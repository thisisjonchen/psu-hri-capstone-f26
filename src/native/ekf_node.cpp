#include "live_ekf.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <pidrone_pkg/Mode.h>
#include <pidrone_pkg/IntegratedImu.h>
#include <pidrone_pkg/State.h>
#include <pidrone_pkg/EkfVertical.h>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace {
void numbers(XmlRpc::XmlRpcValue value,std::vector<double>& out) {
    if (value.getType()==XmlRpc::XmlRpcValue::TypeArray) {
        for (int i=0;i<value.size();++i) numbers(value[i],out);
    } else if (value.getType()==XmlRpc::XmlRpcValue::TypeDouble) out.push_back(static_cast<double>(value));
    else if (value.getType()==XmlRpc::XmlRpcValue::TypeInt) out.push_back(static_cast<int>(value));
    else throw std::invalid_argument("Calibration must contain numbers");
}
std::vector<double> parameter(ros::NodeHandle& node,const std::string& key,std::size_t count) {
    XmlRpc::XmlRpcValue value;
    if (!node.getParam("calibration/"+key,value)) throw std::invalid_argument("Missing calibration: "+key);
    std::vector<double> result; numbers(value,result);
    if (result.size()!=count) throw std::invalid_argument("Invalid calibration size: "+key);
    return result;
}
pals::Profile profile(ros::NodeHandle& node) {
    pals::Profile p;
    p.acceleration_scale=parameter(node,"acceleration_m_s2_per_count",1)[0];
    p.gyro_scale=parameter(node,"gyro_rad_s_per_count",1)[0];
    p.delay=parameter(node,"fusion_delay_s",1)[0];
    node.param("calibration/camera_delay_s",p.camera_delay,0.);
    p.uwb_delay=parameter(node,"uwb_delay_s",1)[0];
    p.acceleration_offset=pals::Vec(parameter(node,"acceleration_offset_msp_counts",3).data());
    p.imu_rotation=pals::Rotation(parameter(node,"rotation_body_msp",9).data());
    p.range_offset=pals::Vec(parameter(node,"range_offset_body",3).data());
    p.range_beam=pals::Vec(parameter(node,"range_beam_body",3).data());
    p.camera_offset=pals::Vec(parameter(node,"camera_offset_body",3).data());
    p.uwb_offset=pals::Vec(parameter(node,"uwb_offset_body",3).data());
    p.camera_rotation=pals::Rotation(parameter(node,"rotation_body_camera",9).data());
    p.validate(); return p;
}
struct Event {
    enum Kind { IMU, RANGE, IMAGE, CAMERA, MODE, UWB } kind;
    double received;
    std::int64_t stamp=0;
    pals::Imu imu;
    sensor_msgs::Range::ConstPtr range;
    sensor_msgs::CompressedImage::ConstPtr image;
    sensor_msgs::CameraInfo::ConstPtr camera;
    pidrone_pkg::Mode::ConstPtr mode;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr uwb;
};

class Node {
public:
    Node():private_("~"),profile_(profile(private_)),filter_(profile_) {
        private_.param<std::string>("body_frame",body_frame_,"pidrone_body");
        private_.param<std::string>("frame_id",frame_,"map");
        cv::setNumThreads(1);
        state_pub_=node_.advertise<pidrone_pkg::State>("pidrone/ekf/state",1);
        odom_pub_=node_.advertise<nav_msgs::Odometry>("pidrone/ekf/odometry",1);
        ready_pub_=node_.advertise<std_msgs::Bool>("pidrone/ekf/local_ready",1);
        vertical_pub_=node_.advertise<pidrone_pkg::EkfVertical>("pidrone/ekf/vertical",1);
        const auto transport=ros::TransportHints().tcpNoDelay();
        imu_sub_=node_.subscribe("pidrone/imu_integrated",100,&Node::imu,this,transport);
        range_sub_=node_.subscribe("pidrone/range",10,&Node::range,this,transport);
        image_sub_=node_.subscribe("raspicam_node/image/compressed",10,&Node::image,this,transport);
        camera_sub_=node_.subscribe("raspicam_node/camera_info",1,&Node::camera,this);
        mode_sub_=node_.subscribe("pidrone/mode",10,&Node::mode,this,transport);
        uwb_sub_=node_.subscribe("pidrone/localization/uwb",10,&Node::uwb,this,transport);
        timer_=node_.createWallTimer(ros::WallDuration(.05),&Node::watchdog,this);
        worker_=std::thread(&Node::work,this);
    }
    ~Node() {
        { std::lock_guard<std::mutex> lock(queue_mutex_); stopping_=true; }
        available_.notify_one(); worker_.join();
    }
private:
    ros::NodeHandle node_,private_;
    pals::Profile profile_;
    pals::LiveEKF filter_;
    ros::Publisher state_pub_,odom_pub_,ready_pub_,vertical_pub_;
    ros::Subscriber imu_sub_,range_sub_,image_sub_,camera_sub_,mode_sub_,uwb_sub_;
    ros::WallTimer timer_;
    std::string body_frame_,frame_;
    std::thread worker_;
    std::mutex queue_mutex_,snapshot_mutex_;
    std::condition_variable available_;
    std::deque<Event> queue_;
    bool stopping_=false;
    std::atomic<bool> failed_{false};
    pals::Snapshot snapshot_;
    // Only the ROS callback thread owns the current-time IMU history and output.
    struct TimedImu { std::int64_t stamp; pals::Imu sample; pals::Vec rate; };
    std::deque<TimedImu> history_;
    std::int64_t last_imu_=0,last_published_=0;
    std::uint32_t last_sequence_=0,last_sample_us_=0;
    pals::Filter last_output_;
    std::array<double,2> vertical_output_ = {};
    std::deque<pals::VerticalOutputSample> vertical_history_;
    bool ready_sent_=false,last_ready_=false;
    std::string last_ready_reason_;
    ros::WallTime ready_at_;

    pals::Snapshot snapshot() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_); return snapshot_;
    }
    void fail(const std::string& reason) {
        if (failed_.exchange(true)) return;
        { std::lock_guard<std::mutex> lock(snapshot_mutex_); snapshot_.fault=reason; }
        ROS_ERROR("EKF stopped: %s",reason.c_str());
    }
    void enqueue(Event event) {
        if (failed_) return;
        event.received=ros::Time::now().toSec();
        bool full=false;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            full=queue_.size()>=256;
            if (!full) queue_.push_back(std::move(event));
        }
        if (full) fail("Input queue overflow; restart the EKF node");
        else available_.notify_one();
    }
    void range(const sensor_msgs::Range::ConstPtr& msg) { Event e; e.kind=Event::RANGE; e.range=msg; enqueue(e); }
    void image(const sensor_msgs::CompressedImage::ConstPtr& msg) { Event e; e.kind=Event::IMAGE; e.image=msg; enqueue(e); }
    void camera(const sensor_msgs::CameraInfo::ConstPtr& msg) { Event e; e.kind=Event::CAMERA; e.camera=msg; enqueue(e); }
    void mode(const pidrone_pkg::Mode::ConstPtr& msg) { Event e; e.kind=Event::MODE; e.mode=msg; enqueue(e); }
    void uwb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        if (msg->header.frame_id!=frame_) { ROS_WARN_THROTTLE(5.,"Ignoring UWB in frame %s",msg->header.frame_id.c_str()); return; }
        Event e; e.kind=Event::UWB; e.uwb=msg; enqueue(e);
    }
    void imu(const pidrone_pkg::IntegratedImu::ConstPtr& msg) {
        const std::int64_t stamp=msg->header.stamp.toNSec();
        if (stamp<=0 || stamp<=last_imu_) return;
        if (!msg->valid) {
            ROS_WARN_THROTTLE(1.,"FC reported an invalid IMU interval; skipping sample");
            return;
        }
        if (last_imu_ && (std::uint32_t(msg->sequence-last_sequence_)==0
            || std::uint32_t(msg->sequence-last_sequence_)>=0x80000000U
            || std::uint32_t(msg->sample_time_us-last_sample_us_)!=msg->integration_time_us)) {
            fail("Integrated IMU continuity lost; restart localization on the floor"); return;
        }
        pals::Imu sample; pals::Vec force,rate;
        try {
            sample=pals::calibrated_imu(last_imu_ ? (stamp-last_imu_)/1.e9 : msg->integration_time_us*1.e-6,
                pals::Quaternion(msg->delta_rotation[0],msg->delta_rotation[1],msg->delta_rotation[2],msg->delta_rotation[3]),
                pals::Vec(msg->delta_velocity[0],msg->delta_velocity[1],msg->delta_velocity[2]),profile_);
            sample.means(force,rate);
        } catch (const std::exception& error) { fail(error.what()); return; }
        last_sequence_=msg->sequence; last_sample_us_=msg->sample_time_us;
        auto s=snapshot();
        if (last_imu_ && (stamp-last_imu_)/1.e9>.2 && s.initialized) fail("IMU gap; restart the EKF node on the floor");
        last_imu_=stamp;
        history_.push_back({stamp,sample,rate});
        while (!history_.empty() && (stamp-history_.front().stamp)/1.e9>1.) history_.pop_front();
        Event e; e.kind=Event::IMU; e.stamp=stamp; e.imu=sample; enqueue(e);
        try { publish(s,stamp); }
        catch (const std::exception& error) { fail(error.what()); publish_ready(false,error.what()); }
    }
    void publish_ready(bool value,const std::string& reason) {
        const ros::WallTime now=ros::WallTime::now();
        if (reason!=last_ready_reason_) {
            ROS_INFO("EKF: %s",reason.c_str());
            last_ready_reason_=reason;
        }
        if (!ready_sent_ || value!=last_ready_ || (now-ready_at_).toSec()>=.05) {
            std_msgs::Bool msg; msg.data=value; ready_pub_.publish(msg);
            ready_sent_=true; last_ready_=value; ready_at_=now;
        }
    }
    void watchdog(const ros::WallTimerEvent&) {
        const auto s=snapshot(); std::string reason;
        if (s.uwb_rejections>=20) ROS_WARN_THROTTLE(5.,"EKF: %d consecutive UWB fixes rejected",s.uwb_rejections);
        const bool ready=s.ready(ros::Time::now().toSec(),last_published_ ? last_output_ : s.filter,reason);
        publish_ready(!failed_ && ready,reason);
    }
    void publish(const pals::Snapshot& s,std::int64_t stamp) {
        const double now=ros::Time::now().toSec(),age=now-stamp/1.e9;
        if (failed_ || !s.fault.empty() || !s.initialized || stamp<=last_published_
            || age<0. || age>.15 || !s.range_ready(now)) { watchdog(ros::WallTimerEvent()); return; }
        if ((stamp-s.origin_ns)/1.e9-s.filter.stamp>profile_.delay+.35) {
            publish_ready(false,"Delayed fusion is more than 0.35 seconds behind its horizon"); return;
        }
        std::vector<pals::Imu> intervals;
        const double first=last_published_ ? std::min(s.filter.stamp,last_output_.stamp) : s.filter.stamp;
        for (const auto& raw:history_) {
            const double relative=(raw.stamp-s.origin_ns)/1.e9;
            if (relative>first) {
                auto sample=raw.sample; sample.stamp=relative; intervals.push_back(sample);
            }
        }
        pals::Filter estimate=s.filter;
        if (!estimate.project(intervals)) {
            fail("Invalid output IMU interval or projection");
            publish_ready(false,"Invalid output IMU interval or projection"); return;
        }
        auto vertical=vertical_output_;
        if (!pals::vertical_output(estimate,last_published_ ? &last_output_ : nullptr,intervals,vertical)) {
            fail("Invalid vertical output prediction");
            publish_ready(false,"Invalid vertical output prediction"); return;
        }
        pals::Output output=pals::body_output(estimate,history_.back().rate);
        const double height_error=vertical[0]-estimate.state[2];
        const pals::Vec velocity_error=pals::rotation(output.quaternion).t()
            *pals::Vec(0.,0.,vertical[1]-estimate.state[5]);
        output.velocity+=velocity_error;
        // Include output tracking error without changing the EKF covariance or readiness.
        output.pose_covariance[14]+=height_error*height_error;
        for (int i=0;i<3;++i) for (int j=0;j<3;++j)
            output.twist_covariance[6*i+j]+=velocity_error[i]*velocity_error[j];
        nav_msgs::Odometry odom;
        odom.header.stamp.fromNSec(stamp); odom.header.frame_id=frame_; odom.child_frame_id=body_frame_;
        auto& p=odom.pose.pose.position; p.x=estimate.state[0]; p.y=estimate.state[1]; p.z=vertical[0];
        auto& q=odom.pose.pose.orientation;
        q.x=output.quaternion[0]; q.y=output.quaternion[1]; q.z=output.quaternion[2]; q.w=output.quaternion[3];
        auto& v=odom.twist.twist.linear; v.x=output.velocity[0]; v.y=output.velocity[1]; v.z=output.velocity[2];
        auto& w=odom.twist.twist.angular; w.x=output.rate[0]; w.y=output.rate[1]; w.z=output.rate[2];
        std::copy(output.pose_covariance.begin(),output.pose_covariance.end(),odom.pose.covariance.begin());
        std::copy(output.twist_covariance.begin(),output.twist_covariance.end(),odom.twist.covariance.begin());
        const double after=ros::Time::now().toSec();
        if (failed_ || after-stamp/1.e9<0. || after-stamp/1.e9>.15 || !s.range_ready(after)) {
            watchdog(ros::WallTimerEvent()); return;
        }
        pidrone_pkg::State state;
        state.header=odom.header; state.pose_with_covariance=odom.pose; state.twist_with_covariance=odom.twist;
        state_pub_.publish(state); odom_pub_.publish(odom);
        last_output_=estimate; vertical_output_=vertical; last_published_=stamp;
        std::string reason; publish_ready(s.ready(after,estimate,reason),reason);
        // Diagnostics must not turn a publication failure into an estimator fault.
        try { publish_vertical(s,estimate,vertical,odom.header); }
        catch (const std::exception& error) { ROS_WARN_THROTTLE(5.,"Vertical diagnostics: %s",error.what()); }
    }
    void publish_vertical(const pals::Snapshot& s,const pals::Filter& estimate,
                          const std::array<double,2>& vertical,const std_msgs::Header& header) {
        if (!vertical_pub_.getNumSubscribers()) return;
        vertical_history_.push_back({estimate.stamp,vertical[0],vertical[1]});
        while (vertical_history_.size()>1 && estimate.stamp-vertical_history_[1].stamp>1.)
            vertical_history_.pop_front();
        std::array<double,2> tracking;
        pidrone_pkg::EkfVertical msg;
        msg.header=header;
        msg.fusion_stamp.fromNSec(s.origin_ns+std::llround(s.filter.stamp*1.e9));
        msg.height_fusion=s.filter.position()[2]; msg.velocity_fusion=s.filter.velocity()[2];
        msg.output_tracking_valid=pals::vertical_tracking_error(s.filter,vertical_history_,tracking);
        msg.height_tracking_error=tracking[0]; msg.velocity_tracking_error=tracking[1];
        msg.height_ekf=estimate.position()[2]; msg.height_output=vertical[0];
        msg.velocity_ekf=estimate.velocity()[2]; msg.velocity_output=vertical[1];
        msg.accel_bias_body_z=estimate.acceleration_bias()[2];
        msg.accel_bias_up=(pals::rotation(estimate.quaternion())*estimate.acceleration_bias())[2];
        const auto& range=s.range_correction;
        msg.range_sequence=range.sequence;
        if (range.sequence) msg.range_stamp.fromNSec(s.origin_ns+std::llround(range.stamp*1.e9));
        msg.range_innovation=range.innovation;
        msg.range_accepted=range.accepted; msg.range_ground_reset=range.ground_reset;
        vertical_pub_.publish(msg);
    }
    void work() {
        while (true) {
            Event e;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                available_.wait(lock,[this]{ return stopping_ || !queue_.empty(); });
                if (stopping_) return;
                e=std::move(queue_.front()); queue_.pop_front();
            }
            if (failed_) continue;
            try {
                if (e.kind==Event::IMU) {
                    filter_.imu(e.stamp,e.imu,e.received);
                } else if (e.kind==Event::RANGE) {
                    const auto& m=*e.range; filter_.range(m.header.stamp.toNSec(),m.range,m.min_range,m.max_range);
                } else if (e.kind==Event::IMAGE) filter_.image(e.image->header.stamp.toNSec(),e.image->data);
                else if (e.kind==Event::CAMERA) {
                    const auto& m=*e.camera;
                    filter_.camera(m.width,m.height,std::vector<double>(m.K.begin(),m.K.end()),m.D,m.distortion_model);
                } else if (e.kind==Event::UWB) {
                    const auto& m=*e.uwb;
                    filter_.uwb(m.header.stamp.toNSec(),m.pose.pose.position.x,m.pose.pose.position.y,
                                std::max(m.pose.covariance[0],m.pose.covariance[7]));
                } else filter_.mode(e.mode->mode,e.received);
                const auto completed=filter_.snapshot();
                if (!completed.fault.empty()) { fail(completed.fault); continue; }
                // Never hold this lock during tracking, fusion, or projection.
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                if (!failed_) snapshot_=completed;
            } catch (const std::exception& error) { fail(error.what()); }
        }
    }
};
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"ekf_localization");
    try { Node node; ros::spin(); }
    catch (const std::exception& error) { ROS_FATAL("EKF startup failed: %s",error.what()); return 1; }
    return 0;
}
