#include "live_ekf.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <pidrone_pkg/Mode.h>
#include <pidrone_pkg/MspImu.h>
#include <pidrone_pkg/State.h>
#include <atomic>
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
    p.acceleration_offset=pals::Vec(parameter(node,"acceleration_offset_msp_counts",3).data());
    p.imu_rotation=pals::Rotation(parameter(node,"rotation_body_msp",9).data());
    p.range_offset=pals::Vec(parameter(node,"range_offset_body",3).data());
    p.range_beam=pals::Vec(parameter(node,"range_beam_body",3).data());
    p.camera_offset=pals::Vec(parameter(node,"camera_offset_body",3).data());
    p.camera_rotation=pals::Rotation(parameter(node,"rotation_body_camera",9).data());
    p.validate(); return p;
}
struct Event {
    enum Kind { IMU, RANGE, IMAGE, CAMERA, MODE } kind;
    double received;
    pidrone_pkg::MspImu::ConstPtr imu;
    sensor_msgs::Range::ConstPtr range;
    sensor_msgs::CompressedImage::ConstPtr image;
    sensor_msgs::CameraInfo::ConstPtr camera;
    pidrone_pkg::Mode::ConstPtr mode;
};

class Node {
public:
    Node():private_("~"),profile_(profile(private_)),filter_(profile_) {
        private_.param<std::string>("body_frame",body_frame_,"pidrone_body");
        cv::setNumThreads(1);
        state_pub_=node_.advertise<pidrone_pkg::State>("pidrone/ekf/state",1);
        odom_pub_=node_.advertise<nav_msgs::Odometry>("pidrone/ekf/odometry",1);
        ready_pub_=node_.advertise<std_msgs::Bool>("pidrone/ekf/local_ready",1);
        const auto transport=ros::TransportHints().tcpNoDelay();
        imu_sub_=node_.subscribe("pidrone/imu_msp",100,&Node::imu,this,transport);
        range_sub_=node_.subscribe("pidrone/range",10,&Node::range,this,transport);
        image_sub_=node_.subscribe("raspicam_node/image/compressed",10,&Node::image,this,transport);
        camera_sub_=node_.subscribe("raspicam_node/camera_info",1,&Node::camera,this);
        mode_sub_=node_.subscribe("pidrone/mode",10,&Node::mode,this,transport);
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
    ros::Publisher state_pub_,odom_pub_,ready_pub_;
    ros::Subscriber imu_sub_,range_sub_,image_sub_,camera_sub_,mode_sub_;
    ros::WallTimer timer_;
    std::string body_frame_;
    std::thread worker_;
    std::mutex queue_mutex_,snapshot_mutex_;
    std::condition_variable available_;
    std::deque<Event> queue_;
    bool stopping_=false;
    std::atomic<bool> failed_{false};
    pals::Snapshot snapshot_;
    // Only the ROS callback thread owns the current-time IMU history and output.
    struct RawImu { std::int64_t stamp; pals::Vec force,rate; };
    std::deque<RawImu> history_;
    std::int64_t last_imu_=0,last_published_=0;
    pals::Filter last_output_;
    bool ready_sent_=false,last_ready_=false;
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
    void imu(const pidrone_pkg::MspImu::ConstPtr& msg) {
        const std::int64_t stamp=msg->header.stamp.toNSec();
        if (stamp<=0 || stamp<=last_imu_) return;
        auto s=snapshot();
        if (last_imu_ && (stamp-last_imu_)/1.e9>.2 && s.initialized) fail("IMU gap; restart the EKF node on the floor");
        last_imu_=stamp;
        const pals::Vec acceleration(msg->accelerometer[0],msg->accelerometer[1],msg->accelerometer[2]);
        const pals::Vec gyro(msg->gyroscope[0],msg->gyroscope[1],msg->gyroscope[2]);
        history_.push_back({stamp,profile_.imu_rotation*((acceleration-profile_.acceleration_offset)*profile_.acceleration_scale),
                           profile_.imu_rotation*(gyro*profile_.gyro_scale)});
        while (!history_.empty() && (stamp-history_.front().stamp)/1.e9>1.) history_.pop_front();
        Event e; e.kind=Event::IMU; e.imu=msg; enqueue(e);
        try { publish(s,stamp); }
        catch (const std::exception& error) { fail(error.what()); publish_ready(false,error.what()); }
    }
    void publish_ready(bool value,const std::string& reason) {
        const ros::WallTime now=ros::WallTime::now();
        if (!ready_sent_ || value!=last_ready_ || (now-ready_at_).toSec()>=.05) {
            std_msgs::Bool msg; msg.data=value; ready_pub_.publish(msg);
            ready_sent_=true; last_ready_=value; ready_at_=now;
        }
        if (!value) ROS_INFO_THROTTLE(5,"EKF: %s",reason.c_str());
    }
    void watchdog(const ros::WallTimerEvent&) {
        const auto s=snapshot(); std::string reason;
        const bool ready=s.ready(ros::Time::now().toSec(),last_published_ ? last_output_ : s.filter,reason);
        publish_ready(!failed_ && ready,reason);
    }
    void publish(const pals::Snapshot& s,std::int64_t stamp) {
        const double now=ros::Time::now().toSec(),age=now-stamp/1.e9;
        if (failed_ || !s.fault.empty() || !s.initialized || stamp<=last_published_
            || age<0. || age>.15 || !s.range_ready(now)) { watchdog(ros::WallTimerEvent()); return; }
        if ((stamp-s.origin_ns)/1.e9-s.filter.stamp>.5) {
            publish_ready(false,"Delayed fusion is more than 0.5 seconds behind"); return;
        }
        std::vector<pals::Imu> intervals;
        for (const auto& raw:history_) {
            const double relative=(raw.stamp-s.origin_ns)/1.e9;
            if (relative>s.filter.stamp) intervals.push_back({relative,raw.force,raw.rate});
        }
        pals::Filter estimate=s.filter;
        if (!estimate.project(intervals)) {
            fail("Invalid output IMU interval or projection");
            publish_ready(false,"Invalid output IMU interval or projection"); return;
        }
        const pals::Output output=pals::body_output(estimate,history_.back().rate);
        nav_msgs::Odometry odom;
        odom.header.stamp.fromNSec(stamp); odom.header.frame_id="ekf_odom"; odom.child_frame_id=body_frame_;
        auto& p=odom.pose.pose.position; p.x=estimate.state[0]; p.y=estimate.state[1]; p.z=estimate.state[2];
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
        last_output_=estimate; last_published_=stamp;
        std::string reason; publish_ready(s.ready(after,estimate,reason),reason);
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
                    const auto& m=*e.imu;
                    filter_.imu(m.header.stamp.toNSec(),pals::Vec(m.accelerometer[0],m.accelerometer[1],m.accelerometer[2]),
                                pals::Vec(m.gyroscope[0],m.gyroscope[1],m.gyroscope[2]),e.received);
                } else if (e.kind==Event::RANGE) {
                    const auto& m=*e.range; filter_.range(m.header.stamp.toNSec(),m.range,m.min_range,m.max_range);
                } else if (e.kind==Event::IMAGE) filter_.image(e.image->header.stamp.toNSec(),e.image->data);
                else if (e.kind==Event::CAMERA) {
                    const auto& m=*e.camera;
                    filter_.camera(m.width,m.height,std::vector<double>(m.K.begin(),m.K.end()),m.D,m.distortion_model);
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
