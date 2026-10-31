#include "live_ekf.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pals {
namespace {
const Vec gravity(0., 0., -9.8);
std::array<double, 225> density() {
    std::array<double, 225> result = {};
    const double noise[] = {0., .3*.3, .015*.015, .02*.02, .001*.001};
    for (int i=0; i<15; ++i) result[16*i]=noise[i/3];
    return result;
}
const auto noise = density();
bool finite(const Vec& v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }
void require(bool valid, const char* reason) { if (!valid) throw std::invalid_argument(reason); }
bool proper(const Rotation& r) {
    return cv::checkRange(cv::Mat(r)) && cv::norm(cv::Mat(r.t()*r-Rotation::eye()), cv::NORM_INF)<=1.e-8
        && std::abs(cv::determinant(r)-1.)<=1.e-8;
}
cv::Mat covariance_view(const Filter& f) {
    return cv::Mat(15,15,CV_64F,const_cast<double*>(f.covariance.data()));
}
void block(cv::Mat& m, int row, int column, const Rotation& value) {
    cv::Mat(value).copyTo(m(cv::Rect(column,row,3,3)));
}
cv::Mat column(const Vec& v) { return cv::Mat(v).clone(); }
std::vector<int> spread(int size, int limit) {
    std::vector<int> result;
    const int count=std::min(size,limit);
    for (int i=0; i<count; ++i) result.push_back(size<=limit ? i : int(double(i)*(size-1)/(limit-1)));
    return result;
}
double median(std::vector<double> values) {
    std::sort(values.begin(),values.end());
    const std::size_t n=values.size();
    return (values[n/2]+values[(n-1)/2])/2.;
}
}

Rotation skew(const Vec& v) { return Rotation(0.,-v[2],v[1],v[2],0.,-v[0],-v[1],v[0],0.); }
Rotation rotation(const Quaternion& q) {
    const double n=cv::norm(q);
    require(std::isfinite(n) && n>=1.e-6,"Invalid quaternion");
    const double x=q[0]/n,y=q[1]/n,z=q[2]/n,w=q[3]/n;
    return Rotation(1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                    2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                    2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y));
}
Quaternion product(const Quaternion& a, const Quaternion& b) {
    return Quaternion(a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
        a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
        a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
        a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]);
}
void Profile::validate() const {
    require(proper(imu_rotation) && proper(camera_rotation),"Mounting must be a proper rotation");
    require(std::isfinite(acceleration_scale) && acceleration_scale>0. && std::isfinite(gyro_scale)
            && gyro_scale>0.,"Positive finite IMU scales are required");
    require(finite(acceleration_offset) && finite(range_offset) && finite(camera_offset)
            && finite(range_beam) && std::abs(cv::norm(range_beam)-1.)<=1.e-8,"Invalid sensor geometry");
    require(std::isfinite(delay) && delay>=0. && delay<=.4,"Live fusion delay must be in [0, 0.4]");
    require(std::isfinite(camera_delay) && camera_delay>=0. && camera_delay<=delay,
            "Camera delay must fit within the fusion delay");
}
void Filter::initialize(double height, const Quaternion& q, const Vec& ba, const Vec& bg) {
    state.fill(0.); state[2]=height;
    std::copy(q.val,q.val+4,state.begin()+6);
    std::copy(ba.val,ba.val+3,state.begin()+10);
    std::copy(bg.val,bg.val+3,state.begin()+13);
    const double pi=std::acos(-1.);
    const double sd[]={.1,.1,.1,.2,.2,.2,pi/18,pi/18,pi/4,.5,.5,.5,.05,.05,.05};
    cv::Mat initial=cv::Mat::zeros(15,15,CV_64F), transform=cv::Mat::eye(15,15,CV_64F);
    for (int i=0; i<15; ++i) initial.at<double>(i,i)=sd[i]*sd[i];
    const Rotation r=rotation(q);
    block(transform,0,6,skew(position())*r); block(transform,3,6,skew(velocity())*r); block(transform,6,6,r);
    cv::Mat converted=transform*initial*transform.t();
    std::copy(converted.ptr<double>(),converted.ptr<double>()+225,covariance.begin());
    stamp=0.;
}
bool Filter::predict(const Imu& sample, double end) {
    if (!(end>stamp && end-stamp<=.2)) return false;
    auto s=state; auto p=covariance;
    const Vec force=sample.force-acceleration_bias(), rate=sample.rate-gyro_bias();
    if (pals_ekf_predict(s.data(),p.data(),force.val,rate.val,gravity.val,noise.data(),end-stamp)) return false;
    state=s; covariance=p; stamp=end;
    return true;
}
bool Filter::project(const std::vector<Imu>& samples) {
    if (samples.empty()) return true;
    std::vector<double> intervals; intervals.reserve(samples.size()*7);
    for (const auto& s:samples) {
        if (s.stamp<=stamp) continue;
        intervals.push_back(s.stamp);
        intervals.insert(intervals.end(),s.force.val,s.force.val+3);
        intervals.insert(intervals.end(),s.rate.val,s.rate.val+3);
    }
    if (intervals.empty()) return true;
    auto s=state; auto p=covariance;
    if (pals_ekf_project(s.data(),p.data(),intervals.data(),intervals.size()/7,
        state.data()+10,state.data()+13,gravity.val,noise.data(),stamp,.2)) return false;
    state=s; covariance=p; stamp=intervals[intervals.size()-7];
    return true;
}
bool Filter::correct(const cv::Mat& innovation, const cv::Mat& h, const cv::Mat& r, double gate) {
    if (!cv::checkRange(innovation) || !cv::checkRange(h) || !cv::checkRange(r)) return false;
    auto s=state; auto p=covariance;
    cv::Mat total(r.rows,r.cols,CV_64F); double nis=0.;
    if (pals_ekf_correct(s.data(),p.data(),innovation.ptr<double>(),h.ptr<double>(),r.ptr<double>(),
                         innovation.rows,gate,total.ptr<double>(),&nis)) return false;
    state=s; covariance=p;
    return true;
}
cv::Mat Filter::public_covariance() const {
    cv::Mat t=cv::Mat::eye(15,15,CV_64F);
    block(t,0,6,-skew(position())); block(t,3,6,-skew(velocity()));
    block(t,6,6,rotation(quaternion()).t());
    cv::Mat p=t*covariance_view(*this)*t.t();
    return .5*(p+p.t());
}
bool Filter::navigation_variance_valid() const {
    const Rotation r=rotation(quaternion());
    for (int start:{0,3}) for (int axis=0; axis<3; ++axis) {
        double variance=0.;
        for (int i=0;i<3;++i) for (int j=0;j<3;++j)
            variance+=r(i,axis)*covariance[15*(start+i)+start+j]*r(j,axis);
        if (!std::isfinite(variance) || variance<0. || variance>.25) return false;
    }
    return true;
}
Output body_output(const Filter& f, const Vec& angular_rate) {
    const Rotation r=rotation(f.quaternion()), flu_to_rfu(0.,-1.,0.,1.,0.,0.,0.,0.,1.);
    const Vec body_velocity=r.t()*f.velocity();
    Output result;
    result.quaternion=product(f.quaternion(),Quaternion(0.,0.,-std::sqrt(.5),std::sqrt(.5)));
    result.velocity=flu_to_rfu*body_velocity;
    result.rate=flu_to_rfu*(angular_rate-f.gyro_bias());
    cv::Mat pose=cv::Mat::zeros(6,15,CV_64F), twist=cv::Mat::zeros(6,15,CV_64F);
    block(pose,0,0,Rotation::eye()); block(pose,3,6,r);
    block(twist,0,3,flu_to_rfu*r.t()); block(twist,0,6,flu_to_rfu*skew(body_velocity));
    block(twist,3,12,-flu_to_rfu);
    const cv::Mat p=f.public_covariance();
    cv::Mat pc=pose*p*pose.t(),tc=twist*p*twist.t();
    for (int i=3;i<6;++i) tc.at<double>(i,i)+=.015*.015/.01;
    require(cv::checkRange(pc) && cv::checkRange(tc) && finite(result.velocity) && finite(result.rate),"Nonfinite output");
    std::copy(pc.ptr<double>(),pc.ptr<double>()+36,result.pose_covariance.begin());
    std::copy(tc.ptr<double>(),tc.ptr<double>()+36,result.twist_covariance.begin());
    return result;
}
bool Snapshot::range_ready(double now) const {
    const double age=now-origin_ns/1.e9-range_aid;
    return initialized && age>=0. && age<=.5;
}
bool Snapshot::ready(double now, const Filter& estimate, std::string& reason) const {
    const double current=now-origin_ns/1.e9;
    if (!fault.empty()) reason=fault;
    else if (!initialized) reason="Hold still on the floor for eight seconds while disarmed";
    else if (!reference_valid) reason="Local reference lost; restart on the floor before flight";
    else if (!(current-estimate.stamp>=0. && current-estimate.stamp<=.15)) reason="Waiting for fresh IMU";
    else if (!range_ready(now)) reason="Waiting for accepted range";
    else if (current-horizontal_aid>.5) reason="Coasting; waiting for accepted camera flow";
    else if (!estimate.navigation_variance_valid()) reason="Position or velocity uncertainty is too large";
    else { reason="Ready"; return true; }
    return false;
}

Camera::Camera(int w,int h,const std::vector<double>& k,const std::vector<double>& d,const std::string& name)
    :width(w),height(h),model(name) {
    require(w>0 && h>0 && w%16==0 && h%16==0 && k.size()==9,"Invalid camera dimensions or matrix");
    matrix=cv::Mat(3,3,CV_64F,const_cast<double*>(k.data())).clone();
    distortion=cv::Mat(d).clone();
    require(cv::checkRange(matrix) && cv::checkRange(distortion) && k[0]>0. && k[4]>0.
        && std::abs(k[6])<=1.e-8 && std::abs(k[7])<=1.e-8 && std::abs(k[8]-1.)<=1.001e-5,"Invalid lens calibration");
    require((model=="equidistant" && d.size()==4) || model=="plumb_bob" || model=="rational_polynomial","Unsupported lens model");
    for (int y=8;y<h;y+=16) for (int x=8;x<w;x+=16) centers.emplace_back(x,y);
    rays=undistort(centers);
}
std::vector<Vec> Camera::undistort(const std::vector<cv::Point2d>& pixels) const {
    if (pixels.empty()) return {};
    std::vector<cv::Point2d> points;
    if (model=="equidistant") cv::fisheye::undistortPoints(pixels,points,matrix,distortion);
    else cv::undistortPoints(pixels,points,matrix,distortion);
    std::vector<Vec> result; result.reserve(points.size());
    for (const auto& p:points) result.emplace_back(p.x,p.y,1.);
    return result;
}
LiveEKF::LiveEKF(const Profile& p):profile_(p) { p.validate(); }
void LiveEKF::clear_rest() { rest_since_=0; initial_.clear(); }
void LiveEKF::mode(const std::string& value,double received) {
    mode_=value; mode_at_=received;
    if (!ground_rest_allowed()) clear_rest();
}
bool LiveEKF::ground_rest_allowed() const {
    // FLYING can continue confirmed ground rest, but cannot start a new rest period.
    return mode_=="DISARMED" || (origin_ns_ && (mode_=="ARMED"
        || (mode_=="FLYING" && rest_since_ && was_stationary_)));
}
void LiveEKF::camera(int width,int height,const std::vector<double>& k,const std::vector<double>& d,const std::string& model) {
    if (camera_ && camera_->width==width && camera_->height==height && camera_->model==model
        && camera_->matrix.total()==k.size() && camera_->distortion.total()==d.size()
        && std::equal(k.begin(),k.end(),camera_->matrix.ptr<double>())
        && std::equal(d.begin(),d.end(),camera_->distortion.ptr<double>())) return;
    camera_=std::make_shared<Camera>(width,height,k,d,model);
    last_aid_.erase("camera");
}
void LiveEKF::range(std::int64_t stamp,double distance,double minimum,double maximum) {
    if (stamp<=0 || !std::isfinite(distance) || !std::isfinite(minimum) || !std::isfinite(maximum)
        || !(0.<=minimum && minimum<maximum && distance>=minimum && distance<=maximum)) {
        range_.stamp=0; clear_rest(); return;
    }
    if (stamp<=range_.stamp) return;
    range_.stamp=stamp; range_.value=distance;
    if (origin_ns_ && fault_.empty()) {
        Measurement m; m.stamp=(stamp-origin_ns_)/1.e9; m.sensor="range";
        m.value=distance; queue(m);
    }
}
void LiveEKF::image(std::int64_t stamp,const std::vector<unsigned char>& encoded) {
    if (!origin_ns_ || !camera_ || !fault_.empty() || stamp<=0) return;
    stamp-=std::llround(profile_.camera_delay*1.e9);
    const double end=(stamp-origin_ns_)/1.e9;
    if (std::isfinite(camera_->previous_stamp) && end<=camera_->previous_stamp) return;
    cv::Mat decoded;
    try { decoded=cv::imdecode(encoded,cv::IMREAD_GRAYSCALE); } catch (const cv::Exception&) {}
    Measurement m; m.previous=camera_->previous; m.image=decoded;
    const double start=camera_->previous_stamp;
    camera_->previous=decoded; camera_->previous_stamp=end;
    if (decoded.empty() || decoded.rows!=camera_->height || decoded.cols!=camera_->width) {
        camera_->previous.release(); return;
    }
    if (m.previous.empty() || end-start>.2) return;
    m.sensor="camera"; m.stamp=(start+end)/2.; m.end=end; m.value=start; m.camera=camera_;
    queue(m);
}
bool LiveEKF::ground_rest(std::int64_t stamp,double received,const Vec& force,const Vec& rate) {
    const double range_age=(stamp-range_.stamp)/1.e9;
    bool quiet=ground_rest_allowed()
        && received-mode_at_>=0. && received-mode_at_<=.5 && range_.stamp>0
        && range_age>=-.1 && range_age<=.3 && range_.value>=.015 && range_.value<=.08
        && cv::norm(rate)<.03 && std::abs(cv::norm(force)-9.8)<.5 && force[2]>9.;
    if (quiet && origin_ns_) quiet=cv::norm(rate-filter_.gyro_bias())<.015;
    if (quiet && ground_range_>=0.) quiet=std::abs(range_.value-ground_range_)<.015;
    if (quiet && rest_since_) quiet=cv::norm(force-rest_force_)<.3 && std::abs(range_.value-rest_range_)<.015;
    if (!quiet) { clear_rest(); return false; }
    if (!rest_since_) { rest_since_=stamp; rest_force_=force; rest_range_=range_.value; }
    return (stamp-rest_since_)/1.e9>=.5;
}
void LiveEKF::check_reference(double stamp) {
    const double aiding=std::max(last_aid_.count("camera") ? last_aid_.at("camera") : 0.,
                                 last_aid_.count("zero_velocity") ? last_aid_.at("zero_velocity") : 0.);
    if (stamp-aiding>2.) reference_valid_=false;
}
void LiveEKF::imu(std::int64_t ns,const Vec& acceleration,const Vec& gyro,double received) {
    if (!fault_.empty() || ns<=0) return;
    const Vec force=profile_.imu_rotation*((acceleration-profile_.acceleration_offset)*profile_.acceleration_scale);
    const Vec rate=profile_.imu_rotation*(gyro*profile_.gyro_scale);
    require(finite(force) && finite(rate),"Invalid IMU");
    if (last_imu_ns_) {
        if (ns<=last_imu_ns_) return;
        if ((ns-last_imu_ns_)/1.e9>.2) {
            if (origin_ns_) { fault_="IMU gap; restart the EKF node on the floor"; return; }
            clear_rest();
        }
    }
    last_imu_ns_=ns;
    const bool stationary=ground_rest(ns,received,force,rate);
    if (!origin_ns_) {
        if (!rest_since_) return;
        initial_.push_back({force,rate,range_.value,ns});
        if ((ns-initial_.front().stamp)/1.e9<8.) return;
        Vec mean_force(0.,0.,0.),mean_rate(0.,0.,0.); double distance=0.;
        for (const auto& s:initial_) { mean_force+=s.force; mean_rate+=s.rate; distance+=s.range; }
        const double count=initial_.size(); mean_force/=count; mean_rate/=count; distance/=count;
        const Vec up=mean_force/cv::norm(mean_force),cross=up.cross(Vec(0.,0.,1.));
        Quaternion q(cross[0],cross[1],cross[2],1.+up[2]); q/=cv::norm(q);
        const Rotation r=rotation(q);
        const double height=-(r*profile_.range_offset+distance*(r*profile_.range_beam))[2];
        if (!std::isfinite(height) || height<=0.) { fault_="Invalid initial floor geometry"; return; }
        filter_.initialize(height,q,mean_force-r.t()*Vec(0.,0.,9.8),mean_rate);
        origin_ns_=ns; ground_range_=distance; initial_.clear(); return;
    }
    const double stamp=(ns-origin_ns_)/1.e9,dt=stamp-last_imu_;
    angular_.push_back({last_imu_,stamp,rate});
    while (!angular_.empty() && angular_.front().end<stamp-profile_.delay-.4) angular_.pop_front();
    imu_.push_back({stamp,force,rate}); last_imu_=stamp;
    if (stationary) {
        Measurement m; m.stamp=stamp; m.sensor="zero_velocity"; queue(m);
        if (was_stationary_) {
            stationary_angle_+=rate*dt; stationary_duration_+=dt;
            if (stationary_duration_>=.2) {
                m.sensor="stationary_gyro"; m.rate=stationary_angle_/stationary_duration_; m.value=stationary_duration_; queue(m);
                stationary_angle_=Vec(0.,0.,0.); stationary_duration_=0.;
            }
        }
    } else { stationary_angle_=Vec(0.,0.,0.); stationary_duration_=0.; }
    was_stationary_=stationary;
    drain(stamp-profile_.delay); check_reference(filter_.stamp);
}
void LiveEKF::queue(const Measurement& m) {
    if (m.stamp<0. || m.stamp<filter_.stamp || m.stamp<=closed_through_
        || m.stamp>last_imu_+std::max(profile_.delay,.1) || measurements_.size()>=512) return;
    measurements_.emplace(std::make_pair(m.stamp,m.sensor),m);
}
void LiveEKF::drain(double through) {
    while (!imu_.empty() && fault_.empty()) {
        const Imu sample=imu_.front();
        const double aid=measurements_.empty() ? std::numeric_limits<double>::infinity() : measurements_.begin()->first.first;
        const double stamp=std::min(sample.stamp,aid);
        if (stamp>through) break;
        if (stamp>filter_.stamp && !filter_.predict(sample,stamp)) { fault_="Invalid IMU prediction"; break; }
        if (sample.stamp==stamp) imu_.pop_front();
        while (!measurements_.empty() && measurements_.begin()->first.first==stamp) {
            Measurement m=measurements_.begin()->second; measurements_.erase(measurements_.begin());
            const bool accepted=correct(m);
            check_reference(stamp);
            if (accepted && (m.sensor=="range" || m.sensor=="camera" || m.sensor=="zero_velocity")) {
                last_aid_[m.sensor]=stamp;
            }
        }
    }
    closed_through_=std::max(closed_through_,through);
}
bool LiveEKF::mean_rate(double start,double end,Vec& rate) const {
    if (end<=start || angular_.empty() || start<angular_.front().start || end>angular_.back().end) return false;
    rate=Vec(0.,0.,0.);
    for (const auto& a:angular_) {
        const double overlap=std::min(a.end,end)-std::max(a.start,start);
        if (overlap>0.) rate+=a.rate*overlap;
        if (a.end>=end) break;
    }
    rate/=(end-start); return true;
}
bool LiveEKF::correct(const Measurement& m) {
    if (m.sensor=="camera") {
        Vec rate; if (!mean_rate(m.value,m.end,rate)) return false;
        try { return correct_camera(m,rate); } catch (const cv::Exception&) { return false; }
    }
    cv::Mat h=cv::Mat::zeros(m.sensor=="range" ? 1 : 3,15,CV_64F);
    if (m.sensor=="range") {
        const Rotation r=rotation(filter_.quaternion());
        const Vec center=filter_.position()+r*profile_.range_offset,direction=r*profile_.range_beam;
        // Range can correct a height prediction that has crossed below the floor.
        if (direction[2]>=-.1) return false;
        const double predicted=-center[2]/direction[2];
        h.at<double>(0,2)=-1./direction[2];
        const Rotation a=r*skew(profile_.range_offset),b=r*skew(profile_.range_beam);
        for (int j=0;j<3;++j) h.at<double>(0,6+j)=(a(2,j)+predicted*b(2,j))/direction[2];
        return filter_.correct((cv::Mat_<double>(1,1)<<m.value-predicted),h,cv::Mat::eye(1,1,CV_64F)*(.02*.02),6.63);
    }
    if (m.sensor=="zero_velocity") {
        block(h,0,3,Rotation::eye()); block(h,0,6,skew(filter_.velocity())*rotation(filter_.quaternion()));
        return filter_.correct(column(-filter_.velocity()),h,cv::Mat::eye(3,3,CV_64F)*(.02*.02),11.34);
    }
    block(h,0,12,Rotation::eye());
    return filter_.correct(column(m.rate-filter_.gyro_bias()),h,cv::Mat::eye(3,3,CV_64F)*(.015*.015/m.value),11.34);
}
bool LiveEKF::correct_camera(const Measurement& m,const Vec& rate) {
    const double duration=m.end-m.value;
    if (!(duration>0. && duration<=.2)) return false;
    const auto& camera=*m.camera;
    const Rotation r=rotation(filter_.quaternion());
    const double height=(filter_.position()+r*profile_.camera_offset)[2];
    const Vec velocity=filter_.velocity()+r*((rate-filter_.gyro_bias()).cross(profile_.camera_offset));
    if (height-std::abs(velocity[2])*duration/2.<.1) return false;
    std::vector<std::int64_t> indices(camera.rays.size());
    const auto& s=filter_.state;
    const auto count=pals_camera_indices(s.data(),s.data()+6,profile_.camera_rotation.val,
        profile_.camera_offset.val,camera.rays.front().val,camera.rays.size(),0.,.1,1.5,indices.data());
    if (count<8) return false;
    std::vector<cv::Point2f> points,back,forward;
    for (int i:spread(count,48)) points.emplace_back(camera.centers[indices[i]]);
    std::vector<unsigned char> found_back,found_forward; std::vector<float> error_back,error_forward;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01);
    cv::calcOpticalFlowPyrLK(m.image,m.previous,points,back,found_back,error_back,cv::Size(15,15),2,criteria,0,1.e-4);
    cv::calcOpticalFlowPyrLK(m.previous,m.image,back,forward,found_forward,error_forward,cv::Size(15,15),2,criteria,0,1.e-4);
    std::vector<cv::Point2d> current,previous;
    for (std::size_t i=0;i<points.size();++i) {
        if (found_back[i] && found_forward[i] && cv::norm(forward[i]-points[i])<=1.
            && error_back[i]<=15. && error_forward[i]<=15. && std::isfinite(back[i].x) && std::isfinite(back[i].y)
            && back[i].x>=0. && back[i].x<camera.width && back[i].y>=0. && back[i].y<camera.height) {
            current.emplace_back(points[i]); previous.emplace_back(back[i]);
        }
    }
    std::vector<cv::Point2d> selected_current,selected_previous;
    for (int i:spread(current.size(),24)) { selected_current.push_back(current[i]); selected_previous.push_back(previous[i]); }
    if (selected_current.size()<8) return false;
    const auto rays=camera.undistort(selected_current),previous_rays=camera.undistort(selected_previous);
    std::vector<Vec> midpoint; for (std::size_t i=0;i<rays.size();++i) midpoint.push_back((rays[i]+previous_rays[i])/2.);
    std::vector<double> predicted(2*rays.size()),jacobians(30*rays.size()); indices.resize(rays.size());
    const auto used=pals_camera_linearize(s.data(),s.data()+3,s.data()+6,s.data()+13,rate.val,
        profile_.camera_rotation.val,profile_.camera_offset.val,midpoint.front().val,midpoint.size(),0.,.1,1.5,
        indices.data(),predicted.data(),jacobians.data());
    if (used<8) return false;
    std::vector<cv::Point2d> shifted_x=selected_previous,shifted_y=selected_previous;
    for (auto& p:shifted_x) p.x+=.01;
    for (auto& p:shifted_y) p.y+=.01;
    const auto rx=camera.undistort(shifted_x),ry=camera.undistort(shifted_y);
    std::vector<cv::Vec2d> measured,residuals; std::vector<cv::Matx22d> noises;
    std::vector<double> residual_x,residual_y;
    for (int i=0;i<used;++i) {
        const int n=indices[i];
        const Vec diff=(previous_rays[n]-rays[n])/duration,dx=(rx[n]-previous_rays[n])/.01,dy=(ry[n]-previous_rays[n])/.01;
        const cv::Matx22d j(dx[0],dy[0],dx[1],dy[1]);
        measured.emplace_back(diff[0],diff[1]);
        residuals.emplace_back(diff[0]-predicted[2*i],diff[1]-predicted[2*i+1]);
        residual_x.push_back(residuals.back()[0]); residual_y.push_back(residuals.back()[1]);
        noises.push_back(j*j.t()*std::pow(1.5/duration,2));
    }
    const cv::Vec2d center(median(residual_x),median(residual_y));
    cv::Mat h=cv::Mat::zeros(2,15,CV_64F),information=cv::Mat::zeros(2,2,CV_64F),innovation=cv::Mat::zeros(2,1,CV_64F);
    int kept=0;
    for (int i=0;i<used;++i) {
        const auto& covariance=noises[i]; const cv::Vec2d delta=residuals[i]-center;
        const double determinant=cv::determinant(covariance);
        if (!std::isfinite(determinant) || determinant<=0.) return false;
        const double score=delta.dot(covariance.inv()*delta);
        if (!std::isfinite(score) || score>9.21) continue;
        ++kept;
        const cv::Mat weight=cv::Mat(covariance.inv());
        h+=weight*cv::Mat(2,15,CV_64F,jacobians.data()+30*i); information+=weight;
        innovation+=weight*cv::Mat(residuals[i]);
    }
    if (kept<8) return false;
    // Weight each match by its normalized-image uncertainty.
    cv::Mat n=information.inv(); h=n*h; innovation=n*innovation;
    const cv::Mat gyro_projection=h(cv::Rect(12,0,3,2));
    n+=gyro_projection*gyro_projection.t()*(.015*.015/duration);
    return filter_.correct(innovation,h,n,9.21);
}
Snapshot LiveEKF::snapshot() const {
    Snapshot s; s.initialized=origin_ns_!=0; s.origin_ns=origin_ns_; s.filter=filter_;
    s.reference_valid=reference_valid_; s.fault=fault_;
    if (last_aid_.count("range")) s.range_aid=last_aid_.at("range");
    for (const char* name:{"camera","zero_velocity"})
        if (last_aid_.count(name)) s.horizontal_aid=std::max(s.horizontal_aid,last_aid_.at(name));
    return s;
}
}
