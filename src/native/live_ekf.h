#pragma once
#include "ekf.h"
#include <opencv2/core.hpp>
#include <array>
#include <deque>
#include <map>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace pals {
using Vec = cv::Vec3d;
using Rotation = cv::Matx33d;
using Quaternion = cv::Vec4d;
Rotation rotation(const Quaternion& q);
Rotation skew(const Vec& v);
Quaternion product(const Quaternion& a, const Quaternion& b);

struct Profile {
    double acceleration_scale, gyro_scale, delay;
    double camera_delay = 0.;
    Vec acceleration_offset, range_offset, range_beam, camera_offset;
    Rotation imu_rotation, camera_rotation;
    void validate() const;
};

struct Imu {
    double stamp;
    Vec force, rate;
};

struct Filter {
    std::array<double, 16> state = {};
    std::array<double, 225> covariance = {};
    double stamp = 0.;
    Vec position() const { return Vec(state.data()); }
    Vec velocity() const { return Vec(state.data()+3); }
    Quaternion quaternion() const { return Quaternion(state.data()+6); }
    Vec acceleration_bias() const { return Vec(state.data()+10); }
    Vec gyro_bias() const { return Vec(state.data()+13); }
    void initialize(double height, const Quaternion&, const Vec& acceleration_bias, const Vec& gyro_bias);
    bool predict(const Imu&, double end);
    bool project(const std::vector<Imu>&);
    bool correct(const cv::Mat& innovation, const cv::Mat& observation, const cv::Mat& noise, double gate);
    cv::Mat public_covariance() const;
    bool navigation_variance_valid() const;
};

struct Output {
    Quaternion quaternion;
    Vec velocity, rate;
    std::array<double, 36> pose_covariance, twist_covariance;
};
Output body_output(const Filter&, const Vec& angular_rate);

struct Snapshot {
    Filter filter;
    std::int64_t origin_ns = 0;
    bool initialized = false, reference_valid = true;
    double range_aid = -1.e100, horizontal_aid = -1.e100;
    std::string fault;
    bool range_ready(double now) const;
    bool ready(double now, const Filter& estimate, std::string& reason) const;
};

struct Camera {
    int width, height;
    cv::Mat matrix, distortion;
    std::string model;
    std::vector<cv::Point2d> centers;
    std::vector<Vec> rays;
    cv::Mat previous;
    double previous_stamp = std::numeric_limits<double>::quiet_NaN();
    Camera(int width, int height, const std::vector<double>& matrix,
           const std::vector<double>& distortion, const std::string& model);
    std::vector<Vec> undistort(const std::vector<cv::Point2d>& pixels) const;
};

class LiveEKF {
public:
    explicit LiveEKF(const Profile&);
    void mode(const std::string&, double received);
    void camera(int width, int height, const std::vector<double>& matrix,
                const std::vector<double>& distortion, const std::string& model);
    void range(std::int64_t stamp, double distance, double minimum, double maximum);
    void image(std::int64_t stamp, const std::vector<unsigned char>& encoded);
    void imu(std::int64_t stamp, const Vec& acceleration, const Vec& gyro, double received);
    Snapshot snapshot() const;
    const Filter& filter() const { return filter_; }
private:
    struct Measurement {
        double stamp, end = 0., value = 0.;
        std::string sensor;
        Vec rate;
        cv::Mat previous, image;
        std::shared_ptr<Camera> camera;
    };
    struct Range { std::int64_t stamp = 0; double value = 0.; } range_;
    struct Initial { Vec force, rate; double range; std::int64_t stamp; };
    struct AngularInterval { double start, end; Vec rate; };
    Profile profile_;
    Filter filter_;
    std::shared_ptr<Camera> camera_;
    std::deque<Imu> imu_;
    std::deque<AngularInterval> angular_;
    std::map<std::pair<double, std::string>, Measurement> measurements_;
    std::vector<Initial> initial_;
    std::int64_t origin_ns_ = 0, last_imu_ns_ = 0, rest_since_ = 0;
    double last_imu_ = 0., closed_through_ = -1.e-9, mode_at_ = -1.e100;
    double ground_range_ = -1., rest_range_ = 0., stationary_duration_ = 0.;
    Vec rest_force_, stationary_angle_ = Vec(0., 0., 0.);
    bool was_stationary_ = false, reference_valid_ = true;
    std::map<std::string, double> last_aid_;
    std::string mode_, fault_;
    void clear_rest();
    bool ground_rest_allowed() const;
    bool ground_rest(std::int64_t stamp, double received, const Vec&, const Vec&);
    void queue(const Measurement&);
    void drain(double through);
    bool correct(const Measurement&);
    bool correct_camera(const Measurement&, const Vec& rate);
    bool mean_rate(double start, double end, Vec& rate) const;
    void check_reference(double stamp);
};
}
