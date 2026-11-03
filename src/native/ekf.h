#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
int pals_ekf_imu_means(const double*, const double*, double, double*, double*);
int pals_ekf_predict(double*, double*, const double*, const double*, const double*, const double*, double);
// Integrated interval layout: start, end, quaternion xyzw, delta velocity in start-body axes.
int pals_ekf_predict_integrated(double*, double*, const double*, const double*, const double*, double, double);
int pals_ekf_project_integrated(double*, double*, const double*, std::size_t,
                              const double*, const double*, double);
// Continuous vertical output: [height, climb rate], current EKF reference, IMU delta velocity.
int pals_ekf_vertical_output(double*, double, double, double, double);
int pals_ekf_correct(double*, double*, const double*, const double*, const double*, std::size_t,
                     double, double*, double*);
std::ptrdiff_t pals_camera_indices(const double*, const double*, const double*, const double*,
                                   const double*, std::size_t, double, double, double, std::int64_t*);
std::ptrdiff_t pals_camera_linearize(const double*, const double*, const double*, const double*,
    const double*, const double*, const double*, const double*, std::size_t, double, double, double,
    std::int64_t*, double*, double*);
}
