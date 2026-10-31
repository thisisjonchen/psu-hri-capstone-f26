#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
int pals_ekf_predict(double*, double*, const double*, const double*, const double*, const double*, double);
int pals_ekf_project(double*, double*, const double*, std::size_t, const double*, const double*,
                     const double*, const double*, double, double);
int pals_ekf_correct(double*, double*, const double*, const double*, const double*, std::size_t,
                     double, double*, double*);
std::ptrdiff_t pals_camera_indices(const double*, const double*, const double*, const double*,
                                   const double*, std::size_t, double, double, double, std::int64_t*);
std::ptrdiff_t pals_camera_linearize(const double*, const double*, const double*, const double*,
    const double*, const double*, const double*, const double*, std::size_t, double, double, double,
    std::int64_t*, double*, double*);
}
