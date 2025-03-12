#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include <array>

extern "C" {
#define _float_t double
#define EKF_N 4
#define EKF_M 2
#include "src/tinyekf.h"
}

class KalmanFilter {
   public:
    KalmanFilter();

    void initialize(double lat, double lon, double v_lat, double v_lon);
    void update(double measured_lat, double measured_lon, double R_lat,
                double R_lon);
    std::array<double, 4> get_state() const;

   private:
    bool is_initialized;
    ekf_t ekf;
};

#endif  // KALMANFILTER_HPP