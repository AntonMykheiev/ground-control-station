#include "KalmanFilter.hpp"

#include <cmath>
#include <iostream>

extern "C" {
#define _float_t double
#define EKF_N 4
#define EKF_M 2
#include "src/tinyekf.h"
}

const double dt = 1.0;

KalmanFilter::KalmanFilter() : is_initialized(false) {}

void KalmanFilter::initialize(double lat, double lon, double v_lat = 0.0,
                              double v_lon = 0.0) {
    _float_t pdiag[EKF_N] = {1e-1, 1e-1, 1e-1, 1e-1};
    ekf_initialize(&ekf, pdiag);

    ekf.x[0] = lat;
    ekf.x[1] = lon;
    ekf.x[2] = v_lat;
    ekf.x[3] = v_lon;

    is_initialized = true;
}

void KalmanFilter::update(double measured_lat, double measured_lon,
                          double R_lat, double R_lon) {
    if (!is_initialized) {
        initialize(measured_lat, measured_lon);
        return;
    }

    _float_t fx[EKF_N];
    fx[0] = ekf.x[0] + dt * ekf.x[2];
    fx[1] = ekf.x[1] + dt * ekf.x[3];
    fx[2] = ekf.x[2];
    fx[3] = ekf.x[3];

    _float_t F[EKF_N * EKF_N] = {0};
    F[0] = 1;
    F[1] = 0;
    F[2] = dt;
    F[3] = 0;
    F[4] = 0;
    F[5] = 1;
    F[6] = 0;
    F[7] = dt;
    F[8] = 0;
    F[9] = 0;
    F[10] = 1;
    F[11] = 0;
    F[12] = 0;
    F[13] = 0;
    F[14] = 0;
    F[15] = 1;

    const double sigma_a2 = 1e-4;
    _float_t Q_mat[EKF_N * EKF_N] = {0};
    Q_mat[0] = sigma_a2 * (dt * dt * dt / 3);
    Q_mat[1] = 0;
    Q_mat[2] = sigma_a2 * (dt * dt / 2);
    Q_mat[3] = 0;

    Q_mat[4] = 0;
    Q_mat[5] = sigma_a2 * (dt * dt * dt / 3);
    Q_mat[6] = 0;
    Q_mat[7] = sigma_a2 * (dt * dt / 2);

    Q_mat[8] = sigma_a2 * (dt * dt / 2);
    Q_mat[9] = 0;
    Q_mat[10] = sigma_a2 * dt;
    Q_mat[11] = 0;

    Q_mat[12] = 0;
    Q_mat[13] = sigma_a2 * (dt * dt / 2);
    Q_mat[14] = 0;
    Q_mat[15] = sigma_a2 * dt;

    ekf_predict(&ekf, fx, F, Q_mat);

    _float_t z[EKF_M] = {measured_lat, measured_lon};
    _float_t hx[EKF_M];
    hx[0] = ekf.x[0];
    hx[1] = ekf.x[1];

    _float_t H_mat[EKF_M * EKF_N] = {0};
    H_mat[0] = 1;
    H_mat[1] = 0;
    H_mat[2] = 0;
    H_mat[3] = 0;
    H_mat[4] = 0;
    H_mat[5] = 1;
    H_mat[6] = 0;
    H_mat[7] = 0;

    _float_t R_mat[EKF_M * EKF_M] = {0};
    R_mat[0] = R_lat * R_lat;
    R_mat[1] = 0;
    R_mat[2] = 0;
    R_mat[3] = R_lon * R_lon;

    bool ok = ekf_update(&ekf, z, hx, H_mat, R_mat);
    if (!ok) {
        std::cerr << "EKF update failed due to singular matrix." << std::endl;
    }
}

std::array<double, 4> KalmanFilter::get_state() const {
    return {ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3]};
}