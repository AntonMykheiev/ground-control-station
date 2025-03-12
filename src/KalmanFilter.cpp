#include "KalmanFilter.hpp"

#include <cmath>
#include <iostream>

// Constructor
KalmanFilter::KalmanFilter() : is_initialized(false) {
    P = {{{1.0, 0.0}, {0.0, 1.0}}};
}

void KalmanFilter::initialize(double lat, double lon) {
    state = {lat, lon};
    is_initialized = true;
}

void KalmanFilter::update(double measured_lat, double measured_lon,
                          double R_lat, double R_lon) {
    if (!is_initialized) {
        initialize(measured_lat, measured_lon);
        return;
    }

    std::array<double, 2> z = {measured_lat, measured_lon};
    std::array<std::array<double, 2>, 2> R = {
        {{R_lat * R_lat, 0.0}, {0.0, R_lon * R_lon}}};

    std::array<double, 2> y = {z[0] - state[0], z[1] - state[1]};

    std::array<std::array<double, 2>, 2> S = {
        {{P[0][0] + R[0][0], P[0][1] + R[0][1]},
         {P[1][0] + R[1][0], P[1][1] + R[1][1]}}};

    double innovation_norm = std::sqrt(y[0] * y[0] + y[1] * y[1]);
    double S_trace = S[0][0] + S[1][1];
    double threshold = 3.0 * std::sqrt(S_trace);
    if (innovation_norm > threshold) {
        std::cerr << "Measurement rejected as outlier: innovation norm = "
                  << innovation_norm << std::endl;
        return;
    }

    double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (det == 0) {
        std::cerr << "Singular innovation covariance matrix. Update skipped."
                  << std::endl;
        return;
    }

    std::array<std::array<double, 2>, 2> S_inv = {
        {{S[1][1] / det, -S[0][1] / det}, {-S[1][0] / det, S[0][0] / det}}};

    std::array<std::array<double, 2>, 2> K;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            K[i][j] = P[i][0] * S_inv[0][j] + P[i][1] * S_inv[1][j];
        }
    }

    state[0] += K[0][0] * y[0] + K[0][1] * y[1];
    state[1] += K[1][0] * y[0] + K[1][1] * y[1];

    std::array<std::array<double, 2>, 2> I = {{{1.0, 0.0}, {0.0, 1.0}}};
    std::array<std::array<double, 2>, 2> newP;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 2; ++k) {
                sum += (I[i][k] - K[i][k]) * P[k][j];
            }
            newP[i][j] = sum;
        }
    }
    P = newP;
}

std::array<double, 2> KalmanFilter::get_state() const { return state; }