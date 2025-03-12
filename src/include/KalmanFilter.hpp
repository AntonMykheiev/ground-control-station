#ifndef KALMANFILTER_HPP
#define KALMANFILTER_HPP

#include <array>

class KalmanFilter {
   public:
    KalmanFilter();

    void initialize(double lat, double lon);
    void update(double measured_lat, double measured_lon, double R_lat,
                double R_lon);
    std::array<double, 2> get_state() const;

   private:
    bool is_initialized;
    std::array<double, 2> state;
    std::array<std::array<double, 2>, 2> P;
};

#endif  // KALMANFILTER_HPP