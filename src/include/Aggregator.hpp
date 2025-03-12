#ifndef AGGREGATOR_HPP
#define AGGREGATOR_HPP

#include <boost/asio/steady_timer.hpp>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "KalmanFilter.hpp"
#include "ResultSender.hpp"

struct MessagePair {
    bool has_rover = false;
    bool has_base = false;
    double rover_lat = 0.0;
    double rover_lon = 0.0;
    double base_lat_error = 0.0;
    double base_lon_error = 0.0;
    std::chrono::steady_clock::time_point
        arrival_time{};  // default initialized
};

class Aggregator {
   public:
    Aggregator(boost::asio::io_context& io_context, ResultSender* sender);
    void process_message(const std::string& device, std::int64_t time,
                         double v1, double v2);

   private:
    void start_cleanup();
    void cleanup_buffer();

    std::unordered_map<std::int64_t, MessagePair> buffer;
    boost::asio::steady_timer timer;
    ResultSender* result_sender;
    KalmanFilter kalman_filter;
};

#endif  // AGGREGATOR_HPP