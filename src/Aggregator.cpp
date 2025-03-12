#include "Aggregator.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

// Global variables as in the original snippet
bool is_first_measurement = true;
double smoothed_lat = 0.0;
double smoothed_lon = 0.0;

Aggregator::Aggregator(boost::asio::io_context& io_context,
                       ResultSender* sender)
    : timer(io_context, std::chrono::minutes(1)), result_sender(sender) {
    start_cleanup();
}

void Aggregator::process_message(const std::string& device, std::int64_t time,
                                 double v1, double v2) {
    auto now = std::chrono::steady_clock::now();
    auto& entry = buffer[time];

    if (entry.arrival_time == std::chrono::steady_clock::time_point{}) {
        entry.arrival_time = now;
    }

    if (device == "rover") {
        entry.has_rover = true;
        entry.rover_lat = v1;
        entry.rover_lon = v2;
    } else if (device == "base") {
        entry.has_base = true;
        entry.base_lat_error = v1;
        entry.base_lon_error = v2;
    } else {
        std::cerr << "Unknown device: " << device << std::endl;
        return;
    }

    if (entry.has_rover && entry.has_base) {
        double R_lat = entry.base_lat_error;
        double R_lon = entry.base_lon_error;

        kalman_filter.update(entry.rover_lat, entry.rover_lon, R_lat, R_lon);

        auto state = kalman_filter.get_state();
        std::cout << "Time: " << time << " Kalman filtered lat: " << std::fixed
                  << std::setprecision(14) << state[0]
                  << ", Kalman filtered lon: " << std::fixed
                  << std::setprecision(14) << state[1] << std::endl;

        result_sender->send_result(time, state[0], state[1]);

        buffer.erase(time);
    }
}

void Aggregator::start_cleanup() {
    timer.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) {
            cleanup_buffer();
            timer.expires_after(std::chrono::minutes(1));
            start_cleanup();
        }
    });
}

void Aggregator::cleanup_buffer() {
    auto now = std::chrono::steady_clock::now();
    for (auto it = buffer.begin(); it != buffer.end();) {
        auto duration = std::chrono::duration_cast<std::chrono::minutes>(
            now - it->second.arrival_time);
        if (duration.count() >= 1) {
            std::cout << "Cleaning up message with time: " << it->first
                      << std::endl;
            it = buffer.erase(it);
        } else {
            ++it;
        }
    }
}