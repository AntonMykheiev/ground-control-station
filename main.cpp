#include <array>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

using boost::asio::ip::udp;
using json = nlohmann::json;

bool is_first_measurement = true;
double smoothed_lat = 0.0;
double smoothed_lon = 0.0;

struct MessagePair {
    bool has_rover = false;
    bool has_base = false;
    double rover_lat = 0.0;
    double rover_lon = 0.0;
    double base_lat_error = 0.0;
    double base_lon_error = 0.0;
    std::chrono::steady_clock::time_point arrival_time;
};

class ResultSender {
   public:
    ResultSender(boost::asio::io_context& io_context, const std::string& host,
                 unsigned short port)
        : socket(io_context, udp::endpoint(udp::v4(), 0)),
          endpoint(boost::asio::ip::make_address(host), port) {}

    void send_result(std::int64_t time, double aggregated_lat,
                     double aggregated_lon) {
        json j;
        j["time"] = time;
        j["lat"] = aggregated_lat;
        j["lon"] = aggregated_lon;
        std::string msg = j.dump();
        boost::system::error_code ec;
        socket.send_to(boost::asio::buffer(msg), endpoint, 0, ec);

        if (ec) {
            std::cerr << "Error when send UDP: " << ec.message() << std::endl;
        } else {
            std::cout << "Sent to QGIS: " << msg << std::endl;
        }
    }

   private:
    udp::socket socket;
    udp::endpoint endpoint;
};

class KalmanFilter {
   public:
    KalmanFilter() : is_initialized(false) { P = {{{1.0, 0.0}, {0.0, 1.0}}}; }

    void initialize(double lat, double lon) {
        state = {lat, lon};
        is_initialized = true;
    }

    void update(double measured_lat, double measured_lon, double R_lat,
                double R_lon) {
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
            std::cerr
                << "Singular innovation covariance matrix. Update skipped."
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

    std::array<double, 2> get_state() const { return state; }

   private:
    bool is_initialized;
    std::array<double, 2> state;
    std::array<std::array<double, 2>, 2> P;
};

class Aggregator {
   public:
    Aggregator(boost::asio::io_context& io_context,
               ResultSender* sender = nullptr)
        : timer(io_context, std::chrono::minutes(1)), result_sender(sender) {
        start_cleanup();
    }

    void process_message(const std::string& device, std::int64_t time,
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
            double measured_lat = entry.rover_lat - entry.base_lat_error;
            double measured_lon = entry.rover_lon - entry.base_lon_error;

            double rounded_lat = std::round(measured_lat * 1e8) / 1e8;
            double rounded_lon = std::round(measured_lon * 1e8) / 1e8;

            double R_lat = entry.base_lat_error;
            double R_lon = entry.base_lon_error;

            kalman_filter.update(measured_lat, measured_lon, R_lat, R_lon);

            auto state = kalman_filter.get_state();
            std::cout << "Time: " << time
                      << " Kalman filtered lat: " << std::fixed
                      << std::setprecision(8) << state[0]
                      << ", Kalman filtered lon: " << std::fixed
                      << std::setprecision(8) << state[1] << std::endl;

            if (result_sender) {
                result_sender->send_result(time, state[0], state[1]);
            }

            buffer.erase(time);
        }
    }

   private:
    void start_cleanup() {
        timer.async_wait([this](const boost::system::error_code& ec) {
            if (!ec) {
                cleanup_buffer();
                timer.expires_after(std::chrono::minutes(1));
                start_cleanup();
            }
        });
    }

    void cleanup_buffer() {
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

    std::unordered_map<std::int64_t, MessagePair> buffer;
    boost::asio::steady_timer timer;
    ResultSender* result_sender;
    KalmanFilter kalman_filter;
};

class UdpReceiver {
   public:
    UdpReceiver(boost::asio::io_context& io_context, unsigned short port,
                Aggregator& aggregator)
        : socket(io_context, udp::endpoint(udp::v4(), port)),
          aggregator(aggregator) {
        start_receive();
    }

   private:
    void start_receive() {
        socket.async_receive_from(
            boost::asio::buffer(data), sender_endpoint,
            boost::bind(&UdpReceiver::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error,
                        std::size_t bytes_received) {
        if (!error && bytes_received > 0) {
            std::string message(data.data(), bytes_received);

            try {
                auto j = json::parse(message);
                std::string device = j.at("device").get<std::string>();
                std::int64_t time = j.at("time").get<std::int64_t>();

                if (device == "rover") {
                    std::string lat_str = j.at("lat").get<std::string>();
                    std::string lon_str = j.at("lon").get<std::string>();
                    double lat = std::stod(lat_str);
                    double lon = std::stod(lon_str);

                    aggregator.process_message(device, time, lat, lon);
                } else if (device == "base") {
                    std::string lat_err_str =
                        j.at("lat_error").get<std::string>();
                    std::string lon_err_str =
                        j.at("lon_error").get<std::string>();
                    double lat_error = std::stod(lat_err_str);
                    double lon_error = std::stod(lon_err_str);

                    aggregator.process_message(device, time, lat_error,
                                               lon_error);
                } else {
                    std::cerr << "Unknown device: " << device << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "JSON parsing error: " << e.what()
                          << "\nmessage: " << message << std::endl;
            }
        }

        start_receive();
    }

    udp::socket socket;
    udp::endpoint sender_endpoint;
    std::array<char, 1024> data;
    Aggregator& aggregator;
};

int main() {
    try {
        boost::asio::io_context io_context;
        ResultSender result_sender(io_context, "127.0.0.1", 9100);
        Aggregator aggregator(io_context, &result_sender);

        UdpReceiver base(io_context, 12345, aggregator);
        UdpReceiver receiver(io_context, 12346, aggregator);

        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}