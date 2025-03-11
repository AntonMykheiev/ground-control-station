#include <array>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

using boost::asio::ip::udp;
using json = nlohmann::json;

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
            double aggregated_lat = entry.rover_lat + entry.base_lat_error;
            double aggregated_lon = entry.rover_lon + entry.base_lon_error;
            std::cout << "Time: " << time
                      << " Aggregated lat: " << aggregated_lat
                      << ", Aggregated lon: " << aggregated_lon << std::endl;

            if (result_sender) {
                result_sender->send_result(time, aggregated_lat,
                                           aggregated_lon);
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
                std::cout << "Clean up message with time: " << it->first
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