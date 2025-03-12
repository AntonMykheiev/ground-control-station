#include "UdpReceiver.hpp"

#include <boost/bind/bind.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

UdpReceiver::UdpReceiver(boost::asio::io_context& io_context,
                         unsigned short port, Aggregator& aggregator)
    : socket(io_context, udp::endpoint(udp::v4(), port)),
      aggregator(aggregator) {
    start_receive();
}

void UdpReceiver::start_receive() {
    socket.async_receive_from(
        boost::asio::buffer(data), sender_endpoint,
        boost::bind(&UdpReceiver::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void UdpReceiver::handle_receive(const boost::system::error_code& error,
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
                std::string lat_err_str = j.at("lat_error").get<std::string>();
                std::string lon_err_str = j.at("lon_error").get<std::string>();
                double lat_error = std::stod(lat_err_str);
                double lon_error = std::stod(lon_err_str);

                aggregator.process_message(device, time, lat_error, lon_error);
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