#include "ResultSender.hpp"

#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ResultSender::ResultSender(boost::asio::io_context& io_context,
                           const std::string& host, unsigned short port)
    : socket(io_context, udp::endpoint(udp::v4(), 0)),
      endpoint(boost::asio::ip::make_address(host), port) {}

void ResultSender::send_result(std::int64_t time, double aggregated_lat,
                               double aggregated_lon) {
    json j;
    j["time"] = time;
    j["lat"] = aggregated_lat;
    j["lon"] = aggregated_lon;

    std::string msg = j.dump();
    boost::system::error_code ec;
    socket.send_to(boost::asio::buffer(msg), endpoint, 0, ec);

    if (ec) {
        std::cerr << "Error when sending UDP: " << ec.message() << std::endl;
    } else {
        std::cout << "Sent to QGIS: " << msg << std::endl;
    }
}