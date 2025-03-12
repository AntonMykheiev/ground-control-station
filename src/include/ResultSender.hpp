#ifndef RESULTSENDER_HPP
#define RESULTSENDER_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <string>

using boost::asio::ip::udp;

class ResultSender {
   public:
    ResultSender(boost::asio::io_context& io_context, const std::string& host,
                 unsigned short port);
    void send_result(std::int64_t time, double aggregated_lat,
                     double aggregated_lon);

   private:
    udp::socket socket;
    udp::endpoint endpoint;
};

#endif  // RESULTSENDER_HPP