#ifndef UDPRECEIVER_HPP
#define UDPRECEIVER_HPP

#include <array>
#include <boost/asio.hpp>

#include "Aggregator.hpp"

using boost::asio::ip::udp;

class UdpReceiver {
   public:
    UdpReceiver(boost::asio::io_context& io_context, unsigned short port,
                Aggregator& aggregator);

   private:
    void start_receive();
    void handle_receive(const boost::system::error_code& error,
                        std::size_t bytes_received);

    udp::socket socket;
    udp::endpoint sender_endpoint;
    std::array<char, 1024> data;
    Aggregator& aggregator;
};

#endif  // UDPRECEIVER_HPP