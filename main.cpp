#include <array>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>

using boost::asio::ip::udp;

class UdpReceiver {
   public:
    UdpReceiver(boost::asio::io_context& io_context, unsigned short port)
        : socket_(io_context, udp::endpoint(udp::v4(), port)) {
        start_receive();
    }

   private:
    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(data_), sender_endpoint_,
            boost::bind(&UdpReceiver::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error,
                        std::size_t bytes_received) {
        if (!error && bytes_received > 0) {
            std::cout.write(data_.data(), bytes_received);
            std::cout << std::endl;
        }

        start_receive();
    }

    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    std::array<char, 1024> data_;
};

int main() {
    try {
        boost::asio::io_context io_context;

        UdpReceiver base(io_context, 12345);
        UdpReceiver receiver(io_context, 12346);

        io_context.run();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}