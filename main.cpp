#include <boost/asio.hpp>
#include <iostream>

#include "Aggregator.hpp"
#include "ResultSender.hpp"
#include "UdpReceiver.hpp"

int main() {
    try {
        boost::asio::io_context io_context;
        ResultSender result_sender(io_context, "127.0.0.1", 9100);
        Aggregator aggregator(io_context, &result_sender);

        UdpReceiver base(io_context, 12345, aggregator);
        UdpReceiver receiver(io_context, 12346, aggregator);

        io_context.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}