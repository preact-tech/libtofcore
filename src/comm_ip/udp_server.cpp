#include <iostream>
#include "udp_server.hpp"

using boost::asio::ip::udp;
using namespace std;

namespace tofcore
{
    UdpServer::UdpServer(boost::asio::io_service &ios) : socket(ios, udp::endpoint(udp::v4(), PORT)),
                                                         recvBuffer(Packet(RECV_BUFF_SIZE))
    {
        startReceive();
    }

    UdpServer::~UdpServer()
    {
        boost::system::error_code error;
        socket.shutdown(boost::asio::ip::udp::socket::shutdown_both, error);
        if (!error)
        {
            socket.close(error);
        }
    }

    void UdpServer::subscribe(std::function<void(Packet &)> onDataReady)
    {
        dataReady = onDataReady;
    }

    void UdpServer::startReceive()
    {
        auto f = [&](const auto& error, std::size_t bytesReceived )
        {
            this->handleReceive(error, bytesReceived);
        };
        socket.async_receive_from(boost::asio::buffer(recvBuffer), remoteEndpoint, f);
                                  
    }

    void UdpServer::handleReceive(const boost::system::error_code &error, std::size_t bytesReceived)
    {
        (void)bytesReceived;
        if (!error || error == boost::asio::error::message_size)
        {
            if(dataReady)
            {
                dataReady(recvBuffer);
            }
        }
        startReceive();
    }

} // end namespace tofcore
