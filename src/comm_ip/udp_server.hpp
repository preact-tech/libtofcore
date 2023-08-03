#ifndef __TOFCORE_UDPSERVER_H__
#define __TOFCORE_UDPSERVER_H__

#include <boost/asio.hpp>
#include <functional>

using boost::asio::ip::udp;

namespace tofcore {

    typedef std::vector<std::byte> Packet;

    class UdpServer {
      static const int PORT = 45454;
      static const int RECV_BUFF_SIZE = 2048;

    public:
      UdpServer(boost::asio::io_service &);
      ~UdpServer();

      void subscribe(std::function<void(Packet &)>);

    private:
      udp::socket socket;
      udp::endpoint remoteEndpoint;
      Packet recvBuffer;

      std::function<void (Packet &)> dataReady;

      void startReceive();
      void handleReceive(const boost::system::error_code &, std::size_t);
    };

} //end namespace tofcore

#endif
