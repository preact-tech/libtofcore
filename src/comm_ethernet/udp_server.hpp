#ifndef __TOFCORE_UDPSERVER_H__
#define __TOFCORE_UDPSERVER_H__

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

using boost::asio::ip::udp;

namespace tofcore {

    typedef std::vector<uint8_t> Packet;

    class UdpServer {
      static const int PORT = 45454;
      static const int RECV_BUFF_SIZE = 2048;

    public:
      UdpServer(boost::asio::io_service &);
      ~UdpServer();

      boost::signals2::connection subscribe(std::function<void(Packet &)>);

    private:
      udp::socket socket;
      udp::endpoint remoteEndpoint;
      Packet recvBuffer;

      boost::signals2::signal<void (Packet &)> dataReady;

      void startReceive();
      void handleReceive(const boost::system::error_code &, std::size_t);
    };

} //end namespace tofcore

#endif
