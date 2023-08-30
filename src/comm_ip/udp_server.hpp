#ifndef __TOFCORE_UDPSERVER_H__
#define __TOFCORE_UDPSERVER_H__

#include <boost/asio.hpp>
#include <cstdint>
#include <functional>

using boost::asio::ip::udp;

constexpr uint16_t DEFAULT_UDP_PORT = 0; // Zero to use ephemeral port

namespace tofcore {

    class UdpServer {

    public:
      typedef std::function<void (const std::vector<std::byte> &)> on_data_ready_t;
      UdpServer(boost::asio::io_service& ios, uint16_t udpPort = DEFAULT_UDP_PORT);
      ~UdpServer();

      uint16_t getDataPort() const
      {
          return m_socket.local_endpoint().port();
      }

      void subscribe(on_data_ready_t);

    private:
      udp::socket m_socket;
      udp::endpoint m_remoteEndpoint;
      std::vector<std::byte> m_recvBuffer;
      std::vector<std::byte> m_measurement;
      uint16_t m_currentMeasurementNum {0};

      on_data_ready_t m_dataReady;

      void startReceive();
      void handleReceive(const boost::system::error_code &, std::size_t);
    };

} //end namespace tofcore

#endif
