#include "udp_server.hpp"
#include "TofEndian.hpp"

using boost::asio::ip::udp;
using namespace std;
using namespace TofComm;

namespace tofcore
{
constexpr int RECV_BUFF_SIZE = 2048;

UdpServer::UdpServer(boost::asio::io_service &ios, uint16_t udpPort) :
            m_socket(ios, udp::endpoint(udp::v4(), udpPort)), m_recvBuffer(RECV_BUFF_SIZE)
{
    startReceive();
}

UdpServer::~UdpServer()
{
    boost::system::error_code error;
    m_socket.shutdown(boost::asio::ip::udp::socket::shutdown_both, error);
    if (!error)
    {
        m_socket.close(error);
    }
}

void UdpServer::subscribe(on_data_ready_t onDataReady)
{
    m_dataReady = onDataReady;
}

void UdpServer::startReceive()
{
    auto f = [&](const auto& error, std::size_t bytesReceived )
    {
        this->handleReceive(error, bytesReceived);
    };
    m_socket.async_receive_from(boost::asio::buffer(m_recvBuffer), m_remoteEndpoint, f);
                                
}

void UdpServer::handleReceive(const boost::system::error_code &error, std::size_t bytesReceived)
{
    (void)bytesReceived;
    if (!error || error == boost::asio::error::message_size)
    {
        uint16_t measurementNum {0};
        uint32_t totalSize {0};
        uint16_t payloadSize {0};
        uint32_t numPackets {0};
        uint32_t packetNum {0};
        uint32_t offset {0};
        auto data = m_recvBuffer.data();
    
        //Every packet has a header describing the measurement and the packets
        // location in the overall measurement payload.
        BE_Get(measurementNum, data); //counter identifying new measuements
        data += sizeof(measurementNum); 
        BE_Get(totalSize, data); //total size for complete measurement
        data += sizeof(totalSize);
        BE_Get(payloadSize, data); //size in bytes of current packet
        data += sizeof(payloadSize);
        BE_Get(offset, data); //offset where the payload goes into the receiving buffer for the measurement
        data += sizeof(offset);
        BE_Get(numPackets, data); //total number of packets for this measuremnt
        data += sizeof(numPackets);
        BE_Get(packetNum, data); //packet number for this measurement
        data += sizeof(packetNum);

        //TODO: What should we do if a packet is dropped or comes in out of order? 
        //     (both are possible with UDP).
        // The current code will still deliver the packet if a packet is dropped but the last packet
        //   the packet will have data from the previous frame in place of the dropped packets.
        //   is received. If the last packet is dropped then the whole measurement is dropped.
        // Out of order packets don't matter unless they occur after the last packet of a measurement
        //   in which case they might affect the next packet depending on when they come in relation
        //   to that frames actual packet at the position.
        if(packetNum == 0)
        {
            //start of a new measurement
            m_measurement.resize(totalSize);
            m_currentMeasurementNum = measurementNum;
            std::copy(data, data + payloadSize, m_measurement.data() + offset);
        }
        else if((m_currentMeasurementNum == measurementNum) &&
                (m_measurement.size() == totalSize) &&
                ((offset + payloadSize) <= m_measurement.size()))
        {
            //continuation of recieving a measurement in progress
            std::copy(data, data + payloadSize, m_measurement.data() + offset);
            if(packetNum == (numPackets - 1))
            { //final packet of measurement received
                if(m_dataReady)
                {
                    m_dataReady(m_measurement);
                }
            }
        }
    }
    startReceive();
}

} // end namespace tofcore
