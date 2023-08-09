

#include "ip_connection.hpp"

#include "crc32.h"
#include "tcp_connection.hpp"
#include "TofEndian.hpp"
#include "udp_server.hpp"
#include <array>
#include <boost/scope_exit.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

namespace tofcore
{
using namespace boost;
using namespace boost::system;
using namespace boost::asio;
using namespace TofComm;


struct IpConnection::Impl
{
    TcpConnection m_tcp;
    UdpServer m_udp;
    uint16_t m_protocol_version { 0 };

    std::function<void(bool, const std::vector<std::byte>&)> on_command_response_ {};

    Impl(io_service &io, const std::string &uri, uint16_t protocolVersion) :
            m_tcp(io),
            m_udp(io),
            m_protocol_version(protocolVersion)
    {
        (void)uri;
    }
};


IpConnection::IpConnection(boost::asio::io_service& io, const std::string &uri, uint16_t protocolVersion) :
    pimpl { new Impl(io, uri, protocolVersion) }
{
}


IpConnection::~IpConnection()
{
    //TODO: Is there anything to do here? 
}


uint16_t IpConnection::get_protocol_version() const
{
    return pimpl->m_protocol_version;
}

std::vector<std::byte> IpConnection::generateCommandStream(uint16_t command, const std::vector<ScatterGatherElement> &data)
{
    //TODO: This is a quick implementation that requires an extra copy.
     /*
      *           ------------ ---------- ----------- ----------- ------------- -----------
      * VERS. 1  | 0xFFFFAA55 | PID      |   CID     | LEN       |  Payload    | CRC32     |
      * COMMAND: |            | (1 byte) | (2 bytes) | (4 bytes) | (LEN bytes) | (4 bytes) |
      *           ------------ ---------- ----------- ----------- ------------- -----------
      *          |<----------------- prolog ------------------->|
      */
     constexpr uint32_t START_MARKER = 0xFFFFAA55;
     constexpr size_t OVERHEAD_SIZE = 3 * sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint16_t); // non-payload portion
     uint32_t payloadSize = 0;
     for(const auto &n : data) // .. and now find out the payload portion
     {
         payloadSize += n.size_bytes();
     }
     const size_t totalSize { payloadSize + OVERHEAD_SIZE };
     std::vector<std::byte> result(totalSize);

     auto dest = result.data();
     BE_Put(dest, START_MARKER);
     dest += sizeof(START_MARKER);
     static uint8_t s_pid;
     *dest = static_cast<std::byte>(s_pid);
     dest += sizeof(s_pid);
     ++s_pid;
     BE_Put(dest, command);
     dest += sizeof(command);
     BE_Put(dest, payloadSize);
     dest += sizeof(payloadSize);
     for(const auto &n : data)
     {
         dest = std::copy(n.begin(), n.end(), dest);
     }
     const uint8_t* buf = reinterpret_cast<const uint8_t*>(result.data());
     uint32_t crc = calcCrc32(buf, totalSize - sizeof(uint32_t));
     BE_Put(dest, crc);

     return result;
}

void IpConnection::send(uint16_t command, const std::vector<ScatterGatherElement> &data)
{
    auto cmdStream = generateCommandStream(command, data);
    pimpl->m_tcp.sendCommand(cmdStream);
}


void IpConnection::send(uint16_t command, const uint8_t *data, uint32_t size)
{
    const auto tmp = ScatterGatherElement{(std::byte*)data, (size_t)size};
    const std::vector<ScatterGatherElement> singleDataChunk { tmp };
    this->send(command, singleDataChunk);
}


void IpConnection::send(uint16_t command, const std::vector<uint8_t> &buf)
{
    this->send(command, buf.data(), buf.size());
}


std::optional<std::vector<std::byte> > IpConnection::send_receive(uint16_t command,
    const std::vector<ScatterGatherElement> &data, std::chrono::steady_clock::duration timeout)
{
    //TODO: Implement timeout
    (void)timeout;
    auto cmdStream = generateCommandStream(command, data);
    std::vector<std::byte> answer;
    bool error = false;
    try
    {
        pimpl->m_tcp.sendCommand(cmdStream, answer);
    }
    catch(...)
    {
        error = true;
    }
    if (error)
    {
        return std::nullopt;
    }
    else
    {
        return {answer};
    }
}


std::optional<std::vector<std::byte> > IpConnection::send_receive(uint16_t command, const std::vector<uint8_t> &buf,
    std::chrono::steady_clock::duration timeout)
{
    return this->send_receive(command, buf.data(), buf.size(), timeout);
}


std::optional<std::vector<std::byte> > IpConnection::send_receive(uint16_t command, const uint8_t *data, uint32_t size,
    std::chrono::steady_clock::duration timeout)
{
    const std::vector<ScatterGatherElement> singleDataChunk { { (std::byte*)data, (size_t)size } };
    return this->send_receive(command, singleDataChunk, timeout);
}


bool IpConnection::set_protocol_version(uint16_t version)
{
    //TODO: for now only supporting version 0 (which is the default so just check
    // the client only asks for version 0)
    return (version == 0);
}


void IpConnection::send_receive_async(uint16_t command, const std::vector<ScatterGatherElement> &data,
    std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback)
{
    (void)command;
    (void)data;
    (void)timeout;
    (void)callback;
    //TODO:
}


void IpConnection::reset_parser()
{
    //TODO:
}


void IpConnection::subscribe(on_measurement_callback_t callback)
{
    pimpl->m_udp.subscribe(callback);
}

} //end namespace tofcore
