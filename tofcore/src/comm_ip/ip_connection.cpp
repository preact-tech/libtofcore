

#include "ip_connection.hpp"

#include "crc32.h"
#include "tcp_connection.hpp"
#include "TofCommand_IF.hpp"
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
using namespace std::chrono_literals;
using namespace TofComm;


struct IpConnection::Impl
{
    TcpConnection m_tcp;
    UdpServer m_udp;

    std::function<void(bool, const std::vector<std::byte>&)> on_command_response_ {};

    std::vector<std::byte> generateCommandStream(uint16_t command, const std::vector<ScatterGatherElement> &data);

    Impl(io_service &io,
         const uri& uri,
         log_callback_t log_callback = nullptr,
         cmd_descr_callback_t cmd_descr_callback = nullptr) :
            m_tcp(io, uri, log_callback, cmd_descr_callback),
            m_udp(io)
    {
    }
};


std::vector<std::byte> IpConnection::Impl::generateCommandStream(uint16_t command, const std::vector<ScatterGatherElement> &data)
{
    //This is a quick implementation that requires an extra copy.
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


IpConnection::IpConnection(boost::asio::io_service& io,
                           const uri& uri,
                           log_callback_t log_callback,
                           cmd_descr_callback_t cmd_descr_callback) :
    pimpl { new Impl(io, uri, log_callback, cmd_descr_callback) }
{
}


IpConnection::~IpConnection()
{
}


void IpConnection::send(uint16_t command, const std::vector<ScatterGatherElement> &data)
{
    auto cmdStream = pimpl->generateCommandStream(command, data);
    //we must use the TCP connection send_receive_async() method so that the response bytes are consumed, pass in a no-op
    // function pointer to satisfy call requirements
    auto noop_callback = [](auto, auto) {};
    pimpl->m_tcp.send_receive_async(cmdStream, std::chrono::seconds(1), noop_callback);
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
    std::mutex m;
    std::condition_variable cv;
    bool ready = false;
    bool errOccurred = false;
    std::vector<std::byte> retval;
    auto f = [&](bool err, const std::vector<std::byte> &response_payload)
    {
        {
            std::unique_lock<std::mutex> lk(m);
            ready = true;
        }

        errOccurred = err;
        retval = response_payload;
        cv.notify_one();
    };

    std::unique_lock<std::mutex> lk(m);
    auto cmdStream = pimpl->generateCommandStream(command, data);
    pimpl->m_tcp.send_receive_async(cmdStream, timeout, f);
    cv.wait(lk, [&] { return ready; });
    if(errOccurred) 
    {
        return std::nullopt;
    }
    return std::make_optional(retval);
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


void IpConnection::setupStreamingLocation()
{
    // Tell the sensor where to send the data (ipv4:port)
    const uint32_t ipv4Addr { pimpl->m_tcp.getIpV4Addr() };
    const uint16_t dataPort { pimpl->m_udp.getDataPort() };
    uint8_t payload[sizeof(uint32_t) + sizeof(uint16_t)];
    BE_Put(payload, ipv4Addr);
    BE_Put(payload + sizeof(uint32_t), dataPort);
    this->send_receive(COMMAND_SET_DATA_IP_ADDRESS, payload, sizeof(payload), 5s);
}


void IpConnection::reset_parser()
{
    //TODO:
}


void IpConnection::subscribe(on_measurement_callback_t callback)
{
    pimpl->m_udp.subscribe(callback);
    setupStreamingLocation();
}

} //end namespace tofcore
