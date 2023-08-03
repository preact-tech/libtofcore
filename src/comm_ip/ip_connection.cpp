

#include "ip_connection.hpp"
#include "tcp_connection.hpp"
#include "udp_server.hpp"

#include "TofEndian.hpp"
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


void IpConnection::send(uint16_t command, const std::vector<ScatterGatherElement> &data)
{
    //TODO: This is a quick implementation that requires an extra copy.
    size_t total = sizeof(command);
    for(const auto &n : data)
    {
        total += n.size_bytes();
    }

    std::vector<std::byte> tmp(total);
    BE_Put(tmp.data(), command);
    auto dest = tmp.begin() + 2;
    for(const auto &n : data)
    {
        dest = std::copy(n.begin(), n.end(), dest);
    }

    pimpl->m_tcp.sendCommand(tmp);
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
    //TODO: This is a quick implementation that requires an extra copy.
    (void)timeout;
    size_t total = sizeof(command);
    for(const auto &n : data)
    {
        total += n.size_bytes();
    }

    std::vector<std::byte> tmp(total);
    BE_Put(tmp.data(), command);
    auto dest = tmp.begin() + 2;
    for(const auto &n : data)
    {
        dest = std::copy(n.begin(), n.end(), dest);
    }

    std::vector<std::byte> answer;
    pimpl->m_tcp.sendCommand(tmp, answer);

    return {answer};
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