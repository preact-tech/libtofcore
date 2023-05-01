/**
 * @file serial_connection.h
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for serial connection to T10 sensor
 * @{
 */
#ifndef _SERIAL_CONNECTION_H_
#define _SERIAL_CONNECTION_H_

#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

namespace tofcore
{

struct ScatterGatherElement
{
    const uint8_t *m_ptr { nullptr };
    uint32_t m_size { 0 };
};

class SerialConnection
{
public:
    typedef std::function<void(const std::vector<uint8_t>&)> on_measurement_callback_t;
    typedef std::function<void(bool, const std::vector<uint8_t>&)> on_command_response_callback_t;

public:
    SerialConnection(boost::asio::io_service&, const std::string &portName, uint32_t baudrate, uint16_t protocolVersion);

    ~SerialConnection();

    uint16_t get_protocol_version() const;

    void send(uint16_t command, const std::vector<ScatterGatherElement> &data);
    void send(uint16_t command, const uint8_t *data, uint32_t size);
    void send(uint16_t command, const std::vector<uint8_t> &buf);

    std::tuple<bool, std::vector<uint8_t> > send_receive(uint16_t command, const std::vector<ScatterGatherElement> &data,
                                                         std::chrono::steady_clock::duration timeout);

    std::tuple<bool, std::vector<uint8_t> > send_receive(uint16_t command, const std::vector<uint8_t> &buf,
                                                         std::chrono::steady_clock::duration timeout);

    std::tuple<bool, std::vector<uint8_t> > send_receive(uint16_t command, const uint8_t *data, uint32_t size,
                                                         std::chrono::steady_clock::duration timeout);
    bool set_protocol_version(uint16_t version);

    /// Callback function that will be called when a complete measurement data packet has been received.
    void subscribe(on_measurement_callback_t callback);

protected:

    void send_receive_async(uint16_t command, const std::vector<ScatterGatherElement> &data,
                            std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback);

    void sendv0(uint16_t command, const std::vector<ScatterGatherElement> &data);
    void sendv1(uint16_t command, const std::vector<ScatterGatherElement> &data);

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace

#endif // _SERIAL_CONNECTION_H_

/** @} */
