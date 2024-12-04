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

#include "connection.hpp"
#include "uri.hpp"

namespace tofcore
{

class SerialConnection : public Connection_T
{
public:
    SerialConnection(boost::asio::io_service&,
                     const uri& uri,
                     log_callback_t log_callback = nullptr,
                     cmd_descr_callback_t cmd_descr_callback = nullptr);

    virtual ~SerialConnection();

    virtual void send(uint16_t command, const std::vector<ScatterGatherElement> &data) override;
    virtual void send(uint16_t command, const uint8_t *data, uint32_t size) override;
    virtual void send(uint16_t command, const std::vector<uint8_t> &buf) override;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command,
        const std::vector<ScatterGatherElement> &data, std::chrono::steady_clock::duration timeout) override;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command, const std::vector<uint8_t> &buf,
        std::chrono::steady_clock::duration timeout) override;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command, const uint8_t *data, uint32_t size,
                                                         std::chrono::steady_clock::duration timeout) override;

    virtual void reset_parser() override;

    /// Callback function that will be called when a complete measurement data packet has been received.
    virtual void subscribe(on_measurement_callback_t callback) override;

private:
    typedef std::function<void(bool, const std::vector<std::byte>&)> on_command_response_callback_t; 
    void send_receive_async(uint16_t command, const std::vector<ScatterGatherElement> &data,
                            std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback);


    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace

#endif // _SERIAL_CONNECTION_H_

/** @} */
