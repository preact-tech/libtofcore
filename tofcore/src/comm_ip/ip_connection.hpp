/**
 * @file ip_connection.h
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for serial connection to T10 sensor
 * @{
 */
#ifndef _IP_CONNECTION_H_
#define _IP_CONNECTION_H_

#include "connection.hpp"
#include "uri.hpp"

namespace tofcore
{

/// @brief Connection class used for communication with a PreACt ToF camera over IP protocols (TCP/UDP)
class IpConnection : public Connection_T
{
public:
    /// @param uri A URI (with scheme tofnet) specifying the IP address (or hostname) and optional port to connect
    ///  to the device. The URI can include optional parameters used to configure the connection and or device.
    IpConnection(boost::asio::io_service&, const uri& uri);

    virtual ~IpConnection() override;

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

    void setupStreamingLocation();

    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace

#endif // _IP_CONNECTION_H_

/** @} */
