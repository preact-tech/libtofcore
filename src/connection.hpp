#if !defined(_TOFCORE_COMMS_HPP)
#define _TOFCORE_COMMS_HPP

#include "span.hpp"
#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace tofcore
{

class Connection_T
{
public:
    typedef std::function<void(const std::vector<std::byte>&)> on_measurement_callback_t;
    typedef tcb::span<std::byte> ScatterGatherElement;

    //TODO: MZS this probably doesn't belong here.
    typedef std::function<void(bool, const std::vector<std::byte>&)> on_command_response_callback_t; 

public:
    virtual ~Connection_T() = default;

    virtual uint16_t get_protocol_version() const = 0;

    virtual void send(uint16_t command, const std::vector<ScatterGatherElement> &data) = 0;
    virtual void send(uint16_t command, const uint8_t *data, uint32_t size) = 0;
    virtual void send(uint16_t command, const std::vector<uint8_t> &buf) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command, const std::vector<ScatterGatherElement> &data,
                                                         std::chrono::steady_clock::duration timeout) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command, const std::vector<uint8_t> &buf,
                                                         std::chrono::steady_clock::duration timeout) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command, const uint8_t *data, uint32_t size,
                                                         std::chrono::steady_clock::duration timeout) = 0;

    virtual void send_receive_async(uint16_t command, const std::vector<ScatterGatherElement> &data,
                            std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback) = 0;

    virtual bool set_protocol_version(uint16_t version) = 0;

    virtual void reset_parser() = 0;

    /// Callback function that will be called when a complete measurement data packet has been received.
    virtual void subscribe(on_measurement_callback_t callback) = 0;

    //TODO: Move to factory
    static std::unique_ptr<Connection_T> create(boost::asio::io_service& io, const std::string& uri);

}; //end class Connection_T

} //end namespace tofcore

#endif
