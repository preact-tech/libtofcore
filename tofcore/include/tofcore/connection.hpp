#if !defined(_TOFCORE_COMMS_HPP)
#define _TOFCORE_COMMS_HPP

#include "CommandTypes.hpp"
#if __cplusplus >= 202002L
#   include <span>
#else
#   define TCB_SPAN_NAMESPACE_NAME std
#   include "span.hpp"
#endif
#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace tofcore
{

typedef std::function<std::tuple<bool, std::string> (const uint16_t cmdId, const bool verbose)> cmd_descr_callback_t;

class Connection_T
{
public:
    typedef std::function<void(const std::vector<std::byte>&)> on_measurement_callback_t;
    typedef std::span<std::byte> ScatterGatherElement;

public:
    virtual ~Connection_T() = default;


    virtual void send(uint16_t command, const std::vector<ScatterGatherElement> &data) = 0;
    virtual void send(uint16_t command, const uint8_t *data, uint32_t size) = 0;
    virtual void send(uint16_t command, const std::vector<uint8_t> &buf) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command,
                                                                const std::vector<ScatterGatherElement> &data,
                                                                std::chrono::steady_clock::duration timeout) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command,
                                                                const std::vector<uint8_t> &buf,
                                                                std::chrono::steady_clock::duration timeout) = 0;

    virtual std::optional<std::vector<std::byte> > send_receive(uint16_t command,
                                                                const uint8_t *data,
                                                                uint32_t size,
                                                                std::chrono::steady_clock::duration timeout) = 0;

    virtual void reset_parser() = 0;

    /// Callback function that will be called when a complete measurement data packet has been received.
    virtual void subscribe(on_measurement_callback_t callback) = 0;

    /// @brief Construct a concrete Connection object based on the provided URI string.
    /// @param io service object that manages the context the connection will run under
    /// @param uri_str URI specifing how to connect to the device to communicate with.
    /// @return 
    static std::unique_ptr<Connection_T> create(boost::asio::io_service& io,
                                                const std::string& uri,
                                                log_callback_t log_callback = nullptr,
                                                cmd_descr_callback_t cmd_descr_callback = nullptr);
}; //end class Connection_T

} //end namespace tofcore

#endif
