/**
 * @file serial_connection.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Implements serial connection to T10 sensor
 * @{
 */
#include "crc32.h"
#include "serial_connection.hpp"
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

#define DBG(l) //std::cout << l << std::endl
#define ERR(l) std::cerr << l << std::endl
/*
 * If CHECK_DATA_CRC is set to 0, CRC errors for streaming data are ignored.
 * Unfortunately this is happening sporadically with the TOFCam660
 */
#if !defined(CHECK_DATA_CRC)
#   define CHECK_DATA_CRC 1
#endif
#if !defined(DATA_CRC_CHUNKS)
#   define DATA_CRC_CHUNKS 0
#endif

/*
 * VERSION 0 COMMAND:
 *  ---------- ----------- ------------- ---------------------
 * |   0xF5   |  CID (BE) |   PAYLOAD   | 0xBE 0xBA 0xFE 0xCA |
 * | (1 byte) | (2 bytes) |  (32 bytes) |      (4 bytes)      |
 *  ---------- ----------- ------------- ---------------------
 *
 * VERSION 1 COMMAND:
 *  ---------- ---------- ----------- ------------- ------------- ------------
 * |   0xF6   |   PID    |  CID (BE) | Length (BE) |   PAYLOAD   | CRC32 (BE) |
 * | (1 byte) | (1 byte) | (2 bytes) |  (4 bytes)  | (LEN bytes) |  (4 bytes) |
 *  ---------- ---------- ----------- ------------- ------------- ------------
 *
 * VERSION 0 RESPONSE:
 *  ---------- ---------- ------------- ---------------- ---------------------
 * |   0xFA   |   TYPE   | Length (LE) |    PAYLOAD     | 0xBE 0xBA 0xFE 0xCA |
 * | (1 byte) | (1 byte) |  (4 bytes)  | (Length bytes) |      (4 bytes)      |
 *  ---------- ---------- ------------- ---------------- ---------------------
 *
 * VERSION 1 RESPONSE:
 *  ---------- ---------- ---------- ------------- ---------------- ------------
 * |   0xFB   |   PID    |   TYPE   | Length (BE) |    PAYLOAD     | CRC32 (BE) |
 * | (1 byte) | (1 byte) | (1 byte) |  (4 bytes)  | (Length bytes) | (4 bytes)  |
 *  ---------- ---------- ---------- ------------- ---------------- ------------
 */

constexpr uint16_t MAX_PROTOCOL_VERSION = 1;
/*
 * Values for version 0 commands
 */
constexpr uint32_t V0_CMD_CID_INDEX = 1;
constexpr uint32_t V0_CMD_PROLOG_SIZE = V0_CMD_CID_INDEX + sizeof(uint16_t);
constexpr uint32_t V0_CMD_PAYLOAD_SIZE = 32;
constexpr auto V0_CMD_START_MARK = std::byte{0XF5};
/*
 * Values for version 1 commands
 */
constexpr uint32_t V1_CMD_PID_INDEX = 1;
constexpr uint32_t V1_CMD_CID_INDEX    = V1_CMD_PID_INDEX + sizeof(uint8_t);
constexpr uint32_t V1_CMD_LENGTH_INDEX = V1_CMD_CID_INDEX + sizeof(uint16_t);
constexpr uint32_t V1_CMD_PROLOG_SIZE  = V1_CMD_LENGTH_INDEX + sizeof(uint32_t);
constexpr uint32_t V1_CMD_MAX_PAYLOAD_SIZE = 4 * 1024;
constexpr auto V1_CMD_START_MARK = std::byte{0XF6};
/*
 * Values for version 0 responses
 */
constexpr auto V0_RESP_START_MARK = std::byte{0XFA};
constexpr uint32_t V0_RESP_PROLOG_TYPE_OFFSET   = 0;
constexpr uint32_t V0_RESP_PROLOG_LENGTH_OFFSET = V0_RESP_PROLOG_TYPE_OFFSET + sizeof(uint8_t);
constexpr uint32_t V0_RESP_PROLOG_SIZE = V0_RESP_PROLOG_LENGTH_OFFSET  + sizeof(uint32_t);
/*
 * Values for version 1 responses
 */
constexpr auto V1_RESP_START_MARK = std::byte{0XFB};
constexpr uint32_t V1_RESP_PROLOG_PID_OFFSET    = 0;
constexpr uint32_t V1_RESP_PROLOG_TYPE_OFFSET   = V1_RESP_PROLOG_PID_OFFSET  + sizeof(uint8_t);
constexpr uint32_t V1_RESP_PROLOG_LENGTH_OFFSET = V1_RESP_PROLOG_TYPE_OFFSET + sizeof(uint8_t);
constexpr uint32_t V1_RESP_PROLOG_SIZE = V1_RESP_PROLOG_LENGTH_OFFSET + sizeof(uint32_t);

struct SerialConnection::Impl
{
    boost::asio::serial_port port_;
    boost::asio::steady_timer response_timer_;
    uint16_t protocol_version_ { 1 };

    std::vector<std::byte> payload_{};
    std::array<std::byte, V1_RESP_PROLOG_SIZE> prolog_epilog_buf_ {};

    std::byte response_type_ { 0 };
    std::byte response_marker_ { 0 };
    std::uint8_t response_pid_ { 0 };
    std::uint8_t v1_cmd_pid_ { 0xFF };   ///< PID of most recent command (V1 protocol)
    uint16_t v1_cmd_cid_ { 0 };        ///< CID of most recent command (V1 protocol)

    uint32_t response_crc_accum_ { 0 };

    std::function<void(const std::vector<std::byte>&)> on_measurement_data_ {};
    std::function<void(bool, const std::vector<std::byte>&)> on_command_response_ {};

    Impl(io_service &io, const std::string &portName, uint32_t baud_rate, uint16_t protocolVersion) :
                port_(io, portName), 
                response_timer_(io), 
                protocol_version_(protocolVersion)
    {

        this->port_.set_option(serial_port_base::baud_rate(baud_rate));
        this->port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        this->port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        this->port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        this->port_.set_option(serial_port_base::character_size(8));

        //start a receive operation to receive the start of the next data packet. 
        this->begin_receive_start();
    }

    Impl(io_service &io, const uri& uri) :
                port_(io, uri.get_path()),
                response_timer_(io)
    {
        auto query_dict = uri.get_query_dictionary();
        auto it = query_dict.find("protocol_version");
        if(it != query_dict.end())
        {
            this->protocol_version_ = std::atoi(it->second.c_str());
        }

        uint32_t baud_rate = 115200;
        it = query_dict.find("baudrate");
        if(it != query_dict.end())
        {
            baud_rate = std::atoi(it->second.c_str());
        }

        this->port_.set_option(serial_port_base::baud_rate(baud_rate));
        this->port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        this->port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        this->port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        this->port_.set_option(serial_port_base::character_size(8));

        //start a receive operation to receive the start of the next data packet. 
        this->begin_receive_start();
    }


    void reset_parser();

    void begin_receive_start();
    void on_receive_start(const system::error_code &error);

    void begin_receive_v0_prolog();
    void on_receive_v0_prolog(const system::error_code &error);

    void begin_receive_v1_prolog();
    void on_receive_v1_prolog(const system::error_code &error);

    void begin_receive_payload(uint32_t payload_length);
    void on_receive_payload(const system::error_code &error);

    bool is_valid_v1_response();
    void begin_receive_end();
    void on_receive_end(const system::error_code &error);

    void begin_response_timer(std::chrono::steady_clock::duration timeout);
    void on_response_timeout(const system::error_code &error);

    void process_v0_response();
    void process_v1_response();

    bool process_error(const system::error_code &error, const char* where);
};

/* =========================================================================
 *
 * SerialConnection public methods
 *
 * ========================================================================= */

SerialConnection::SerialConnection(boost::asio::io_service& io, const uri& uri) :
    pimpl { new Impl(io, uri) }
{
}


SerialConnection::~SerialConnection()
{
    //TODO: Do I need to explicity cancel pending waits?
    pimpl->port_.close();
}

uint16_t SerialConnection::get_protocol_version() const
{
    return pimpl->protocol_version_;
}

void SerialConnection::reset_parser()
{
    pimpl->reset_parser();
}

void SerialConnection::send(uint16_t command, const std::vector<uint8_t> &buf)
{
    this->send(command, buf.data(), buf.size());
}

void SerialConnection::send(uint16_t command, const uint8_t *data, uint32_t size)
{
    const auto tmp = ScatterGatherElement{(std::byte*)data, (size_t)size};
    const std::vector<ScatterGatherElement> singleDataChunk { tmp };
    this->send(command, singleDataChunk);
}

void SerialConnection::send(uint16_t command, const std::vector<ScatterGatherElement>& data)
{
    if (0 == pimpl->protocol_version_)
    {
        this->sendv0(command, data);
    }
    else
    {
        this->sendv1(command, data);
    }
}

std::optional<std::vector<std::byte> > SerialConnection::send_receive(uint16_t command, const std::vector<uint8_t> &buf,
                                                                       std::chrono::steady_clock::duration timeout)
{
    return this->send_receive(command, buf.data(), buf.size(), timeout);
}

std::optional<std::vector<std::byte> > SerialConnection::send_receive(uint16_t command, const uint8_t *data, uint32_t size,
                                                                       std::chrono::steady_clock::duration timeout)
{
    const std::vector<ScatterGatherElement> singleDataChunk { { (std::byte*)data, (size_t)size } };
    return this->send_receive(command, singleDataChunk, timeout);
}

std::optional<std::vector<std::byte> > SerialConnection::send_receive(uint16_t command, const std::vector<ScatterGatherElement>& data,
                                                                       std::chrono::steady_clock::duration timeout)
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
    this->send_receive_async(command, data, timeout, f);
    cv.wait(lk, [&] { return ready; });
    if(errOccurred) 
    {
        return std::nullopt;
    }
    return std::make_optional(retval);
}

bool SerialConnection::set_protocol_version(uint16_t version)
{
    if (version <= MAX_PROTOCOL_VERSION)
    {
        pimpl->protocol_version_ = version;
        return true;
    }
    return false;
}

void SerialConnection::subscribe(on_measurement_callback_t callback)
{
    pimpl->on_measurement_data_ = callback;
}


/* #########################################################################
 *
 * SerialConnection protected methods
 *
 * ######################################################################### */

void SerialConnection::send_receive_async(uint16_t command, const std::vector<ScatterGatherElement>& data,
                                          std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback)
{
    //TODO: Do I need to check if there is already a pending response? (there can be only one!)
    pimpl->on_command_response_ = callback;
    this->send(command, data);
    pimpl->begin_response_timer(timeout);
}

void SerialConnection::sendv0(uint16_t command, const std::vector<ScatterGatherElement>& data)
{
    /* VERSION 0 COMMAND:
     *  ---------- ----------- ------------- ---------------------
     * |   0xF5   |  CID (BE) |   PAYLOAD   | 0xBE 0xBA 0xFE 0xCA |
     * | (1 byte) | (2 bytes) |  (32 bytes) |      (4 bytes)      |
     *  ---------- ----------- ------------- ---------------------
     */
    std::array<std::byte, V0_CMD_PROLOG_SIZE> prolog { V0_CMD_START_MARK };
    BE_Put(&prolog[V0_CMD_CID_INDEX], command);
    uint32_t size { 0 };
    for (const auto& d: data)
    {
        size += d.size();
    }
    if (size > V0_CMD_PAYLOAD_SIZE)
    {
        ERR("Cannot send payloads larger than " << V0_CMD_PAYLOAD_SIZE << " bytes with version 0 protocol");
        return;
    }

    static std::array<std::byte, V0_CMD_PAYLOAD_SIZE> filler; // 0 fill for unused part
    auto filler_size = filler.size() - size; // pad payload to always send V0_CMD_PAYLOAD_SIZE

    static const auto epilog = std::array<std::byte, sizeof(uint32_t)> { std::byte{0xbe}, std::byte{0xba}, std::byte{0xfe}, std::byte{0xca} };

    std::vector<boost::asio::const_buffer> bufs { buffer(prolog) };
    for (const auto& d: data)
    {
        bufs.push_back(buffer(d.begin(), d.size()));
    }
    bufs.push_back(buffer(filler, filler_size));
    bufs.push_back(buffer(epilog));
    boost::asio::write(pimpl->port_, bufs);
}

void SerialConnection::sendv1(uint16_t command, const std::vector<ScatterGatherElement>& data)
{
    /* VERSION 1 COMMAND:
     *  ---------- ---------- ----------- ------------- ------------- ------------
     * |   0xF6   |   PID    |  CID (BE) | Length (BE) |   PAYLOAD   | CRC32 (BE) |
     * | (1 byte) | (1 byte) | (2 bytes) |  (4 bytes)  | (LEN bytes) |  (4 bytes) |
     *  ---------- ---------- ----------- ------------- ------------- ------------
     */
    std::array<std::byte, V1_CMD_PROLOG_SIZE> prolog { V1_CMD_START_MARK, (std::byte)(++pimpl->v1_cmd_pid_) };
    pimpl->v1_cmd_cid_ = command;
    BE_Put(&prolog[V1_CMD_CID_INDEX], command);
    uint32_t size { 0 };
    for (const auto& d: data)
    {
        size += d.size();
    }
    BE_Put(&prolog[V1_CMD_LENGTH_INDEX], size);

    //Calculate the CRC32 of everything but the CRC32 epilog
    uint32_t crc32 { calcCrc32((uint8_t*)&prolog[0], V1_CMD_PROLOG_SIZE) };

    std::vector<boost::asio::const_buffer> bufs { buffer(prolog) };
    for (const auto& d: data)
    {
        crc32 = updateCrc32(crc32, (const uint8_t*)d.begin(), d.size());
        bufs.push_back(buffer(d.begin(), d.size()));
    }
    std::array<uint8_t, sizeof(crc32)> epilog {};
    BE_Put(&epilog[0], crc32);
    bufs.push_back(buffer(epilog));
    boost::asio::write(pimpl->port_, bufs);
}

void SerialConnection::Impl::reset_parser()
{
    //Cancel any pending reads and timers and start fresh. 
    this->port_.cancel();
    this->response_timer_.cancel();

#if defined(_WIN32)
    auto handle = this->port_.native_handle();
    PurgeComm(handle, PURGE_RXCLEAR);
#endif

    this->begin_receive_start();
}

/* #########################################################################
 *
 * Impl Receive methods (in order of receive sequence)
 *
 * ######################################################################### */

void SerialConnection::Impl::begin_response_timer(std::chrono::steady_clock::duration timeout)
{
    auto f = [&](const auto &error)
    {
        this->on_response_timeout(error);
    };

    this->response_timer_.expires_after(timeout);
    this->response_timer_.async_wait(f);
}

void SerialConnection::Impl::on_response_timeout(const system::error_code &error)
{
    // If the timer expires (timeout), there is no error
    if (error && ( (error == boost::asio::error::operation_aborted) ||
                   !this->process_error(error, __FUNCTION__)
                 ))
    {
        return;
    }
    // The timer went off and so a "timeout" occurred waiting for the response
    if (this->on_command_response_)
    {
        std::vector<std::byte> tmp;
        this->on_command_response_(true, tmp);
        this->on_command_response_ = nullptr;
    }
}

void SerialConnection::Impl::begin_receive_start()
{
    auto f = [&](const auto &error, auto)
    {
        this->on_receive_start(error);
    };
    this->response_marker_ = (std::byte)0; // invalidate any previous marker
    async_read(this->port_, buffer(&this->response_marker_, 1), f);
}

void SerialConnection::Impl::on_receive_start(const system::error_code &error)
{
    if (error && !this->process_error(error, __FUNCTION__))
    {
        return;
    }
    // Must be either the version 0 or version 1 marker
    if (V0_RESP_START_MARK == this->response_marker_)
    {
        this->begin_receive_v0_prolog();
    }
    else if (V1_RESP_START_MARK == this->response_marker_)
    {
        this->response_crc_accum_ = calcCrc32((uint8_t*)&this->response_marker_, sizeof(this->response_marker_));
        this->begin_receive_v1_prolog();
    }
    else
    {
        //Not the start of packet so try again
        this->begin_receive_start();
    }
}

void SerialConnection::Impl::begin_receive_v0_prolog()
{
    auto f = [&](const auto &error, auto)
    {
        this->on_receive_v0_prolog(error);
    };
    async_read(this->port_, buffer(this->prolog_epilog_buf_, V0_RESP_PROLOG_SIZE), f);
}

void SerialConnection::Impl::on_receive_v0_prolog(const system::error_code &error)
{
    if (error && !this->process_error(error, __FUNCTION__))
    {
        return;
    }
    /* VERSION 0 RESPONSE:
     *  ---------- ---------- ------------- ---------------- ---------------------
     * |   0xFA   |   TYPE   | Length (LE) |    PAYLOAD     | 0xBE 0xBA 0xFE 0xCA |
     * | (1 byte) | (1 byte) |  (4 bytes)  | (Length bytes) |      (4 bytes)      |
     *  ---------- ---------- ------------- ---------------- ---------------------
     */
    this->response_type_ = this->prolog_epilog_buf_[V0_RESP_PROLOG_TYPE_OFFSET];
    uint32_t payload_length; 
    LE_Get(payload_length, &this->prolog_epilog_buf_[V0_RESP_PROLOG_LENGTH_OFFSET]);
    DBG("Received V0 type: " << (unsigned)this->response_type_ << "; size: " << payload_length);
    this->begin_receive_payload(payload_length);
}

void SerialConnection::Impl::begin_receive_v1_prolog()
{
    auto f = [&](const auto &error, auto)
    {
        this->on_receive_v1_prolog(error);
    };
    async_read(this->port_, buffer(this->prolog_epilog_buf_, V1_RESP_PROLOG_SIZE), f);
}

void SerialConnection::Impl::on_receive_v1_prolog(const system::error_code &error)
{
    if (error && !this->process_error(error, __FUNCTION__))
    {
        return;
    }
    /* VERSION 1 RESPONSE:
     *  ---------- ---------- ---------- ------------- ---------------- ------------
     * |   0xFB   |   PID    |   TYPE   | Length (BE) |    PAYLOAD     | CRC32 (BE) |
     * | (1 byte) | (1 byte) | (1 byte) |  (4 bytes)  | (Length bytes) | (4 bytes)  |
     *  ---------- ---------- ---------- ------------- ---------------- ------------
     */
    this->response_crc_accum_ = updateCrc32(this->response_crc_accum_,  (const uint8_t*)&this->prolog_epilog_buf_[0], V1_RESP_PROLOG_SIZE);

    this->response_pid_ = (uint8_t)this->prolog_epilog_buf_[V1_RESP_PROLOG_PID_OFFSET];
    this->response_type_ = this->prolog_epilog_buf_[V1_RESP_PROLOG_TYPE_OFFSET];
    uint32_t payload_length; BE_Get(payload_length, &this->prolog_epilog_buf_[V1_RESP_PROLOG_LENGTH_OFFSET]);
    DBG("Received V1 type: " << (unsigned)this->response_type_ << "; size: " << payload_length);
    this->begin_receive_payload(payload_length);
}

void SerialConnection::Impl::begin_receive_payload(uint32_t payload_length)
{
    this->payload_.resize(payload_length);
    auto f = [&](const auto &error, auto)
    {
        this->on_receive_payload(error);
    };

    async_read(this->port_, buffer(this->payload_), f);
}

void SerialConnection::Impl::on_receive_payload(const system::error_code &error)
{
    if (error && !this->process_error(error, __FUNCTION__))
    {
        return;
    }
    this->response_crc_accum_ = updateCrc32(this->response_crc_accum_, (const uint8_t*)&this->payload_[0], this->payload_.size());
    //Now wait for the end bytes before acknowledging the data and passing it off to clients. 
    this->begin_receive_end();
}

void SerialConnection::Impl::begin_receive_end()
{
    auto f = [&](const auto &error, auto)
    {
        this->on_receive_end(error);
    };

    async_read(this->port_, buffer(this->prolog_epilog_buf_, sizeof(uint32_t)), f);
}

void SerialConnection::Impl::on_receive_end(const system::error_code &error)
{
    if (error && !this->process_error(error, __FUNCTION__))
    {
        return;
    }

    BOOST_SCOPE_EXIT(this_)
    {
        //Start all over again when we exit this method
        this_->begin_receive_start();
    }
    BOOST_SCOPE_EXIT_END

    if (V0_RESP_START_MARK == this->response_marker_)
    {
        process_v0_response();
    }
    else if (V1_RESP_START_MARK == this->response_marker_)
    {
        process_v1_response();
    }
    else
    {
        ERR("Invalid response marker");
    }
}

void SerialConnection::Impl::process_v0_response()
{
    DBG(__FUNCTION__ << ": type: " << (unsigned)this->response_type_ << "; payload size: " << this->payload_.size());
    switch (std::to_integer<uint8_t>(this->response_type_))
    {
        case 0: //command answer
            if (this->on_command_response_)
            {
                //Make sure to clear the handler `on_command_response_` 
                // member variable and cancel the timer before calling 
                // the handler. Otherwise a race condition can ensue due
                // to the fact that the client is likely running in another
                // thread that could become ublocked and even run far enough
                // to start another command sequence prior to this thread
                // returning from the handler callback.
                auto on_command_response = this->on_command_response_;
                this->on_command_response_ = nullptr;
                this->response_timer_.cancel();
                on_command_response(false, this->payload_);
            }
            break;
        case 1: //measurement data
            if (this->on_measurement_data_)
            {
                this->on_measurement_data_(this->payload_);
            }
            break;
    }
}

bool SerialConnection::Impl::is_valid_v1_response()
{
    bool isValid { true };
    if (this->response_pid_ != this->v1_cmd_pid_)
    {
        ERR("PID in response (" << (unsigned)this->response_pid_
            << " does not match PID in command (" << (unsigned)this->v1_cmd_pid_);
        isValid = false;
    }
    const size_t preDataSize { sizeof(uint16_t) + sizeof(uint8_t) }; // CID, result code
    if (this->payload_.size() < preDataSize)
    {
        isValid = false;
    }
    else
    {
        uint16_t responseCID; BE_Get(responseCID, &this->payload_[0]);
        if (responseCID != this->v1_cmd_cid_)
        {
            ERR("CID in response (0X" << std::hex << (unsigned)responseCID << " does not match CID in command (0X" << this->v1_cmd_cid_);
            isValid = false;
        }
        const auto resultCode { this->payload_[sizeof(uint16_t)] };
        if (resultCode != std::byte{0})
        {
            DBG("Non-zero result code for CID 0X" << std::hex << responseCID << ": " << (unsigned)resultCode);
            isValid = false;
        }
        // Strip the CID/Result code from the version 1 "payload"
        this->payload_.erase(this->payload_.begin(),
                             ( (this->payload_.size() >= preDataSize) ?
                                   (this->payload_.begin() + preDataSize) : this->payload_.end()) );
    }
    return isValid;
}

void SerialConnection::Impl::process_v1_response()
{
    DBG(__FUNCTION__ << ": type: " << (unsigned)this->response_type_ << "; payload size: " << this->payload_.size());
    uint32_t response_crc { 0 }; BE_Get(response_crc, &this->prolog_epilog_buf_[0]);
    switch (std::to_integer<uint8_t>(this->response_type_))
    {
        case 0: //command answer
        {
            bool isValid { this->response_crc_accum_ == response_crc };
            if (!isValid)
            {
                ERR("-INVALID COMMAND CRC received: 0X" << std::hex << response_crc << "; stream: 0X" << this->response_crc_accum_ << std::dec);
            }
            if (this->on_command_response_)
            {
                //Make sure to clear the handler `on_command_response_` 
                // member variable and cancel the timer before calling 
                // the handler. Otherwise a race condition can ensue due
                // to the fact that the client is likely running in another
                // thread that could become ublocked and even run far enough
                // to start another command sequence prior to this thread
                // returning from the handler callback.
                auto on_command_response = this->on_command_response_;
                this->on_command_response_ = nullptr;
                this->response_timer_.cancel();
                isValid &= this->is_valid_v1_response();
                on_command_response(!isValid, this->payload_);
            }
            break;
        }
        case 1: //measurement data
        {
#if CHECK_DATA_CRC
#   if DATA_CRC_CHUNKS > 0
            const uint32_t crcOffset = payload_.size();
            const uint32_t chunkSize { (crcOffset + DATA_CRC_CHUNKS) / DATA_CRC_CHUNKS };
            uint32_t chunkCrcs[DATA_CRC_CHUNKS] { };
            uint32_t startOffset { 0 };
            for (uint32_t chunk = 0; chunk < DATA_CRC_CHUNKS; ++chunk)
            {
                const uint32_t crcSize { std::min(chunkSize, (crcOffset - startOffset)) }; // last chunk may be smaller
                chunkCrcs[chunk] = calcCrc32(payload_.data() + startOffset, crcSize);
                startOffset += chunkSize;
            }
            startOffset = 0;
            ERR("PID " << (unsigned)response_pid_ << " Data chunk CRCs {chunk, offset, CRC}:");
            for (uint32_t chunk = 0; chunk < DATA_CRC_CHUNKS; ++chunk)
            {
                ERR("  {" << chunk << std::hex << ", 0X" << startOffset << ", 0X" << chunkCrcs[chunk] << std::dec << "}");
                startOffset += chunkSize;
            }
#   endif
            if (this->response_crc_accum_ != response_crc)
            {
                ERR("-INVALID DATA CRC received: 0X" << std::hex << response_crc << "; stream: 0X" << this->response_crc_accum_ << std::dec);
            }
            else
#endif
            if (this->on_measurement_data_)
            {
                this->on_measurement_data_(this->payload_);
            }
            break;
        }
        default:
            ERR("Unknown response type: " << (unsigned)this->response_type_);
            break;
    }
}

bool SerialConnection::Impl::process_error(const system::error_code &error, const char* where)
{
    if (error)
    {
        ERR("ERROR in " << where << ": " << error.message());
    }
    else
    {
        DBG("NO ERROR in " << where );
    }
    return true; // continue for now
}

} //end namespace
