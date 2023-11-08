/**
 * @file tof_sensor.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Implements API for libtofcore
 */
#include "tof_sensor.hpp"
#include "connection.hpp"
#include "comm_serial/serial_connection.hpp"
#include "tofcore/device_discovery.hpp"
#include "tofcore/TofEndian.hpp"
#include "TofCommand_IF.hpp"
#include "TofEndian.hpp"
#include "Measurement_T.hpp"
#include <boost/endian/conversion.hpp>
#include <boost/scope_exit.hpp>
#include <iostream>
#include <mutex>
#include <thread>

namespace tofcore {

using namespace std;
using namespace std::chrono_literals;
using namespace boost::endian;
using namespace TofComm;

struct Sensor::Impl
{
    Impl(const std::string &uri) :
        connection(Connection_T::create(ioService, uri)),
        measurement_timer_(ioService)
    {
    }

    void init();

    boost::asio::io_service ioService;
    std::thread serverThread_;
    std::mutex measurementReadyMutex;
    on_measurement_ready_t measurementReady;
    std::unique_ptr<Connection_T> connection;

    /// The items below are specifically used for streaming via polling.
    /// Note: This mode is currently only used on Windows systems due to a strange Windows only phenomenon.
    ///  Data from the sensor can be randomly dropped for no reason that we can discern.
    ///  When data is dropped our parser gets out sync and cannot easily recover.
    ///  By using polling to request each frame we can easily detected a dropped frame (via a timeout)
    ///  at which time we can reset our parser and then request the next frame.
    ///  Some day we hope to figure out the real problem and or make our parser more resilient to dropped data
    //   but for now this is what we have.
    bool stream_via_polling_ { false };
    uint16_t measurement_command_ = 0;
    boost::asio::steady_timer measurement_timer_;

    /// Method used to initiate streaming of measurement data
    /// 
    /// @param measurement_command The measurement command ID to to send
    /// @return true on successful
    bool request_measurement_stream(uint16_t measurement_command)
    {
        this->measurement_command_ = measurement_command;
#if defined(_WIN32)
        stream_via_polling_ = true;
#else
        stream_via_polling_ = false;
#endif

        if(stream_via_polling_)
        {
            return this->request_next_measurement();
        }
        else
        {
            //TODO Consider changing this to send_receive() so we can really know if it succeeds.
            this->connection->send(this->measurement_command_, &TofComm::CONTINUOUS_MEASUREMENT, sizeof(TofComm::CONTINUOUS_MEASUREMENT));
            return true;
        }
    }

    /// Method used to request the next measurement item when stream_via_polling_ is true.
    ///
    /// @return true on successful
    bool request_next_measurement()
    {
        auto f = [this](const boost::system::error_code error)
        {
            if ((error != boost::asio::error::operation_aborted) && this->stream_via_polling_)
            {
                this->connection->reset_parser();
                this->request_next_measurement();
            }
        };

        this->measurement_timer_.cancel();
        this->measurement_timer_.expires_after(250ms);
        this->measurement_timer_.async_wait(f);

        this->connection->send(this->measurement_command_, &TofComm::SINGLE_MEASUREMENT, sizeof(TofComm::SINGLE_MEASUREMENT));
        //Assume success: Note we can't use send_receive() here because this method could be called
        // from the context of the measurement callback and we would deadlock.
        return true;
    }
};

/* #########################################################################
 *
 * Sensor Class Implementation
 *
 * ######################################################################### */

Sensor::Sensor(const std::string& uri /*= std::string()*/)
{
    std::string connection_uri = uri;
    if (connection_uri.empty())
    {
        // Get all PreAct Devices
        const auto devices = find_all_devices();
        
        // No PreAct Devices
        if (devices.empty())
        {
            throw std::runtime_error("No PreAct Devices Found");
        }

        connection_uri = devices[0].connector_uri;
    }
    this->pimpl = std::unique_ptr<Impl>(new Impl(connection_uri));
    pimpl->init();
}


Sensor::Sensor(uint16_t protocolVersion, const std::string &portName, uint32_t baudrate) 
{
    std::string connection_uri = portName;
    // If no port name given. Find first PreAct device avaiable
    if (connection_uri.empty()){

        // Get all PreAct Devices
        std::vector<device_info_t> devices = find_all_devices();
        
        // No PreAct Devices
        if (devices.empty()){
            throw std::runtime_error("No PreAct Devices Found");
        }
        connection_uri = devices[0].connector_uri;
    }

    //Add protocol version and baudrate to uri query parameters
    connection_uri += "?baudrate=";
    connection_uri += std::to_string(baudrate);
    connection_uri += ";protocol_version=";
    connection_uri += std::to_string(protocolVersion);

    this->pimpl = std::unique_ptr<Impl>(new Impl(connection_uri));
    pimpl->init();
}

Sensor::~Sensor()
{
    pimpl->ioService.stop();
    pimpl->serverThread_.join();
}

std::optional<std::vector<uint16_t>> Sensor::getIntegrationTimes()
{
    auto result = this->send_receive(COMMAND_GET_INT_TIMES);

    if (!result)
    {
        return std::nullopt;
    }
    const auto &payload = *result;
    const auto payloadSize = payload.size();
    const size_t numIntegrationTimes = std::min((payloadSize / sizeof(uint16_t)), TofComm::KLV_NUM_INTEGRATION_TIMES);
    std::vector<uint16_t> integrationTimes { };

    for (size_t i = 0; i < numIntegrationTimes; ++i)
    {
        uint16_t intTime;
        BE_Get(intTime, &payload[2 * i]);
        integrationTimes.push_back(intTime);
    }

    return { integrationTimes };
}

bool Sensor::getLensInfo(std::vector<double>& rays_x, std::vector<double>& rays_y, std::vector<double>& rays_z)
{
    auto result = this->send_receive(COMMAND_GET_LENS_INFO, (uint8_t)1);

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = answer.size();

    //The first byte in the answer is the message ID, so we can skip that to get to the data words.
    //The data is packed x,y,z scaled int32_t (big-endian) in row major order.
    const int16_t* begin = reinterpret_cast<const int16_t*>(answer.data()+1);
    const int16_t* end = begin + (size/sizeof(int16_t));
    const double imax = std::numeric_limits<int16_t>::max();
    const auto ray_count = std::distance(begin, end)/3;
    rays_x.reserve(ray_count);
    rays_y.reserve(ray_count);
    rays_z.reserve(ray_count);
    int16_t v;
    for( auto i = begin; i != end;)
    {
        BE_Get(v, i++);
        rays_x.push_back(v / imax);
        BE_Get(v, i++);
        rays_y.push_back(v / imax);
        BE_Get(v, i++);
        rays_z.push_back(v / imax);
    }
    return true;
}

std::optional<LensIntrinsics_t> Sensor::getLensIntrinsics()
{
    auto result = this->send_receive(COMMAND_GET_LENS_INFO, (uint8_t)0);

    if (!result || (result->size() != RAW_SENSOR_INFO_DATA_SIZE))
    {
        return std::nullopt; // failed to get information
    }
    LensIntrinsics_t li { };

    const uint8_t* rawPtr = reinterpret_cast<const uint8_t*>(result->data());
    int16_t s16 { 0 };
    // Row offset
    BE_Get(s16, rawPtr);
    li.m_rowOffset = s16 / 1.0E2;
    rawPtr += sizeof(s16);
    // Column offset
    BE_Get(s16, rawPtr);
    li.m_columnOffset = s16 / 1.0E2;
    rawPtr += sizeof(s16);
    // Row focal length
    uint32_t u32 { 0 };
    BE_Get(u32, rawPtr);
    li.m_rowFocalLength = u32 / 1.0E6;
    rawPtr += sizeof(u32);
    // Column focal length
    BE_Get(u32, rawPtr);
    li.m_columnFocalLength = u32 / 1.0E6;
    rawPtr += sizeof(u32);
    // Un-distortion coefficients
    for (size_t n = 0; n < 5; ++n)
    {
        int32_t i32 { 0 };
        BE_Get(i32, rawPtr);
        li.m_undistortionCoeffs[n] = i32 / 1.0E8;
        rawPtr += sizeof(i32);
    }
    return { li };
}

bool Sensor::getSensorInfo(TofComm::versionData_t &versionData)
{
    auto result = this->send_receive(COMMAND_READ_SENSOR_INFO);
    auto ok = bool { result };
    const auto &payload = *result;

    ok &= (payload.size() == sizeof(versionData));

    if (ok)
    {
        memcpy((void*) &versionData, (void*) payload.data(), sizeof(versionData));
    }
    return ok;
}

std::optional<std::string> Sensor::getSensorLocation()
{
    auto result = this->send_receive(COMMAND_GET_SENSOR_LOCATION);
    if (!result)
    {
        return std::nullopt; // failed to get information
    }
    else
    {
        return { std::string(reinterpret_cast<const char*>(result->data()), result->size()) };
    }
}

std::optional<std::string> Sensor::getSensorName()
{
    auto result = this->send_receive(COMMAND_GET_SENSOR_NAME);
    if (!result)
    {
        return std::nullopt; // failed to get information
    }
    else
    {
        return { std::string(reinterpret_cast<const char*>(result->data()), result->size()) };
    }
}

bool Sensor::getSensorStatus(TofComm::Sensor_Status_t &sensorStatus)
{
    auto result = this->send_receive(COMMAND_READ_SENSOR_STATUS);
    auto ok = bool { result };
    const auto &payload = *result;

    ok &= (payload.size() == sizeof(sensorStatus));

    if (ok)
    {
        memcpy((void*) &sensorStatus, (void*) payload.data(), sizeof(sensorStatus));

        sensorStatus.lastTemperature = boost::endian::big_to_native(sensorStatus.lastTemperature);
        sensorStatus.USB_Current = boost::endian::big_to_native(sensorStatus.USB_Current);
        sensorStatus.BIT_Status = boost::endian::big_to_native(sensorStatus.BIT_Status);

    }
    return ok;
}

bool Sensor::getSettings(std::string& jsonSettings)
{
    auto result = this->send_receive(COMMAND_READ_SETTINGS);

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = answer.size();

    jsonSettings = std::string(reinterpret_cast<const char*>(answer.data()+1), size-1);

    return true;
}

/**
 * @brief Check if Horizontal flip is active.
 * 
 * @return std::tuple<bool, bool> - <message success, Hflip active>
 */
std::optional<bool> Sensor::isFlipHorizontallyActive()
{
    bool hIsFlipped { false };
    auto result = this->send_receive(COMMAND_GET_HORIZ_FLIP_STATE);
    if (result)
    {
        const auto& answer = *result;
        if (answer.size() > 0)
        {
            hIsFlipped = (*reinterpret_cast<const uint8_t*>(answer.data()) != 0);
            return hIsFlipped;
        }
    }
    return std::nullopt;
}

/**
 * @brief Check if Vertical flip is active.
 * 
 * @return std::tuple<bool, bool> - <message success, Vflip active>
 */
std::optional<bool> Sensor::isFlipVerticallyActive()
{
    bool vIsFlipped { false };
    auto result = this->send_receive(COMMAND_GET_VERT_FLIP_STATE);
    if (result)
    {
        const auto& answer = *result;
        if (answer.size() > 0)
        {
            vIsFlipped = (*reinterpret_cast<const uint8_t*>(answer.data()) != 0);
            return vIsFlipped;
        }
    }
    return std::nullopt;
}

void Sensor::jumpToBootloader()
{
    this->send_receive(COMMAND_JUMP_TO_BOOLOADER);
}


void Sensor::jumpToBootloader(uint16_t token)
{
    this->send_receive(COMMAND_JUMP_TO_BOOLOADER, token);
}

bool Sensor::setBinning(const bool vertical, const bool horizontal)
{
    uint8_t byte = 0;
    if (vertical && horizontal)
    {
        byte = 3;
    }
    else if (vertical)
    {
        byte = 1;
    }
    else if (horizontal)
    {
        byte = 2;
    }

    return bool{this->send_receive(COMMAND_SET_BINNING, byte)};
}

bool Sensor::setFlipHorizontally(bool flip)
{
    const uint8_t data = (flip ? 1 : 0);
    return bool{this->send_receive(COMMAND_SET_HORIZ_FLIP_STATE, data)};
}

bool Sensor::setFlipVertically(bool flip)
{
    const uint8_t data = (flip ? 1 : 0);
    return bool{this->send_receive(COMMAND_SET_VERT_FLIP_STATE, data)};
}

bool Sensor::setHDRMode(uint8_t mode)
{
    return this->send_receive(COMMAND_SET_HDR, mode).has_value();
}

bool Sensor::setIntegrationTime(uint16_t low)
{
    uint16_t params[] = {native_to_big(low)};
    return this->send_receive(COMMAND_SET_INT_TIMES, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setIntegrationTimes(uint16_t low, uint16_t mid, uint16_t high)
{
    uint16_t params[] = {native_to_big(low), native_to_big(mid), native_to_big(high)};
    return this->send_receive(COMMAND_SET_INT_TIMES, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setIPv4Settings(const std::array<std::byte, 4>& adrs, const std::array<std::byte, 4>& mask, const std::array<std::byte, 4>& gateway)
{
    std::vector<std::byte> ipData { std::begin(adrs), std::end(adrs) };
    std::copy(std::begin(mask), std::end(mask), std::back_inserter(ipData));
    std::copy(std::begin(gateway), std::end(gateway), std::back_inserter(ipData));
    return this->send_receive(COMMAND_SET_CAMERA_IP_SETTINGS, {ipData.data(), ipData.size()}).has_value();
}

bool Sensor::setMinAmplitude(uint16_t minAmplitude)
{
    return this->send_receive(COMMAND_SET_MIN_AMPLITUDE, minAmplitude).has_value();
}

bool Sensor::setModulation(const uint16_t modFreqkHz)
{
    return this->send_receive(COMMAND_SET_MODULATION, modFreqkHz).has_value();
}

std::optional<uint16_t> Sensor::getModulation(void)
{
    auto result = this->send_receive(COMMAND_GET_MODULATION);

    if (!result)
    {
        return std::nullopt;
    }
    const auto &payload = *result;

    uint16_t modFreqkHz = 0;
    BE_Get(modFreqkHz, &payload[0]);

    return {modFreqkHz};
}

bool Sensor::setOffset(int16_t offset)
{
    return this->send_receive(COMMAND_SET_OFFSET, offset).has_value();
}

bool Sensor::setProtocolVersion(uint16_t version)
{
    return pimpl->connection->set_protocol_version(version);
}

bool Sensor::setSensorLocation(std::string location)
{
    return this->send_receive(COMMAND_SET_SENSOR_LOCATION, {(std::byte*)location.data(), location.size()}).has_value();
}

bool Sensor::setSensorName(std::string name)
{
    return this->send_receive(COMMAND_SET_SENSOR_NAME, {(std::byte*)name.data(), name.length()}).has_value();
}

bool Sensor:: getIPv4Settings(std::array<std::byte, 4>& adrs, std::array<std::byte, 4>& mask, std::array<std::byte, 4>& gateway)
{
    auto result = this->send_receive(COMMAND_GET_CAMERA_IP_SETTINGS);
    if (result && (result->size() == 12))
    {
        auto it = std::begin(*result);
        std::copy(it, it + 4, std::begin(adrs));
        it += 4;
        std::copy(it, it + 4, std::begin(mask));
        it += 4;
        std::copy(it, it + 4, std::begin(gateway));
        return true;
    }
    else
    {
        return false;
    }
}

uint16_t Sensor::getProtocolVersion() const
{
    return pimpl->connection->get_protocol_version();
}

bool Sensor::stopStream()
{
    this->pimpl->stream_via_polling_ = false;
    this->pimpl->measurement_timer_.cancel();
    return this->send_receive(COMMAND_STOP_STREAM).has_value();
}

bool Sensor::storeSettings()
{
    return this->send_receive(COMMAND_STORE_SETTINGS).has_value();
}

bool Sensor::streamDCS()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DCS);
}

bool Sensor::streamDCSAmbient()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DCS_AMBIENT);
}

bool Sensor::streamDistance()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DISTANCE);
}

bool Sensor::streamDistanceAmplitude()
{
    return this->pimpl->request_measurement_stream(COMMAND_GET_DIST_AND_AMP);
}

void Sensor::subscribeMeasurement(std::function<void (std::shared_ptr<Measurement_T>)> onMeasurementReady)
{
    //Hook the callback so that if stream via polling is enabled we can request
    // the next measurement just before before delivering to the caller
    auto hook = [this, onMeasurementReady](std::shared_ptr<Measurement_T> measurement) -> void
    {
        if (pimpl->stream_via_polling_)
        {
            pimpl->request_next_measurement();
        }
        
        //Now pass the data on to our client
        if (onMeasurementReady)
        {
            onMeasurementReady(measurement);
        }
    };
    std::lock_guard<std::mutex> guard {pimpl->measurementReadyMutex};
    pimpl->measurementReady = hook;
}

/* #########################################################################
 *
 * Sensor::Impl Implementation
 *
 * ######################################################################### */

void Sensor::Impl::init()
{
    auto server_f = [this]() 
        {
            try
            { 
                ioService.run(); 
            }
            catch(std::exception& e)
            {
                std::cerr << "caught exception in background thread: " << e.what() << std::endl;
            }
        };

    serverThread_ = std::thread{ server_f };

    auto f = [&](const std::vector<std::byte>& p)
        {
            on_measurement_ready_t usr_callback;
            { std::lock_guard<std::mutex> guard {measurementReadyMutex};
                usr_callback = measurementReady;
            }

            if (usr_callback)
            {
                auto new_measurement = tofcore::create_measurement(p);
                try {
                    usr_callback(new_measurement);
                } catch(std::exception& err) {
                    std::string what = err.what();
                    std::cerr << "caught unhandled exception from user callback: \n\t" << what << std::endl;
                }
            }
        };
    connection->subscribe(f);
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, const std::vector<Sensor::send_receive_payload_t>& payload,
        std::chrono::steady_clock::duration timeout /*= 5s*/) const
{
    auto result = pimpl->connection->send_receive(command, payload, timeout);
    if(!result)
    {
        return std::nullopt;
    }
    return {*result};
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, const send_receive_payload_t& payload,
        std::chrono::steady_clock::duration timeout /*= 5s*/) const
{
    const std::vector<Sensor::send_receive_payload_t> one{payload};
    return this->send_receive(command, one, timeout);
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command) const
{
    return this->send_receive(command, send_receive_payload_t{});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint8_t value) const
{
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int8_t value) const
{
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint16_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int16_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, uint32_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, int32_t value) const
{
    boost::endian::native_to_big_inplace(value);
    auto ptr = reinterpret_cast<std::byte*>(&value);
    return this->send_receive(command, {ptr, sizeof(value)});
}

} //end namespace tofcore
