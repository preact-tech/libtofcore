/**
 * @file tof_sensor.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Implements API for libtofcore
 */
#include "comm_serial/serial_connection.hpp"
#include "tof_sensor.hpp"
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
    Impl(uint16_t protocolVersion, const std::string &portName, uint32_t baudrate) :
                connection(ioService, portName, baudrate, protocolVersion)
    {
    }

    void init();

    boost::asio::io_service ioService;
    std::thread serverThread_;
    std::mutex measurementReadyMutex;
    on_measurement_ready_t measurementReady;
    SerialConnection connection;
};

/* #########################################################################
 *
 * Sensor Class Implementation
 *
 * ######################################################################### */

Sensor::Sensor(uint16_t protocolVersion, const std::string &portName, uint32_t baudrate) :
            pimpl { new Impl { protocolVersion, portName, baudrate } }
{
    pimpl->init();
}

Sensor::~Sensor()
{
    pimpl->ioService.stop();
    pimpl->serverThread_.join();
}

bool Sensor::getAccelerometerData(int16_t& x, int16_t& y, int16_t& z, uint8_t& g_range)
{
    auto result = this->send_receive(COMMAND_READ_ACCELEROMETER);
    auto ok = bool {result};
    const auto& payload =*result;
    ok &= (payload.size() == READ_ACCELEROMETER_SIZE) && (READ_ACCELEROMETER_DATA_TYPE == (uint8_t)payload[DATA_TYPE_ID_OFFSET]);
    if (ok)
    {
        BE_Get(x, &payload[ACCELEROMETER_X_OFFSET]);
        BE_Get(y, &payload[ACCELEROMETER_Y_OFFSET]);
        BE_Get(z, &payload[ACCELEROMETER_Z_OFFSET]);
        g_range = (uint8_t)payload[ACCELEROMETER_G_RANGE_OFFSET];
    }

    return ok;
}

/* DEPRECATED - use getSensorInfo() instead. */
bool Sensor::getChipInformation(uint16_t& waferId, uint16_t& chipId)
{
    auto result = this->send_receive(COMMAND_READ_CHIP_INFO);
    auto ok = bool {result};
    const auto& payload = *result;

    ok &= (payload.size() == READ_CHIP_INFO_SIZE) && (READ_CHIP_INFO_DATA_TYPE == (uint8_t)payload[DATA_TYPE_ID_OFFSET]);
    if (ok)
    {
        BE_Get(waferId, &payload[CHIP_WAFER_ID_OFFSET]);
        BE_Get(chipId,  &payload[CHIP_CHIP_ID_OFFSET]);
    }

    return ok;
}

bool Sensor::getLensInfo(std::vector<double>& rays_x, std::vector<double>& rays_y, std::vector<double>& rays_z)
{
    auto result = this->send_receive(COMMAND_GET_LENS_INFO);

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

bool Sensor::getSettings(std::string& jsonSettings)
{
    auto result = this->send_receive(COMMAND_READ_SETTINGS);

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = answer.size();

    jsonSettings = std::string(reinterpret_cast<const char*>(answer.data()+1), size);

    return true;
}

/* DEPRECATED - use getSensorInfo() instead. */
bool Sensor::getSoftwareVersion(std::string& version)
{
    auto result = this->send_receive(COMMAND_READ_FIRMWARE_VERSION);

    if (!result)
    {
        return false; // failed to get information
    }

    const auto& answer = *result;
    const auto size = (answer.size() > 0) ? (answer.size() - 1) : 0; // We're not using leading version 0 protocol ACK, just the string that follows

    version = std::string(reinterpret_cast<const char*>(answer.data() + 1), size); // +1 to skip leading version 0 protocol ACK

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

bool Sensor::setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold, const uint16_t temporalEdgeThresholdLow, const uint16_t temporalEdgeThresholdHigh, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue)
{
    std::vector<uint8_t> payload;

    BE_Put(&payload[V0_T1_TEMPORAL_FILTER_FACTOR_INDEX], temporalFactor);

    BE_Put(&payload[V0_T1_TEMPORAL_FILTER_THRESHOLD_INDEX], temporalThreshold);

    payload[V0_T1_MEDIAN_FILTER_ENABLED_INDEX] = medianFilter ? 1 : 0;

    payload[V0_T1_AVERAGE_FILTER_ENABLED_INDEX] = averageFilter ? 1 : 0;

    BE_Put(&payload[V0_T1_EDGE_DETECTION_THRESHOLD_INDEX], edgeThreshold);

    payload[V0_T1_INTERFERENCE_DETECTION_USE_LAST_VALUE_INDEX] = interferenceDetectionUseLastValue ? 1 : 0;

    BE_Put(&payload[V0_T1_INTERFERENCE_DETECTION_LIMIT_INDEX], interferenceDetectionLimit);

    BE_Put(&payload[V0_T1_TEMPORAL_EDGE_FILTER_THRESHOLD_LOW_INDEX], temporalEdgeThresholdLow);

    BE_Put(&payload[V0_T1_TEMPORAL_EDGE_FILTER_THRESHOLD_HIGH_INDEX], temporalEdgeThresholdHigh);

    return pimpl->connection.send_receive(COMMAND_SET_FILTER, payload, 5s).has_value();
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

bool Sensor::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray)
{
    uint16_t params[4] = {native_to_big(low), native_to_big(mid), native_to_big(high), native_to_big(gray)};
    return this->send_receive(COMMAND_SET_INT_TIMES, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setMinAmplitude(uint16_t minAmplitude)
{
    return this->send_receive(COMMAND_SET_MIN_AMPLITUDE, minAmplitude).has_value();
}

bool Sensor::setModulation(const uint8_t index, const uint8_t channel)
{
    uint8_t params[] = {index, channel};
    return this->send_receive(COMMAND_SET_MODULATION, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setOffset(int16_t offset)
{
    return this->send_receive(COMMAND_SET_OFFSET, offset).has_value();
}

bool Sensor::setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1)
{
    uint16_t params[4] = {native_to_big(x0), native_to_big(y0), native_to_big(x1), native_to_big(y1)};
    return this->send_receive(COMMAND_SET_ROI, {(std::byte*)params, sizeof(params)}).has_value();
}

bool Sensor::setProtocolVersion(uint16_t version)
{
    return pimpl->connection.set_protocol_version(version);
}

uint16_t Sensor::getProtocolVersion() const
{
    return pimpl->connection.get_protocol_version();
}

bool Sensor::stopStream()
{
    return this->send_receive(COMMAND_STOP_STREAM).has_value();
}

bool Sensor::storeSettings()
{
    return this->send_receive(COMMAND_STORE_SETTINGS).has_value();
}

bool Sensor::streamDCS()
{
    return this->send_receive(COMMAND_GET_DCS, (uint8_t)1).has_value();
}

bool Sensor::streamDCSAmbient()
{
    return this->send_receive(COMMAND_GET_DCS_AMBIENT, (uint8_t)1).has_value();
}

bool Sensor::streamDistance()
{
    return this->send_receive(COMMAND_GET_DISTANCE, (uint8_t)1).has_value();
}

bool Sensor::streamDistanceAmplitude()
{

    return this->send_receive(COMMAND_GET_DIST_AND_AMP, (uint8_t)1).has_value();
}

bool Sensor::streamGrayscale()
{
    return this->send_receive(COMMAND_GET_GRAYSCALE, (uint8_t)1).has_value();
}

void Sensor::subscribeMeasurement(std::function<void (std::shared_ptr<Measurement_T>)> onMeasurementReady)
{
    std::lock_guard<std::mutex> guard {pimpl->measurementReadyMutex};
    pimpl->measurementReady = onMeasurementReady;
}


bool Sensor::getSensorInfo(TofComm::versionData_t &versionData){

    auto result = this->send_receive(COMMAND_READ_SENSOR_INFO);

    auto ok = bool{result};

    const auto& payload = *result;

    ok &= (payload.size() == sizeof(versionData));

    if (ok)
    {
        //TODO Not really sure that this is a safe thing to do, it assumes that the data exactly matches the structure.
        memcpy((void*)&versionData, (void*)payload.data(), sizeof(versionData));
    }

    return ok;
}

/* #########################################################################
 *
 * Sensor::Impl Implementation
 *
 * ######################################################################### */

void Sensor::Impl::init()
{
    serverThread_ = std::thread{ [this]() { ioService.run(); } };

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
    connection.subscribe(f);
}


Sensor::send_receive_result_t Sensor::send_receive(const uint16_t command, const std::vector<Sensor::send_receive_payload_t>& payload,
        std::chrono::steady_clock::duration timeout /*= 5s*/) const
{
    auto result = pimpl->connection.send_receive(command, payload, timeout);
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
