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

    typedef std::tuple<bool, std::vector<uint8_t> > send_receive_result_t;
    send_receive_result_t send_receive(uint16_t command);
    send_receive_result_t send_receive(uint16_t command, uint32_t value);
    send_receive_result_t send_receive(uint16_t command, int32_t value);
    send_receive_result_t send_receive(uint16_t command, uint16_t value);
    send_receive_result_t send_receive(uint16_t command, int16_t value);
    send_receive_result_t send_receive(uint16_t command, uint8_t value);
    send_receive_result_t send_receive(uint16_t command, int8_t value);
    send_receive_result_t streamMeasurement(uint16_t);
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
    auto result = pimpl->send_receive(COMMAND_READ_ACCELEROMETER);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == READ_ACCELEROMETER_SIZE) && (READ_ACCELEROMETER_DATA_TYPE == payload[DATA_TYPE_ID_OFFSET]);
    if (ok)
    {
        BE_Get(x, &payload[ACCELEROMETER_X_OFFSET]);
        BE_Get(y, &payload[ACCELEROMETER_Y_OFFSET]);
        BE_Get(z, &payload[ACCELEROMETER_Z_OFFSET]);
        g_range = payload[ACCELEROMETER_G_RANGE_OFFSET];
    }

    return ok;
}

bool Sensor::getChipInformation(uint16_t& waferId, uint16_t& chipId)
{
    auto result = pimpl->send_receive(COMMAND_READ_CHIP_INFO);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == READ_CHIP_INFO_SIZE) && (READ_CHIP_INFO_DATA_TYPE == payload[DATA_TYPE_ID_OFFSET]);
    if (ok)
    {
        BE_Get(waferId, &payload[CHIP_WAFER_ID_OFFSET]);
        BE_Get(chipId,  &payload[CHIP_CHIP_ID_OFFSET]);
    }

    return ok;
}

bool Sensor::getLensInfo(std::vector<double>& rays_x, std::vector<double>& rays_y, std::vector<double>& rays_z)
{
    auto result = pimpl->send_receive(COMMAND_GET_LENS_INFO);

    if (!std::get<bool>(result))
    {
        return false; // failed to get information
    }

    const auto& answer = std::get<1>(result);
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
    auto result = pimpl->send_receive(COMMAND_READ_SETTINGS);

    if (!std::get<bool>(result))
    {
        return false; // failed to get information
    }

    const auto& answer = std::get<1>(result);
    const auto size = answer.size();

    jsonSettings = std::string(reinterpret_cast<const char*>(answer.data()+1), size);

    return true;
}

bool Sensor::getSoftwareVersion(std::string& version)
{
    auto result = pimpl->send_receive(COMMAND_READ_FIRMWARE_VERSION);

    if (!std::get<bool>(result))
    {
        return false; // failed to get information
    }

    const auto& answer = std::get<1>(result);
    const auto size = (answer.size() > 0) ? (answer.size() - 1) : 0; // We're not using leading version 0 protocol ACK, just the string that follows

    version = std::string(reinterpret_cast<const char*>(answer.data() + 1), size); // +1 to skip leading version 0 protocol ACK

    return true;
}

std::tuple<bool, StorageMetadata_T> Sensor::getStorageMetadata(TofComm::StorageId_e id, TofComm::StorageMode_e mode)
{
    StorageMetadata_T metaData { .m_storageId = id, .m_mode = mode };
    bool isOk { false };
    uint16_t startProtocol = pimpl->connection.get_protocol_version();
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(1); // cannot use version 0 protocol for storage commands
    }
    auto result = pimpl->connection.send_receive(COMMAND_STORAGE_GET_METADATA, (const uint8_t*)&metaData, sizeof(metaData), 5s);
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(startProtocol);
    }
    isOk = std::get<bool>(result);
    if (isOk)
    {
        const uint8_t* data = std::get<1>(result).data();
        {
            uint32_t v32;
            BE_Get(v32, data + offsetof(StorageMetadata_T, m_dataSize));
            metaData.m_dataSize = v32;
            BE_Get(v32, data + offsetof(StorageMetadata_T, m_dataCrc32));
            metaData.m_dataCrc32 = v32;
        }
        {
            uint16_t v16;
            BE_Get(v16, data + offsetof(StorageMetadata_T, m_dataType));
            metaData.m_dataType = v16;
            BE_Get(v16, data + offsetof(StorageMetadata_T, m_typeVersion));
            metaData.m_typeVersion = v16;
        }
    }
    return std::make_tuple(isOk, metaData);
}

void Sensor::jumpToBootloader()
{
    pimpl->send_receive(COMMAND_JUMP_TO_BOOLOADER);
}

std::tuple<bool, uint16_t> Sensor::sequencerGetVersion() const
{
    auto result = pimpl->send_receive(COMMAND_SEQU_GET_VERSION);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == sizeof(uint16_t));
    uint16_t version { 0 };
    if (ok)
    {
        BE_Get(version, &payload[0]);
    }

    return std::make_tuple(ok, version);
}

std::tuple<bool, bool> Sensor::sequencerIsVersionSupported(uint16_t version) const
{
    auto result = pimpl->send_receive(COMMAND_SEQU_VERSION_IS_SUPPORTED, version);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == sizeof(uint8_t));
    bool isSupported { false };
    if (ok)
    {
        isSupported = (payload[0] != 0);
    }

    return std::make_tuple(ok, isSupported);
}

bool Sensor::sequencerSetVersion(uint16_t version)
{
    auto result = pimpl->send_receive(COMMAND_SEQU_SET_VERSION, version);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == sizeof(uint8_t)) && (0 == payload[0]);

    return ok;
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

    return std::get<bool>(pimpl->send_receive(COMMAND_SET_BINNING, byte));
}

bool Sensor::setFactoryMode(bool enable)
{
    std::vector<uint8_t> payload { enable };
    return std::get<bool>(pimpl->connection.send_receive(COMMAND_FACTORY_MODE, payload, 5s));
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

    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_FILTER, payload, 5s));
}

bool Sensor::setHDRMode(uint8_t mode)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_HDR, mode));
}

bool Sensor::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray)
{
    uint16_t params[4] = {native_to_big(low), native_to_big(mid), native_to_big(high), native_to_big(gray)};
    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_INT_TIMES, (uint8_t*)params, sizeof(params), 5s));
}

bool Sensor::setMinAmplitude(uint16_t minAmplitude)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_MIN_AMPLITUDE, minAmplitude));
}

bool Sensor::setModulation(const uint8_t index, const uint8_t channel)
{
    uint8_t params[] = {index, channel};
    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_MODULATION, &params[0], sizeof(params), 5s));
}

bool Sensor::setOffset(int16_t offset)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_OFFSET, offset));
}

bool Sensor::setProtocolVersion(uint16_t version)
{
    return pimpl->connection.set_protocol_version(version);
}

bool Sensor::setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1)
{
    uint16_t params[4] = {native_to_big(x0), native_to_big(y0), native_to_big(x1), native_to_big(y1)};
    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_ROI, (uint8_t*)params, sizeof(params), 5s));
}

bool Sensor::startDrnuCalibration()
{
    return std::get<bool>(pimpl->send_receive(COMMAND_CALIBRATE));
}

bool Sensor::startProductionCalibration()
{
    return std::get<bool>(pimpl->send_receive(COMMAND_CALIBRATE_PRODUCTION));
}

bool Sensor::stopStream()
{
    return std::get<bool>(pimpl->send_receive(COMMAND_STOP_STREAM));
}

std::tuple<bool, std::vector<uint8_t>> Sensor::storageRead(StorageId_e id, TofComm::StorageMode_e mode,
                                                           uint32_t storageOffset, uint32_t numBytes)
{
    StorageReadCommand_T readCommand { .m_storageId = id, .m_mode = mode };
    BE_Put(&readCommand.m_offset, storageOffset);
    BE_Put(&readCommand.m_numBytes, numBytes);
    uint16_t startProtocol = pimpl->connection.get_protocol_version();
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(1); // cannot use version 0 protocol for storage commands
    }
    // guarantee we return to starting protocol even if exception is thrown
    BOOST_SCOPE_EXIT(startProtocol, this_)
    {
        if (0 == startProtocol)
        {
            this_->pimpl->connection.set_protocol_version(startProtocol);
        }
    }
    BOOST_SCOPE_EXIT_END

    auto result = pimpl->connection.send_receive(COMMAND_STORAGE_READ, (const uint8_t*)&readCommand, sizeof(readCommand), 5s);

    return result;
}

bool Sensor::storageWriteData(StorageId_e id, TofComm::StorageMode_e mode,
                              uint32_t storageOffset, const uint8_t *data, uint32_t numBytes)
{
    const WriteContinueCommand_T cmd { id, mode, htonl(storageOffset) };
    const std::vector<ScatterGatherElement> cmdAndData
    {
        { (const uint8_t*)&cmd, (uint32_t)sizeof(cmd) },
        { data,                 numBytes              }
    };
    uint16_t startProtocol = pimpl->connection.get_protocol_version();
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(1); // cannot use version 0 protocol for storage commands
    }
    auto result = pimpl->connection.send_receive(COMMAND_STORAGE_WRITE, cmdAndData, 5s);
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(startProtocol);
    }
    return std::get<bool>(result);
}

bool Sensor::storageWriteFinish(TofComm::StorageId_e id, TofComm::StorageMode_e mode, uint32_t totalSize, uint32_t crc32)
{
    const WriteFinishCommand_T cmd { id, mode, htonl(totalSize), htonl(crc32) };
    uint16_t startProtocol = pimpl->connection.get_protocol_version();
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(1); // cannot use version 0 protocol for storage commands
    }
    auto result = pimpl->connection.send_receive(COMMAND_STORAGE_WRITE, (const uint8_t*)&cmd, sizeof(cmd), 5s);
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(startProtocol);
    }
    return std::get<bool>(result);
}

bool Sensor::storageWriteStart(TofComm::StorageId_e id, TofComm::StorageMode_e mode)
{
    const WriteStartCommand_T cmd { id, mode };
    uint16_t startProtocol = pimpl->connection.get_protocol_version();
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(1); // cannot use version 0 protocol for storage commands
    }
    auto result = pimpl->connection.send_receive(COMMAND_STORAGE_WRITE, (const uint8_t*)&cmd, sizeof(cmd), 5s);
    if (0 == startProtocol)
    {
        pimpl->connection.set_protocol_version(startProtocol);
    }
    return std::get<bool>(result);
}

bool Sensor::storeSettings()
{
    return std::get<bool>(pimpl->send_receive(COMMAND_STORE_SETTINGS));
}

bool Sensor::streamDCS()
{
    return std::get<bool>(pimpl->streamMeasurement(COMMAND_GET_DCS));
}

bool Sensor::streamDCSAmbient()
{
    return std::get<bool>(pimpl->streamMeasurement(COMMAND_GET_DCS_AMBIENT));
}

bool Sensor::streamDistance()
{
    return std::get<bool>(pimpl->streamMeasurement(COMMAND_GET_DISTANCE));
}

bool Sensor::streamDistanceAmplitude()
{
    return std::get<bool>(pimpl->streamMeasurement(COMMAND_GET_DIST_AND_AMP));
}

bool Sensor::streamGrayscale()
{
    return std::get<bool>(pimpl->streamMeasurement(COMMAND_GET_GRAYSCALE));
}

void Sensor::subscribeMeasurement(std::function<void (std::shared_ptr<Measurement_T>)> onMeasurementReady)
{
    std::lock_guard<std::mutex> guard {pimpl->measurementReadyMutex};
    pimpl->measurementReady = onMeasurementReady;
}

bool Sensor::setDllStep(bool enable, uint8_t coarseStep, uint8_t fineStep, uint8_t finestStep){

    uint8_t params[] = {(uint8_t)enable, coarseStep, fineStep, finestStep};

    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_DLL_STEP, &params[0], sizeof(params), 5s));  
}

bool Sensor::readRegister(uint8_t regAddress, uint8_t& regData){

    auto result = pimpl->send_receive(COMMAND_READ_REGISTER, regAddress);

    bool ok = std::get<bool>(result);

    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == READ_REGISTER_DATA_SIZE); // Data type field is 6 for REG READ

    if (ok)
    {
        regData = payload[READ_REGISTER_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::writeRegister(uint8_t regAddress, uint8_t regData){

    uint8_t params[] = {regAddress, regData};

    return std::get<bool>(pimpl->connection.send_receive(COMMAND_WRITE_REGISTER, &params[0], sizeof(params), 5s));  
}


/* #########################################################################
* 
* Illuminator Board Commands
*
* ########################################################################## */

bool Sensor::setVledEnables(uint8_t vledEnables){

    uint8_t params[] = {vledEnables};

    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_VLED_ENABLES, &params[0], sizeof(params), 5s));  
}

bool Sensor::getVledEnables(uint8_t& vledEnables)
{
    auto result = pimpl->send_receive(COMMAND_GET_VLED_ENABLES);

    bool ok = std::get<bool>(result);

    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == VLED_ENABLES_DATA_SIZE); 

    if (ok)
    {
        vledEnables = payload[VLED_ENABLES_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::setVled(uint16_t vledMv)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_VLED, vledMv));
}

bool Sensor::getVled(uint16_t& vledMv)
{
    auto result = pimpl->send_receive(COMMAND_GET_VLED);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == VLED_DATA_SIZE);
    if (ok)
    {
        BE_Get(vledMv, &payload[VLED_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIb5V(uint16_t& v5Mv)
{
    auto result = pimpl->send_receive(COMMAND_GET_IB_5V);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == IB_5V_DATA_SIZE);
    if (ok)
    {
        BE_Get(v5Mv, &payload[IB_5V_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIllmnTemperature(int16_t& tempMdegC)
{
    auto result = pimpl->send_receive(COMMAND_GET_IB_TEMPERATURE);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == IB_TEMPERATURE_DATA_SIZE);
    if (ok)
    {
        BE_Get(tempMdegC, &payload[IB_TEMPERATURE_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIbPd(uint16_t& photodiodeMv)
{
    auto result = pimpl->send_receive(COMMAND_GET_IB_PD);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == IB_PD_DATA_SIZE);
    if (ok)
    {
        BE_Get(photodiodeMv, &payload[IB_PD_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::getIbSerial(uint32_t& serialNum)
{
    auto result = pimpl->send_receive(COMMAND_GET_SERIAL_NUM);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == SERIAL_NUM_DATA_SIZE);
    if (ok)
    {
        BE_Get(serialNum, &payload[SERIAL_NUM_DATA_OFFSET]);
    }

    return ok;
}

bool Sensor::setIbSerial(uint32_t serialNum)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_SERIAL_NUM, serialNum));
}

bool Sensor::setIbRgb(uint8_t rgbBitmap, uint16_t rgbBlinkMs)
{
    uint8_t params[sizeof(rgbBitmap) + sizeof(rgbBlinkMs)];
    params[0] = rgbBitmap;
    BE_Put(&params[sizeof(rgbBitmap)], rgbBlinkMs);

    return std::get<bool>(pimpl->connection.send_receive(COMMAND_SET_RGB, &params[0], sizeof(params), 5s));  
}

bool Sensor::getIbRgb(uint8_t& rgbBitmap)
{
    auto result = pimpl->send_receive(COMMAND_GET_RGB);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == RGB_DATA_SIZE);
    if (ok)
    {
        //TODO
        rgbBitmap = payload[RGB_DATA_OFFSET];
    }

    return ok;
}

bool Sensor::setTestVal(uint8_t testVal)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_IB_TEST_8BIT, testVal));
}

bool Sensor::setTestVal(uint16_t testVal)
{
    return std::get<bool>(pimpl->send_receive(COMMAND_SET_IB_TEST_16BIT, testVal));
}

bool Sensor::getTestVal(uint8_t& testVal)
{
    auto result = pimpl->send_receive(COMMAND_GET_IB_TEST_8BIT);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == IB_TEST_8BIT_SIZE);
    if (ok)
    {
        BE_Get(testVal, &payload[IB_TEST_8BIT_OFFSET]);
    }

    return ok;
}

bool Sensor::getTestVal(uint16_t& testVal)
{
    auto result = pimpl->send_receive(COMMAND_GET_IB_TEST_16BIT);

    bool ok = std::get<bool>(result);
    const auto& payload = std::get<1>(result);
    ok &= (payload.size() == IB_TEST_16BIT_SIZE);
    if (ok)
    {
        BE_Get(testVal, &payload[IB_TEST_16BIT_OFFSET]);
    }

    return ok;
}

bool Sensor::getSensorInfo(TofComm::versionData_t &versionData){

    auto result = pimpl->send_receive(COMMAND_READ_SENSOR_INFO);

    bool ok = std::get<bool>(result);

    const auto& payload = std::get<1>(result);

    ok &= (payload.size() == sizeof(versionData));

    if (ok)
    {
        memcpy(&versionData, &payload[0], payload.size());
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

    auto f = [&](const std::vector<uint8_t>& p)
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

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command)
{
    return this->connection.send_receive(command, nullptr, 0, 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, uint8_t value)
{
    return this->connection.send_receive(command, &value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, int8_t value)
{
    return this->connection.send_receive(command, (uint8_t*)&value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, uint16_t value)
{
    boost::endian::native_to_big_inplace(value);
    return this->connection.send_receive(command, (uint8_t*)&value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, int16_t value)
{
    boost::endian::native_to_big_inplace(value);
    return this->connection.send_receive(command, (uint8_t*)&value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, uint32_t value)
{
    boost::endian::native_to_big_inplace(value);
    return this->connection.send_receive(command, (uint8_t*)&value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::send_receive(uint16_t command, int32_t value)
{
    boost::endian::native_to_big_inplace(value);
    return this->connection.send_receive(command, (uint8_t*)&value, sizeof(value), 5s);
}

Sensor::Impl::send_receive_result_t Sensor::Impl::streamMeasurement(uint16_t cmd)
{
    uint8_t enable_streaming = 1; //1 means streaming measurement data as fast as possible
    return this->send_receive(cmd, enable_streaming); 
}

} //end namespace tofcore
