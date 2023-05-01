#ifndef __TOFCORE_T10_SENSOR_H__
#define __TOFCORE_T10_SENSOR_H__
/**
 * @file tof_sensor.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for libtofcore control
 */
#include "CommandTypes.hpp"
#include "Measurement_T.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <tuple>

namespace tofcore
{

constexpr uint32_t      DEFAULT_BAUD_RATE           { 115200 };
constexpr const char*   DEFAULT_PORT_NAME           { "/dev/ttyACM0" };
constexpr uint16_t      DEFAULT_PROTOCOL_VERSION    { 1 };

class Sensor
{
public:

    Sensor(uint16_t protocolVersion = DEFAULT_PROTOCOL_VERSION,
           const std::string &portName = DEFAULT_PORT_NAME,
           uint32_t baudrate = DEFAULT_BAUD_RATE);

    typedef std::function<void (std::shared_ptr<Measurement_T>)> on_measurement_ready_t;

    ~Sensor();

    bool getAccelerometerData(int16_t &x, int16_t &y, int16_t &z, uint8_t &g_range);
    bool getChipInformation(uint16_t &waferId, uint16_t &chipId);
    bool getLensInfo(std::vector<double> &rays_x, std::vector<double> &rays_y, std::vector<double> &rays_z);
    bool getSensorInfo(TofComm::versionData_t &versionData);
    bool getSettings(std::string& jsonSettings);
    bool getSoftwareVersion(std::string& version);
    std::tuple<bool, TofComm::StorageMetadata_T> getStorageMetadata(TofComm::StorageId_e id, TofComm::StorageMode_e mode);

    void jumpToBootloader();

    std::tuple<bool, uint16_t> sequencerGetVersion() const;
    std::tuple<bool, bool> sequencerIsVersionSupported(uint16_t version) const;
    bool sequencerSetVersion(uint16_t version);

    bool setBinning(const bool vertical, const bool horizontal);
    bool setFactoryMode(bool enable);
    bool setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor,
                   const uint16_t temporalThreshold, const uint16_t edgeThreshold, const uint16_t temporalEdgeThresholdLow,
                   const uint16_t temporalEdgeThresholdHigh, const uint16_t interferenceDetectionLimit,
                   const bool interferenceDetectionUseLastValue);
    bool setHDRMode(uint8_t mode);
    bool setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t);
    bool setMinAmplitude(uint16_t minAmplitude);
    bool setModulation(const uint8_t index, const uint8_t channel);
    bool setOffset(int16_t offset);
    bool setProtocolVersion(uint16_t version);
    bool setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1);

    bool startDrnuCalibration();
    bool startProductionCalibration();

    bool stopStream();
    bool storeSettings();

    std::tuple<bool, std::vector<uint8_t>> storageRead(TofComm::StorageId_e id, TofComm::StorageMode_e mode,
                                                       uint32_t storageOffset, uint32_t numBytes);

    bool storageWriteData(TofComm::StorageId_e id, TofComm::StorageMode_e mode,
                          uint32_t storageOffset, const uint8_t *data, uint32_t numBytes);

    bool storageWriteFinish(TofComm::StorageId_e id, TofComm::StorageMode_e mode, uint32_t totalSize, uint32_t crc32);

    bool storageWriteStart(TofComm::StorageId_e id, TofComm::StorageMode_e mode);

    bool streamDCS();
    bool streamGrayscale();
    bool streamDistance();
    bool streamDistanceAmplitude();
    bool streamDCSAmbient();

    void subscribeMeasurement(on_measurement_ready_t);

    bool setDllStep(bool enable, uint8_t coarseStep, uint8_t fineStep, uint8_t finestStep);
    bool readRegister(uint8_t regAddress, uint8_t &regData);
    bool writeRegister(uint8_t regAddress, uint8_t regData);

    //Illuminator board commands
    bool setVledEnables(uint8_t vledEnables);
    bool getVledEnables(uint8_t& vledEnables);
    bool setVled(uint16_t vledMv);
    bool getVled(uint16_t& vledMv);
    bool getIb5V(uint16_t& v5Mv);
    bool getIllmnTemperature(int16_t& tempMdegC);
    bool getIbPd(uint16_t& photodiodeMv);
    bool getIbSerial(uint32_t& serialNum);
    bool setIbSerial(uint32_t serialNum);
    bool setIbRgb(uint8_t rgbBitmap, uint16_t rgbBlinkMs);
    bool getIbRgb(uint8_t& rgbBitmap);
    bool setTestVal(uint8_t testVal);
    bool setTestVal(uint16_t testVal);
    bool getTestVal(uint8_t& testVal);
    bool getTestVal(uint16_t& testVal);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace tofcore

#endif
