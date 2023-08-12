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
#include "device_discovery.hpp"
#include "span.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <tuple>
#include <optional>

namespace tofcore
{

constexpr uint32_t      DEFAULT_BAUD_RATE           { 115200 };
constexpr const char*   DEFAULT_PORT_NAME           { "" }; 
constexpr uint16_t      DEFAULT_PROTOCOL_VERSION    { 1 };

struct LensIntrinsics_t
{
    double m_rowOffset { 0.0 };
    double m_columnOffset { 0.0 };
    double m_rowFocalLength { 0.0 };
    double m_columnFocalLength { 0.0 };
    std::array<double, 5> m_undistortionCoeffs { 0.0, 0.0, 0.0, 0.0, 0.0 };
};

class Sensor
{
public:
    Sensor(const std::string& uri = std::string());
    Sensor(uint16_t protocolVersion = DEFAULT_PROTOCOL_VERSION,
           const std::string &portName = DEFAULT_PORT_NAME,
           uint32_t baudrate = DEFAULT_BAUD_RATE);

    typedef std::function<void (std::shared_ptr<Measurement_T>)> on_measurement_ready_t;

    ~Sensor();

    std::optional<std::vector<uint16_t>> getIntegrationTimes();
    bool getLensInfo(std::vector<double> &rays_x, std::vector<double> &rays_y, std::vector<double> &rays_z);
    std::optional<LensIntrinsics_t> getLensIntrinsics();
    bool getIPv4Settings(std::array<std::byte, 4>& adrs, std::array<std::byte, 4>& mask, std::array<std::byte, 4>& gateway);
    bool getSensorInfo(TofComm::versionData_t &versionData);
    std::optional<std::string> getSensorLocation();
    std::optional<std::string> getSensorName();
    bool getSensorStatus(TofComm::Sensor_Status_t &sensorStatus);
    bool getSettings(std::string& jsonSettings);

    std::optional<bool> isFlipHorizontallyActive();
    std::optional<bool> isFlipVerticallyActive();

    void jumpToBootloader();
    void jumpToBootloader(uint16_t token);

    bool setBinning(const bool vertical, const bool horizontal);
    bool setFlipHorizontally(bool flip);
    bool setFlipVertically(bool flip);
    bool setHDRMode(uint8_t mode);
    bool setIntegrationTime(uint16_t);
    bool setIntegrationTimes(uint16_t, uint16_t, uint16_t);
    bool setIPv4Settings(const std::array<std::byte, 4>& adrs, const std::array<std::byte, 4>& mask, const std::array<std::byte, 4>& gateway);
    bool setMinAmplitude(uint16_t minAmplitude);
    bool setModulation(const uint8_t index, const uint8_t channel);
    bool setOffset(int16_t offset);
    bool setSensorLocation(std::string location);
    bool setSensorName(std::string name);

    bool stopStream();
    bool storeSettings();

    bool streamDCS();
    bool streamDistance();
    bool streamDistanceAmplitude();
    bool streamDCSAmbient();

    void subscribeMeasurement(on_measurement_ready_t);

protected:
    typedef std::optional<std::vector<std::byte>> send_receive_result_t;
    typedef tcb::span<std::byte> send_receive_payload_t;

    uint16_t getProtocolVersion() const;
    bool setProtocolVersion(uint16_t version);
    send_receive_result_t send_receive(const uint16_t command, const std::vector<send_receive_payload_t>& payload, 
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;
    send_receive_result_t send_receive(const uint16_t command, const send_receive_payload_t& payload,
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;
    send_receive_result_t send_receive(const uint16_t command) const;
    send_receive_result_t send_receive(const uint16_t command, uint32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint8_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int8_t value) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace tofcore

#endif
