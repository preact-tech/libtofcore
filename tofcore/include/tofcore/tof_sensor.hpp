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
#if __cplusplus >= 202002L
#   include <span>
#else
#   define TCB_SPAN_NAMESPACE_NAME std
#   include "span.hpp"
#endif
#include "CommandTypes.hpp"
#include "connection.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <tuple>
#include <optional>

namespace tofcore
{

constexpr const char*   DEFAULT_URI                 { "" }; 
constexpr uint32_t      DEFAULT_BAUD_RATE           { 115200 };
constexpr const char*   DEFAULT_PORT_NAME           { "" };

constexpr uint16_t      DEFAULT_MOD_FREQ_STEP_KHZ   { 10 };

constexpr int IMU_SEND_RECEIVE_FAILED {-20};

struct LensIntrinsics_t
{
    double m_rowOffset { 0.0 };
    double m_columnOffset { 0.0 };
    double m_rowFocalLength { 0.0 };
    double m_columnFocalLength { 0.0 };
    std::array<double, 5> m_undistortionCoeffs { 0.0, 0.0, 0.0, 0.0, 0.0 };
    double m_hfov { 0.0 };
    double m_vfov { 0.0 };
};

enum class SensorControlStatus : uint8_t
{
    IDLE,               ///< No ToF activity
    CAPTURE,            ///< Capturing data internally
    SEND,               ///< Sending a single result
    STREAM,             ///< Continually streaming data
    OVERTEMPERATURE,    ///< Disabled due to temperature
    ERROR,              ///< ToF unavailable due to an internal error
};

class Sensor
{
public:

    /// @brief Connect to a ToF sensor using the supplied uri string to locate the device and configure
    ///  the connection.
    /// @param uri The following uri schemes/formats are supported
    ///   - If uri is empty a scan for connected devices is done and a connection is made to the first device found.
    ///   - tofserial: Specifies a serial type connection (i.e. UART, virtual USB COM, or emulated pts device).
    ///                The connection baudrate can be specified as an argument.
    ///                Examples: tofserial:/dev/ttyACM0?baudrate=19200  (Linux only)
    ///                          tofserial:COM1    (Windows only)
    ///   - tofnet: Specifies a IP network type connection (i.e. Ethernet address or hostname and optional port).
    ///                Examples:  tofnet://10.10.31.180:50660
    ///                           tofnet://localhost  (e.g. to connect to emulator)
    ///
    ///   If the uri scheme is not provided then some attempt is made to deduce the scheme based on
    ///   uri content.
    Sensor(const std::string& uri);

    /// @brief Old style constructor for typically used for connecting over serial type devices. 
    ///   (Note: this constructor will accept any URI.)
    Sensor(const std::string &portName, uint32_t baudrate);

    Sensor(std::unique_ptr<Connection_T> connection);

    typedef std::function<void (std::shared_ptr<Measurement_T>)> on_measurement_ready_t;

    virtual ~Sensor();

    uint32_t getDebugLevel() const;
    void setDebugLevel(uint32_t level);

    std::optional<uint16_t> getCalVledMv();
    std::optional<uint8_t> getFrameCrcState();
    std::optional<uint32_t> getFramePeriodMs();
	std::optional<std::tuple<uint32_t, uint32_t, uint32_t>> getFramePeriodMsAndLimits();
    std::optional<TofComm::ImuScaledData_T> getImuInfo();
    std::optional<uint16_t> getIntegrationTime();
    std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> getIntegrationTimeUsAndLimits();
    std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> getIPMeasurementEndpoint();
    bool getIPv4Settings(std::array<std::byte, 4>& adrs, std::array<std::byte, 4>& mask, std::array<std::byte, 4>& gateway);
    bool getLensInfo(std::vector<double> &rays_x, std::vector<double> &rays_y, std::vector<double> &rays_z);
    std::optional<LensIntrinsics_t> getLensIntrinsics();
    std::optional<std::tuple<std::array<std::byte, 4>, uint16_t>> getLogIpv4Destination();
    std::optional<uint16_t> getMinAmplitude();
    std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> getMinAmplitudeAndLimits();
    std::optional<std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>> getModulationFreqKhzAndLimitsAndStepSize();
    std::optional<uint16_t> getModulation();
    bool getSensorInfo(TofComm::versionData_t &versionData);
    std::optional<std::string> getSensorLocation();
    std::optional<std::string> getSensorName();
    bool getSensorStatus(TofComm::Sensor_Status_t &sensorStatus);
    bool getSettings(std::string& jsonSettings);
    std::optional<std::tuple<uint16_t, uint16_t, uint16_t>> getVledSettingAndLimits();
    std::optional<TofComm::VsmControl_T> getVsmSettings();
    std::optional<uint32_t> getVsmMaxNumberOfElements();

    std::optional<SensorControlStatus> getSensorControlState();

    /* Retrieves the supported IMU accelerometr range in the units of G's. 
       The first element of the returned tuple is the communication status code,
       0 being successful and if not, a non-zero error code.
       The second element is the list of supported ranges in G's. */
    std::tuple<int8_t, std::list<uint8_t>> imuAccelerometerAvailableRangesInGs();

    /* Retrieves the IMU accelerometr range in the units of G's. 
       The first element of the returned tuple is the communication status code,
       0 being successful and if not, a non-zero error code.
       and the second element if the range in G's. */
    std::tuple<int8_t, uint8_t> imuAccelerometerRangeInGs();

    /* Sets the IMU accelerometr range in the units of G's. 
       Returns 0 if successful and if not, a non-zero error code.  */
    int8_t imuAccelerometerRangeInGs(uint8_t rangeInGs);

    /* Initiates the IMU accelerometr self-test. 
       Returns 0 if successful and if not, a non-zero error code.  */
    int8_t imuAccelerometerSelfTest();

    /* Initiates the IMU gyro self-test.  NOTE:  At this time, this routine
       does nothing, self-test was not running correctly. 
       Returns 0 if successful and if not, a non-zero error code.  */
    int8_t imuGyroSelfTest();

    std::optional<bool> isFlipHorizontallyActive();
    std::optional<bool> isFlipVerticallyActive();
    std::optional<bool> isRawDataSorted();

    void jumpToBootloader();
    void jumpToBootloader(uint16_t token);

    bool setBinning(const bool vertical, const bool horizontal);
    bool setBinning(const bool binning);
    std::optional<uint8_t> getBinning();
    bool setCalVledMv(uint16_t vledMv);
    bool setFlipHorizontally(bool flip);
    bool setFlipVertically(bool flip);
    bool setFrameCrcState(uint8_t state);
    bool setFramePeriodMs(uint32_t periodMs);
    bool setHdr(bool enable, bool useSpatial=false);
    std::optional<TofComm::HdrSettings_T> getHdrSettings();
    bool setIntegrationTime(uint16_t);
    bool setIntegrationTimes(uint16_t, uint16_t, uint16_t);
    bool setIPMeasurementEndpoint(std::array<std::byte,4> address, uint16_t port);
    bool setIPv4Settings(const std::array<std::byte, 4>& adrs, const std::array<std::byte, 4>& mask, const std::array<std::byte, 4>& gateway);
    bool setLogIPv4Destination(const std::array<std::byte, 4>& adrs, const uint16_t port);
    bool setMinAmplitude(uint16_t minAmplitude);
    bool setModulation(uint16_t modFreqkHz);
    bool setOffset(int16_t offset);
    bool setSensorLocation(std::string location);
    bool setSensorName(std::string name);
    bool setVsm(const TofComm::VsmControl_T& vsmControl);
    bool sortRawData(const bool sortIt);

    bool stopStream();
    bool storeSettings();

    bool streamDCS();
    bool streamDCSDiffAmbient();
    bool streamDistance();
    bool streamDistanceAmplitude();
    bool streamDCSAmbient();

    void subscribeMeasurement(on_measurement_ready_t);

    typedef std::optional<std::vector<std::byte>> send_receive_result_t;
    typedef std::span<std::byte> send_receive_payload_t;

    send_receive_result_t send_receive(const uint16_t command, const send_receive_payload_t& payload,
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;

protected:

    virtual std::tuple<bool, std::string> getCmdName(const uint16_t cmdId, const bool verbose) const;

    void log_cmd_reply(const uint16_t command, const std::optional<std::vector<std::byte>> reply) const;
    void log_cmd_send(const uint16_t command, const send_receive_payload_t& payload) const;

    send_receive_result_t send_receive(const uint16_t command, const std::vector<send_receive_payload_t>& payload, 
                                       std::chrono::steady_clock::duration timeout = std::chrono::seconds(5)) const;
    send_receive_result_t send_receive(const uint16_t command) const;
    send_receive_result_t send_receive(const uint16_t command, uint32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int32_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int16_t value) const;
    send_receive_result_t send_receive(const uint16_t command, uint8_t value) const;
    send_receive_result_t send_receive(const uint16_t command, int8_t value) const;

    log_callback_t m_log_callback { nullptr };

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} //end namespace tofcore

#endif
