#ifndef COMMANDTYPES_HPP
#define COMMANDTYPES_HPP
/**
 * @file CommandTypes.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <array>
#include <cstdint>
#include <functional>
#include <string>

#ifdef __GNUC__
#define PACK_START
#define PACK_END __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK_START __pragma( pack(push, 1) )
#define PACK_END __pragma( pack(pop))
//#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

namespace tofcore
{

typedef std::function<void (const std::string& msg, uint32_t level)> log_callback_t;

inline constexpr uint32_t LOG_LVL_ERROR     { 0 };
inline constexpr uint32_t LOG_LVL_INFO      { 1 };
inline constexpr uint32_t LOG_LVL_DBG_HI    { 2 };
inline constexpr uint32_t LOG_LVL_DBG_MID   { 3 };
inline constexpr uint32_t LOG_LVL_DBG_LOW   { 4 };

} // namespace tofcore


namespace TofComm
{

inline constexpr uint32_t IMAGER_WIDTH  { 320 };
inline constexpr uint32_t IMAGER_HEIGHT { 240 };

/**
 * VSM (Vector Sequence Mode) mode allows a sequence of integration times and
 * modulation frequencies to be specified
 */
PACK_START struct VsmElement_T
{
    uint16_t m_integrationTimeUs { 0 };
    uint16_t m_modulationFreqKhz { 0 };
} PACK_END;
/**
 * The sequence can have a length from 0 to this size.
 */
inline constexpr uint32_t VSM_MAX_NUMBER_OF_ELEMENTS { 16 };
/**
 * VSM mode is controlled by defining between 0 and VSM_MAX_NUMBER_OF_ELEMENTS.
 * Each has an integration time & modulation frequency. When that element becomes
 * "active" (the one currently selected), then those settings are applied. Note
 * that a value of zero can be used to leave the setting unchanged at its current
 * value. If the number of elements is zero, then VSM mode is "disabled" and the
 * non-VSM settings for the integration time and modulation frequency are used;
 * otherwise, the elements are sequenced through for consecutive acquistions.
 * When the end of the sequence when the end is reached, it starts over. This
 * continues until new VSM settings are commanded.
 */
PACK_START struct VsmControl_T
{
    uint16_t m_vsmFlags { 0 };          // potential future expansion: spatial HDR via VSM
    uint8_t m_numberOfElements { 0 };   // # of elements within m_elements
    uint8_t m_vsmIndex { 0 };           // index of currently active element within m_elements
    VsmElement_T m_elements[VSM_MAX_NUMBER_OF_ELEMENTS] { };
} PACK_END;

void vsmEndianConversion(VsmControl_T& vsmControl);

typedef uint8_t(MAC_T)[6];

inline constexpr unsigned SERIAL_NUMBER_SIZE        { 256U };
inline constexpr unsigned SOFTWARE_SOURCE_ID_SIZE   { 32U };
inline constexpr unsigned TEST_STATION_DATA_SIZE    { 2040U };

#define DEFAULT_MAC { 0x00, 0x1A, 0xF1, 0x99, 0x99, 0x99 } ///< Default MAC for Ethernet connection

PACK_START struct ManufacturingData_T
{
    MAC_T m_MAC DEFAULT_MAC;
    uint8_t m_dummy[2] { 0, 0 };
    char m_deviceSerialNumber[SERIAL_NUMBER_SIZE] {};   ///<< Product's device serial number (string)
    char m_cpuBoardSerialNumber[SERIAL_NUMBER_SIZE] {}; ///<< Product's CPU board serial number (string)
    char m_modelName[SERIAL_NUMBER_SIZE] {};            ///<< Product model number (string)
    char m_testStationData[TEST_STATION_DATA_SIZE] {};  ///<< Storage for test station data
    uint8_t m_pad[756] { 0 };                           ///<< pad for future expansion
    uint8_t m_ipv4Address[4] { 10, 10, 31, 180 };       ///<< IPV4 network address
    uint8_t m_ipv4Mask[4] { 255, 255, 255, 0 };         ///<< IPV4 network mask
    uint8_t m_ipv4Gateway[4] { 10, 10, 31, 1 };         ///<< IPV4 network gateway
} PACK_END;

// Mojave Backpack Modules
enum backpackModule_t : uint8_t
{   /*
     * NOTES: * DO NOT change size or order of elements. Add any new elements to
     *            the end.
     *        * There must be an identical type on the sensor side for
     *            this information.
     */
    NO_MODULE = 0,
    EVT_BREAKOUT_BOARD,
    BACKPACK_SAHARA,
    BACKPACK_BORREGO,
    MODULE_UNKOWN
};

PACK_START struct versionData_t
{   /*
     * NOTES: * DO NOT change size or order of elements. Add any new elements to
     *            the end.
     *        * There must be an identical type on the sensor side for
     *            this information.
     */
    char m_deviceSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_cpuBoardSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_illuminatorBoardSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_modelName[SERIAL_NUMBER_SIZE] { };
    char m_lastResetType[SOFTWARE_SOURCE_ID_SIZE] { };
    char m_softwareSourceID[SOFTWARE_SOURCE_ID_SIZE] { };
    char m_softwareVersion[SOFTWARE_SOURCE_ID_SIZE] { };
    uint8_t m_cpuBoardRevision { };
    uint32_t m_sensorChipId { };
    char m_illuminatorSwSourceId[SOFTWARE_SOURCE_ID_SIZE] { };
    char m_illuminatorSwVersion[SOFTWARE_SOURCE_ID_SIZE] { };
    uint8_t m_illuminatorHwCfg { };
    backpackModule_t m_backpackModule { };
} PACK_END;

PACK_START struct Sensor_Status_t
{
    int16_t  lastTemperature;
    int16_t  USB_Current;
    uint32_t BIT_Status;
} PACK_END;

enum class HdrMode_e
{
    UNKOWN, TEMPORAL, SPATIAL
};

struct HdrSettings_T
{
    bool enabled {false};
    HdrMode_e mode {HdrMode_e::UNKOWN};
};

struct ImuScaledData_T
{
    std::array<int16_t, 3> accelerometer_millig { 0, 0, 0 };
    std::array<int32_t, 3> gyro_milliDegreesPerSecond { 0, 0, 0 }; 
    int32_t temperature_milliDegreesC { 0 };
    uint32_t timestamp { 0 };
};

} //end namespace TofComm

#endif
