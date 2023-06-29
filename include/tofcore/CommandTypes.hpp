#ifndef COMMANDTYPES_HPP
#define COMMANDTYPES_HPP
/**
 * @file CommandTypes.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <cstdint>

#ifdef __GNUC__
#define PACK_START
#define PACK_END __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK_START __pragma( pack(push, 1) )
#define PACK_END __pragma( pack(pop))
//#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

namespace TofComm
{

typedef uint8_t(MAC_T)[6];

constexpr unsigned SERIAL_NUMBER_SIZE       { 256U };
constexpr unsigned TEST_STATION_DATA_SIZE   { 2040U };
PACK_START struct ManufacturingData_T
{
    MAC_T m_MAC { 0x00, 0x1A, 0xF1, 0x99, 0x99, 0x99 };     ///< Default MAC for Ethernet connection
    uint8_t m_dummy[2] { 0, 0 };
    char m_deviceSerialNumber[SERIAL_NUMBER_SIZE] { 0 };    ///<< Product serial number (string)
    char m_cpuBoardSerialNumber[SERIAL_NUMBER_SIZE] { 0 };  ///<< Product model number (string)
    char m_modelName[SERIAL_NUMBER_SIZE] { 0 };             ///<< Product model number (string)
    char m_testStationData[TEST_STATION_DATA_SIZE] { 0 };   ///<< Storage for test station data
    uint8_t m_pad[768] { 0 };                               ///<< pad for future expansion
} PACK_END;

// Mojave Backpack Modules
enum backpackModule_t : uint8_t
{
    NO_MODULE = 0,
    EVT_BREAKOUT_BOARD = 1

};

PACK_START struct versionData_t
{

    char m_deviceSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_cpuBoardSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_illuminatorBoardSerialNumber[SERIAL_NUMBER_SIZE] { };
    char m_modelName[SERIAL_NUMBER_SIZE] { };
    char m_lastResetType[32] { };
    char m_softwareSourceID[32] { };
    char m_softwareVersion[32] { };
    uint8_t m_cpuVersion { };
    uint32_t m_sensorChipId { };
    char m_illuminatorSwSourceId[32] { };
    char m_illuminatorSwVersion[32] { };
    uint8_t m_illuminatorHwCfg { };
    backpackModule_t m_backpackModule { };

} PACK_END;

} //end namespace TofComm

#endif
