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

#if !defined(PACKED)
#    define PACKED __attribute__((__packed__))
#endif

namespace TofComm
{

enum class StorageId_e: uint8_t
{
    UNDEFINED, CALIBRATON, LENS_DATA, MANUFACTURING_DATA, PERSISTENT_SETTINGS, LOG
};

enum class StorageMode_e: uint8_t
{
    RAW, EXTRACTED
};

enum class WriteOperation_e: uint8_t
{
    START, CONTINUE, FINISH
};

struct StorageMetadata_T
{
    StorageId_e m_storageId { StorageId_e::UNDEFINED };
    StorageMode_e m_mode { StorageMode_e::RAW };
    uint16_t m_dataType { 0 };      ///< Identifies the type of data in the data set
    uint16_t m_typeVersion { 0 };   ///< The version of this data set's type
    uint32_t m_dataSize { 0 };      ///< The number of bytes in the data set (following this header's page)
    uint32_t m_dataCrc32 { 0 };     ///< The CRC32 of the data
} PACKED;

struct StorageReadCommand_T
{
    StorageId_e m_storageId { StorageId_e::UNDEFINED };
    StorageMode_e m_mode { StorageMode_e::RAW };
    uint32_t m_offset { 0 };    ///< Offset into related storage of first byte to read
    uint32_t m_numBytes { 0 };  ///< Number of bytes to read (all of related storage ID if 0)
} PACKED;;

struct WriteStartCommand_T
{
    WriteStartCommand_T(StorageId_e id, StorageMode_e mode) :
                m_storageId(id), m_mode(mode)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::START };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
} PACKED;

struct WriteContinueCommand_T
{
    WriteContinueCommand_T(StorageId_e id, StorageMode_e mode, uint32_t offset) :
                m_storageId(id), m_mode(mode), m_offset(offset)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::CONTINUE };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
    const uint32_t m_offset { 0 };    ///< Offset associated with first byte of data that follows
} PACKED;

struct WriteFinishCommand_T
{
    WriteFinishCommand_T(StorageId_e id, StorageMode_e mode, uint32_t size, uint32_t crc32) :
                m_storageId(id), m_mode(mode), m_size(size), m_crc32(crc32)
    {
    }
    const WriteOperation_e m_operation { WriteOperation_e::FINISH };
    const StorageId_e m_storageId { StorageId_e::UNDEFINED };
    const StorageMode_e m_mode { StorageMode_e::RAW };
    const uint8_t m_dummy[1] { 0 }; // force 4 byte alignment of data that follows
    // DO NOT CHANGE THE ORDER OF THE ABOVE MEMBERS !
    const uint32_t m_size { 0 };  ///< size of entire section's data
    const uint32_t m_crc32 { 0 }; ///< CRC32 of entire section's data
} PACKED;

typedef uint8_t(MAC_T)[6];

struct ManufacturingData_T
{
    MAC_T m_MAC { 0x00, 0x1A, 0xF1, 0x99, 0x99, 0x99 };    ///< Default MAC for Ethernet connection
    uint8_t m_dummy[2] { 0, 0 };
    char m_serialNumber[256] { 0 };     ///<< Product serial number (string)
    char m_modelNumber[256] { 0 };      ///<< Product model number (string)
    char m_modelName[256] { 0 };        ///<< Product model number (string)

} PACKED;

// Mojave Backpack Modules
enum backpackModule_t : uint8_t
{
    NO_MODULE = 0,
    EVT_BREAKOUT_BOARD = 1

};
struct versionData_t
{
    char m_serialNumber[256] { };
    char m_modelNumber[256] { };
    char m_modelName[256] { };
    char m_softwareSourceID[32] { };
    char m_softwareVersion[32] { };
    uint8_t m_cpuVersion { };
    char m_illuminatorSwSourceId[32] { };
    char m_illuminatorSwVersion[32] { };
    uint8_t m_illuminatorHwCfg { };
    backpackModule_t m_backpackModule { };

} PACKED;

} //end namespace TofComm

#endif
