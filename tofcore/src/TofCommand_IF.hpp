#ifndef TOFCOMMAND_IF_HPP
#define TOFCOMMAND_IF_HPP
/**
 * @file TofCommand_IF.hpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <cstdint>
#include <string>
#include <tuple>

#if !defined(PACKED)
#    define PACKED __attribute__((__packed__))
#endif

namespace TofComm
{

/*
 * XMacro to define list of all TofCore commands and their values
 */
#define TOF_CORE_CMDS                                                   \
   TOF_CORE_CMD(COMMAND_SET_INT_TIMES                           , 0x01) \
   TOF_CORE_CMD(COMMAND_GET_DIST_AND_AMP                        , 0x02) \
   TOF_CORE_CMD(COMMAND_GET_DISTANCE                            , 0x03) \
   TOF_CORE_CMD(COMMAND_GET_DCS_DIFF_AMBIENT                    , 0x04) \
   TOF_CORE_CMD(COMMAND_STOP_STREAM                             , 0x06) \
   TOF_CORE_CMD(COMMAND_GET_DCS                                 , 0x07) \
   TOF_CORE_CMD(COMMAND_GET_INT_TIMES                           , 0x08) \
   TOF_CORE_CMD(COMMAND_GET_STREAMING_STATE                     , 0x09) \
   TOF_CORE_CMD(COMMAND_SET_RAW_DATA_SORT                       , 0x0A) \
   TOF_CORE_CMD(COMMAND_GET_DCS_AMBIENT                         , 0x0B) \
   TOF_CORE_CMD(COMMAND_GET_RAW_DATA_SORT_STATE                 , 0x0C) \
   TOF_CORE_CMD(COMMAND_GET_FRAME_CRC_STATE                     , 0x0D) \
   TOF_CORE_CMD(COMMAND_SET_FRAME_CRC_STATE                     , 0x0E) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_FRAME_PERIOD_MS                     , 0x10) \
   TOF_CORE_CMD(COMMAND_GET_FRAME_PERIOD_MS                     , 0x11) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_OFFSET                              , 0x14) \
   TOF_CORE_CMD(COMMAND_SET_MIN_AMPLITUDE                       , 0x15) \
   TOF_CORE_CMD(COMMAND_GET_MIN_AMPLITUDE                       , 0x16) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_MODULATION                          , 0x17) \
   TOF_CORE_CMD(COMMAND_SET_BINNING                             , 0x18) \
   TOF_CORE_CMD(COMMAND_SET_HDR                                 , 0x19) \
   TOF_CORE_CMD(COMMAND_GET_HDR                                 , 0x1A) \
   TOF_CORE_CMD(COMMAND_SET_VSM                                 , 0x1B) \
   TOF_CORE_CMD(COMMAND_GET_VSM                                 , 0x1C) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_CAL_VLED_MV                         , 0x1E) \
   TOF_CORE_CMD(COMMAND_GET_CAL_VLED_MV                         , 0x1F) \
   TOF_CORE_CMD(COMMAND_GET_BINNING                             , 0x20) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_DATA_IP_ADDRESS                     , 0x25) \
   TOF_CORE_CMD(COMMAND_SET_DATA_IP_ADDRESS                     , 0x26) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_CAMERA_IP_SETTINGS                  , 0x28) \
   TOF_CORE_CMD(COMMAND_GET_CAMERA_IP_SETTINGS                  , 0x29) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_MODULATION                          , 0x2F) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_HORIZ_FLIP_STATE                    , 0x30) \
   TOF_CORE_CMD(COMMAND_GET_VERT_FLIP_STATE                     , 0x31) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_HORIZ_FLIP_STATE                    , 0x33) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_VERT_FLIP_STATE                     , 0x36) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_LENS_INFO                           , 0x50) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SETUP_UDP_LOG                           , 0x6C) \
   TOF_CORE_CMD(COMMAND_GET_UDP_LOG_SETUP                       , 0x6D) \
                                                                        \
   TOF_CORE_CMD(COMMAND_JUMP_TO_BOOLOADER                       , 0x6F) \
                                                                        \
   TOF_CORE_CMD(COMMAND_STORE_SETTINGS                          , 0x71) \
   TOF_CORE_CMD(COMMAND_READ_SETTINGS                           , 0x72) \
                                                                        \
   TOF_CORE_CMD(COMMAND_SET_SENSOR_NAME                         , 0x78) \
   TOF_CORE_CMD(COMMAND_GET_SENSOR_NAME                         , 0x79) \
   TOF_CORE_CMD(COMMAND_SET_SENSOR_LOCATION                     , 0x7A) \
   TOF_CORE_CMD(COMMAND_GET_SENSOR_LOCATION                     , 0x7B) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_INTEG_TIME_LIMITS                   , 0xB0) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_MOD_FREQ_LIMITS                     , 0xB2) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_FRAME_PERIOD_LIMITS                 , 0xB4) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_AVAILABLE_FEATURES                  , 0xB6) \
   TOF_CORE_CMD(COMMAND_GET_VSM_MAX_NUMBER_ELEMENTS             , 0xB7) \
   TOF_CORE_CMD(COMMAND_GET_MIN_AMPLITUDE_LIMITS                , 0xB8) \
                                                                        \
   TOF_CORE_CMD(COMMAND_GET_VLED_LIMITS                         , 0xBA) \
                                                                        \
   TOF_CORE_CMD(COMMAND_READ_SENSOR_INFO                        , 0xD0) \
   TOF_CORE_CMD(COMMAND_READ_SENSOR_STATUS                      , 0xD1) \
   TOF_CORE_CMD(COMMAND_IMU_ACCELEROMETER_GET_AVAILABLE_RANGES  , 0xD2) \
   TOF_CORE_CMD(COMMAND_IMU_ACCELEROMETER_GET_RANGE             , 0xD3) \
   TOF_CORE_CMD(COMMAND_IMU_ACCELEROMETER_SELF_TEST             , 0xD4) \
   TOF_CORE_CMD(COMMAND_IMU_ACCELEROMETER_SET_RANGE             , 0xD5) \
   TOF_CORE_CMD(COMMAND_IMU_GYRO_SELF_TEST                      , 0xD6) \
   TOF_CORE_CMD(COMMAND_IMU_READ_INFO                           , 0xD7)

/*
 * Create an enumeration that has the command names as tags that are associated
 * with the values for those commands.
 */
#undef TOF_CORE_CMD
#define TOF_CORE_CMD(name,value) name=value,

enum TofCoreCommandValues
{
    TOF_CORE_CMDS
};

/*
 * Constants associated with specific commands
 */

// TOF_CORE_CMD(COMMAND_GET_DCS_AMBIENT          , 0x0B)
    constexpr std::uint8_t SINGLE_MEASUREMENT           = 0;
    constexpr std::uint8_t CONTINUOUS_MEASUREMENT       = 1;

// TOF_CORE_CMD(COMMAND_GET_LENS_INFO            , 0x50)
    constexpr size_t RAW_SENSOR_INFO_DATA_SIZE          = 40;

constexpr uint8_t DATA_TYPE_ID_OFFSET = 0;

/*
 * Offsets into version 0 protocol type 1 (measurement data) header
 */
constexpr uint32_t HEADER_VERSION_INDEX     = 0;        ///<Index of the header version
constexpr uint32_t HEADER_TYPE_INDEX        = 1;        ///<Index of the data type
constexpr uint32_t HEADER_WIDTH_INDEX       = 3;        ///<Index of the width
constexpr uint32_t HEADER_HEIGHT_INDEX      = 5;        ///<Index of the height
constexpr uint32_t HEADER_ORIGIN_X_INDEX    = 7;        ///<Index of the origin X
constexpr uint32_t HEADER_ORIGIN_Y_INDEX    = 9;        ///<Index of the origin y
constexpr uint32_t HEADER_DATA_FLAGS_INDEX  = 11;       ///<Index of flags describing image order
    inline constexpr uint8_t RAW_DATA_IN_NATIVE_ORDER = 0x01;   ///<Rows are in native order, not sorted
    inline constexpr uint8_t HORIZONTAL_FLIP    = 0x02;         ///<Rows flipped (bottom-to-top)
    inline constexpr uint8_t VERTICAL_FLIP      = 0x04;         ///<Columns flipped (right-to-left)
constexpr uint32_t HEADER_TEMPERATURE_INDEX = 21;       ///<Index of the last temperature
constexpr uint32_t HEADER_DATA_OFFSET_INDEX = 23;       ///<Index of the data offset_INDEX = offset where the measurement data starts
constexpr uint32_t HEADER_USER_DATA_INDEX   = 25;       ///<Index of the user data (start of KLV data)

} //end namespace TofComm

namespace tofcore
{
std::tuple<bool, std::string>getTofCoreCmdName(const uint16_t cmdId, const bool verbose = false);
} // namespace tofcore

#endif
