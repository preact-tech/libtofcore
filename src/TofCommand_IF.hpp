#ifndef TOFCOMMAND_IF_HPP
#define TOFCOMMAND_IF_HPP
/**
 * @file TofCommand_IF.hpp
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
constexpr uint16_t COMMAND_SET_ROI                  = 0x00;
constexpr uint16_t COMMAND_SET_INT_TIMES            = 0x01;
constexpr uint16_t COMMAND_GET_DIST_AND_AMP         = 0x02;
constexpr uint16_t COMMAND_GET_DISTANCE             = 0x03;
constexpr uint16_t COMMAND_GET_GRAYSCALE            = 0x05;
constexpr uint16_t COMMAND_STOP_STREAM              = 0x06;
constexpr uint16_t COMMAND_GET_DCS                  = 0x07;
constexpr uint16_t COMMAND_GET_DCS_AMBIENT          = 0x0B;
    constexpr std::uint8_t SINGLE_MEASUREMENT       = 0;
    constexpr std::uint8_t CONTINUOUS_MEASUREMENT   = 1;

constexpr uint16_t COMMAND_SET_OFFSET               = 0x14;
constexpr uint16_t COMMAND_SET_MIN_AMPLITUDE        = 0x15;
constexpr uint16_t COMMAND_SET_FILTER               = 0x16;
constexpr uint16_t COMMAND_SET_MODULATION           = 0x17;
constexpr uint16_t COMMAND_SET_BINNING              = 0x18;
constexpr uint16_t COMMAND_SET_HDR                  = 0x19;
constexpr uint16_t COMMAND_SET_DLL_STEP             = 0x1D;
constexpr uint16_t COMMAND_CALIBRATE                = 0x1E;
constexpr uint16_t COMMAND_CALIBRATE_PRODUCTION     = 0x1F;

constexpr uint16_t COMMAND_READ_CHIP_INFO           = 0x24;
    constexpr uint8_t READ_CHIP_INFO_DATA_TYPE          = 3;
    constexpr size_t  CHIP_WAFER_ID_OFFSET              = 1;
    constexpr size_t  CHIP_CHIP_ID_OFFSET               = CHIP_WAFER_ID_OFFSET + sizeof(uint16_t);
    constexpr size_t  READ_CHIP_INFO_SIZE               = CHIP_CHIP_ID_OFFSET + sizeof(uint16_t);
constexpr uint16_t COMMAND_READ_FIRMWARE_VERSION    = 0x25;
    constexpr uint8_t READ_FIRMWARE_VERSION_DATA_TYPE   = 2;
    constexpr size_t  BUILD_ID_OFFSET                   = 1;
    constexpr size_t  MINOR_VERSION_OFFSET              = BUILD_ID_OFFSET + sizeof(uint16_t);
    constexpr size_t  MAJOR_VERSION_OFFSET              = MINOR_VERSION_OFFSET + sizeof(uint8_t);
    constexpr size_t  READ_FIRMWARE_VERSION_SIZE        = MAJOR_VERSION_OFFSET + sizeof(uint8_t);
constexpr uint16_t COMMAND_GET_HORIZ_FLIP_STATE     = 0x30;
constexpr uint16_t COMMAND_GET_VERT_FLIP_STATE      = 0x31;
constexpr uint16_t COMMAND_SET_HORIZ_FLIP_STATE     = 0x33;
constexpr uint16_t COMMAND_SET_VERT_FLIP_STATE      = 0x36;
constexpr uint16_t COMMAND_READ_ACCELEROMETER       = 0x4B;
    constexpr uint8_t READ_ACCELEROMETER_DATA_TYPE      = 9;
    constexpr size_t  ACCELEROMETER_X_OFFSET            = 1;
    constexpr size_t  ACCELEROMETER_Y_OFFSET            = ACCELEROMETER_X_OFFSET + sizeof(uint16_t);
    constexpr size_t  ACCELEROMETER_Z_OFFSET            = ACCELEROMETER_Y_OFFSET + sizeof(uint16_t);
    constexpr size_t  ACCELEROMETER_G_RANGE_OFFSET      = ACCELEROMETER_Z_OFFSET + sizeof(uint16_t);
    constexpr size_t  READ_ACCELEROMETER_SIZE           = ACCELEROMETER_G_RANGE_OFFSET + sizeof(uint8_t);

constexpr uint16_t COMMAND_WRITE_REGISTER       = 0x2A;
constexpr uint16_t COMMAND_READ_REGISTER        = 0x2B; //0x27;
    constexpr size_t  READ_REGISTER_DATA_OFFSET         = 1;
    constexpr size_t  READ_REGISTER_DATA_SIZE           = 2; // Account for data type in return

constexpr uint16_t COMMAND_SEQU_GET_VERSION     = 0x38;
constexpr uint16_t COMMAND_SEQU_SET_VERSION     = 0x39;
constexpr uint16_t COMMAND_SEQU_VERSION_IS_SUPPORTED = 0x3A;

constexpr uint16_t COMMAND_GET_LENS_INFO        = 0x50;

constexpr uint16_t COMMAND_JUMP_TO_BOOLOADER    = 0x6F;

constexpr uint16_t COMMAND_STORE_SETTINGS       = 0x71;
constexpr uint16_t COMMAND_READ_SETTINGS        = 0x72;

/*
* Illuminator board commands
*/
constexpr uint16_t COMMAND_SET_VLED_ENABLES     = 0x80;
constexpr uint16_t COMMAND_GET_VLED_ENABLES     = 0x81;
    constexpr size_t  VLED_ENABLES_DATA_OFFSET         = 1;
    constexpr size_t  VLED_ENABLES_DATA_SIZE           = 2;
constexpr uint16_t COMMAND_SET_VLED             = 0x82;
constexpr uint16_t COMMAND_GET_VLED             = 0x83;
    constexpr size_t VLED_DATA_OFFSET               = 0;
    constexpr size_t VLED_DATA_SIZE                 = 2;
constexpr uint16_t COMMAND_GET_IB_5V            = 0x84;
    constexpr size_t IB_5V_DATA_OFFSET               = 0;
    constexpr size_t IB_5V_DATA_SIZE                 = 2;
constexpr uint16_t COMMAND_GET_IB_TEMPERATURE      = 0x85;
    constexpr size_t IB_TEMPERATURE_DATA_OFFSET         = 0;
    constexpr size_t IB_TEMPERATURE_DATA_SIZE           = 2;
constexpr uint16_t COMMAND_GET_IB_PD            = 0x86;
    constexpr size_t IB_PD_DATA_OFFSET               = 0;
    constexpr size_t IB_PD_DATA_SIZE                 = 2;
constexpr uint16_t COMMAND_SET_RGB              = 0x88;
    constexpr size_t RGB_COLOR_OFFSET           = 0;
    constexpr size_t RGB_BLINK_OFFSET           = 1;
    constexpr size_t RGB_SET_DATA_SIZE          = 3;
constexpr uint16_t COMMAND_GET_RGB              = 0x89;
    constexpr size_t RGB_DATA_OFFSET                 = 0;
    constexpr size_t RGB_DATA_SIZE                   = 1;
constexpr uint16_t COMMAND_SET_IB_TEST_8BIT      = 0x8A;
constexpr uint16_t COMMAND_SET_IB_TEST_16BIT     = 0x8B;
constexpr uint16_t COMMAND_GET_IB_TEST_8BIT      = 0x8C;
    constexpr size_t IB_TEST_8BIT_OFFSET             = 0;
    constexpr size_t IB_TEST_8BIT_SIZE               = 1;
constexpr uint16_t COMMAND_GET_IB_TEST_16BIT     = 0x8D;
    constexpr size_t IB_TEST_16BIT_OFFSET             = 0;
    constexpr size_t IB_TEST_16BIT_SIZE               = 2;
constexpr uint16_t COMMAND_SET_SERIAL_NUM        = 0x8F;
constexpr uint16_t COMMAND_GET_SERIAL_NUM        = 0x90;
    constexpr size_t SERIAL_NUM_DATA_OFFSET           = 0;
    constexpr size_t SERIAL_NUM_DATA_SIZE             = 4;


constexpr uint16_t COMMAND_STORAGE_GET_METADATA = 0xCC;
constexpr uint16_t COMMAND_STORAGE_WRITE        = 0xCD;
constexpr uint16_t COMMAND_STORAGE_READ         = 0xCE;
constexpr uint16_t COMMAND_FACTORY_MODE         = 0xCF;

constexpr uint16_t COMMAND_READ_SENSOR_INFO     = 0xD0;

constexpr uint8_t DATA_TYPE_ID_OFFSET = 0;

/*
 * Offsets into version 0 protocol type 1 (measurement data) header
 */
constexpr uint32_t V0_T1_VERSION_INDEX = 0;                 ///<Index of the header version
constexpr uint32_t V0_T1_TYPE_INDEX = 1;                    ///<Index of the data type
constexpr uint32_t V0_T1_WIDTH_INDEX = 3;                   ///<Index of the width
constexpr uint32_t V0_T1_HEIGHT_INDEX = 5;                  ///<Index of the height
constexpr uint32_t V0_T1_ORIGIN_X_INDEX = 7;                ///<Index of the origin X
constexpr uint32_t V0_T1_ORIGIN_Y_INDEX = 9;                ///<Index of the origin y
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_0_INDEX = 15;  ///<Index of the integration time 3D 0
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_1_INDEX = 17;  ///<Index of the integration time 3D 1
constexpr uint32_t V0_T1_INTEGRATION_TIME_3D_2_INDEX = 19;  ///<Index of the integration time 3D 2
constexpr uint32_t V0_T1_TEMPERATURE_INDEX = 21;            ///<Index of the last temperature
constexpr uint32_t V0_T1_DATA_OFFSET_INDEX = 23;            ///<Index of the data offset_INDEX = offset where the measurement data starts
constexpr uint32_t V0_T1_USER_DATA_INDEX = 25;              ///<Index of the user data (start of KLV data)

/*
 * Offsets into version 0 protocol type 0 (command) payload for filter parameters
 */
constexpr uint32_t V0_T1_TEMPORAL_FILTER_FACTOR_INDEX = 0;                ///<Factor for the temporal filter
constexpr uint32_t V0_T1_TEMPORAL_FILTER_THRESHOLD_INDEX = 2;             ///<Threshold for the temporal filter
constexpr uint32_t V0_T1_MEDIAN_FILTER_ENABLED_INDEX = 4;                 ///<Flag to enable/disable the median filter
constexpr uint32_t V0_T1_AVERAGE_FILTER_ENABLED_INDEX = 5;                ///<Flag to enable/disable the average filter
constexpr uint32_t V0_T1_EDGE_DETECTION_THRESHOLD_INDEX = 6;              ///<Threshold to setup the edge detection
constexpr uint32_t V0_T1_INTERFERENCE_DETECTION_USE_LAST_VALUE_INDEX = 8; ///<Flag to select if the last valid value should be used in case of interference
constexpr uint32_t V0_T1_INTERFERENCE_DETECTION_LIMIT_INDEX = 9;          ///<Interference detection limit
constexpr uint32_t V0_T1_TEMPORAL_EDGE_FILTER_THRESHOLD_LOW_INDEX = 11;   ///<Lower threshold for temporal edge filter
constexpr uint32_t V0_T1_TEMPORAL_EDGE_FILTER_THRESHOLD_HIGH_INDEX = 13;  ///<Higher threshold for temporal edge filter

} //end namespace TofComm

#endif
