#ifndef _METADATATYPES_HPP
#define _METADATATYPES_HPP
/**
 * @file CommandTypes.hpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Declares constants and types associated with the TOF communication interface.
 */
#include <cstddef>
#include <cstdint>

namespace TofComm
{
/**
 * KLV meta-data keys which are included as a block in the Measurement header
 */
constexpr uint16_t KLV_SENSOR_TEMPERATURE_KEY {1};
constexpr uint16_t KLV_DLL_SETTINGS_KEY {2};
constexpr uint16_t KLV_ILLUMINATOR_INFO_KEY {3};
constexpr uint16_t KLV_MODULATION_FREQUENCY_KEY {4};
constexpr uint16_t KLV_INTEGRATION_TIME_KEY {5};
constexpr uint16_t KLV_BINNING_KEY {6};
constexpr uint16_t KLV_VSM_KEY {7};
constexpr uint16_t KLV_FRAME_TIMESTAMP_KEY {8};
constexpr uint16_t KLV_FRAME_CRC_KEY {9};


constexpr size_t KLV_NUM_DLL_BYTES { 4 };
constexpr size_t KLV_NUM_TEMPERATURES { 4 };

/// @brief Decoded illuminator info data included with the ILLUMINATOR_INFO KLV key
struct illuminator_info_t
{
    uint8_t led_segments_enabled {0}; ///bit field representing which possible segments were enabled
    float temperature_c {0.0};
    float vled_v {0.0};
    float photodiode_v {0.0};
};

}

#endif //_METADATATYPES_HPP
