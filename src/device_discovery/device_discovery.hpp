/**
 * @file device_discovery.h
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for ToF Sensor Device Discovery
 * @{
 */
#ifndef _DEVICE_DISCOVERY_H_
#define _DEVICE_DISCOVERY_H_

#include "span.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

namespace tofcore
{
    constexpr uint16_t PREACT_VENDOR_ID = 0x35FA;

    // MOJAVE DEFS
    constexpr uint16_t MOJAVE_PRODUCT_ID = 0x0D0F;

    // T10C DEFS
    constexpr uint16_t T10C_PRODUCT_ID = 0x0A03U;

    // USB Channel Info
    constexpr uint8_t USB_INTERFACE_COMM_CH = 0;
    constexpr uint8_t USB_INTERFACE_DEBUG_CH = 2;

    constexpr char FILE_PREFIX[] = "file:///";
    constexpr char SERIAL_PREFIX[] = "serial:///";
    constexpr char HTTP_PREFIX[] = "http://";
    constexpr char VIDEO_PREFIX[] = "video:///";

    struct device_info_t
    {
        std::string connector_uri;  //examples /dev/ttyACM1, COM2, (file:///dev/ttyAM1 serial://COM1 
                                    // serial:///dev/ttyACM1, serial:///devttyUSB10?baud=5600 
                                    // in the future it might be network http://192.12.23.0:6000
                                    // video:///video1)
        std::string serial_num;
        std::string model;
    };


    std::vector<device_info_t> find_all_devices();

    device_info_t get_device_info(const std::string defaultPort);

} //end namespace

#endif // _DEVICE_DISCOVERY_H_

/** @} */
