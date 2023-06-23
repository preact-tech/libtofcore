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

#include <vector>
#include <string>

namespace tofcore
{
    /// @brief PreAct Tof Device Info
    struct device_info_t
    {
        std::string connector_uri;  //examples /dev/ttyACM1, COM2, (file:///dev/ttyAM1 serial://COM1 
                                    // serial:///dev/ttyACM1, serial:///devttyUSB10?baud=5600 
                                    // in the future it might be network http://192.12.23.0:6000
                                    // video:///video1)
        std::string serial_num;     // Device serial number string of alpha-numeric characters
        std::string model;          // Device model name/number string. ex: Mojave
    };

    /// @brief Find all PreAct ToF Devices attached to machine
    /// @return std::vector<device_info_t>, a list of connected devices, their connection uri, serial number, and model name/number 
    std::vector<device_info_t> find_all_devices();

} //end namespace

#endif // _DEVICE_DISCOVERY_H_

/** @} */
