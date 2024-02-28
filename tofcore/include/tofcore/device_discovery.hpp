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

#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <vector>

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
        std::string sensor_name;    // Device name string. ex: Mojave
        std::string location;       // Device location string.
    };

    /// @brief Find all PreAct ToF Devices attached to machine
    /// @param wait_time Sets the wait time, for Ethernet devices to respond
    /// @param max_count stop searching early if found devices matches max_count
    /// @return std::vector<device_info_t>, a list of connected devices, their connection uri, serial number, and model name/number 
    std::vector<device_info_t> find_all_devices(
        std::chrono::steady_clock::duration wait_time = std::chrono::seconds(2), std::size_t max_count = std::numeric_limits<std::size_t>::max());

    /// @brief Find all PreAct ToF Devices attached by USB to the machine
    /// @param max_count stop searching early if found devices matches max_count
    /// @return std::vector<device_info_t>, a list of connected devices, their connection uri, serial number, and model name/number 
    std::vector<device_info_t> find_usb_cdc_devices(std::size_t max_count = std::numeric_limits<std::size_t>::max());

    /// @brief Find all PreAct ToF Devices attached by Ethernet to the machine
    /// @param wait_time The amount of time, to wait for devices to respond
    /// @param max_count Stop searching early if the count of devices matches this.
    /// @return std::vector<device_info_t>, a list of connected devices, their connection uri, serial number, and model name/number 
    std::vector<device_info_t> find_ip_devices(
        std::chrono::steady_clock::duration wait_time = std::chrono::seconds(2),
        std::size_t max_count = std::numeric_limits<std::size_t>::max());

    /// @brief Find all PreAct ToF Video Devices attached to machine
    /// @param max_count Stop searching early if the count of devices matches this.
    /// @return std::vector<device_info_t>, a list of connected devices, their connection uri, serial number, and model name/number 
    std::vector<device_info_t> find_video_devices(std::size_t max_count = std::numeric_limits<std::size_t>::max());

} //end namespace

#endif // _DEVICE_DISCOVERY_H_

/** @} */
