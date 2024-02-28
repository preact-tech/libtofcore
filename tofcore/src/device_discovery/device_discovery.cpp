/**
 * @file device_discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Discovers Connected ToF Sensors
 * @{
 */
#include "device_discovery.hpp"

namespace tofcore
{


    /// @brief Find all PreAct ToF Devices
    /// @return 
    std::vector<device_info_t> find_all_devices(std::chrono::steady_clock::duration wait_time, std::size_t max_count)
    {
        std::vector<device_info_t> devices;

        // Check if environment variable TOFCORE_DEVICE_URI is set
        if(const char* env_p = std::getenv("TOFCORE_DEVICE_URI")) {
            // Create a new device_info_t instance with the environment variable as connector_uri
            device_info_t device;
            device.connector_uri = env_p;

            // Add it to the devices vector
            devices.insert(devices.end(), device);
        }

        // Gather device by connection type
        if(devices.size() < max_count)
        {
            auto found = find_usb_cdc_devices(max_count - devices.size());
            devices.insert(devices.end(), found.begin(), found.end());
        }

        if(devices.size() < max_count)
        {
            auto found = find_ip_devices(wait_time, max_count - devices.size());
            devices.insert(devices.end(), found.begin(), found.end());
            max_count -= devices.size();
        }

        if(devices.size() < max_count)
        {
            auto found = find_video_devices(max_count - devices.size());
            devices.insert(devices.end(), found.begin(), found.end());
            max_count -= devices.size();
        }

        return devices;
    }

} //end namespace
