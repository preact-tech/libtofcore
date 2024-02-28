/**
 * @file video_device_discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Discovers Connected ToF Video Devices
 * @{
 */
#include "device_discovery.hpp"

namespace tofcore
{
    std::vector<device_info_t> find_video_devices(std::size_t max_count)
    {
        (void)max_count;
        return std::vector<device_info_t>();
    }

} //end namespace
