/**
 * @file usb_device_discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Discovers USB Connected ToF Sensors
 * @{
 */
#include "crc32.h"
#include "device_discovery.hpp"
#include "TofEndian.hpp"
#include <array>
#include <condition_variable>
#include <iostream>
#include "json.hpp"
#include <mutex>
#include <libusbp.hpp>
#include <cstdlib>  // for std::getenv

namespace tofcore
{
    using namespace std::string_literals;

    constexpr uint16_t PREACT_VENDOR_ID = 0x35FA;

    // MOJAVE DEFS
    constexpr uint16_t MOJAVE_PRODUCT_ID = 0x0D0F;

    // T10C DEFS
    constexpr uint16_t T10C_PRODUCT_ID = 0x0A03U;

    // USB Channel Info
    constexpr uint8_t USB_INTERFACE_COMM_CH = 0;
    constexpr uint8_t USB_INTERFACE_DEBUG_CH = 2;

    constexpr uint16_t UDP_MULTICAST_PORT = 5352;

    /// @brief Check whether a USB based device has PreAct's Vendor Id
    /// @param device, device in question
    /// @return true, if device VID matches PreAct's VID
    bool IsUsbDeviceAPreactDevice(libusbp::device &device){

        return device.get_vendor_id() == PREACT_VENDOR_ID;
    }

    /// @brief Get the usb device port name
    /// @param device, pointer to libusbp device
    /// @return std::string, port name/handle. Ex: /dev/ttyACM0 on Linux
    std::string GetUsbCdcDeviceSerialPortName(libusbp::device &device){

        const bool composite = true;

        libusbp::serial_port port(device, USB_INTERFACE_COMM_CH, composite);

        return port.get_name();
    }

    /// @brief Get the usb device model name from PID
    /// @param device, pointer to libusbp device
    /// @return std::string, model name
    std::string GetUsbDeviceModelName(libusbp::device &device){

        std::string modelName = "Unknown Device";

        // If device is a Mojave Unit, add to list
        if (IsUsbDeviceAPreactDevice(device)){

            // Get PID
            uint16_t product_id = device.get_product_id();

            // Capture Unit Type
            if (MOJAVE_PRODUCT_ID == product_id){

                modelName = "Mojave";
            }

            else if (T10C_PRODUCT_ID == product_id){

                modelName = "T10C";
            }

            // This should NOT occur. Unknown preact PID
            else {

                modelName = "UnknownPreActDevice";
            }

        }

        return modelName;
    }

    /// @brief Get the usb devic serial number string
    /// @param device, pointer to libusbp device
    /// @return std::string, serial number string
    std::string GetUsbDeviceSerialNumber(libusbp::device &device){

        // Attempt to Get Serial Number
        std::string serial = {};
        try
        {
            serial = device.get_serial_number();
            return serial;
        }
        catch(const std::exception & error) {
            return serial;
        }
    }

    /// @brief Find USB Serial / Communications Device Class Devices
    /// @return std::vector<device_info_t> vector of found devices
    std::vector<device_info_t> find_usb_cdc_devices(std::size_t max_count)
    {
        // Get all usb devices
        std::vector<libusbp::device> usbDevices = libusbp::list_connected_devices();
        
        // List to add preact devices to
        // Maybe this evolves into several list, based on product. This'll work for now
        std::vector<device_info_t> preactDevices;

        for (auto devPtr = usbDevices.begin(); devPtr != usbDevices.end(); ++devPtr)
        {
            
            libusbp::device &device = *devPtr;

            // Only add devices that have PreAct VID
            if (IsUsbDeviceAPreactDevice(device)){

                device_info_t deviceEntry;

                deviceEntry.connector_uri = "tofserial:"s + GetUsbCdcDeviceSerialPortName(device);
                deviceEntry.model = GetUsbDeviceModelName(device);
                deviceEntry.serial_num = GetUsbDeviceSerialNumber(device);

                // Add Preact device to list
                preactDevices.push_back(deviceEntry);
                if(preactDevices.size() == max_count)
                { 
                    break;
                }
            }
        }
        return preactDevices;
    }

} //end namespace
