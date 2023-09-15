/**
 * @file device_discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Discovers Connected ToF Sensors
 * @{
 */
#include "crc32.h"
#include "device_discovery.hpp"
#include "TofEndian.hpp"
#include <array>
#include <boost/scope_exit.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
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
    std::vector<device_info_t> find_usb_cdc_devices(){

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
            }

        }

        return preactDevices;

    }

    /// @brief Find Eth Devices (TODO: for future development)
    /// @return std::vector<device_info_t> vector of found devices
    std::vector<device_info_t> find_eth_devices(){

        // TODO: for future development
        return std::vector<device_info_t>();
    }

    /// @brief Find Eth Devices (TODO: for future development)
    /// @return std::vector<device_info_t> vector of found devices
    std::vector<device_info_t> find_video_devices(){

        // TODO: for future development
        return std::vector<device_info_t>();
    }

    /// @brief Find all PreAct ToF Devices
    /// @return 
    std::vector<device_info_t> find_all_devices(){

        std::vector<device_info_t> devices;

        // Check if environment variable TOFCORE_DEVICE_URI is set
        if(const char* env_p = std::getenv("TOFCORE_DEVICE_URI")) {
            // Create a new device_info_t instance with the environment variable as connector_uri
            device_info_t dut_device;
            dut_device.connector_uri = env_p;
            dut_device.serial_num = "";
            dut_device.model = "";

            // Add it to the devices vector
            devices.insert(devices.begin(), dut_device);
        }

        // Gather device by connection type
        // For furture development, might want seperate lists. 
        std::vector<device_info_t> usbCdcDevice = find_usb_cdc_devices();
        devices.insert(devices.end(), usbCdcDevice.begin(), usbCdcDevice.end());

        std::vector<device_info_t> ethDevices = find_eth_devices();
        devices.insert(devices.end(), ethDevices.begin(), ethDevices.end());

        std::vector<device_info_t> videoDevices = find_video_devices();
        devices.insert(devices.end(), videoDevices.begin(), videoDevices.end());

        return devices;
    }

} //end namespace
