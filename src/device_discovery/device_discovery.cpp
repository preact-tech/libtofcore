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
#include <libusbp-1/libusbp.hpp>

namespace tofcore
{

    /// @brief TODO: this is for future development, will need to determine device connection type
    /// @return 
    std::string GetDeviceType(){
        return SERIAL_PREFIX;
    }

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

        // Attemp to Get Serial Number
        std::string serial = "No Serial Number";
        try
        {
            serial = device.get_serial_number();
        }
        catch(const std::exception & error) {
            std::cout << "- WARNING UNIT DOES NOT HAVE SERIAL NUMBER" << std::endl;
        }

        return serial;
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

                // TODO: will eventually want to prepend device type prefix
                //deviceEntry.connector_uri = GetDeviceType() + GetUsbCdcDeviceSerialPortName(device);

                deviceEntry.connector_uri = GetUsbCdcDeviceSerialPortName(device);
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

    /// @brief Get device info
    /// @param defaultPort, default port name
    /// @return device_info_t, struct containing uri, model, and serial number strings
    device_info_t get_device_info(const std::string defaultPort){

        // No port name given (Note: empty default causes all kinds of problems)
        if (defaultPort.compare("") == 0){

            std::vector<device_info_t> devices = find_all_devices();
            
            if (devices.empty()){
                throw std::runtime_error("No PreAct Devices Found");
            }

            return devices[0];
        }

        else {

            device_info_t deviceEntry = {
                defaultPort,
                "",
                ""
            };

            return deviceEntry;
        }
    }

} //end namespace
