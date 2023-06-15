/**
 * @file usb_connection.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Gathers USB related information from Preact sensors
 * @{
 */
#include "crc32.h"
#include "usb_connection.hpp"
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
    UsbConnection::UsbConnection(){
        m_preactDevices = GetPreActDevices();
    }

    UsbConnection::~UsbConnection(){
        m_preactDevices.clear();
    }

    /// @brief Get the usb device port name
    /// @param device, pointer to libusbp device
    /// @return std::string, port name/handle. Ex: /dev/ttyACM0 on Linux
    std::string UsbConnection::GetSerialPortName(libusbp::device &device ){

        const bool composite = true;

        libusbp::serial_port port(device, USB_INTERFACE_COMM_CH, composite);

        return port.get_name();
    }

    /// @brief Get All PreAct Devices connected to this host
    /// @param displayAllDevices, bool display all connected devices in scan if TRUE 
    /// @return std::vector<libusbp::device>, list of PreAct Devices found by known VID/PID (defined in header)
    std::vector<libusbp::device> UsbConnection::GetPreActDevices(){

        // Get all usb devices
        std::vector<libusbp::device> allDevices = libusbp::list_connected_devices();
        
        // List to add preact devices to
        // Maybe this evolves into several list, based on product. This'll work for now
        std::vector<libusbp::device> preactDevices;

        for (auto devPtr = allDevices.begin(); devPtr != allDevices.end(); ++devPtr)
        {
            libusbp::device &device = *devPtr;

            // Get VID/PID
            uint16_t vendor_id = device.get_vendor_id();
            uint16_t product_id = device.get_product_id();

            // If device is a Mojave Unit, add to list
            if (PREACT_VENDOR_ID == vendor_id){

                // Capture Unit Type
                if (MOJAVE_PRODUCT_ID == product_id){

                    std::cout << "Found Mojave Unit!" << std::endl;
                }

                else if (T10C_PRODUCT_ID == product_id){

                    std::cout << "Found T10C Unit!" << std::endl;
                }

                // This should NOT occur. Unknown preact PID
                else {
                    std::cout << "Found Unknown Preact Unit!?" << std::endl;
                }

                // Attemp to Get Serial Number
                std::string serial = "No Serial Number";
                try
                {
                    serial = device.get_serial_number();
                    std::cout << "- Device Serial Number: " << serial << std::endl;
                }
                catch(const std::exception & error) {
                    std::cout << "- WARNING UNIT DOES NOT HAVE SERIAL NUMBER" << std::endl;
                }

                // Get the port name
                std::cout << "- Port Name: " << GetSerialPortName(device) << std::endl;

                std::cout << std::endl;

                // Add Preact device to list
                preactDevices.push_back(device);
            }

        }

        return preactDevices;
    }

} //end namespace
