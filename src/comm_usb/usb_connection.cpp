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
    UsbConnection::UsbConnection(const std::string defaultPortName){

        m_portName = defaultPortName;
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

    /// @brief Check Whether Port Name Given is a PreAct Device
    /// @param givenPortName, std::string, of port name. May be default
    /// @return true if port name 
    bool UsbConnection::IsPortNameAPreactDevice(const std::string &givenPortName){

        for (auto devPtr = m_preactDevices.begin(); devPtr != m_preactDevices.end(); ++devPtr){

            libusbp::device &device = *devPtr;

            std::string devicePortName = GetSerialPortName(device);

            // If port name matches device in PreAct list
            if (devicePortName.compare(givenPortName) == 0) {

                return true;
            }
        }

        return false;
    }

    /// @brief Check if given port name is a PreAct Device, otherwise, return first avaiable (if any)
    /// @return std::string, port name
    std::string UsbConnection::GetAvailablePreactDevicePortName(){

        std::cout << "Initial port name: " << m_portName << std::endl;

        std::cout << "Number of PreAct Devices: " << m_preactDevices.size() << std::endl;

        // Check if given port name is a preact device
        // If so, return to use for serial connection
        if (IsPortNameAPreactDevice(m_portName)){
            std::cout << "Using Port Name Given: " << m_portName << std::endl;
        }

        // Else if other PreAct Devices are available, return first available 
        else if (m_preactDevices.size() > 0){

            std::cout << "Port Unavailable or NOT a PreAct Device: " << m_portName << std::endl;

            m_portName = GetSerialPortName(m_preactDevices[0]);

            std::cout << "Using First Available PreAct Device: " << m_portName << std::endl;
        }

        // No PreAct Devices
        else {
            std::cout << "No PreAct Devices Available" << std::endl;
        }

        return m_portName;
    }

} //end namespace
