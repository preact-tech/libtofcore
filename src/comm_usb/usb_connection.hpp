/**
 * @file serial_connection.h
 *
 * Copyright 2023 PreAct Technologies
 *
 * API for serial connection to T10 sensor
 * @{
 */
#ifndef _USB_CONNECTION_H_
#define _USB_CONNECTION_H_

#include "span.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>
#include <cstdint>
#include <libusbp-1/libusbp.hpp>

namespace tofcore
{

class UsbConnection
{

public:
    UsbConnection(const std::string defaultPortName);

    ~UsbConnection();

    std::string GetSerialPortName(libusbp::device &device);

    std::vector<libusbp::device> GetPreActDevices();

    bool IsPortNameAPreactDevice(const std::string &givenPortName);

    std::string GetAvailablePreactDevicePortName();

    const uint16_t PREACT_VENDOR_ID = 0x35FA;

    // MOJAVE DEFS
    constexpr uint16_t MOJAVE_PRODUCT_ID = 0x0D0F;

    // T10C DEFS
    const uint16_t T10C_PRODUCT_ID = 0x0A03U;

    // USB Channel Info
    const uint8_t USB_INTERFACE_COMM_CH = 0;
    const uint8_t USB_INTERFACE_DEBUG_CH = 2;

    // TODO: make our own struct and store pertinent information. 
    // DO NOT bring library struct up out of libs. Less flexible in the future
    std::vector<libusbp::device> m_preactDevices;
    std::string m_portName;

protected:


};

} //end namespace

#endif // _USB_CONNECTION_H_

/** @} */
