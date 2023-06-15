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

typedef tcb::span<std::byte> ScatterGatherElement;

class UsbConnection
{

public:
    UsbConnection();

    ~UsbConnection();

    std::string GetSerialPortName(libusbp::device &device);

    std::vector<libusbp::device> GetPreActDevices();

    const uint16_t PREACT_VENDOR_ID = 0x35FA;

    // MOJAVE DEFS
    const uint16_t MOJAVE_PRODUCT_ID = 0x0D0F;

    // T10C DEFS
    const uint16_t T10C_PRODUCT_ID = 0x0A03U;

    // USB Channel Info
    const uint8_t USB_INTERFACE_COMM_CH = 0;
    const uint8_t USB_INTERFACE_DEBUG_CH = 2;

    std::vector<libusbp::device> m_preactDevices;

protected:


};

} //end namespace

#endif // _USB_CONNECTION_H_

/** @} */
