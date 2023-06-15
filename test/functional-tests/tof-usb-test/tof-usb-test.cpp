/**
 * @file tof-usb-test.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libusbp to get Preact USB based devices connected to host.
 */

#include "tofcore/tof_sensor.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include "tof-usb-test.h"
#include "../src/comm_usb/usb_connection.hpp"

#include <libusbp-1/libusbp.hpp>
// https://github.com/pololu/libusbp/blob/master/examples/port_name/port_name.cpp


std::string serial_number_or_default(const libusbp::device & device,
    const std::string & def)
{
    try
    {
        return device.get_serial_number();
    }
    catch (const libusbp::error & error)
    {
        if (error.has_code(LIBUSBP_ERROR_NO_SERIAL_NUMBER))
        {
            return def;
        }
        throw;
    }
}

void print_device(libusbp::device & device)
{
    uint16_t vendor_id = device.get_vendor_id();
    uint16_t product_id = device.get_product_id();
    uint16_t revision = device.get_revision();
    std::string serial_number = serial_number_or_default(device, "-");
    std::string os_id = device.get_os_id();

    // Note: The serial number might have spaces in it, so it should be the last
    // field to avoid confusing programs that are looking for a field after the
    // serial number.

    std::ios::fmtflags flags(std::cout.flags());
    std::cout
        << std::hex << std::setfill('0') << std::right
        << std::setw(4) << vendor_id
        << ':'
        << std::setw(4) << product_id
        << ' '
        << std::setfill(' ') << std::setw(2) << (revision >> 8)
        << '.'
        << std::setfill('0') << std::setw(2) << (revision & 0xFF)
        << ' '
        << os_id
        << ' '
        << std::setfill(' ') << std::left << serial_number
        << std::endl;
    std::cout.flags(flags);
}


int main() {

    try
    {

        printf("Get Devices list\n");

        auto preactDevices = tofcore::UsbConnection();
    }

    catch(const std::exception & error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
    }
        
}