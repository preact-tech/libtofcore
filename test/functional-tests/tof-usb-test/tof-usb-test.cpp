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
#include "../src/device_discovery/device_discovery.hpp"

static std::string devicePort { };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hp:sv:")) != -1)
    {
        switch (opt)
        {
            case 'h':
                std::cout   << "Test to list PreAct USB Devices. Also used to test whether a particular port is a PreAct device" << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-h] [-p <port>]" << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< "/dev/ttyACM3" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                  devicePort = optarg;
                  break;
            default:
                break;
        }
    }
}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);

    try
    {
        tofcore::device_info_t device = tofcore::get_device_info(devicePort);

        std::cout << "Device URI: " << device.connector_uri << std::endl
                  << "Model Name: " << device.model << std::endl
                  << "Serial Number: " << device.serial_num << std::endl;
    }

    catch(const std::exception & error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
    }
        
}