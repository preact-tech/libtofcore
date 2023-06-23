/**
 * @file tof-discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libusbp to get Preact USB based devices connected to host.
 */

#include "tofcore/device_discovery.hpp"
#include <iostream>
#include <unistd.h>

static std::string devicePort { };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "h")) != -1)
    {
        switch (opt)
        {
            case 'h':
                std::cout   << "Utiliity to scan for PreAct ToF Devices\n\n"
                            << "Usage: " << argv[0] << " [-h]\n"
                            << "  -h            Print help and exit"
                            << std::endl;
                exit(0);
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
        std::vector<tofcore::device_info_t> devices = tofcore::find_all_devices();

        for (auto devPtr = devices.begin(); devPtr != devices.end(); ++devPtr)
        {
            std::cout << "------------------------" << std::endl
                      << "Device URI: " << devPtr->connector_uri << std::endl
                      << "Model Name: " << devPtr->model << std::endl
                      << "Serial Number: " << devPtr->serial_num << std::endl;
        }
    }

    catch(const std::exception & error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
    }
        
}