/**
 * @file tof-discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libusbp to get Preact USB based devices connected to host.
 */

#include "tofcore/device_discovery.hpp"
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

static std::string devicePort { };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Discover and enumerate connected devices");
    desc.add_options()
        ("help,h", "produce help message")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
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