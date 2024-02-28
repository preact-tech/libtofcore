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

static uint32_t timeout_sec = 5;
static std::size_t max_count = std::numeric_limits<int>::max();
static bool continuous = false;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Discover and enumerate connected devices");
    desc.add_options()
        ("help,h", "produce help message")
        ("timeout,t", po::value<uint32_t>(&timeout_sec)->default_value(5), "Max time (in seconds) to search for each scan iteration")
        ("max_count,m", po::value<size_t>(&max_count)->default_value(std::numeric_limits<int>::max()), "The point at which to stop searching for more devices in each scan")
        ("continuous,c", po::bool_switch(&continuous), "Continuously search for devices until process is killed")
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

    do
    {
        try
        {
            std::vector<tofcore::device_info_t> devices = tofcore::find_all_devices(std::chrono::seconds(timeout_sec), max_count);

            for (auto devPtr = devices.begin(); devPtr != devices.end(); ++devPtr)
            {
                std::cout << "------------------------" << std::endl
                        << "Device URI: " << devPtr->connector_uri << std::endl
                        << "Sensor Name: " << devPtr->sensor_name << std::endl
                        << "Model Name: " << devPtr->model << std::endl
                        << "Serial Number: " << devPtr->serial_num << std::endl
                        << "Location: " << devPtr->location << std::endl;
            }
        }

        catch(const std::exception & error)
        {
            std::cerr << "Error: " << error.what() << std::endl;
        }
            /* code */
    } while (continuous);
}
