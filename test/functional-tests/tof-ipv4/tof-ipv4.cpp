/**
 * @file tof-ipv4.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that sets/gets IPv4 settings
 */
#include "tofcore/tof_sensor.hpp"
#include <array>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { 1 };
static std::vector<unsigned> ipv4Data {};


static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc(
                "IPv4 write[optional]/read utility\n\n"
                "  Usage: [options] [-i IPv4 address quartet, IPv4 mask quartet, IPv4 gateway quartet]\n\n"
                "  EXAMPLE: To set the IPv4 address, mask, gateway to 10.10.31.180, 255.255.255.0, 10.10.31.1\n\n"
                "     tof-ipv4 -i 10 10 31 180 255 255 255 0 10 10 31 1\n\n"
                "  NOTE:\n"
                "    1) If no -i option is specified, the IPv4 values are only read.\n"
                );
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("ipv4Data,i", po::value<std::vector<unsigned>>(&ipv4Data), "Set IPv4 (requires 12 unsigned values)")
        ;

    po::positional_options_description pos_desc;
    pos_desc.add("ipv4Data", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);

    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }
}

static void signalHandler(int signum)
{
    (void)signum;
    exitRequested = true;
}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);
    /*
     * Change default action of ^C, ^\ from abnormal termination in order to
     * perform a controlled shutdown.
     */
    signal(SIGINT, signalHandler);
#if defined(SIGQUIT)
    signal(SIGQUIT, signalHandler);
#endif
    {
        tofcore::Sensor sensor { protocolVersion, devicePort, baudRate };
        // Set the IPv4 values
        if (ipv4Data.size() == 12)
        {
            const std::array<std::byte, 4> ipv4Addr { (std::byte)ipv4Data[0], (std::byte)ipv4Data[1], (std::byte)ipv4Data[2], (std::byte)ipv4Data[3] };
            const std::array<std::byte, 4> ipv4Mask { (std::byte)ipv4Data[4], (std::byte)ipv4Data[5], (std::byte)ipv4Data[6], (std::byte)ipv4Data[7] };
            const std::array<std::byte, 4> ipv4Gway { (std::byte)ipv4Data[8], (std::byte)ipv4Data[9], (std::byte)ipv4Data[10], (std::byte)ipv4Data[11] };
            if (sensor.setIPv4Settings(ipv4Addr, ipv4Mask, ipv4Gway))
            {
                std::cout << "SUCCESS in setting:" << std::endl;
            }
            else
            {
                std::cout << "FAILED in setting:" << std::endl;
            }
            std::cout << "  ipv4Addr: " << ipv4Data[0] << "." << ipv4Data[1] << "." << ipv4Data[2] << "."  << ipv4Data[3] << std::endl;
            std::cout << "  ipv4Mask: " << ipv4Data[4] << "." << ipv4Data[5] << "." << ipv4Data[6] << "."  << ipv4Data[7] << std::endl;
            std::cout << "  ipv4GW:   " << ipv4Data[8] << "." << ipv4Data[9] << "." << ipv4Data[10] << "."  << ipv4Data[11] << std::endl;
        }
        else
        {
            std::cout << "Skipping setting of IPv4 data since 12 values were not provided" << std::endl;
        }
        // Read the IPv4 values
        std::array<std::byte, 4> adrs;
        std::array<std::byte, 4> mask;
        std::array<std::byte, 4> gway;
        auto result = sensor.getIPv4Settings(adrs, mask, gway);

        if (result)
        {
            std::cout << "IPv4 values reported:" << std::endl;
            std::cout << "  ipv4Addr: " << (unsigned)adrs[0] << "."<< (unsigned)adrs[1] << "."<< (unsigned)adrs[2] << "."<< (unsigned)adrs[3] << std::endl;
            std::cout << "  ipv4Mask: " << (unsigned)mask[0] << "."<< (unsigned)mask[1] << "."<< (unsigned)mask[2] << "."<< (unsigned)mask[3] << std::endl;
            std::cout << "  ipv4GW:   " << (unsigned)gway[0] << "."<< (unsigned)gway[1] << "."<< (unsigned)gway[2] << "."<< (unsigned)gway[3] << std::endl;
        }
        else
        {
            std::cout << "FAILED read of IPv4 values" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
