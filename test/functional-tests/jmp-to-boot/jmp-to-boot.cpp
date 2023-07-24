/**
 * @file tof-stat.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libt10 to get various sensor values.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;
namespace po = boost::program_options;

constexpr uint16_t FALLBACK_LOADER_TOKEN { 0x04a5 };

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { 1 };
static std::string tokenStr {};
static uint16_t token = 0;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Command sensor to jump to bootloader");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("fallback-loader,f", "Jump to fallback loader to perform update of the primary loader")
        ("token,t", po::value<std::string>(&tokenStr), "Pass 16bit token value with command to perform special boot-loader operations")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if (vm.count("fallback-loader"))
    {
        token = FALLBACK_LOADER_TOKEN;
    }

    if (vm.count("token") != 0) 
    {
        //use stoi() with 0 for the radix (3rd) parameter to allow automatic conversion of hex or
        // decimal inputs.
        token = std::stoi(tokenStr, 0, 0); 
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
        if(0 == token)
        {
            std::cout << "Requesting sensor jump to bootloader with default token" << std::endl;
            sensor.jumpToBootloader();
        }
        else
        {
            if (FALLBACK_LOADER_TOKEN == token)
            {
                std::cout << "Requesting sensor jump to FALLBACK bootloader (token 0x" << std::hex
                          << std::setw(4) << std::setfill('0') << token << ")" << std::endl;
            }
            else
            {
                std::cout << "Requesting sensor jump to bootloader (token 0x" << std::hex
                          << std::setw(4) << std::setfill('0') << token << ")" << std::endl;
            }
            sensor.jumpToBootloader(token);
        }
        exit(0);

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
