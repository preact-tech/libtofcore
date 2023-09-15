/**
 * @file tof-flip.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t flipHorizontal { 0 };
static uint16_t flipVertical { 0 };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };
static bool setHorizontal { false };
static bool setVertical { false };
static bool storeSettings { false };



static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Set/Get horizontal/vertical flip state.");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("horizontal-flip,H", po::value<uint16_t>(&flipHorizontal), "Turn horizontal flip off/on")
        ("vertical-flip,V", po::value<uint16_t>(&flipVertical), "turn vertical flip off/on")
        ("store-settings,s", po::bool_switch(&storeSettings), "Store settings in persistent memory")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    if (vm.count("horizontal-flip"))
    {
        setHorizontal = true;
    }
    if (vm.count("vertical-flip"))
    {
        setVertical = true;
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

        if (setHorizontal)
        {
            const bool flip { flipHorizontal != 0 };
            if (sensor.setFlipHorizontally(flip))
            {
                std::cout << "Horizontal flip commanded " << (flip ? "ON" : "OFF") << std::endl;
            }
            else
            {
                std::cerr << "FAILED to command horizontal flip state" << std::endl;
            }
        }

        if (setVertical)
        {
            const bool flip { flipVertical != 0 };
            if (sensor.setFlipVertically(flip))
            {
                std::cout << "Vertical flip commanded " << (flip ? "ON" : "OFF") << std::endl;
            }
            else
            {
                std::cerr << "FAILED to command vertical flip state" << std::endl;
            }
        }

        auto hResult = sensor.isFlipHorizontallyActive();
        auto vResult = sensor.isFlipVerticallyActive();

        const char* hDescr = (hResult) ? (*hResult ? "ON" : "OFF") : "??";
        const char* vDescr = (vResult) ? (*vResult ? "ON" : "OFF") : "??";

        std::cout << "Horizontal Flip: " << hDescr << "; Vertical Flip: " << vDescr << std::endl;

        if (storeSettings)
        {
            if (sensor.storeSettings())
            {
                std::cout << "Settings stored in persistent memory." << std::endl;
            }
            else
            {
                std::cerr << "Failed to store settings in persistent memory" << std::endl;
            }
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
