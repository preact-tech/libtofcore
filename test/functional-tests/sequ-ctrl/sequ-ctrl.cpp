/**
 * @file tof-stat.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <unistd.h>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static int checkVersion { -1 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool getVersion { false };
static const uint16_t protocolVersion { 1 };
static int setVersion { -1 };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:c:ghp:s:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'c':
                checkVersion = atoi(optarg);
                break;
            case 'g':
                getVersion = true;
                break;
            case 'h':
                std::cout   << "Get/Set EPC sequencer version." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-c <version>] [-g] [-h] [-p <port>] [-s <version>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -c <version>  Check if version is supported" << std::endl
                            << "  -g            Get current sequencer version" << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -s <version>  Set sequencer version" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                  devicePort = optarg;
                  break;
            case 's':
                setVersion = atoi(optarg);
                break;
            default:
                break;
        }
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
    signal(SIGQUIT, signalHandler);
    {
        tofcore::Sensor sensor { protocolVersion, devicePort, baudRate };
        /*
         * Check sequencer version for support
         */
        if (checkVersion >= 0)
        {
            const auto result { sensor.sequencerIsVersionSupported(checkVersion) };
            if (std::get<0>(result))
            {
                const bool isSupported { std::get<1>(result) };
                std::cout << "Sequencer version " << checkVersion << (isSupported ? " is" : " is not") << " supported" << std::endl;
            }
            else
            {
                std::cerr << "Failed to check sequencer version" << std::endl;
            }
        }
        /*
         * Set sequencer version
         */
        if (setVersion >= 0)
        {
            const bool result { sensor.sequencerSetVersion(setVersion) };
            if (result)
            {
                std::cout << "Sequencer version set to " << setVersion << std::endl;
            }
            else
            {
                std::cerr << "Failed to set sequencer version to " << setVersion << std::endl;
            }
        }
       /*
         * Get sequencer version
         */
        if (getVersion)
        {
            const auto result { sensor.sequencerGetVersion() };
            if (std::get<bool>(result))
            {
                const uint16_t version { std::get<uint16_t>(result) };
                std::cout << "Sequencer version is " << version << std::endl;
            }
            else
            {
                std::cerr << "Failed to get sequencer version" << std::endl;
            }
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
