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
#include <unistd.h>

using namespace tofcore;

constexpr uint16_t FALLBACK_LOADER_TOKEN { 0x04a5 };

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static const uint16_t protocolVersion { 1 };
static uint16_t token = 0;

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hfp:t:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Command sensor to jump to bootloader" << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -f            Jump to fallback loader" << std::endl
                            << "  -t <token>    Pass 16bit token value with command to peform special bootloader operations" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                devicePort = optarg;
                break;
            case 'f':
                token = FALLBACK_LOADER_TOKEN;
                break;
            case 't':
                token = strtoul(optarg, nullptr, 0);
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
