/**
 * @file rapid-changes.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program to make sensor changes as rapidly as possible.
 *
 * See MOS-426
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <unistd.h>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static const uint16_t protocolVersion { 1 };
static uint32_t repetitions { 100000 };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hp:r:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Continually and rapidly set integration time." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -r <count>    Repetition count (defaults to " << repetitions << ")" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                devicePort = optarg;
                break;
            case 'r':
                repetitions = atoi(optarg);
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
        uint32_t setCount { 0 };
        uint32_t integrationOffset { 0 };

        for (uint32_t i = 0; i < repetitions; ++i)
        {
            ++setCount;
            if (!sensor.setIntegrationTimes((100 + integrationOffset),
                                           (200 + integrationOffset),
                                           (300 + integrationOffset)))
            {
                std::cerr << "setIntegration() FAILED on " << setCount << "th attempt" << std::endl;
                break;
            }
            integrationOffset = (integrationOffset + 1) % 100;
        }
        exit(0);

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
