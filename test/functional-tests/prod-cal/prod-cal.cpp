/**
 * @file tof-stat.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to initiate the Espros calibration sequence.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <unistd.h>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool doDrnuCal { false };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:dhp:v:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'd':
                doDrnuCal = true;
                break;
            case 'h':
                std::cout   << "Start calibration sequence and exit." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>] [-s] [-v <ver>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -d            Initiate DRNU cal. instead of production cal." << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -v <ver>      Use version <ver> of the command protocol. Default = " << DEFAULT_PROTOCOL_VERSION << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                  devicePort = optarg;
                  break;
            case 'v':
                protocolVersion = atoi(optarg);
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
        if (doDrnuCal)
        {
            std::cout << "Initiating DRNU calibration sequence" << std::endl;
            sensor.startDrnuCalibration();
        }
        else
        {
            std::cout << "Initiating PRODUCTION calibration sequence" << std::endl;
            sensor.startProductionCalibration();
        }
        exit(0);
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
