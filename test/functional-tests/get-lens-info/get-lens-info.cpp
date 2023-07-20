/**
 * @file get-lens-info.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that libtofcore to read Lens information.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <unistd.h>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static const uint16_t protocolVersion { 1 };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hp:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Read Lens information from the sensor" << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                devicePort = optarg;
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

        auto lensInfo = sensor.getLensIntrinsics();
        if (lensInfo)
        {
            std::cout << "rowOffset="           << lensInfo->m_rowOffset
                      << ", columnOffset="      << lensInfo->m_columnOffset
                      << ", rowFocalLength="    << lensInfo->m_rowFocalLength
                      << ", columnFocalLength=" << lensInfo->m_columnFocalLength
                      << ", undistortionCoeff=[";
            size_t n;
            for (n = 0; n < 4; ++n)
            {
                std::cout << lensInfo->m_undistortionCoeffs[n] << ", ";
            }
            std::cout << lensInfo->m_undistortionCoeffs[n] << "]" << std::endl;
        }
        else
        {
            std::cerr << "Unable to read Lens Info data" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
