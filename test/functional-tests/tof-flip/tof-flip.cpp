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
#include <unistd.h>

using namespace TofComm;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool flipHorizontal { false };
static bool flipVertical { false };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };
static bool setHorizontal { false };
static bool setVertical { false };
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hH:p:sv:V:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Set/Get horizontal/vertical flip state." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-H(0|1)] [-p <port>] [-s] [-v <ver>] [-V(0|1)]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -H 0|1        Turn horizontal flip off/on" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -s            Store settings in persistent memory" << std::endl
                            << "  -v <ver>      Use version <ver> of the command protocol. Default = " << DEFAULT_PROTOCOL_VERSION << std::endl
                            << "  -V 0|1        Turn vertical flip off/on" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'H':
                setHorizontal = true;
                flipHorizontal = atoi(optarg);
                break;
            case 'p':
                  devicePort = optarg;
                  break;
            case 'v':
                protocolVersion = atoi(optarg);
                break;
            case 's':
                storeSettings = true;
                break;
            case 'V':
                setVertical = true;
                flipVertical = atoi(optarg);
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
