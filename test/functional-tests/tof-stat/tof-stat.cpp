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

using namespace TofComm;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:hp:sv:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Read various sensor status data." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>] [-s] [-v <ver>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -s            Have sensor store current sensor settings in persistent memory" << std::endl
                            << "  -v <ver>      Use version <ver> of the command protocol. Default = " << DEFAULT_PROTOCOL_VERSION << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                  devicePort = optarg;
                  break;
            case 's':
                storeSettings = true;
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
        /*
         * Version information
         */
        versionData_t versionData { };
        if (sensor.getSensorInfo(versionData))
        {
            std::cout << "Version Information" << std::endl
                        << "  Device Serial #    : " << versionData.m_deviceSerialNumber << std::endl
                        << "  CPU Board Serial # : " << versionData.m_cpuBoardSerialNumber << std::endl
                        << "  Illuminator Serial # : " << versionData.m_illuminatorBoardSerialNumber << std::endl
                        << "  Model Name         : " << versionData.m_modelName << std::endl
                        << "  Last Reset Type    : " << versionData.m_lastResetType << std::endl
                        << "  Software Version   : " << versionData.m_softwareVersion << std::endl
                        << "  CPU version        : " << (unsigned)versionData.m_cpuVersion << std::endl
                        << "  Chip ID : " << std::hex << versionData.m_sensorChipId << std::dec << std::endl
                        << "  Illuminator SW Version: " << versionData.m_illuminatorSwVersion << '.' << 
                                                            versionData.m_illuminatorSwSourceId << std::endl
                        << "  Backpack Module    : " << (unsigned)versionData.m_backpackModule << std::endl;
        }
        else
        {
            std::cerr << "Failed to read version data" << std::endl;
        }
        /*
         * Persistent Sensor Settings
         */
        std::string jsonSettings { };
        if (sensor.getSettings(jsonSettings))
        {
            std::cout << "Settings: '" << jsonSettings.c_str() << "'" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read sensor settings" << std::endl;
        }
        /*
         * Accelerometer Reading
         */
        int16_t x, y, z;
        uint8_t g_range;
        if (sensor.getAccelerometerData(x, y, z, g_range))
        {
            std::cout <<     "AccelX: " << (g_range * x / 32768.0)
                      << " g; AccelY: " << (g_range * y / 32768.0)
                      << " g; AccelZ: " << (g_range * z / 32768.0) << " g" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read accelerometer" << std::endl;
        }
        /*
         * Store settings (optional)
         */
        if (storeSettings)
        {
            if (sensor.storeSettings())
            {
                std::cout << "Sensor settings stored in persistent memory." << std::endl;
            }
            else
            {
                std::cerr << "Failed to store sensor settings in persistent memory" << std::endl;
            }
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
