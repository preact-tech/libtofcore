/**
 * @file tof-stat.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace TofComm;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };
static std::string sensorLocation { };
static std::string sensorName { };
static bool setSensorLocation { false };
static bool setSensorName { false };
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Tof sensor status");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("store-settings,s", po::bool_switch(&storeSettings), "Have sensor store current sensor settings in persistent memory")
        ("sensor-location,l", po::value<std::string>(&sensorLocation), "Set location of sensor")
        ("sensor-name,n", po::value<std::string>(&sensorName), "Set name of sensor")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }

    setSensorLocation = (vm.count("sensor-location") > 0);
    setSensorName = (vm.count("sensor-name") > 0);
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
            std::cerr << "Failed to retrieve static sensor data" << std::endl;
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
         * Set sensor name/location
         */
        if (setSensorName)
        {
            if (sensor.setSensorName(sensorName))
            {
                std::cout << "Sensor name set to '" << sensorName << "'" << std::endl;
            }
            else
            {
                std::cerr << "FAILED to set sensor name" << std::endl;
            }
        }
        if (setSensorLocation)
        {
            if (sensor.setSensorLocation(sensorLocation))
            {
                std::cout << "Sensor location set to '" << sensorLocation << "'" << std::endl;
            }
            else
            {
                std::cerr << "FAILED to set sensor location" << std::endl;
            }
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
        /*
         * Read Sensor Status
         */
        Sensor_Status_t sensorStatus { };
        if (sensor.getSensorStatus(sensorStatus))
        {
            std::cout << "Last Temperature: " << sensorStatus.lastTemperature
                      << "; USB Current: " << sensorStatus.USB_Current
                      << "; BIT Status: " << std::hex << "0X" << sensorStatus.BIT_Status << "\n";
        }
        else
        {
            std::cerr << "Failed to read sensor status" << std::endl;
        }
        /*
         * Read Sensor Name and Location
         */
        auto sensorName = sensor.getSensorName();
        if (sensorName)
        {
            std::cout << "Sensor name: '" << sensorName->c_str() << "'" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read sensor name" << std::endl;
        }
        auto sensorLocation = sensor.getSensorLocation();
        if (sensorLocation)
        {
            std::cout << "Sensor location: '" << sensorLocation->c_str() << "'" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read sensor location" << std::endl;
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
