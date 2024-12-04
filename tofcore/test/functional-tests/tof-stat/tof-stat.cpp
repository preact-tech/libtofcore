/**
 * @file tof-stat.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>

using namespace test;
using namespace TofComm;
using namespace tofcore;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint16_t calVledMv { 0 };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static std::string sensorLocation { };
static std::string sensorName { };
static bool setCalVledMv { false };
static bool setSensorLocation { false };
static bool setSensorName { false };
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Tof sensor status");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("cal-vled-mv,c", po::value<uint16_t>(&calVledMv), "Set value of calibration VLED mV")
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("help,h", "produce help message")
        ("sensor-location,l", po::value<std::string>(&sensorLocation), "Set location of sensor")
        ("sensor-name,n", po::value<std::string>(&sensorName), "Set name of sensor")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("store-settings,s", po::bool_switch(&storeSettings), "Have sensor store current sensor settings in persistent memory")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        dbg_out << desc << "\n";
        exit(0);
    }

    setCalVledMv = (vm.count("cal-vled-mv") > 0);
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
        tofcore::Sensor sensor { devicePort, baudRate };
        sensor.setDebugLevel(debugLevel);

        if (setCalVledMv)
        {
            if (sensor.setCalVledMv(calVledMv))
            {
                dbg_out << "Sensor cal. VLED mV set to " << calVledMv << " mV\n\n";
            }
            else
            {
                err_out << "FAILED to set cal. VLED mV\n\n";
            }
        }
        /*
         * Version information
         */
        versionData_t versionData { };
        uint16_t calVledMv { 0 };
        auto respondsGetCalVledMv = sensor.getCalVledMv();
        if (respondsGetCalVledMv)
        {
            calVledMv = *respondsGetCalVledMv;
        }
        else
        {
            err_out << "Failed to retrieve calibration VLED mv\n\n";
        }
        if (sensor.getSensorInfo(versionData))
        {
            dbg_out << "Version Information"
                    << "\n  Device Serial #    : " << versionData.m_deviceSerialNumber
                    << "\n  CPU Board Serial # : " << versionData.m_cpuBoardSerialNumber
                    << "\n  Illum. Bd Serial # : " << versionData.m_illuminatorBoardSerialNumber
                    << "\n  Model Name         : " << versionData.m_modelName
                    << "\n  Last Reset Type    : " << versionData.m_lastResetType
                    << "\n  Software Version   : " << versionData.m_softwareVersion
                    << "\n  CPU Board Revision : " << (unsigned)versionData.m_cpuBoardRevision
                    << "\n  Chip ID            : " << std::hex << versionData.m_sensorChipId << std::dec
                    << "\n  Illum. SW Version  : " << versionData.m_illuminatorSwVersion << '.' << versionData.m_illuminatorSwSourceId
                    << "\n  Backpack Module    : " << (unsigned)versionData.m_backpackModule
                    << "\n  Calibration VLED   : " << (unsigned)calVledMv << " mV\n\n";
        }
        else
        {
            err_out << "Failed to retrieve static sensor data\n\n";
        }

        /*
         * Persistent Sensor Settings
         */
        std::string jsonSettings { };
        if (sensor.getSettings(jsonSettings))
        {
            dbg_out << "Settings: '" << jsonSettings.c_str() << "'\n\n";
        }
        else
        {
            err_out << "Failed to read sensor settings\n\n";
        }
        std::optional<uint8_t> binningSetting = sensor.getBinning();
        if(binningSetting.has_value())
        {
            dbg_out << "Binning: " << static_cast<unsigned>(binningSetting.value()) << "\n\n";
        }
        else
        {
            dbg_out << "Failed to get binning setting!\n\n";
        }
        /*
         * Set sensor name/location
         */
        if (setSensorName)
        {
            if (sensor.setSensorName(sensorName))
            {
                dbg_out << "Sensor name set to '" << sensorName << "'\n\n";
            }
            else
            {
                err_out << "FAILED to set sensor name\n\n";
            }
        }
        if (setSensorLocation)
        {
            if (sensor.setSensorLocation(sensorLocation))
            {
                dbg_out << "Sensor location set to '" << sensorLocation << "'\n\n";
            }
            else
            {
                err_out << "FAILED to set sensor location\n\n";
            }
        }
        /*
         * Store settings (optional)
         */
        if (storeSettings)
        {
            if (sensor.storeSettings())
            {
                dbg_out << "Sensor settings stored in persistent memory.\n\n";
            }
            else
            {
                err_out << "Failed to store sensor settings in persistent memory\n\n";
            }
        }
        /*
         * Read Sensor Status
         */
        Sensor_Status_t sensorStatus { };
        if (sensor.getSensorStatus(sensorStatus))
        {
            dbg_out << "Last Temperature: " << sensorStatus.lastTemperature
                    << "; USB Current: " << sensorStatus.USB_Current
                    << "; BIT Status: " << std::hex << "0X" << sensorStatus.BIT_Status << "\n\n";
        }
        else
        {
            err_out << "Failed to read sensor status\n\n";
        }
        /*
         * Read Sensor Name and Location
         */
        auto sensorName = sensor.getSensorName();
        if (sensorName)
        {
            dbg_out << "Sensor name: '" << sensorName->c_str() << "'\n\n";
        }
        else
        {
            err_out << "Failed to read sensor name\n\n";
        }
        auto sensorLocation = sensor.getSensorLocation();
        if (sensorLocation)
        {
            dbg_out << "Sensor location: '" << sensorLocation->c_str() << "'\n\n";
        }
        else
        {
            err_out << "Failed to read sensor location\n\n";
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
