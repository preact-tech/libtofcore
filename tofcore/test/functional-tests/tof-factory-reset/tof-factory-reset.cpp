/**
 * @file tof-factory-reset.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to reset a sensor to factory defaults.
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
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool resetSettingsToFactoryDefault { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Tof sensor status");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("help,h", "produce help message")
        ("confirm", po::bool_switch(&resetSettingsToFactoryDefault), "Erase sensor settings and reset to factory defaults")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        dbg_out << desc << "\n";
        exit(0);
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
        tofcore::Sensor sensor { devicePort, baudRate };
        sensor.setDebugLevel(debugLevel);
  
        /*
         * Version information
         */
        versionData_t versionData { };
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
                    << "\n  Backpack Module    : " << (unsigned)versionData.m_backpackModule;
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
         * Erase settings
         */
        if (resetSettingsToFactoryDefault)
        {
            if (sensor.resetSettingsToFactoryDefault())
            {
                dbg_out << "Sensor settings erased from persistent memory. Default settings will be applied at next power on.\n\n";
            }
            else
            {
                err_out << "Failed to erase sensor settings from persistent memory.\n\n";
            }
        }
        else
        {
            dbg_out << "Reset not performed! Use --confirm to execute factory reset.\n\n";
        }
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
