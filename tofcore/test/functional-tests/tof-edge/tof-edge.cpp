/**
 * @file tof-vsm.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that sets/gets IPv4 settings
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <thread>
#include <vector>

using namespace test;
using namespace tofcore;
using namespace TofComm;
using namespace std::chrono_literals;
using namespace std::chrono;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

static bool doEnableDisable { false };
static bool enableEdge { false };

static bool doAutoStart { false };
static bool autoStart   { false };

static bool captureRefFrame { false };
static bool eraseRefFrame   { false };
static bool getVolume       { false };
static std::string jsonParam{ };
static bool readParams      { false };
static bool storeRefFrame   { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc(
                "Edge example for volume edge application\n\n"
                "  Usage: tof-edge [options]\n\n"
                "  EXAMPLES:\n"
                "    Disable edge app:                 tof-edge -e 0\n"
                "    Enable edge app:                  tof-edge -e 1\n"
                "    Configure edge app to auto-start: tof-edge -a 1\n"
                "    Get volume:                       tof-edge -g\n"
                "    Erase reference frame:            tof-edge -E\n"
                "    Capture new reference frame:      tof-edge -c\n"
                "    Store reference frame in NOR:     tof-edge -s\n"
                "    Read volume algo. parameters:     tof-edge -r\n"
                "    Modify HDR integ. time params:    tof-edge -w '{\"integTime_us\":[500,2500]}'\n\n"
                "  NOTE: To see list of parameters and their formats, study the output of the -r option.\n\n"
                );
    desc.add_options()
        ("auto-start,a",    po::value<bool>(&autoStart), "Enable/disable auto-start of edge application")
        ("baud-rate,b",     po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE), "Set baud rate")
        ("capture,c",                                       "Capture reference frame")
        ("debug,G", new     CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p",    po::value<std::string>(&devicePort),                    "Specify sensor address")
        ("erase,E",                                         "Erase reference frame")
        ("enable-edge,e",   po::value<bool>(&enableEdge),   "Enable/disable edge application")
        ("get-volume,g",                                    "Get volume results")
        ("help,h",                                          "produce help message")
        ("quiet,q",         po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("read-params,r",                                   "Read volume algorithm parameters")
        ("store,s",                                         "Store reference frame to NOR")
        ("write,w",         po::value<std::string>(&jsonParam), "Write JSON parameter setting")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
        dbg_out << desc << "\n";
        exit(0);
    }

    captureRefFrame = (vm.count("capture") != 0);
    eraseRefFrame = (vm.count("erase") != 0);
    getVolume = (vm.count("get-volume") != 0);
    readParams = (vm.count("read-params") != 0);
    storeRefFrame = (vm.count("store") != 0);

    if (vm.count("enable-edge"))
    {
        doEnableDisable = true;
    }

    if (vm.count("auto-start"))
    {
        doAutoStart = true;
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

        if (captureRefFrame || eraseRefFrame || storeRefFrame)
        {
            constexpr uint8_t ERASE_REF_FRAME   { 0x01 };
            constexpr uint8_t CAPTURE_REF_FRAME { 0x02 };
            constexpr uint8_t STORE_REF_FRAME   { 0x04 };
            uint8_t cmdFlags = 0;
            std::string description { "" };
            if (eraseRefFrame)
            {
                cmdFlags |= ERASE_REF_FRAME;
                description.append(" Erase");
            }
            if (captureRefFrame)
            {
                cmdFlags |= CAPTURE_REF_FRAME;
                description.append(" Capture");
            }
            if (storeRefFrame)
            {
                cmdFlags |= STORE_REF_FRAME;
                description.append(" Store");
            }
            constexpr uint16_t CONFIG_VOLUME_REF_FRAME { 0x1005 };
            std::byte cmdByte = (std::byte)cmdFlags;
            if (!sensor.send_receive(CONFIG_VOLUME_REF_FRAME, Sensor::send_receive_payload_t(&cmdByte, 1)))
            {
                err_out << "Failed to configure reference frame\n";
            }
            else
            {
                dbg_out << "Reference frame configuration: " << description.c_str() << "\n";
            }
        }

        if (doAutoStart)
        {
            constexpr uint16_t DISABLE_AUTO_START { 0x1002 };
            constexpr uint16_t ENABLE_AUTO_START  { 0x1003 };
            uint16_t command = (autoStart ? ENABLE_AUTO_START : DISABLE_AUTO_START);
            if (!sensor.send_receive(command, Sensor::send_receive_payload_t { }))
            {
                err_out << "Failed to enable/disable auto-start of edge application\n";
            }
            else
            {
                dbg_out << "Edge Application AUTO-START " << (autoStart ? "ENABLED\n" : "DISABLED\n");
            }
        }

        if (doEnableDisable)
        {
            constexpr uint16_t EDGE_APP_DISABLE { 0x1000 };
            constexpr uint16_t EDGE_APP_ENABLE  { 0x1001 };
            uint16_t command = (enableEdge ? EDGE_APP_ENABLE : EDGE_APP_DISABLE);
            if (!sensor.send_receive(command, Sensor::send_receive_payload_t { }))
            {
                err_out << "Failed to enable/disable edge application\n";
            }
            else
            {
                dbg_out << "Edge application " << (enableEdge ? "ENABLED\n" : "DISABLED\n");
            }
        }

        if (getVolume)
        {
            constexpr uint16_t GET_VOLUME_INFO { 0x1004 };
            auto volumeData = sensor.send_receive(GET_VOLUME_INFO, Sensor::send_receive_payload_t { });

             if (volumeData)
             {
                 dbg_out << "Received " << volumeData->size() <<" bytes of volume data from sensor\n";
                 auto data = *volumeData;
                 uint32_t scaledResult = ((uint32_t)data[1] << 24) +
                                         ((uint32_t)data[2] << 16) +
                                         ((uint32_t)data[3] << 8)  + (uint32_t)data[4];
                 dbg_out << "Scaled result: " << scaledResult << "\n";
             }
             else
             {
                 err_out << "ERROR: No valid Volume data received\n";
             }
        }

        if (readParams)
        {
            constexpr uint16_t GET_VOLUME_PARAMETERS { 0x1006 };
            auto volumeParms = sensor.send_receive(GET_VOLUME_PARAMETERS, Sensor::send_receive_payload_t { });
            if (volumeParms)
            {
                const auto& answer = *volumeParms;
                const auto size = answer.size();

                std::string paramString = std::string(reinterpret_cast<const char*>(answer.data()+1), size-1);

                dbg_out << "Volume parameters: '" << paramString.c_str() << "'\n";
            }
            else
            {
                err_out << "ERROR: Failed to read volume parameters\n";
            }
        }

        if (jsonParam.size() > 0)
        {
            constexpr uint16_t SET_VOLUME_PARAMETERS { 0x1007 };

            std::vector<std::byte> commandPayload(jsonParam.size() + 1);
            std::transform(jsonParam.begin(),
                           jsonParam.end(),
                           commandPayload.begin(),
                           [] (char c) { return std::byte(c); });

            if (!sensor.send_receive(SET_VOLUME_PARAMETERS,
                                     Sensor::send_receive_payload_t(commandPayload.data(), commandPayload.size()) ))
            {
                err_out << "Setting volume parameter FAILED: '" << jsonParam.c_str() << "'\n";
            }
            else
            {
                dbg_out << "Setting volume parameter SUCCEEDED: '" << jsonParam.c_str() << "'\n";
            }
       }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
