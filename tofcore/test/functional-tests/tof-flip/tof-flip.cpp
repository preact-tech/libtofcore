/**
 * @file tof-flip.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to get/set horizontal/vertical image flip.
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>

using namespace test;
using namespace tofcore;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t flipHorizontal { 0 };
static uint16_t flipVertical { 0 };
static bool setHorizontal { false };
static bool setVertical { false };
static bool storeSettings { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Set/Get horizontal/vertical flip state.");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("help,h", "produce help message")
        ("horizontal-flip,H", po::value<uint16_t>(&flipHorizontal), "Turn horizontal flip off/on")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("store-settings,s", po::bool_switch(&storeSettings), "Store settings in persistent memory")
        ("vertical-flip,V", po::value<uint16_t>(&flipVertical), "turn vertical flip off/on")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        dbg_out << desc << "\n";
        exit(0);
    }

    if (vm.count("horizontal-flip"))
    {
        setHorizontal = true;
    }
    if (vm.count("vertical-flip"))
    {
        setVertical = true;
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

        if (setHorizontal)
        {
            const bool flip { flipHorizontal != 0 };
            if (sensor.setFlipHorizontally(flip))
            {
                dbg_out << "Horizontal flip commanded " << (flip ? "ON" : "OFF") << "\n";
            }
            else
            {
                err_out << "FAILED to command horizontal flip state\n";
            }
        }

        if (setVertical)
        {
            const bool flip { flipVertical != 0 };
            if (sensor.setFlipVertically(flip))
            {
                dbg_out << "Vertical flip commanded " << (flip ? "ON" : "OFF") << "\n";
            }
            else
            {
                err_out << "FAILED to command vertical flip state\n";
            }
        }

        auto hResult = sensor.isFlipHorizontallyActive();
        auto vResult = sensor.isFlipVerticallyActive();

        const char* hDescr = (hResult) ? (*hResult ? "ON" : "OFF") : "??";
        const char* vDescr = (vResult) ? (*vResult ? "ON" : "OFF") : "??";

        dbg_out << "Horizontal Flip: " << hDescr << "; Vertical Flip: " << vDescr << "\n";

        if (storeSettings)
        {
            if (sensor.storeSettings())
            {
                dbg_out << "Settings stored in persistent memory.\n";
            }
            else
            {
                err_out << "Failed to store settings in persistent memory\n";
            }
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
