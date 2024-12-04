/**
 * @file rapid-changes.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program to make sensor changes as rapidly as possible.
 *
 * See MOS-426
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>

using namespace test;
using namespace tofcore;

static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static uint32_t repetitions { 100000 };

static volatile bool exitRequested { false };

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Test sensor with rapid changes to settings");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("help,h", "produce help message")
        ("repeat,r", po::value<uint32_t>(&repetitions)->default_value(100000))
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
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
    #if defined(SIGQUUIT)
    signal(SIGQUIT, signalHandler);
    #endif
    {
        tofcore::Sensor sensor { devicePort, baudRate };
        sensor.setDebugLevel(debugLevel);

        uint32_t setCount { 0 };
        uint32_t integrationOffset { 0 };

        for (uint32_t i = 0; i < repetitions; ++i)
        {
            ++setCount;
            if (!sensor.setIntegrationTime(100 + integrationOffset))
            {
                err_out << "setIntegration() FAILED on " << setCount << "th attempt\n";
                break;
            }
            integrationOffset = (integrationOffset + 1) % 100;
        }
        exit(0);

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
