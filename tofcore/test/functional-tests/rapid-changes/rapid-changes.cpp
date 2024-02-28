/**
 * @file rapid-changes.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program to make sensor changes as rapidly as possible.
 *
 * See MOS-426
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static uint32_t repetitions { 100000 };

static volatile bool exitRequested { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Test sensor with rapid changes to settings");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
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
        uint32_t setCount { 0 };
        uint32_t integrationOffset { 0 };

        for (uint32_t i = 0; i < repetitions; ++i)
        {
            ++setCount;
            if (!sensor.setIntegrationTime(100 + integrationOffset))
            {
                std::cerr << "setIntegration() FAILED on " << setCount << "th attempt" << std::endl;
                break;
            }
            integrationOffset = (integrationOffset + 1) % 100;
        }
        exit(0);

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
