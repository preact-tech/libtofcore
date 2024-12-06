/**
 * @file jmp-to-boot.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Program to command sensor to jump to the bootloader or reset
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>

using namespace test;
using namespace tofcore;

constexpr uint16_t FALLBACK_LOADER_TOKEN { 0x04a5 };

static DebugOutput dbg_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static std::string tokenStr {};
static uint16_t token = 0;

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Command sensor to jump to bootloader");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("fallback-loader,f", "Jump to fallback loader to perform update of the primary loader")
        ("help,h", "produce help message")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("token,t", po::value<std::string>(&tokenStr), "Pass 16bit token value with command to perform special boot-loader operations")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        dbg_out << desc << "\n";
        exit(0);
    }

    if (vm.count("fallback-loader"))
    {
        token = FALLBACK_LOADER_TOKEN;
    }

    if (vm.count("token") != 0) 
    {
        //use stoi() with 0 for the radix (3rd) parameter to allow automatic conversion of hex or
        // decimal inputs.
        token = std::stoi(tokenStr, 0, 0); 
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

        if(0 == token)
        {
            dbg_out << "Requesting sensor jump to bootloader with default token\n";
            sensor.jumpToBootloader();
        }
        else
        {
            if (FALLBACK_LOADER_TOKEN == token)
            {
                dbg_out << "Requesting sensor jump to FALLBACK bootloader (token 0x" << std::hex
                          << std::setw(4) << std::setfill('0') << token << ")\n";
            }
            else
            {
                dbg_out << "Requesting sensor jump to bootloader (token 0x" << std::hex
                          << std::setw(4) << std::setfill('0') << token << ")\n";
            }
            sensor.jumpToBootloader(token);
        }
        exit(0);

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
