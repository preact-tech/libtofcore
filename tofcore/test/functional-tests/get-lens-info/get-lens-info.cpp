/**
 * @file get-lens-info.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that libtofcore to read Lens information.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <iostream>
#include <boost/program_options.hpp>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { 1 };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("illuminator board test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("protocol-version,v", po::value<uint16_t>(&protocolVersion)->default_value(DEFAULT_PROTOCOL_VERSION))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
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
#if defined(SIGQUIT)
    signal(SIGQUIT, signalHandler);
#endif
    {
        tofcore::Sensor sensor { protocolVersion, devicePort, baudRate };

        auto lensInfo = sensor.getLensIntrinsics();
        if (lensInfo)
        {
            std::cout << "rowOffset="           << lensInfo->m_rowOffset
                      << ", columnOffset="      << lensInfo->m_columnOffset
                      << ", rowFocalLength="    << lensInfo->m_rowFocalLength
                      << ", columnFocalLength=" << lensInfo->m_columnFocalLength
                      << ", undistortionCoeff=[";
            size_t n;
            for (n = 0; n < 4; ++n)
            {
                std::cout << lensInfo->m_undistortionCoeffs[n] << ", ";
            }
            std::cout << lensInfo->m_undistortionCoeffs[n] << "]" << std::endl;
        }
        else
        {
            std::cerr << "Unable to read Lens Info data" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
