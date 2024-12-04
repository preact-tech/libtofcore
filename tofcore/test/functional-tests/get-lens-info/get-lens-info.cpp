/**
 * @file get-lens-info.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that libtofcore to read Lens information.
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <boost/program_options.hpp>

using namespace test;
using namespace tofcore;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Get Lens Information Test");
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("help,h", "produce help message")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
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

        auto lensInfo = sensor.getLensIntrinsics();
        if (lensInfo)
        {
            dbg_out << "rowOffset="           << lensInfo->m_rowOffset
                    << ", columnOffset="      << lensInfo->m_columnOffset
                    << ", rowFocalLength="    << lensInfo->m_rowFocalLength
                    << ", columnFocalLength=" << lensInfo->m_columnFocalLength
                    << ", undistortionCoeff=[";
            size_t n;
            for (n = 0; n < 4; ++n)
            {
                dbg_out << lensInfo->m_undistortionCoeffs[n] << ", ";
            }
            dbg_out << lensInfo->m_undistortionCoeffs[n] << "]";
           
            dbg_out << ", hfov=" << lensInfo->m_hfov << ", vfov=" << lensInfo->m_vfov << "\n";
        }
        else
        {
            err_out << "Unable to read Lens Info data\n";
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
