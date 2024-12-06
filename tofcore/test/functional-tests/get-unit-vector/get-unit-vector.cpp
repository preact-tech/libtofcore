/**
 * @file get-unit-vector.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that libtofcore to read Lens unit vector.
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iomanip>
#include <boost/program_options.hpp>

using namespace test;
using namespace tofcore;

inline constexpr unsigned NUM_ROWS { 240 };
inline constexpr unsigned NUM_COLS { 320 };

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
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
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

static void printRay(const std::vector<double>& ray, const char* rayName)
{
    dbg_out << "double " << rayName << "[" << NUM_ROWS << "][" << NUM_COLS << "] =\n{\n";
    for (unsigned row = 0; row < NUM_ROWS; ++row)
    {
        dbg_out << "  /* ROW: " << std::setw(3) << std::setfill('0') << row << " */ {";
        for (unsigned col = 0; col < NUM_COLS; ++col)
        {
            dbg_out << ray[row];
            if (col != (NUM_COLS - 1))
            {
                dbg_out << ", ";
            }
        }
        dbg_out << "}";
        if (row != (NUM_ROWS - 1))
        {
            dbg_out << ",";
        }
        dbg_out << "\n";
    }
    dbg_out << "};\n";

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

        std::vector<double> rays_x;
        std::vector<double> rays_y;
        std::vector<double> rays_z;
        if (sensor.getLensInfo(rays_x, rays_y, rays_z))
        {
            printRay(rays_x, "rays_x");
            printRay(rays_y, "rays_y");
            printRay(rays_z, "rays_z");
        }
        else
        {
            err_out << "Unable to read unit vector data\n";
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
