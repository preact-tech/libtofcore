/**
 * @file tof-vsm.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that sets/gets IPv4 settings
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <array>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <thread>

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

static uint16_t integrationUsStart { 0 };
static uint16_t integrationUsIncrement { 0 };
static uint16_t mfKhzStart { 0 };
static uint16_t mfKhzIncrement { 0 };
static int32_t numElements { -1 };
static bool enableHdr {false};

static void parseArgs(int argc, char *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc(
                "VSM (Vector Sequence Mode) read/write utility\n\n"
                "  Usage: [options] [-n Number in sequence]\n"
                "                   [-i Start integration time in uS]\n"
                "                   [-j Integration time increment in uS]\n"
                "                   [-f Start modulation frequency in Kh]\n"
                "                   [-m Modulation frequency increment in Khz]\n\n"
                "  EXAMPLES:\n"
                "    1) To sequence through 8 integration times starting at 100 uS, with 200 uS steps:\n\n"
                "         tof-vsm -n 8 -i 100 -j 200\n\n"
                "    2) To cancel VSM mode:\n\n"
                "         tof-vsm -n 0\n\n"
                "    3) To read current VSM settings without making any changes:\n\n"
                "         tof-vsm\n\n"
                "  NOTE:\n"
                "    The VSM data is always read whether any modifications are made\n"
                );
    desc.add_options()
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("enable-hdr,e", po::value<bool>(&enableHdr), "Enable or disable HDR.")
        ("help,h", "produce help message")
        ("integrationUsIncrement,j", po::value<uint16_t>(&integrationUsIncrement), "Integration time increment")
        ("integrationUsStart,i", po::value<uint16_t>(&integrationUsStart), "Starting integration time value")
        ("mfKhzIncrement,m", po::value<uint16_t>(&mfKhzIncrement), "Modulation frequency increment")
        ("mfKhzStart,f", po::value<uint16_t>(&mfKhzStart), "Starting modulation frequency value")
        ("numElements,n", po::value<int32_t>(&numElements), "Specify number of VSM elements")
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

        // Set the IPv4 values
        if (numElements > (int32_t)VSM_MAX_NUMBER_OF_ELEMENTS)
        {
            err_out << "ERROR: Number of elements must be <= " << VSM_MAX_NUMBER_OF_ELEMENTS << "\n";
            return -1;
        }
        else if (numElements >= 0)
        {
            VsmControl_T vsmControl { };
            vsmControl.m_numberOfElements = numElements;
            uint16_t integrationTimeUs = integrationUsStart;
            uint16_t modulationFreqKhz = mfKhzStart;
            for (int32_t n = 0; n < numElements; ++n)
            {
                vsmControl.m_elements[n].m_integrationTimeUs = integrationTimeUs;
                vsmControl.m_elements[n].m_modulationFreqKhz = modulationFreqKhz;
                integrationTimeUs += integrationUsIncrement;
                modulationFreqKhz += mfKhzIncrement;
            }
            sensor.setVsm(vsmControl);
        }
        else
        {
            dbg_out << "Skipping setting of VSM since -n argument not used\n";
        }

        if(enableHdr)
        {
            dbg_out << "Enabling Temporal HDR.\n";
            sensor.setHdr(true);
        }
        else
        {
            dbg_out << "Disabling Temporal HDR.\n";
            sensor.setHdr(false);
        }

        auto result = sensor.getVsmSettings();

        if (result)
        {
            const VsmControl_T& vsm = *result;
            dbg_out << "VSM Flags: " << vsm.m_vsmFlags << "\n";
            dbg_out << "    N    : " << (unsigned)vsm.m_numberOfElements << "\n";
            dbg_out << "    I    : " << (unsigned)vsm.m_vsmIndex << "\n";
            for (size_t n = 0; n < vsm.m_numberOfElements; ++n)
            {
                dbg_out << "         [" << n << "]  { ";
                dbg_out << vsm.m_elements[n].m_integrationTimeUs << ", "
                        << vsm.m_elements[n].m_modulationFreqKhz << "}\n";
            }
        }
        else
        {
            dbg_out << "FAILED read VSM settings\n";
        }

        dbg_out << "\n";
        auto hdrSettings = sensor.getHdrSettings();
        if(hdrSettings)
        {
            const HdrSettings_T& settings = *hdrSettings;
            dbg_out << "HDR Settings\n";
            dbg_out << "Enabled: " << (settings.enabled ? "True" : "False") << "\n";
            dbg_out << "Mode: " << (settings.mode==HdrMode_e::SPATIAL ? "Spatial" : "Temporal") << "\n";
        }
        else
        {
            dbg_out << "FAILED read HDR settings\n";
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
