/**
 * @file tof-vsm.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that sets/gets IPv4 settings
 */
#include "tofcore/tof_sensor.hpp"
#include <array>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <boost/program_options.hpp>

using namespace tofcore;
using namespace TofComm;
using namespace std::chrono_literals;
using namespace std::chrono;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };

static uint16_t integrationUsStart { 0 };
static uint16_t integrationUsIncrement { 0 };
static uint16_t mfKhzStart { 0 };
static uint16_t mfKhzIncrement { 0 };
static int32_t numElements { -1 };


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
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("numElements,n", po::value<int32_t>(&numElements), "Specify number of VSM elements")
        ("integrationUsStart,i", po::value<uint16_t>(&integrationUsStart), "Starting integration time value")
        ("integrationUsIncrement,j", po::value<uint16_t>(&integrationUsIncrement), "Integration time increment")
        ("mfKhzStart,f", po::value<uint16_t>(&mfKhzStart), "Starting modulation frequency value")
        ("mfKhzIncrement,m", po::value<uint16_t>(&mfKhzIncrement), "Modulation frequency increment")
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
        tofcore::Sensor sensor { devicePort, baudRate };
        // Set the IPv4 values
        if (numElements > (int32_t)VSM_MAX_NUMBER_OF_ELEMENTS)
        {
            std::cerr << "ERROR: Number of elements must be <= " << VSM_MAX_NUMBER_OF_ELEMENTS << std::endl;
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
            std::cout << "Skipping setting of VSM since -n argument not used" << std::endl;
        }

        auto result = sensor.getVsmSettings();

        if (result)
        {
            const VsmControl_T& vsm = *result;
            std::cout << "VSM Flags: " << vsm.m_vsmFlags << std::endl;
            std::cout << "    N    : " << (unsigned)vsm.m_numberOfElements << std::endl;
            std::cout << "    I    : " << (unsigned)vsm.m_vsmIndex << std::endl;
            for (size_t n = 0; n < vsm.m_numberOfElements; ++n)
            {
                std::cout << "         [" << n << "]  { ";
                std::cout << vsm.m_elements[n].m_integrationTimeUs << ", "
                          << vsm.m_elements[n].m_modulationFreqKhz << "}" << std::endl;
            }
        }
        else
        {
            std::cout << "FAILED read VSM settings" << std::endl;
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
