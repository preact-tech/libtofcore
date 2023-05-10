/**
 * @file simple-streamer.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to stream DCS or DCS+Ambient data
 */
#include "tofcore/tof_sensor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool captureAmxxx { false };
static bool captureDistance { false };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static uint16_t protocolVersion { DEFAULT_PROTOCOL_VERSION };
static uint32_t verbosity { 0 };

static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> distanceCount;
static std::atomic<uint32_t> grayscaleCount;


static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{
    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
        case DataType::DISTANCE_AMPLITUDE:
            ++amplitudeCount;
            ++distanceCount;
            if (verbosity > 0)
            {
                std::cout << "received DISTANCE-AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::DCS:
            ++dcsCount;
            if (verbosity > 0)
            {
                std::cout << "received DCS measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::GRAYSCALE:
            ++grayscaleCount;
            if (verbosity > 0)
            {
                std::cout << "received GRAYSCALE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            if (verbosity > 0)
            {
                std::cout << "received DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            if (verbosity > 0)
            {
                std::cout << "received AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::AMBIENT:
            ++amplitudeCount;
            if (verbosity > 0)
            {
                std::cout << "received AMBIENT measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        
        default:
            std::cout << "UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << std::endl;
    }
    if(verbosity > 0)
    {
        auto chip_temps = pData->sensor_temperatures();
        if(chip_temps) 
        {
            std::cout << "Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << std::endl;
        } 
        else 
        {
            std::cout << "No sensor temperature data" << std::endl;
        }
        auto integration_times = pData->integration_times();
        if(integration_times)
        {
            std::cout << "Integration time settings (ms): "; 
            for(auto& v : *integration_times)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
        else 
        {
            std::cout << "No integration time data" << std::endl;
        }
        auto mod_frequencies = pData->modulation_frequencies();
        if(mod_frequencies)
        {
            std::cout << "Modulation Frequency settings (Hz): ";
            for(auto& v : *mod_frequencies)
            {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
        else 
        {
            std::cout << "No modulation frequency data" << std::endl;
        }
        auto v_binning = pData->vertical_binning();
        auto h_binning = pData->horizontal_binning();
        if(v_binning && h_binning)
        {
            std::cout << "Binning settings: " << (int)(*h_binning) << " " << (int)(*v_binning) << std::endl;
        }
        else 
        {
            std::cout << "No binning data" << std::endl;
        }

        auto dll_settings = pData->dll_settings();
        if(dll_settings)
        {
            std::cout << "DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ") << (int)(*dll_settings)[1] << " " <<  (int)(*dll_settings)[2] << " " <<  (int)(*dll_settings)[3] << std::endl;
        }
        else 
        {
            std::cout << "No DLL settings" << std::endl;
        }
    }
}

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "ab:dhp:v:V")) != -1)
    {
        switch (opt)
        {
            case 'a':
                 captureAmxxx = true;
                 break;
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'd':
                 captureDistance = true;
                 break;
            case 'h':
                std::cout   << "Stream data from T10 and report message sizes." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-a] [-b <baud>] [-h] [-p <port>] [-v <ver>]" << std::endl
                            << "  -a        Capture DCS+Ambient or Distance Amplitude frames, (not just DCS or Distance)" << std::endl
                            << "  -b <baud> Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -d        Capture distance (or amplitude) frames instead of DCS frames" << std::endl
                            << "  -h        Print help and exit" << std::endl
                            << "  -p <port> Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -v <ver>      Use version <ver> of the command protocol. Default = " << DEFAULT_PROTOCOL_VERSION << std::endl
                            << "  -V        Increase verbosity of output." << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'p':
                 devicePort = optarg;
                 break;
            case 'v':
                protocolVersion = atoi(optarg);
                break;
            case 'V':
                ++verbosity;
                break;
            default:
                break;
        }
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
    signal(SIGQUIT, signalHandler);
    {
        tofcore::Sensor sensor { protocolVersion, devicePort, baudRate };
        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        if (captureDistance)
        {
            if (captureAmxxx)
            {
                sensor.streamDistanceAmplitude();
            }
            else
            {
                sensor.streamDistance();
            }
        }
        else
        {
            if (captureAmxxx)
            {
                sensor.streamDCSAmbient();
            }
            else
            {
                sensor.streamDCS();
            }
        }
        auto lastTime = steady_clock::now();
        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_until(lastTime + 1000ms);
            lastTime = steady_clock::now();
            const uint32_t amplitude { amplitudeCount };
            amplitudeCount = 0;
            const uint32_t dcs { 4 * dcsCount };
            dcsCount = 0;
            const uint32_t distance { distanceCount };
            distanceCount = 0;
            const uint32_t grayscale { grayscaleCount };
            grayscaleCount = 0;
            std::cout << "RAW FPS: amplitude = " << amplitude << "; dcs = " << dcs
                      << "; distance = " << distance << "; grayscale = " << grayscale
                      << "; total = " << (amplitude + dcs + distance + grayscale) << std::endl;
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
