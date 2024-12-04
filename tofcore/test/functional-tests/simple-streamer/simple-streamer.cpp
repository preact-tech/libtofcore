/**
 * @file simple-streamer.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to stream data
 */
#include "dbg_out.hpp"
#include "po_count.hpp"
#include "tofcore/tof_sensor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <thread>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace test;
using namespace tofcore;
using namespace TofComm;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool captureAmbient { false };
static bool captureAmplitude { false };
static bool captureDcsDiff { false };
static bool captureDistance { false };
static uint32_t crcState { 0 };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool enableBinning { false };
static volatile bool exitRequested { false };
static uint32_t verbosity { 0 };
static uint16_t modulation { 0 };
static uint16_t integration_time { 0 };
static bool setFramePeriod { false };
static uint32_t framePeriodMs { 1000 };

static bool setSortingState { false };
static bool sortRawData { true };

static std::atomic<uint32_t> ambientCount;
static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> dcsDiffCount;
static std::atomic<uint32_t> distanceCount;


static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{
    static high_resolution_clock::time_point lastTime { high_resolution_clock::now() };
    high_resolution_clock::time_point timeNow = high_resolution_clock::now();

    duration<double> framePeriod { duration_cast<duration<double>>(timeNow - lastTime) };
    lastTime = timeNow;

    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
        case DataType::DISTANCE_AMPLITUDE:
            ++amplitudeCount;
            ++distanceCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] DISTANCE-AMPLITUDE measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
                auto distanceData = pData->distance();
                uint32_t adcErrors { 0 };
                uint32_t amplitudeErrors { 0 };
                uint32_t goodPixels { 0 };
                uint32_t ignoredPixels { 0 };
                uint32_t pixelErrors { 0 };
                uint32_t saturatedPixels { 0 };
                for (auto p = distanceData.begin(); p != distanceData.end(); ++p)
                {
                    auto distance = *p;
                    if ((distance & 0x0001) != 0)
                    {
                        ++pixelErrors;
                        auto errCode = (distance & 0x000E);
                        if (errCode == 0x0002)
                        {
                            ++saturatedPixels;
                        }
                        else if (errCode == 0x0004)
                        {
                            ++ignoredPixels;
                        }
                        else if (errCode == 0x0008)
                        {
                            ++amplitudeErrors;
                        }
                        else
                        {
                            ++adcErrors;
                        }
                    }
                    else
                    {
                        ++goodPixels;
                    }
                }
                dbg_out << "\tGood pixels: " << goodPixels << "; Errors: " << pixelErrors
                        << "; amplitude: " << amplitudeErrors << "; saturated: " << saturatedPixels
                        << "; ADC: " << adcErrors << "\n";

            }
            break;
        case DataType::DCS:
            ++dcsCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] DCS measurement data, packet size "
                        << pData->pixel_buffer().size() << "\n";
            }
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] DISTANCE measurement data, packet size "
                        << pData->pixel_buffer().size() << "\n";
            }
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] AMPLITUDE measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;
        case DataType::AMBIENT:
        case DataType::GRAYSCALE:
            ++ambientCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] AMBIENT measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;
        case DataType::DCS_DIFF_AMBIENT:
            ++ambientCount;
            ++dcsDiffCount;
            if (verbosity > 0)
            {
                dbg_out << "[" << framePeriod.count() << "] DCS_DIFF+AMBIENT measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;

        default:
            dbg_out << "[" << framePeriod.count() << "] UNRECOGNIZED data type: "
                    << static_cast<int16_t>(pData->type()) << "\n";
    }
    if(verbosity > 0)
    {
        auto chip_temps = pData->sensor_temperatures();
        if(chip_temps) 
        {
            dbg_out << "  Sensor temperatures: " << (*chip_temps)[0] << ", "
                    << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << "\n";
        } 
        else 
        {
            dbg_out << "  No sensor temperature data\n";
        }
        auto integration_time = pData->integration_time();
        if(integration_time)
        {
            dbg_out << "  Integration time setting (uS): " << *integration_time << "\n";
        }
        else 
        {
            dbg_out << "  No integration time data\n";
        }
        auto mod_frequency = pData->modulation_frequency();
        if(mod_frequency)
        {
            dbg_out << "  Modulation Frequency setting (Hz): " << *mod_frequency << "\n";
        }
        else 
        {
            dbg_out << "  No modulation frequency data\n";
        }
        auto v_binning = pData->vertical_binning();
        auto h_binning = pData->horizontal_binning();
        if(v_binning && h_binning)
        {
            dbg_out << "  Binning settings: " << (int)(*h_binning) << " " << (int)(*v_binning) << "\n";
        }
        else 
        {
            dbg_out << "  No binning data\n";
        }

        auto dll_settings = pData->dll_settings();
        if(dll_settings)
        {
            dbg_out << "  DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ")
                    << (int)(*dll_settings)[1] << " " <<  (int)(*dll_settings)[2] << " " <<  (int)(*dll_settings)[3] << "\n";
        }
        else 
        {
            dbg_out << "  No DLL settings\n";
        }
        auto illum = pData->illuminator_info();
        if(illum)
        {
            const auto& illum_info = *illum;
            dbg_out << "  Illuminator info: 0x" << std::hex
                    << (int)illum_info.led_segments_enabled << std::dec << " "
                    << illum_info.temperature_c << "C "
                    << illum_info.vled_v << "V "
                    << illum_info.photodiode_v << "V\n";
        }
        else
        {
            dbg_out << "  No Illuminator information\n";
        }

        auto vsmControl = pData->vsm_info();
        if(vsmControl)
        {
            dbg_out << "  VSM: Flags=" << vsmControl->m_vsmFlags << "; N = "
                    << (unsigned)vsmControl->m_numberOfElements  << "; I = "
                    << (unsigned)vsmControl->m_vsmIndex << ";";
            uint8_t numElements = std::min(vsmControl->m_numberOfElements, (uint8_t) VSM_MAX_NUMBER_OF_ELEMENTS);
            for (decltype(numElements) n = 0; n < numElements; ++n)
            {
                VsmElement_T& element = vsmControl->m_elements[n];
                dbg_out << " [" << element.m_integrationTimeUs << ", "
                        << element.m_modulationFreqKhz << "]";
            }
            dbg_out << "\n";
        }
        else
        {
            dbg_out << "  No VSM data\n";
        }

        auto timestamp = pData->frame_timestamp();
        if(timestamp)
        {
            dbg_out << "  Frame timestamp: " << (int) *timestamp << "\n";
        }
        else
        {
            dbg_out << "  No timestamp found in frame data\n";
        }

        dbg_out << "  Sorted: " << pData->is_raw_data_sorted()
                << "; HFlip: " << pData->is_flipped_horizontally()
                << "; VFlip: " << pData->is_flipped_vertically() << "\n";

        dbg_out << "\n\n";
    }
}

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Simple Streamer Test");
    desc.add_options()
        ("ambient,A", po::bool_switch(&captureAmbient), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("amplitude,a", po::bool_switch(&captureAmplitude), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("Binning,B", po::bool_switch(&enableBinning)->default_value(false),"Enable full binning")
        ("crc-state,C", new CountValue(&crcState), "Increase frame CRC state")
        ("Dcs-diff,D", po::bool_switch(&captureDcsDiff),  "Capture DCS_DIFF + Ambient frames")
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("distance,d", po::bool_switch(&captureDistance),  "Capture distance (or amplitude) frames instead of DCS frames")
        ("frame-period,f", po::value<uint32_t>(&framePeriodMs),"Set target frame period (mS)")
        ("help,h", "produce help message")
        ("integration,i", po::value<uint16_t>(&integration_time)->default_value(0),"Set integration time to this value (uS)")
        ("modulation,m", po::value<uint16_t>(&modulation)->default_value(0),"Set modulation frequency to this value (kHz)")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("sort,s", po::value<bool>(&sortRawData), "Sort raw data")
        ("verbose,V", new  CountValue(&verbosity), "Increase verbosity of output")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("sort"))
    {
        setSortingState = true;
    }
    if (vm.count("frame-period"))
    {
        setFramePeriod = true;
    }
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
    try
    {
        parseArgs(argc, argv);
    }
    catch (po::error &x)
    {
        err_out << x.what() << "\n";
        return 1;
    }
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

        if (sensor.setFrameCrcState(crcState))
        {
            dbg_out << "Frame CRC state set to " << crcState << "\n";
        }
        else
        {
            err_out << "FAILED to set frame CRC state set to " << crcState << "\n";
        }

        if (setSortingState)
        {
            sensor.sortRawData(sortRawData);
            dbg_out << "Configured RAW data sorting: " << (sortRawData ? "ON" : "OFF") << "\n";
        }
        else
        {
            auto sortState = sensor.isRawDataSorted();
            if (sortState)
            {
                dbg_out << "RAW data sorting: " << (*sortState ? "ON" : "OFF") << "\n";
            }
            else
            {
                err_out << "Unable to get state of RAW data sorting\n";
            }
        }

        if (enableBinning)
        {
            sensor.setBinning(true);
        }
        else
        {
            sensor.setBinning(false);
        }
        if(modulation)
        {
            sensor.setModulation(modulation);
        }

        if(integration_time)
        {
            sensor.setIntegrationTime(integration_time);
        }

        if (setFramePeriod)
        {
            if (!sensor.setFramePeriodMs(framePeriodMs))
            {
                err_out << "FAILED to set frame period to " << framePeriodMs << " mS\n";
            }
        }

        auto framePeriodData = sensor.getFramePeriodMsAndLimits();
        if (framePeriodData)
        {
            auto [framePeriodMs, framePeriodMsMin, framePeriodMsMax] = *framePeriodData;
            dbg_out << "Frame Period: " << framePeriodMs << " mS; Min: " << framePeriodMsMin
                    << " mS; Max: " << framePeriodMsMax << " mS\n";
        }
        else
        {
            err_out << "FAILED to get frame period data\n";
        }

        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        uint32_t rawFramesPerRangeFrame { 1 };
        if (captureDcsDiff)
        {
            sensor.streamDCSDiffAmbient();
            rawFramesPerRangeFrame = 3;
        }
        else if (captureDistance)
        {
            if (captureAmbient || captureAmplitude)
            {
                sensor.streamDistanceAmplitude();
                rawFramesPerRangeFrame = 2;
            }
            else
            {
                sensor.streamDistance();
                rawFramesPerRangeFrame = 1;
            }
        }
        else
        {
            if (captureAmbient || captureAmplitude)
            {
                sensor.streamDCSAmbient();
                rawFramesPerRangeFrame = 5;
            }
            else
            {
                sensor.streamDCS();
                rawFramesPerRangeFrame = 4;
            }
        }

        auto startTime { high_resolution_clock::now() };
        auto lastTime { startTime };
        uint32_t outputCount = 1;
        uint32_t totalFrames { 0 };
        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_until(lastTime + 1000ms);
            lastTime = high_resolution_clock::now();
            duration<double> totalTime { duration_cast<duration<double>>(lastTime - startTime) };
            const uint32_t ambient { ambientCount };
            ambientCount = 0;
            const uint32_t amplitude { amplitudeCount };
            amplitudeCount = 0;
            const uint32_t dcs { 4 * dcsCount };
            dcsCount = 0;
            const uint32_t dcsDiff { 2 * dcsDiffCount };
            dcsDiffCount = 0;
            const uint32_t distance { distanceCount };
            distanceCount = 0;
            totalFrames += (ambient + amplitude + dcs + dcsDiff + distance);
            dbg_out << "[" << std::setw(5) << std::setfill('0') << outputCount
                    <<"] Frames: ambient = " << std::setw(2) << ambient
                    << "; amplitude = " << std::setw(2) << amplitude
                    << "; dcs = " << std::setw(2) << dcs
                    << "; dcsDiff = " << std::setw(2) << dcsDiff
                    << "; distance = " << std::setw(2) << distance
                    << "; total = " << std::setw(3) << (ambient + amplitude + dcs + dcsDiff + distance)
                    << "; FPS: " << (totalFrames / totalTime.count() / rawFramesPerRangeFrame) << "\n";
            ++outputCount;
        }
        dbg_out << "Shutting down...\n";
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
