/**
 * @file tof-mf-test.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to test modifications of the
 * modulation frequency.
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
static bool captureDistance { false };
static uint32_t debugLevel { 0 };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool enableBinning { false };
static volatile bool exitRequested { false };
static uint32_t verbosity { 0 };
static uint16_t modulation { 0 };
static uint16_t integration_time { 0 };

static std::atomic<uint32_t> ambientCount;
static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> dcsDiffCount;
static std::atomic<uint32_t> distanceCount;


static void measurement_callback(std::shared_ptr<tofcore::Measurement_T> pData)
{
    using DataType = tofcore::Measurement_T::DataType;
    switch (pData->type())
    {
        case DataType::DISTANCE_AMPLITUDE:
            ++amplitudeCount;
            ++distanceCount;
            if (verbosity > 1)
            {
                dbg_out << "received DISTANCE-AMPLITUDE measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;
        case DataType::DCS:
            ++dcsCount;
            if (verbosity > 1)
            {
                dbg_out << "received DCS measurement data, packet size " << pData->pixel_buffer().size() << "\n";
            }
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            if (verbosity > 1)
            {
                dbg_out << "received DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << "\n";
            }
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            if (verbosity > 1)
            {
                dbg_out << "received AMPLITUDE measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;
        case DataType::AMBIENT:
        case DataType::GRAYSCALE:
           ++ambientCount;
            if (verbosity > 1)
            {
                dbg_out << "received AMBIENT measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;
        case DataType::DCS_DIFF_AMBIENT:
            ++ambientCount;
            ++dcsDiffCount;
            if (verbosity > 1)
            {
                dbg_out << "received DCS_DIFF+AMBIENT measurement data, packet size "
                        << (pData->pixel_buffer().size()) << "\n";
            }
            break;

        default:
            dbg_out << "UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << "\n";
    }
    if(verbosity > 1)
    {
        auto chip_temps = pData->sensor_temperatures();
        if(chip_temps) 
        {
            dbg_out << "  Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << "\n";
        } 
        else 
        {
            dbg_out << "  No sensor temperature data\n";
        }
        auto integration_time = pData->integration_time();
        if(integration_time)
        {
            dbg_out << "  Integration time settings (uS): " << *integration_time << "\n";
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
            dbg_out << "  DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ") << (int)(*dll_settings)[1] << " " <<  (int)(*dll_settings)[2] << " " <<  (int)(*dll_settings)[3] << "\n";
        }
        else 
        {
            dbg_out << "  No DLL settings\n";
        }
        auto illum = pData->illuminator_info();
        if(illum)
        {
            const auto& illum_info = *illum;
            dbg_out << "  Illuminator info: 0x" << std::hex << (int)illum_info.led_segments_enabled << std::dec << " "
                    << illum_info.temperature_c << "C "
                    <<  illum_info.vled_v << "V "
                    << illum_info.photodiode_v << "V"
                    << "\n";
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
                dbg_out << " [" << element.m_integrationTimeUs << ", " << element.m_modulationFreqKhz << "]";
            }
            dbg_out << "\n\n";
        }
        else
        {
            dbg_out << "  No VSM data" << "\n\n";
        }
    }
}

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("ToF Modulation Frequency Test");
    desc.add_options()
        ("ambient,A", po::bool_switch(&captureAmbient), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("amplitude,a", po::bool_switch(&captureAmplitude), "Capture DCS+Ambient or Distance+Amplitude frames, (not just DCS or Distance)")
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("Binning,B", po::bool_switch(&enableBinning)->default_value(false),"Enable full binning")
        ("debug,G", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("distance,d", po::bool_switch(&captureDistance),  "Capture distance (or amplitude) frames instead of DCS frames")
        ("help,h", "produce help message")
        ("integration,i", po::value<uint16_t>(&integration_time)->default_value(0),"Set integration time to this value (uS)")
        ("modulation,m", po::value<uint16_t>(&modulation)->default_value(0),"Set modulation frequency to this value (kHz)")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("verbose,V", new  CountValue(&verbosity), "Increase verbosity of output")
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

static void test_mf(tofcore::Sensor& sensor, uint16_t testFreqKhz)
{
    if (verbosity > 0)
    {
        dbg_out << "Setting MF to " << testFreqKhz << " kHz\n";
    }
    sensor.setModulation(testFreqKhz);
    sensor.streamDistance();
    /*
     * It can take several hundred mS for the sensor to initially calculate
     * the correction values for modulation frequencies that are not exactly
     * the calibration frequencies (it has to interpolate)
     */
    std::this_thread::sleep_for(500ms);

    sensor.stopStream();

    uint16_t expectedKhz;
    if (testFreqKhz < 6000)
    {
        expectedKhz = 6000;
    }
    else if (testFreqKhz > 24000)
    {
        expectedKhz = 24000;
    }
    else
    {
        expectedKhz = (uint16_t)((testFreqKhz/10.0) + 0.5) * 10;
    }

    auto actualKhz = sensor.getModulation();

    if (actualKhz)
    {
        auto v = actualKhz.value();
        if (v != expectedKhz)
        {
            err_out << "Expected " << expectedKhz << " kHz, got " << v << " kHz\n";
            exit(-1);
        }
        else if (verbosity > 0)
        {
            dbg_out << "Got expected value: " << expectedKhz << " kHz\n";
        }
    }
    else
    {
        err_out << "Error reading modulation frequency\n";
        exit(-1);
    }
}

static void run_mf_tests(tofcore::Sensor& sensor)
{
    const uint16_t testValues[]
        {6001, 6009, 6050, 8000, 10100, 13000, 22000, 6000, 7000, 12000, 23000, 24000};
    for (auto testKhz : testValues)
    {
        test_mf(sensor, testKhz);
    }
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

        if (enableBinning)
        {
            sensor.setBinning(true, true);
        }
        else
        {
            sensor.setBinning(false, false);
        }
        if(modulation)
        {
            sensor.setModulation(modulation);
        }

        if(integration_time)
        {
            sensor.setIntegrationTime(integration_time);
        }

        sensor.subscribeMeasurement(&measurement_callback); // callback is called from background thread
        if (captureDistance)
        {
            if (captureAmbient || captureAmplitude)
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
            if (captureAmbient || captureAmplitude)
            {
                sensor.streamDCSAmbient();
            }
            else
            {
                sensor.streamDCS();
            }
        }
//        sensor.stopStream();
        std::this_thread::sleep_for(100ms);
        uint32_t testCount { 0 };
        while (!exitRequested) // wait for ^\ or ^C
        {
            run_mf_tests(sensor);
            ++testCount;
            dbg_out << "Tests run: " << testCount << "\n";
            std::this_thread::sleep_for(500ms);
        }
        dbg_out << "Shutting down...\n";
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
