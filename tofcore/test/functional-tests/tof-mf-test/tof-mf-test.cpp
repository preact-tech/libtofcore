/**
 * @file tof-mf-test.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libtofcore to test modifications of the
 * modulation frequency.
 */
#include "tofcore/tof_sensor.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <iomanip>
#include <thread>
#include <boost/program_options.hpp>


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace tofcore;
using namespace TofComm;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool captureAmxxx { false };
static bool captureDistance { false };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool enableBinning { false };
static volatile bool exitRequested { false };
static size_t verbosity { 0 };
static uint16_t modulation { 0 };
static uint16_t integration_time { 0 };

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
            if (verbosity > 1)
            {
                std::cout << "received DISTANCE-AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::DCS:
            ++dcsCount;
            if (verbosity > 1)
            {
                std::cout << "received DCS measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::GRAYSCALE:
            ++grayscaleCount;
            if (verbosity > 1)
            {
                std::cout << "received GRAYSCALE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::DISTANCE:
            ++distanceCount;
            if (verbosity > 1)
            {
                std::cout << "received DISTANCE measurement data, packet size " << pData->pixel_buffer().size() << std::endl;
            }
            break;
        case DataType::AMPLITUDE:
            ++amplitudeCount;
            if (verbosity > 1)
            {
                std::cout << "received AMPLITUDE measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        case DataType::AMBIENT:
            ++amplitudeCount;
            if (verbosity > 1)
            {
                std::cout << "received AMBIENT measurement data, packet size "
                          << (pData->pixel_buffer().size()) << std::endl;
            }
            break;
        
        default:
            std::cout << "UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type()) << std::endl;
    }
    if(verbosity > 1)
    {
        auto chip_temps = pData->sensor_temperatures();
        if(chip_temps) 
        {
            std::cout << "  Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << std::endl;
        } 
        else 
        {
            std::cout << "  No sensor temperature data" << std::endl;
        }
        auto integration_time = pData->integration_time();
        if(integration_time)
        {
            std::cout << "  Integration time settings (uS): " << *integration_time << std::endl;
        }
        else 
        {
            std::cout << "  No integration time data" << std::endl;
        }
        auto mod_frequency = pData->modulation_frequency();
        if(mod_frequency)
        {
            std::cout << "  Modulation Frequency setting (Hz): " << *mod_frequency << std::endl;
        }
        else 
        {
            std::cout << "  No modulation frequency data" << std::endl;
        }
        auto v_binning = pData->vertical_binning();
        auto h_binning = pData->horizontal_binning();
        if(v_binning && h_binning)
        {
            std::cout << "  Binning settings: " << (int)(*h_binning) << " " << (int)(*v_binning) << std::endl;
        }
        else 
        {
            std::cout << "  No binning data" << std::endl;
        }

        auto dll_settings = pData->dll_settings();
        if(dll_settings)
        {
            std::cout << "  DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ") << (int)(*dll_settings)[1] << " " <<  (int)(*dll_settings)[2] << " " <<  (int)(*dll_settings)[3] << std::endl;
        }
        else 
        {
            std::cout << "  No DLL settings" << std::endl;
        }
        auto illum = pData->illuminator_info();
        if(illum)
        {
            const auto& illum_info = *illum;
            std::cout << "  Illuminator info: 0x" << std::hex << (int)illum_info.led_segments_enabled << std::dec << " "
                      << illum_info.temperature_c << "C " 
                      <<  illum_info.vled_v << "V " 
                      << illum_info.photodiode_v << "V" 
                      << std::endl;
        }
        else
        {
            std::cout << "  No Illuminator information" << std::endl;
        }

        auto vsmControl = pData->vsm_info();
        if(vsmControl)
        {
            std::cout << "  VSM: Flags=" << vsmControl->m_vsmFlags << "; N = "
                      << (unsigned)vsmControl->m_numberOfElements  << "; I = "
                      << (unsigned)vsmControl->m_vsmIndex << ";";
            uint8_t numElements = std::min(vsmControl->m_numberOfElements, (uint8_t) VSM_MAX_NUMBER_OF_ELEMENTS);
            for (decltype(numElements) n = 0; n < numElements; ++n)
            {
                VsmElement_T& element = vsmControl->m_elements[n];
                std::cout << " [" << element.m_integrationTimeUs << ", " << element.m_modulationFreqKhz << "]";
            }
            std::cout << std::endl << std::endl;
        }
        else
        {
            std::cout << "  No VSM data" << std::endl << std::endl;
        }
    }
}

namespace po = boost::program_options;

class CountValue : public po::typed_value<std::size_t>
{
public:
    CountValue():
        CountValue(nullptr)
    {
    }

    CountValue(std::size_t* store):
        po::typed_value<std::size_t>(store)
    {
        // Ensure that no tokens may be passed as a value.
        default_value(0);
        zero_tokens();
    }

private:

    virtual void xparse(boost::any& store, const std::vector<std::string>& /*tokens*/) const
    {
        // Replace the stored value with the access count.
        store = boost::any(++count_);
    }

    mutable std::size_t count_{ 0 };
};

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("ToF Modulation Frequency Test");
    desc.add_options()
        ("help,h", "produce help message")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("Binning,B", po::bool_switch(&enableBinning)->default_value(false),"Enable full binning")
        ("amplitude,a", po::bool_switch(&captureAmxxx), "Capture DCS+Ambient or Distance Amplitude frames, (not just DCS or Distance)")
        ("ambient", po::bool_switch(&captureAmxxx), "Capture DCS+Ambient or Distance Amplitude frames, (not just DCS or Distance)")
        ("distance,d", po::bool_switch(&captureDistance),  "Capture distance (or amplitude) frames instead of DCS frames")
        ("modulation,m", po::value<uint16_t>(&modulation)->default_value(0),"Set modulation frequency to this value (kHz)")
        ("integration,i", po::value<uint16_t>(&integration_time)->default_value(0),"Set integration time to this value (uS)")
        ("verbose,V",               
           new  CountValue(&verbosity),
            "Increase verbosity of output")
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

static void test_mf(tofcore::Sensor& sensor, uint16_t testFreqKhz)
{
    if (verbosity > 0)
    {
        std::cout << "Setting MF to " << testFreqKhz << " kHz" << std::endl;
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
            std::cerr << "Expected " << expectedKhz << " kHz, got " << v << " kHz" << std::endl;
            exit(-1);
        }
        else if (verbosity > 0)
        {
            std::cout << "Got expected value: " << expectedKhz << " kHz" << std::endl;
        }
    }
    else
    {
        std::cerr << "Error reading modulation frequency" << std::endl;
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
        std::cerr << x.what() << std::endl;
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
//        sensor.stopStream();
        std::this_thread::sleep_for(100ms);
        uint32_t testCount { 0 };
        while (!exitRequested) // wait for ^\ or ^C
        {
            run_mf_tests(sensor);
            ++testCount;
            std::cout << "Tests run: " << testCount << std::endl;
            std::this_thread::sleep_for(500ms);
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
