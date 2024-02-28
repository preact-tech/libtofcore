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
            std::cout << "  Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", "<< (*chip_temps)[2] << ", "<< (*chip_temps)[3] << std::endl;
        } 
        else 
        {
            std::cout << "  No sensor temperature data" << std::endl;
        }
        auto integration_time = pData->integration_time();
        if(integration_time)
        {
            std::cout << "  Integration time setting (uS): " << *integration_time << std::endl;
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
            std::cout << "\n\n";
        }
        else
        {
            std::cout << "  No VSM data" << "\n\n";
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
    mutable std::size_t count_{ 0 };

    virtual void xparse(boost::any& store, const std::vector<std::string>& /*tokens*/) const
    {
        // Replace the stored value with the access count.
        store = boost::any(++count_);
    }
};

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Simple Streamer Test");
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

static uint16_t integrationUsStart { 250 };
static uint16_t integrationUsIncrement { 250 };
static uint16_t mfKhzStart { 0 };
static uint16_t mfKhzIncrement { 0 };
static int32_t numElements { 8 };

static void setVsm(tofcore::Sensor& sensor)
{
    integrationUsStart += 50;
    if (integrationUsStart > 1000)
    {
        integrationUsStart = 250;
    }
    std::cout << "SETTING VSM" << std::endl;
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

static void getVsm(tofcore::Sensor& sensor)
{
    auto result = sensor.getVsmSettings();

    if (result)
    {
        const VsmControl_T& vsm = *result;
        std::cout << "VSM Flags: " << vsm.m_vsmFlags << "; ";
        std::cout << "N: " << (unsigned)vsm.m_numberOfElements << "; ";
        std::cout << "I: " << (unsigned)vsm.m_vsmIndex << "; ";
        for (size_t n = 0; n < vsm.m_numberOfElements; ++n)
        {
            std::cout << " {";
            std::cout << vsm.m_elements[n].m_integrationTimeUs << ", "
                      << vsm.m_elements[n].m_modulationFreqKhz << "} ";
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "FAILED read VSM settings" << std::endl;
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
        auto lastTime = steady_clock::now();
        uint32_t delayCount { 0 };

        while (!exitRequested) // wait for ^\ or ^C
        {
            std::this_thread::sleep_until(lastTime + 1000ms);
            lastTime = steady_clock::now();

            getVsm(sensor);
            setVsm(sensor);

            if (++delayCount >= 2)
            {
                delayCount = 0;
                const uint32_t amplitude { amplitudeCount };
                amplitudeCount = 0;
                const uint32_t dcs { 4 * dcsCount };
                dcsCount = 0;
                const uint32_t distance { distanceCount };
                distanceCount = 0;
                const uint32_t grayscale { grayscaleCount };
                grayscaleCount = 0;
                std::cout << "RAW FPS: amplitude = " << amplitude << "; dcs = " << dcs << "; distance = " << distance
                          << "; grayscale = " << grayscale << "; total = " << (amplitude + dcs + distance + grayscale)
                          << std::endl;
            }
        }
        std::cout << "Shutting down..." << std::endl;
        sensor.stopStream();
    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
