/**
 * @file tof-imu.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that uses libtofcore to get various sensor values.
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
using namespace TofComm;
using namespace tofcore;

static DebugOutput dbg_out {};
static ErrorOutput err_out {};

static bool accelerometerAvailableRanges { false };
static bool accelerometerRange { false };
static unsigned accelerometerRangeValue { 0 };
static unsigned accelerometerRangeValueSet { 0 };
static bool accelerometerSelfTest { false };
static uint32_t baudRate { DEFAULT_BAUD_RATE };
static bool captureDcsAmbient { false };
static bool captureDcsDiff { false };
static bool continuous { false };
static uint32_t debugLevel { 0 };
static bool displayAccel { false };
static bool displayAll { false };
static bool displayFrameCounts { false };
static bool displayGyro { false };
static bool displayTemperature { false };
static bool displayTime { false };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static bool gyroSelfTest { false };
static uint32_t pollDelay { 0 };
static uint32_t verbosity;
static bool vsmEnable { false };

static std::atomic<uint32_t> ambientCount;
static std::atomic<uint32_t> amplitudeCount;
static std::atomic<uint32_t> dcsCount;
static std::atomic<uint32_t> dcsDiffCount;
static std::atomic<uint32_t> distanceCount;

static high_resolution_clock::time_point s_startTime;

namespace po = boost::program_options;

static void parseArgs(int argc, char *argv[])
{
    po::variables_map vm;
    po::options_description desc("Tof IMU data");
    desc.add_options()
        ("accel-selftest,a", po::bool_switch(&accelerometerSelfTest)->default_value(false), "Runs IMU accelerometer self-test")
        ("baud-rate,b", po::value<uint32_t>(&baudRate)->default_value(DEFAULT_BAUD_RATE))
        ("cont,c", po::bool_switch(&continuous)->default_value(false), "Continuous output of values, cntl-c to stop")
        ("dcs-ambient,d", po::bool_switch(&captureDcsAmbient),  "Capture DCS + Ambient frames")
        ("dcs-diff,D", po::bool_switch(&captureDcsDiff),  "Capture DCS_DIFF + Ambient frames")
        ("debug,g", new  CountValue(&debugLevel),"Increase debug level of libtofcore")
        ("device-uri,p", po::value<std::string>(&devicePort))
        ("accel-range,r", po::bool_switch(&accelerometerRange)->default_value(false), "Returns IMU accelerometer range")
        ("display-accel,C", po::bool_switch(&displayAccel)->default_value(false), "Display Accel values")
        ("display-all,A", po::bool_switch(&displayAll)->default_value(false), "Display all values")
        ("display-frame-counts,F", po::bool_switch(&displayFrameCounts)->default_value(false), "Display frame count data")
        ("display-gyro,G", po::bool_switch(&displayGyro) ->default_value(false), "Display Gyro data")
        ("display-temperature,t", po::bool_switch(&displayTemperature)->default_value(false), "Display temperature data")
        ("set-accel-range,R", po::value<unsigned>(&accelerometerRangeValueSet), "Sets IMU accelerometer range")
        ("supported-accel-ranges,S", po::bool_switch(&accelerometerAvailableRanges)->default_value(false), "Returns the supported IMU accelerometer ranges")
        ("display-time,T", po::bool_switch(&displayTime) ->default_value(false), "Display time data")
        ("help,h", "Program to access the IMU sensors: accelerometer, gyro and temperature.")
        ("poll-delay,P", po::value<uint32_t>(&pollDelay)->default_value(0), "Delay (mS) between IMU data polls.")
        ("quiet,q", po::bool_switch(&dbg_out.quiet)->default_value(false), "Disable output")
        ("verbosity,v", po::value<uint32_t>(&verbosity)->default_value(0), "Verbosity for debugging.")
        ("VSM,V", po::bool_switch(&vsmEnable),  "Enable VSM mode")
        ;

    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        dbg_out << desc << "\n"
                << "EXAMPLE - Grab IMU data once and display it:\n\n"
                << "  tof-imu -A\n\n"
                << "EXAMPLE - Poll IMU data as fast as possible (while not streaming) and display only the Gyro data:\n\n"
                << "  tof-imu -G -c\n\n"
                << "EXAMPLE - Poll IMU data while streaming DCS+AMBIENT with 100 mS delay between polls and displaying all data:\n\n"
                << "  tof-imu -p /dev/ttyACM0 -d -P 100 -c"
                << "\n\n";

        exit(0);
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

static void logFailure(const char *msg)
{
    high_resolution_clock::time_point now { high_resolution_clock::now() };
    duration<double> commandDuration { duration_cast<duration<double>>(now - s_startTime) };
    err_out << "[" << std::fixed << std::setprecision(6) << commandDuration.count() << "] FAILED: " << msg << "\n";
    exit(-1);
}

static void logSuccess(const char *msg)
{
    high_resolution_clock::time_point now { high_resolution_clock::now() };
    duration<double> commandDuration { duration_cast<duration<double>>(now - s_startTime) };
    dbg_out << "[" << std::fixed << std::setprecision(6) << commandDuration.count() << "] SUCCESS: " << msg << "\n";
}

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
                dbg_out << "\tGood pixels: " << goodPixels << "; Errors: " << pixelErrors << "; amplitude: "
                        << amplitudeErrors << "; saturated: " << saturatedPixels << "; ADC: " << adcErrors << "\n";

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
            dbg_out << "[" << framePeriod.count() << "] UNRECOGNIZED data type: " << static_cast<int16_t>(pData->type())
                    << "\n";
    }
    if (verbosity > 0)
    {
        auto chip_temps = pData->sensor_temperatures();
        if (chip_temps)
        {
            dbg_out << "  Sensor temperatures: " << (*chip_temps)[0] << ", " << (*chip_temps)[1] << ", " << (*chip_temps)[2]
                    << ", " << (*chip_temps)[3] << "\n";
        }
        else
        {
            dbg_out << "  No sensor temperature data\n";
        }
        auto integration_time = pData->integration_time();
        if (integration_time)
        {
            dbg_out << "  Integration time setting (uS): " << *integration_time << "\n";
        }
        else
        {
            dbg_out << "  No integration time data\n";
        }
        auto mod_frequency = pData->modulation_frequency();
        if (mod_frequency)
        {
            dbg_out << "  Modulation Frequency setting (Hz): " << *mod_frequency << "\n";
        }
        else
        {
            dbg_out << "  No modulation frequency data\n";
        }
        auto v_binning = pData->vertical_binning();
        auto h_binning = pData->horizontal_binning();
        if (v_binning && h_binning)
        {
            dbg_out << "  Binning settings: " << (int) (*h_binning) << " " << (int) (*v_binning) << "\n";
        }
        else
        {
            dbg_out << "  No binning data\n";
        }

        auto dll_settings = pData->dll_settings();
        if (dll_settings)
        {
            dbg_out << "  DLL settings: " << ((*dll_settings)[0] != 0 ? "True " : "False ") << (int) (*dll_settings)[1] << " "
                    << (int) (*dll_settings)[2] << " " << (int) (*dll_settings)[3] << "\n";
        }
        else
        {
            dbg_out << "  No DLL settings\n";
        }
        auto illum = pData->illuminator_info();
        if (illum)
        {
            const auto &illum_info = *illum;
            dbg_out << "  Illuminator info: 0x" << std::hex << (int) illum_info.led_segments_enabled << std::dec << " "
                    << illum_info.temperature_c << "C " << illum_info.vled_v << "V " << illum_info.photodiode_v << "V\n";
        }
        else
        {
            dbg_out << "  No Illuminator information\n";
        }

        auto vsmControl = pData->vsm_info();
        if (vsmControl)
        {
            dbg_out << "  VSM: Flags=" << vsmControl->m_vsmFlags << "; N = " << (unsigned) vsmControl->m_numberOfElements
            << "; I = " << (unsigned) vsmControl->m_vsmIndex << ";";
            uint8_t numElements = std::min(vsmControl->m_numberOfElements, (uint8_t) VSM_MAX_NUMBER_OF_ELEMENTS);
            for (decltype(numElements) n = 0; n < numElements; ++n)
            {
                VsmElement_T &element = vsmControl->m_elements[n];
                dbg_out << " [" << element.m_integrationTimeUs << ", " << element.m_modulationFreqKhz << "]";
            }
            dbg_out << "\n";
        }
        else
        {
            dbg_out << "  No VSM data\n";
        }

        auto timestamp = pData->frame_timestamp();
        if (timestamp)
        {
            dbg_out << "  Frame timestamp: " << (int) *timestamp << "\n";
        }
        else
        {
            dbg_out << "No timestamp found in frame data\n";
        }

        dbg_out << "\n\n";
    }
}

static int imuDataDisplay(tofcore::Sensor& sensor)
{
    high_resolution_clock::time_point now { high_resolution_clock::now() };
    duration<double> totalTime { duration_cast<duration<double>>(now - s_startTime) };
    /*
     * Imu information
     */
    auto data {sensor.getImuInfo()};

    if (!data)
    {
        logFailure("getImuInfo()");
    }
    else
    {
        ImuScaledData_T &imuData = *data; 
       
        dbg_out << "[" << std::fixed << std::setprecision(6) << totalTime.count() << "]\n";
        if (displayAll || displayAccel)
        {
            dbg_out << "\taccelerometer (xyz): (" << imuData.accelerometer_millig[0] << " milli-g, " << imuData.accelerometer_millig[1] << " milli-g, " << imuData.accelerometer_millig[2] << " milli-g)\n";
        }
        if (displayAll || displayGyro)
        {
            dbg_out << "\tgyro (xyz): (" << imuData.gyro_milliDegreesPerSecond[0]  << " milli-deg/sec, " << imuData.gyro_milliDegreesPerSecond[1] << " milli-deg/sec, " << imuData.gyro_milliDegreesPerSecond[2] << " milli-deg/sec)\n";
        }
        if (displayAll || displayTemperature)
        {
            dbg_out << "\ttemperature : " << imuData.temperature_milliDegreesC << " milli-DegC\n";
        }
        if (displayAll || displayTime)
        {
            dbg_out << "\tIMU timestamp : " << imuData.timestamp << " mS\n";
        }
        if ((captureDcsAmbient || captureDcsDiff) && (displayAll || displayFrameCounts))
        {
            uint32_t rawFrameCount = (ambientCount + amplitudeCount + (4*dcsCount) + (2*dcsDiffCount) + distanceCount);

            uint32_t rawFramesPerRangeFrame { 1 };
            if (captureDcsAmbient)
            {
                rawFramesPerRangeFrame = 5;
            }
            else if (captureDcsDiff)
            {
                rawFramesPerRangeFrame = 3;
            }

            dbg_out << "\tFrames:" << std::setw(7) << std::setfill('0')
                     << "  ambient = " << std::setw(7) << ambientCount
                     << "; amplitude = " << std::setw(7) << amplitudeCount
                     << "; dcs = " << std::setw(7) << dcsCount
                     << "; dcsDiff = " << std::setw(7) << dcsDiffCount
                     << "; distance = " << std::setw(7) << distanceCount
                     << "; FPS: " << (rawFrameCount / totalTime.count() / rawFramesPerRangeFrame) << "\n";

        }
    }
    return 0;
}

static std::string imuCodeToString(int code)
{
    /* There probably is a clever way to do this, but this will work.
       A maintenance issue - need to keep this in sync with codes 
       defined in BMI270.hpp. 
       The constants here are taken from BMI270.hpp.*/
    constexpr int FATAL_IMU_ERROR                    { -1 };
    constexpr int FATAL_IMU_ERRORS_CANNOT_BE_CLEARED { -2 }; 
    constexpr int FATAL_IMU_CANNOT_BE_READ           { -3 }; 
    constexpr int FATAL_INTERNAL_IMU_ERROR           { -4 };

    constexpr int8_t IMU_ACCELEROMETER_SELF_TEST_FAIL               { -5 };
    constexpr int8_t FAIL_IMU_ACCELEROMETER_SELF_TEST_ENABLE        { -6 };
    constexpr int8_t FAIL_IMU_ACCELEROMETER_SELF_TEST_RANGE_SET     { -7 };
    constexpr int8_t FAIL_IMU_ACCELEROMETER_SELF_TEST_AMPLITUDE_SET { -8 };
    constexpr int8_t FAIL_IMU_ACCELEROMETER_SELF_TEST_POLARITY_SET  { -9 };
    constexpr int8_t FAIL_IMU_ACCELEROMETER_SELF_TEST_CONFIGURATION { -10 };

    constexpr int8_t FAIL_IMU_GYRO_TRIGGER_STATUS    { -11 };
    constexpr int8_t FAIL_IMU_GYRO_SELF_TEST         { -12 };
    constexpr int8_t FAIL_IMU_GYRO_SELF_TEST_TIMEOUT { -13 };
    constexpr int8_t FAIL_IMU_GYRO_SELF_TEST_SETUP   { -14 };

    constexpr int8_t IMU_GYRO_SELF_TEST_NOT_IMPLMENTED { -99 };

    std::string returnString = "test";

    switch (code)
    {
        case 0:
            returnString = "Pass";
            break;
    
        case FATAL_IMU_ERROR:
            returnString = "Fatal IMU error";
            break;

        case FATAL_IMU_ERRORS_CANNOT_BE_CLEARED:
            returnString = "IMU errors cannot be cleared";
            break;

        case FATAL_IMU_CANNOT_BE_READ:
            returnString = "IMU data cannot be read";
            break;

        case FATAL_INTERNAL_IMU_ERROR:
            returnString = "Internal IMU error detected";
            break;

        case IMU_ACCELEROMETER_SELF_TEST_FAIL:
            returnString = "IMU accelerometer self-test failed";
            break;

        case FAIL_IMU_ACCELEROMETER_SELF_TEST_ENABLE:
            returnString = "Failed to enable the IMU accelerometer self-test";
            break;

        case FAIL_IMU_ACCELEROMETER_SELF_TEST_RANGE_SET:
            returnString = "Failed to set the IMU accelerometer test range";
            break;

        case FAIL_IMU_ACCELEROMETER_SELF_TEST_AMPLITUDE_SET:
            returnString = "Failed to set the IMU accelerometer test amplitude";
            break;

        case FAIL_IMU_ACCELEROMETER_SELF_TEST_POLARITY_SET:
            returnString = "Failed to set the IMU accelerometer test polarity";
            break;

        case FAIL_IMU_ACCELEROMETER_SELF_TEST_CONFIGURATION:
            returnString = "Failed to configure the IMU accelerometer for self-test";
            break;

        case FAIL_IMU_GYRO_TRIGGER_STATUS:
            returnString = "The IMU gyro trigger status failed";
            break;

        case FAIL_IMU_GYRO_SELF_TEST:
            returnString = "The IMU gyro self-test failed";
            break;

        case FAIL_IMU_GYRO_SELF_TEST_TIMEOUT:
            returnString = "Timeout occurred waiting for the completion of the IMU gyro self-test";
            break;

        case FAIL_IMU_GYRO_SELF_TEST_SETUP:
            returnString = "Failed to setup the IMU gyro for self-test";
            break;

        case IMU_GYRO_SELF_TEST_NOT_IMPLMENTED:
            returnString = "The IMU gyro self-test is not implemented.";
            break;

        default:
            returnString = "Undefined return code ";
            returnString += std::to_string(code);
            break;
    }
    
    return returnString;
}

static int imuAvailableRangeDisplay(tofcore::Sensor& sensor)
{
    /*
     * Imu range
     */
    std::tuple<int8_t, std::list<uint8_t>>data {sensor.imuAccelerometerAvailableRangesInGs()};

    if (std::get<0>(data) == 0)
    {
        dbg_out << "Imu available accelerometer ranges =\n";
        std::list<uint8_t> l = std::get<1>(data);
        for (auto const& r : l)
            dbg_out << " " << (int)r << " g\n";
    }
    else
    {
        err_out << "Failed to retrieve valid IMU accelerometer range\n";
        return -1;
    }
    return 0;
}

static int imuRangeDisplay(tofcore::Sensor& sensor)
{
    /*
     * Imu range
     */
    std::tuple<int8_t, uint8_t>data {sensor.imuAccelerometerRangeInGs()};

    if (std::get<0>(data) == 0)
    {
        dbg_out << "Imu accelerometer range=" << (int)std::get<1>(data) << "g\n";
    }
    else
    {
        err_out << "Failed to retrieve valid IMU accelerometer range\n";
        return -1;
    }
    return 0;
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

    const char *cmdDescription { "Connected to sensor" };

    s_startTime = high_resolution_clock::now();

    try
    {
        tofcore::Sensor sensor {devicePort, baudRate};
        sensor.setDebugLevel(debugLevel);
        logSuccess(cmdDescription);

        if (captureDcsAmbient || captureDcsDiff)
        {
            // Subscribe
            cmdDescription = "subscribeMeasurement()";
            sensor.subscribeMeasurement(&measurement_callback);
            logSuccess(cmdDescription);

            if (vsmEnable)
            {
                VsmControl_T vsmControl {};
                vsmControl.m_numberOfElements = 3;
                vsmControl.m_elements[0].m_integrationTimeUs = 500;
                vsmControl.m_elements[0].m_modulationFreqKhz = 6010;
                vsmControl.m_elements[1].m_integrationTimeUs = 500;
                vsmControl.m_elements[1].m_modulationFreqKhz = 6010;
                vsmControl.m_elements[2].m_integrationTimeUs = 500;
                vsmControl.m_elements[2].m_modulationFreqKhz = 7010;

                cmdDescription = "setVsm(vsmControl)";
                if (sensor.setVsm(vsmControl))
                {
                    logSuccess(cmdDescription);
                    std::string s { "    VSM Elements: " };
                    for (unsigned i = 0; i < vsmControl.m_numberOfElements; ++i)
                    {
                        s.append("[").append(std::to_string(vsmControl.m_elements[i].m_integrationTimeUs)).
                                    append(", ").append(std::to_string(vsmControl.m_elements[i].m_modulationFreqKhz)).
                                    append("]");
                        if ((int)i < vsmControl.m_numberOfElements - 1)
                        {
                            s.append(", ");
                        }
                    }
                    dbg_out << s.c_str() << "\n";
                }
                else
                {
                    logFailure(cmdDescription);
                }
            }

            if (captureDcsAmbient)
            {
                if (sensor.streamDCSAmbient())
                {
                    logSuccess("streamDCSAmbient()");
                }
                else
                {
                    logFailure("streamDCSAmbient()");
                }
            }
            else
            {
                if (sensor.streamDCSDiffAmbient())
                {
                    logSuccess("streamDCSDiffAmbient()");
                }
                else
                {
                    logFailure("streamDCSDiffAmbient()");
                }
            }
        }

        if (accelerometerRange)
        {
            imuRangeDisplay(sensor);
        }

        if (accelerometerAvailableRanges)
        {
            imuAvailableRangeDisplay(sensor);
        }

        if ((accelerometerRangeValueSet != 0) && (accelerometerRangeValue != accelerometerRangeValueSet))
        {
            int8_t returnValue;

            returnValue = sensor.imuAccelerometerRangeInGs(accelerometerRangeValueSet);
            if (returnValue == 0)
            { 
                dbg_out <<"Imu accelerometer range set to "<< accelerometerRangeValueSet << " g\n";
                accelerometerRangeValue = accelerometerRangeValueSet;
            }
            else
            { 
                dbg_out << " IMU accelerometer setting range failed code=" << imuCodeToString(returnValue) << "\n";
            }
        }

        if (accelerometerSelfTest)
        {
            int8_t returnValue;

            returnValue = sensor.imuAccelerometerSelfTest();
            if (returnValue == 0)
            {
                logSuccess("IMU accelerometer self-test");
            }
            else
            {
                std::string s {"IMU accelerometer self-test: "};
                s.append(imuCodeToString(returnValue));
                logFailure(s.c_str());
            }
        }

        if (gyroSelfTest)
        {
            int8_t returnValue = sensor.imuGyroSelfTest();
            if (returnValue == 0)
            {
                logSuccess("IMU gyro self-test");
            }
            else
            {
                std::string s {"IMU gyro self-test: "};
                s.append(imuCodeToString(returnValue));
                logFailure(s.c_str());
            }
        }

        // If self test was executed, skip grabbing data
        if (!accelerometerSelfTest && !gyroSelfTest)
        {
            do
            {
                imuDataDisplay(sensor);
                if (exitRequested)
                    continuous = false;
                if (pollDelay > 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollDelay));
                }
            }
            while (continuous);
        }
        sensor.stopStream();
    }
    catch (po::error &x)
    {
        err_out << x.what() << "\n";
        return -1;
    }
    catch (...)
    {
        err_out << "No sensor found\n";
        return -1;
    }
    return 0;
}
