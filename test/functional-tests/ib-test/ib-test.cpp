/**
 * @file ib-test.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program that uses libt10 to test the illuminator board.
 */
#include "tofcore/tof_sensor.hpp"
#include <csignal>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace tofcore;

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string devicePort { DEFAULT_PORT_NAME };
static volatile bool exitRequested { false };
static const uint16_t protocolVersion { 1 };
static bool runRgbTest { false };
static int8_t rgbColor { -1 };
static uint16_t rgbBlinkMs { 0 };
static int8_t emitterEnable {-1};

//illuminator board stats
static uint32_t serialNumber { 0 };
static uint16_t vled {0};
static uint16_t psu_5v { 0 };
static uint16_t pdMv { 0 };
static int16_t temp { 0 };

constexpr uint8_t TEST_VAL_8BIT {0xBB};
constexpr uint16_t TEST_VAL_16BIT {0xAACC};

/*********************************************
        RGB bitmask bit positions

NOTE: only one RGB bit can be set at a time.
*********************************************/
#define RGB_OFF_BP 0
#define RGB_RED_BP 1
#define RGB_GREEN_BP 2
#define RGB_BLUE_BP 4

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:c:hl:m:p:rs:v:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'c':
                rgbColor = atoi(optarg);
                break;
            case 'h':
                std::cout   << "Command to get various values from the Illuminator Board" << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-h] [-p <port>]" << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -c <color>    Set the desired RGB color. 0=off, 1=red, 2=green, 4=blue." << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -l <enable>   Set the Emitter LED enable state" <<std::endl
                            << "  -m <blinkMs   Set the blink period in ms. Default = " << (unsigned)rgbBlinkMs << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -r            Run the rgb test to exercise each RGB color."<< std::endl
                            << "  -s <serial>   Write the Illuminator board serial number to illuminator NVM" << std::endl
                            << "  -v <vled>     Set vled to the desired level (mV)." << std::endl 
                            << std::endl << std::endl;
                exit(0);
            case 'l':
                emitterEnable = atoi(optarg);
                break;
            case 'm':
                rgbBlinkMs = atoi(optarg);
                break;
            case 'p':
                devicePort = optarg;
                break;
            case 'r':
                runRgbTest = true;
                break;
            case 's':
                serialNumber = atoi(optarg);
                break;
            case 'v':
                vled = atoi(optarg);
                break;
            default:
                break;
        }
    }
}

static void signalHandler(int signum)
{
    (void)signum;
    /*
    Put shutdown code here.
    */
    exit(0);
}

/*
Convenience functions to check the return values.
*/
static std::string checkError(bool pass, uint16_t& buf)
{
    std::string ret {};
    if(pass){
        ret = std::to_string(buf); 
    }
    else {
        ret = "ERROR";
    }
    return ret;
}

static std::string checkError(bool pass, int16_t& buf)
{
    std::string ret {};
    if(pass){
        ret = std::to_string(buf); 
    }
    else {
        ret = "ERROR";
    }
    return ret;
}

static std::string checkError(bool pass, uint8_t& buf)
{
    std::string ret {};
    if(pass){
        ret = std::to_string(buf); 
    }
    else {
        ret = "ERROR";
    }
    return ret;
}

void dumpStats(tofcore::Sensor& sensor)
{
    TofComm::versionData_t versionData;
    bool ok = sensor.getSensorInfo(versionData);
    
    std::cout << "Firmware version: ";
    if(ok) {
        std::cout << versionData.m_illuminatorSwVersion << '.' << versionData.m_illuminatorSwSourceId << std::endl;
    }
    else
    {
        std::cout << "ERROR" << std::endl;
    }

    std::cout << "Board Config: ";
    if(ok) {
        std::cout << (unsigned) versionData.m_illuminatorHwCfg << std::endl;
    }
    else {
        std::cout << "ERROR" <<std::endl;
    }

    std::cout << "IB Serial: ";
    if(sensor.getIbSerial(serialNumber))
    {
        std::cout << static_cast<unsigned int>(serialNumber) << std::endl;
    }
    else
    {
        std::cout << "ERROR" << std::endl;
    }

    std::cout << std::endl;

    std::cout << "Temperature: " << checkError(sensor.getIllmnTemperature(temp), temp) << std::endl;
    std::cout << std::endl;

    std::cout << "Illuminator Board Voltages (mV)" << std::endl;
    std::cout << "VLED: " << checkError(sensor.getVled(vled), vled) << std::endl;
    std::cout << "5V: " << checkError(sensor.getIb5V(psu_5v), psu_5v) << std::endl;
    std::cout << "V PD: " << checkError(sensor.getIbPd(pdMv), pdMv) << std::endl;
    std::cout << std::endl;
}

void testRgbs(tofcore::Sensor& sensor)
{
    std::cout << "Starting RGB test." << std::endl << std::endl;

    uint8_t rgbState { 0 };
    std::cout << "Turning RGBs OFF (state 0)" << std::endl;
    sensor.setIbRgb(0, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color RED (state 1)" << std::endl;
    sensor.setIbRgb(RGB_RED_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color GREEN (state 2)" << std::endl;
    sensor.setIbRgb(RGB_GREEN_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << "Setting RGB color BLUE (state 4)" << std::endl;
    sensor.setIbRgb(RGB_BLUE_BP, rgbBlinkMs);
    std::cout << "RGB state: " << checkError(sensor.getIbRgb(rgbState), rgbState) << std::endl;
    std::cout << "Press enter to continue." <<std::endl;
    std::getchar();

    std::cout << std::endl << "Finished RGB test." << std::endl;
}

bool setRgb(tofcore::Sensor& sensor, uint8_t state) 
{
    //Make sure we only set one rgb at a time.
    if(state == 0)
    {
        return sensor.setIbRgb(RGB_OFF_BP, rgbBlinkMs);
    }
    else if(state & RGB_RED_BP)
    {
        return sensor.setIbRgb(RGB_RED_BP, rgbBlinkMs);
    }
    else if(state & RGB_GREEN_BP)
    {
        return sensor.setIbRgb(RGB_GREEN_BP, rgbBlinkMs);
    }
    else if(state & RGB_BLUE_BP)
    {
        return sensor.setIbRgb(RGB_BLUE_BP, rgbBlinkMs);
    }

    return false;
}

/**
 * @brief write and retrieve 8 and 16 bit values from the sensor
 * 
 * @param sensor reference to the sensor under test.
 * @return true comms ok
 * @return false comm failure
 */
bool testCommsI2C(tofcore::Sensor& sensor)
{
    bool ok {true};
    std::cout << std::hex;
    std::cout << "Sending test val: " << static_cast<unsigned int>(TEST_VAL_8BIT) << std::endl;
    sensor.setTestVal(TEST_VAL_8BIT);
    uint8_t test8 {0};
    std::cout << "Received test val: ";
    if(sensor.getTestVal(test8))
    {
        std::cout << (unsigned int) test8;
    }
    else
    {
        std::cout << "ERROR";
    } 
    std::cout << std::endl;
    ok &= (test8 == TEST_VAL_8BIT);

    std::cout << "Sending test val: " << static_cast<unsigned int>(TEST_VAL_16BIT) << std::endl;
    sensor.setTestVal(TEST_VAL_16BIT);
    uint16_t test16 {0};
    std::cout << "Received test val: ";
    if(sensor.getTestVal(test16))
    {
        std::cout << (unsigned int) test16;
    } 
    else
    {
        std::cout << "ERROR";
    }

    std::cout << std::endl;
    std::cout << std::dec;
    ok &= (test16 == TEST_VAL_16BIT);

    return ok;
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

        //Exit here if we're just setting the RGB.
        if(rgbColor >= 0)
        {
            if(!setRgb(sensor, rgbColor))
            {
                std::cout << "Failed to set color!" << std::endl;
            }
            exit(0);
        }

        if(serialNumber)
        {
            std::cout << "Setting illuminator serial number: " << (unsigned) serialNumber << std::endl;
            if(!sensor.setIbSerial(serialNumber))
            {
                std::cout << "Failed to set serial number!" << std::endl;
            }
            exit(0);
        }

        if(vled) 
        {
            std::cout << "Setting VLED: " << vled << "mV" << std::endl;
            if(!sensor.setVled(vled))
            {
                std::cout << "Failed to set VLED!" << std::endl;
            }
            exit(0);
        }

        if(emitterEnable >= 0)
        {
            std::cout << "Setting emitter enable: " << (unsigned) emitterEnable << std::endl;
            bool setOk = sensor.setVledEnables((uint8_t) emitterEnable);
            int8_t emitterEnableState { -1 };
            sensor.getVledEnables((uint8_t&) emitterEnableState);
            if(!setOk || (emitterEnableState != emitterEnable))
            {
                std::cout << "Failed to set emitter enable!" << std::endl;
            }
            exit(0);
        }

        std::cout << "Starting Illuminator Board Test!" << std::endl;
        if(testCommsI2C(sensor))
        {
            std::cout << "Passed Comms test." << std::endl;
        }
        else
        {
            std::cout << "Failed comms test!" << std::endl;
        }

        dumpStats(sensor);

        if(runRgbTest)
        {
            testRgbs(sensor);
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
