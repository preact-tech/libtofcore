/**
 * @file tof-rw.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Test program to read Tof persistent storage
 */
#include "crc32.h"
#include "tofcore/tof_sensor.hpp"
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>

using namespace std::chrono;
using namespace tofcore;
using namespace TofComm;

enum class OutputMode_e { BINARY, TEXT, HEX };

static uint32_t baudRate { DEFAULT_BAUD_RATE };
static std::string calFileName { };
static std::string devicePort { DEFAULT_PORT_NAME };
static bool eraseStorage { false };
static volatile bool exitRequested { false };
static bool getMetadata { false };
static uint32_t hexBytesPerLine { 16 };
static ManufacturingData_T manufacturingData { };
static uint32_t maxSizePerRead { 0 };
static OutputMode_e outputMode { OutputMode_e::BINARY };
static uint16_t protocolVersion { 1 }; //Note we really need to use protocolVersion 1 for these large messages
static StorageId_e storageId { StorageId_e::PERSISTENT_SETTINGS };
static StorageMode_e storageMode { StorageMode_e::EXTRACTED };
static uint32_t storageOffset { 0 };
static uint32_t storageSize { 0 };
static bool writeManufacturingData { false };

static bool doErase(tofcore::Sensor& sensor)
{
    const bool ok { sensor.setFactoryMode(true) && sensor.storageWriteStart(storageId, StorageMode_e::EXTRACTED) };
    return ok;
}

static std::tuple<bool, std::vector<std::byte>> doRead(tofcore::Sensor& sensor, const uint32_t offset, const uint32_t size)
{
    bool ok { true };
    std::vector<std::byte> data {};

    uint32_t totalBytesRead { 0 };

    while (true)
    {
        // If maxSizePerRead was specified, limit read size to it; otherwise, use the size passed in
        const uint32_t bytesToRead
            { (0 == maxSizePerRead) ?
                        size : ((0 == size) ? maxSizePerRead : std::min(maxSizePerRead, size)) };

        const auto readResult = sensor.storageRead(storageId, storageMode, offset + totalBytesRead, bytesToRead);

        ok = readResult.has_value();
        if (!ok)
        {
            break; // done if there was an error
        }
        /*
         * Append the data read to the result.
         */
        const auto& v = *readResult;
        data.insert(data.end(), v.begin(), v.end()); // Append to vector that will be returned
        const size_t bytesRead { v.size() };
        totalBytesRead += bytesRead;
//        std::cerr << "Read " << bytesRead << " bytes (" << totalBytesRead << ")" << std::endl;
        /*
         * Check to see if multiple reads are being used to limit the size of
         * each one. We're done when either no size was specified for the read
         * or when that number of bytes have been read.
         */
        if (    (0 == bytesToRead)          // Done if no read size was specified (we got "all")
             || (bytesRead < bytesToRead)   // .. or if we got less than requested (we got "all that remained")
             || ( (size != 0) && (totalBytesRead >= size) ) ) // .. or we got the number requested
        {
            break;
        }
    }

    return std::make_tuple(ok, data);
}

static void outputBinary(const std::vector<std::byte>& pData)
{
    for (auto b : pData)
    {
        std::cout << std::to_integer<uint8_t>(b);
    }
}

static void outputHex(const std::vector<std::byte>& pData)
{
    uint32_t count { 0 };
    std::string pchars { "  " };
    uint32_t offset { 0 };
    std::cout << "[" << std::hex << std::setw(6) << std::setfill('0') << offset << "]";
    for (auto b : pData)
    {
        std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << (unsigned)b;
        const char c { (char)b };
        constexpr char period { '.' };
        if (isprint(c)) pchars.push_back(c);
        else            pchars.push_back(period);
        if ((++count % hexBytesPerLine) == 0)
        {
            offset += hexBytesPerLine;
            std::cout << pchars.c_str() << std::endl
                      << "[" << std::hex << std::setw(6) << std::setfill('0') << offset << "]";
            pchars = "  ";
        }
    }
    std::cout << std::endl << std::dec;
}

static void outputText(const std::vector<std::byte>& pData)
{
    std::cout << reinterpret_cast<const char*>(pData.data()) << std::endl;
}

static void parseArgs(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "b:BcCeEhlLmM:p:P:o:rs:Stw:xX:")) != -1)
    {
        switch (opt)
        {
            case 'b':
                baudRate = atoi(optarg);
                break;
            case 'B':
                outputMode = OutputMode_e::BINARY;
                break;
            case 'c':
                storageId = StorageId_e::CALIBRATON;
                break;
            case 'C':
                getMetadata = true;
                break;
            case 'e':
                 storageMode = StorageMode_e::EXTRACTED;
                 break;
            case 'E':
                 eraseStorage = true;
                 break;
            case 'h':
                std::cout   << "Read sensor storage." << std::endl << std::endl
                            << "Usage: " << argv[0] << " [-b <baud>] [-B] [-c] [-C] [-e] [-h] [-l] [-L]" << std::endl
                            << "      [-m] [-M <mandata>] [-o <offset>] [-p <port>] [-P <calfile>] [-r]  " << std::endl
                            << "      [-s <size>] [-S] [-t] [-w <N>] [-x] [-X <max>] " << std::endl << std::endl
                            << "  -b <baud>     Set baud rate (UART). Default = "<< DEFAULT_BAUD_RATE << std::endl
                            << "  -B            output BINARY data" << std::endl
                            << "  -c            Read Calibration storage" << std::endl
                            << "  -C            Read CRC32 & size metadata instead of actual data" << std::endl
                            << "  -e            EXTRACTED mode access" << std::endl
                            << "  -E            ERASE the specified storage instead of read; e.g., '-m -E' erases manufacturing data storage" << std::endl
                            << "  -h            Print help and exit" << std::endl
                            << "  -l            read Lens Data storage" << std::endl
                            << "  -L            read Log storage" << std::endl
                            << "  -m            read Manufacturing storage" << std::endl
                            << "  -M <mandata>  Store manufacturing data AND EXIT" << std::endl
                            << "  -o <offset>   Start reading <offset> bytes in (default is 0)" << std::endl
                            << "  -p <port>     Set port name. Default = "<< DEFAULT_PORT_NAME << std::endl
                            << "  -P <calfile>  Program calibration data from calfile AND EXIT" << std::endl
                            << "  -r            RAW mode access" << std::endl
                            << "  -s <size>     Read <size> bytes (default is 0, meaning all)" << std::endl
                            << "  -S            read persistent Settings storage" << std::endl
                            << "  -t            Treat received data as TEXT for printing" << std::endl
                            << "  -w <N>        In HEX mode, output N bytes per line" << std::endl
                            << "  -x            Output data as HEX" << std::endl
                            << "  -X <max>      Maximum size of reads; break up larger reads if necessary" << std::endl << std::endl
                            << "<mandata> = <mac>,serialnumstr,modelnumstr,modelnamestr" << std::endl
                            << "<mac> = aa:bb:cc:dd:ee" << std::endl
                            << std::endl << std::endl;
                exit(0);
            case 'l':
                storageId = StorageId_e::LENS_DATA;
                break;
            case 'L':
                storageId = StorageId_e::LOG;
                break;
            case 'm':
                storageId = StorageId_e::MANUFACTURING_DATA;
                break;
            case 'M':
            {
                unsigned mac[6] {0};
                char serialNum[256] {0};
                char modelNum[256] {0};
                char modelName[256] {0};
                int n = sscanf(optarg, "%x:%x:%x:%x:%x:%x,%255[^,],%255[^,],%255[^,]", mac+0, mac+1, mac+2, mac+3, mac+4, mac+5,
                               serialNum, modelNum, modelName);
                if(9 == n)
                {
                    for (size_t i = 0; i < sizeof(manufacturingData.m_MAC); ++i)
                    {
                        manufacturingData.m_MAC[i] = mac[i];
                    }
                    strncpy(manufacturingData.m_serialNumber, serialNum, sizeof(manufacturingData.m_serialNumber));
                    strncpy(manufacturingData.m_modelName, modelName, sizeof(manufacturingData.m_modelName));
                    strncpy(manufacturingData.m_modelNumber, modelNum, sizeof(manufacturingData.m_modelNumber));
                    writeManufacturingData = true;
                }
                else
                {
                    std::cerr << "Invalid manufacturing data: '" << optarg << "' - " << n << std::endl;
                    exit(-1);
                }
                break;
            }
            case 'o':
                storageOffset = atoi(optarg);
                break;
            case 'p':
                  devicePort = optarg;
                  break;
            case 'P':
                calFileName = optarg;
                break;
            case 'r':
                storageMode = StorageMode_e::RAW;
                break;
            case 's':
                storageSize = atoi(optarg);
                break;
            case 'S':
                storageId = StorageId_e::PERSISTENT_SETTINGS;
                break;
            case 't':
                outputMode = OutputMode_e::TEXT;
                break;
            case 'w':
                hexBytesPerLine = atoi(optarg);
                break;
            case 'x':
                outputMode = OutputMode_e::HEX;
                break;
            case 'X':
                maxSizePerRead = atoi(optarg);
                break;
            default:
                break;
        }
    }
}

static int programCalibration(tofcore::Sensor& sensor)
{
    std::ifstream inFile { };
    inFile = std::ifstream(calFileName, (std::ios::in | std::ios::binary));
    if (!inFile.is_open())
    {
        std::cerr << "ERROR: Failed to open input file for calibration data: " << calFileName << std::endl;
        return -1;
    }
    /*
     * Erase existing cal
     */
    if (!sensor.storageWriteStart(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED))
    {
        std::cerr << "FAILED to erase sensor calibration" << std::endl;
        return -1;
    }
    std::cout << "Sensor calibration metadata erased." << std::endl;
    /*
     * Program cal data 4k at a time
     */
    uint32_t offset { 0 };
    uint32_t crc { 0 };
    const auto startTime { high_resolution_clock::now() };
    while (inFile)
    {
        constexpr uint32_t BUFFER_SIZE { 4096 }; // must match sensor NOR's sector size
        uint8_t readBuffer[BUFFER_SIZE];
        ssize_t numRead = inFile.read(reinterpret_cast<char*>(readBuffer), BUFFER_SIZE).gcount();
        if (!sensor.storageWriteData(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED, offset, readBuffer, numRead))
        {
            std::cerr << std::endl << "FAILED writing " << numRead << " bytes to calibration offset " << offset << std::endl;
            return -1;
        }
        crc = updateCrc32(crc, readBuffer, numRead);
        offset += numRead;
        std::cout << std::dec << offset << " (0x" << std::hex << offset << ") bytes programmed\r" << std::flush;
    }
    /*
     * Update size/crc metadata
     */
    if (!sensor.storageWriteFinish(StorageId_e::CALIBRATON, StorageMode_e::EXTRACTED, offset, crc))
    {
        std::cerr << "FAILED to update calibration metadata" << std::endl;
        return -1;
    }
    const auto endTime { high_resolution_clock::now() };
    std::cout << std::endl << "Calibration metadata updated: size = " << std::dec << offset << " (0x" << std::hex << offset
              << "); CRC = 0x" << crc << std::dec << std::endl;
    const auto durationMs = duration_cast<milliseconds>(endTime - startTime);
    const double durationSec { durationMs.count()/1000.0 };
    std::cout << "SUCCESS: Sensor calibration programmed in " << durationSec << " seconds ("
              << (offset/durationSec) << " bytes/sec)" << std::endl;

    return 0;
}

static void readMetadata(tofcore::Sensor& sensor)
{
    auto result = sensor.getStorageMetadata(storageId, storageMode);
    if (!std::get<bool>(result))
    {
        std::cerr << "Failed to read metadata" << std::endl;
    }
    else
    {
        StorageMetadata_T& metadata { std::get<1>(result) };
        std::cout << "size: " << metadata.m_dataSize << " bytes  CRC32: 0x"
                  << std::setw(8) << std::setfill('0') << std::hex << metadata.m_dataCrc32 << std::dec;
        if (StorageMode_e::EXTRACTED == metadata.m_mode)
        {
            std::cout << "  type: " << metadata.m_dataType << "  version: " << metadata.m_typeVersion;
        }
        std::cout << std::endl;
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

        if (calFileName.size() != 0)
        {
            return programCalibration(sensor);
        }
        else if (writeManufacturingData)
        {
            if (!sensor.setFactoryMode(true))
            {
                std::cerr << "setFactoryMode() FAILED" << std::endl;
                exit(-1);
            }
            if (!sensor.storageWriteStart(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED))
            {
                std::cerr << "storageWriteStart() FAILED" << std::endl;
                exit(-1);
            }
            if (!sensor.storageWriteData(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED,
                                         0, (const uint8_t*)&manufacturingData, sizeof(manufacturingData)))
            {
                std::cerr << "storageWriteData() FAILED" << std::endl;
                exit(-1);
            }
            const uint32_t crc { calcCrc32((const uint8_t*)&manufacturingData, sizeof(manufacturingData)) };
            if (!sensor.storageWriteFinish(StorageId_e::MANUFACTURING_DATA, StorageMode_e::EXTRACTED, sizeof(manufacturingData), crc))
            {
                std::cerr << "storageWriteFinish() FAILED" << std::endl;
                exit(-1);
            }
        }
        else if (eraseStorage)
        {
            if (doErase(sensor))
            {
                std::cout << "Erase successful" << std::endl;
            }
            else
            {
                std::cerr << "Erase failed" << std::endl;
            }
        }
        else if (getMetadata)
        {
            readMetadata(sensor);
        }
        else
        {
            auto readResult = doRead(sensor, storageOffset, storageSize);

            if (std::get<bool>(readResult))
            {
                const auto &pData = std::get<1>(readResult);
                if (OutputMode_e::BINARY == outputMode)
                {
                    outputBinary(pData);
                }
                else if (OutputMode_e::HEX == outputMode)
                {
                    outputHex(pData);
                }
                else
                {
                    outputText(pData);
                }
            }
            else
            {
                std::cerr << "Failed to read storage data" << std::endl;
            }
        }

    } // when scope is exited, sensor connection is cleaned up

    return 0;
}
