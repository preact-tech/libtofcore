/**
 * @file log-server.cpp
 *
 * Copyright 2024 PreAct Technologies
 *
 * Test program that receives and prints sensor UDP log output.
 */
#include "IpClient_T.hpp"
#include <cstring>
#include <errno.h>
#include <iostream>
#include <sys/socket.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace preact;

constexpr uint16_t DEFAULT_UDP_PORT { 5001 };
constexpr size_t RCV_BUFFER_SIZE { 1500 };

static uint16_t udpPort { DEFAULT_UDP_PORT };

static int createUdpServer()
{
    int udpFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpFd < 0)
    {
        std::cerr << "socket() FAILED: " << strerror(errno) << "\n";
    }
    else
    {
        struct sockaddr_in udpAddr { };
        udpAddr.sin_family = AF_INET;
        udpAddr.sin_port = htons(udpPort);
        udpAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(udpFd, (const struct sockaddr*) &udpAddr, sizeof(udpAddr)) < 0)
        {
            std::cerr << "bind() FAILED: " << strerror(errno) << "\n";
            close(udpFd);
            udpFd = -1;
        }
    }
    return udpFd;
}

static void parseArgs(int argc, char *argv[])
{
    po::options_description desc("Receive sensor UDP log stream and print to console");
    desc.add_options()
        ("help,h", "produce help message")
        ("udp-port,p", po::value<uint16_t>(&udpPort), "UDP port to which sensor is sending log")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    parseArgs(argc, argv);
    int udpFd = createUdpServer();

    if (udpFd >= 0)
    {
        std::shared_ptr<IpClient_T> udpServer = IpClient_T::create(udpFd);

        uint8_t rcvBuffer[RCV_BUFFER_SIZE];
        while (true)
        {
            ssize_t n = udpServer->recvFrom(rcvBuffer, sizeof(rcvBuffer)-1, 0);
            if (n < 0)
            {
                std::cerr << "recv() FAILED: " << strerror(errno) << "\n";
                break;
            }
            rcvBuffer[n] = 0;
            std::cout << reinterpret_cast<const char*>(rcvBuffer);
        }
    }
    return -1;
}
