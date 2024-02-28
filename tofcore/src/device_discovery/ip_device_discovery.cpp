/**
 * @file ethernet_device_discovery.cpp
 *
 * Copyright 2023 PreAct Technologies
 *
 * Discovers Ethernet Connected ToF Sensors
 * @{
 */
#include "device_discovery.hpp"
#include "json.hpp"
#include <array>
#include <boost/asio.hpp>
#include <condition_variable>
#include <iostream>
#include <set>
#include <thread>
#include <memory>
#include <string_view>

#if defined(_WIN32)
#   include <iphlpapi.h>

typedef IP_ADAPTER_UNICAST_ADDRESS_LH Addr;
typedef IP_ADAPTER_ADDRESSES* AddrList;

#elif defined(__APPLE__) || defined(__linux__)
#   include <ifaddrs.h>
#endif

namespace tofcore
{
    using namespace std::string_literals;
    using json = nlohmann::json;
    using namespace boost::asio;

    constexpr uint16_t UDP_MULTICAST_PORT {5352};
    constexpr auto UDP_MULTICAST_ADDRESS {"224.0.0.250"};

    constexpr auto DEVICE_INFO_CMP = [](auto& a, auto& b) { return a.connector_uri < b.connector_uri; };
    typedef std::set<device_info_t, decltype(DEVICE_INFO_CMP)> device_set_t;

    static std::shared_ptr<ip::udp::socket> create_listening_socket(io_service& io_service);
    static void do_receive(std::shared_ptr<ip::udp::socket> socket, device_set_t& devices, std::size_t max_count);
    static void send_query_for_sensors(boost::asio::io_service& io_service);


    /** 
     * Primary entry point for discovering IP based devices
    */
    std::vector<device_info_t> find_ip_devices(std::chrono::steady_clock::duration wait_time, std::size_t max_count)
    {
        auto devices = device_set_t {DEVICE_INFO_CMP};
        auto io_service = boost::asio::io_service {};
        auto receive_socket = create_listening_socket(io_service);

        //Setup logic to receive udp packets from devices 
        do_receive(receive_socket, devices, max_count);
        send_query_for_sensors(io_service);
        io_service.run_until(std::chrono::steady_clock::now() + wait_time);

        return {devices.begin(), devices.end()};
    }


#if defined(_WIN32)
    /**
     * Return a set of all network interfaces
     */
    static std::set<std::string> find_interfaces()
    {
        std::set<std::string> ethernet_interfaces;
        // It's a windows machine, we assume it has 512KB free memory
        DWORD outBufLen = 1 << 19;
        AddrList ifaddrs = (AddrList) new char[outBufLen];

        std::vector<ip::address> res;

        ULONG err = GetAdaptersAddresses(AF_INET,
            GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_DNS_SERVER,
            NULL,
            ifaddrs,
            &outBufLen);

        if (err == NO_ERROR)
        {
            for (AddrList addr = ifaddrs; addr != 0; addr = addr->Next)
            {
                if (addr->OperStatus != IfOperStatusUp)
                {
                    continue;
                }

                for (
                    Addr* uaddr = addr->FirstUnicastAddress;
                    uaddr != 0;
                    uaddr = uaddr->Next)
                {
                    if (uaddr->Address.lpSockaddr->sa_family != AF_INET)
                    {
                        continue;
                    }

                    ip::address_v4 ip_addr =
                        ip::make_address_v4(
                            ntohl(
                                reinterpret_cast<sockaddr_in*>(
                                    addr->FirstUnicastAddress->Address.lpSockaddr)->sin_addr.s_addr));

                    ethernet_interfaces.insert(ip_addr.to_string());
                }
            }
        }
        else
        {
            std::cerr << "GetAdaptersAddresses() FAILED: " << err << std::endl;
            return ethernet_interfaces; // return early 'cause there's nothing else we can do
        }
        delete[]((char*)ifaddrs);
        return ethernet_interfaces;
    }

#elif defined(__APPLE__) || defined(__linux__)
    /**
     * Return a set of all network interfaces
     */
    static std::set<std::string> find_interfaces()
    {
        struct ifaddrs *ifaddr;
        std::set<std::string> ethernet_interfaces;

        if (getifaddrs(&ifaddr) != 0)
        {
            std::cerr << "getifaddrs() FAILED: " << strerror(errno) << std::endl;
            return ethernet_interfaces; // return early 'cause there's nothing else we can do
        }

        uint32_t numIfs { 0 };

        for (struct ifaddrs *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next)
        {
            ++numIfs;
            if (ifa->ifa_addr == nullptr)
            {
                continue;
            }
            int family = ifa->ifa_addr->sa_family;
            if (AF_INET == family)
            {
                char ifAddr[NI_MAXHOST];
                if (getnameinfo(
                    ifa->ifa_addr,
                    sizeof(struct sockaddr_in),
                    ifAddr,
                    NI_MAXHOST,
                    NULL,
                    0,
                    NI_NUMERICHOST) != 0)
                {
                    std::cerr << "getnameinfo() FAILED: " << strerror(errno) << std::endl;
                }
                else
                {
                    ethernet_interfaces.insert(ifAddr);
                }
            }
        }
        freeifaddrs(ifaddr);
        return ethernet_interfaces;
    }
#endif

    /**
     * Take a json formatted message from a device, parse it, and add the devices set
     */
    static void try_add_device(std::string_view message, device_set_t& devices)
    {
        try
        {
            json j = json::parse(message);
            if (j.contains("_service") && //
                (j["_service"].get<std::string>() == "TOF"))
            {
                if (j.contains("addr"))
                {
                    std::string sensorAddress = j["addr"].get<std::string>();
                    std::string connector_uri = "tofnet://" + sensorAddress;

                    device_info_t deviceEntry;

                    deviceEntry.connector_uri = connector_uri;

                    std::string model_name = j["modelName"].get<std::string>();
                    deviceEntry.model = model_name;

                    std::string serial_num = j["deviceSerialNumber"].get<std::string>();
                    deviceEntry.serial_num = serial_num;

                    std::string name = j["sensorName"].get<std::string>();
                    deviceEntry.sensor_name = name;

                    std::string location = j["sensorLocation"].get<std::string>();
                    deviceEntry.location = location;

                    // Add Preact device to list
                    devices.insert(deviceEntry);
                }
            }
        }
        catch (...)
        {
            std::cerr << "Invalid JSON: " << message << std::endl;
        }
    }

    /**
     * Periodically send query to sensors.
     */
    void send_query_for_sensors(boost::asio::io_service& io_service)
    {
        json j = { { "_service", "query" } };
        static std::string s = j.dump();
        const auto destination_endpoint = ip::udp::endpoint {ip::address::from_string(UDP_MULTICAST_ADDRESS), UDP_MULTICAST_PORT };

        for (const auto &ifAddr : find_interfaces())
        {
            try
            {
                auto socket = std::make_shared<ip::udp::socket>(io_service);
                socket->open(ip::udp::v4());
                socket->set_option(ip::udp::socket::reuse_address(true));
                socket->bind(
                    ip::udp::endpoint(
                        ip::address::from_string(ifAddr),
                        UDP_MULTICAST_PORT));
            
                const_buffer buf(s.c_str(), s.size());
                socket->async_send_to(buf, destination_endpoint,
                    [socket](boost::system::error_code ec, std::size_t /*length*/) 
                    {
                        if(ec)
                        {
                            std::cerr << "failure sending query message: " << ec.message() << std::endl;
                        }
                    });
            }
            catch(const std::exception& e)
            {
                std::cerr << "Error sending UDP query broadcast: " << e.what() << '\n';
            }
        }
    }


    /** Buffer used by do_receive() to receive UDP packets into */
    static auto message =  std::array<char, 1500>{};

    /**
     * Perform receive operations on passed in socket, add any discovered devices passed in devices set. 
     * Stop listening for more devices if max_count devices is reached. 
    */
    void do_receive(std::shared_ptr<ip::udp::socket> socket, device_set_t& devices, std::size_t max_count)
    {
        socket->async_receive(
            boost::asio::buffer(message),
            [socket, &devices, max_count](boost::system::error_code ec, std::size_t cnt)
            {
                if (!ec)
                {
                    auto v = std::string_view(message.data(), cnt);
                    try_add_device(v, devices);
                }
                //Queue up another receive, unless the last one was canceled.
                if(!ec || ec != boost::asio::error::operation_aborted)
                {
                    if(devices.size() < max_count)
                    {
                        do_receive(socket, devices, max_count);
                    }
                }
            });
    }

    /**
     * Setup a socket to receive multicast messages from all
     * interfaces. Start sending query messages and watching for replies.
     */
    std::shared_ptr<ip::udp::socket> create_listening_socket(io_service& io_service)
    {
        ip::udp::endpoint listen_endpoint(
            ip::udp::v4(),
            UDP_MULTICAST_PORT);
        auto socket = std::make_shared<ip::udp::socket>(io_service);

        socket->open(listen_endpoint.protocol());
        socket->set_option(ip::udp::socket::reuse_address(true));
        socket->bind(listen_endpoint);

        for (auto const& ifAddr : find_interfaces())
        {
            socket->set_option(ip::multicast::join_group(
                ip::address::from_string(UDP_MULTICAST_ADDRESS).to_v4(),
                ip::address::from_string(ifAddr).to_v4()));
        }
        return socket;
    }

} //end namespace
