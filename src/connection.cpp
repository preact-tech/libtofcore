#include "connection.hpp"
#include "comm_ip/ip_connection.hpp"
#include "comm_serial/serial_connection.hpp"

namespace tofcore
{

std::unique_ptr<Connection_T> Connection_T::create(boost::asio::io_service& io, const std::string& uri)
{
    if(uri == "ip")
    {
        return std::make_unique<IpConnection>(io, "", 0);
    }
    return std::make_unique<SerialConnection>(io, uri, 19200, 1);
}


} //end namespace tofcore