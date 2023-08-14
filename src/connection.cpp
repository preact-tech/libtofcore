#include "connection.hpp"
#include "comm_ip/ip_connection.hpp"
#include "comm_serial/serial_connection.hpp"
#include "uri.hpp"

#include <string>
#include <regex>
#include <optional>

namespace tofcore
{
using namespace std::string_literals;

uri parse_as_uri(const std::string& str)
{
    try
    {
        //Start out by simply trying to parse str as a uri, if successful we are done
        return uri(str);
    } catch(std::invalid_argument& e)
    {
        //Typical error is no scheme (e.g. tofserial: or tofnet:, etc) so inspect the string and try to deduce the scheme
        // Does it look like a Linux COM device: /dev/<something>
        // or Windows COM device: com##
        const std::regex serial_device_regex( "^\\/dev|^[Cc][Oo][Mm]", std::regex_constants::ECMAScript);
        if(std::regex_search(str, serial_device_regex))
        {
            //Add the tofserial schema and try parsing again.
            return uri("tofserial:" + str);
        }
        else
        {
            //assume it's an IP address or hostname
            return uri("tofnet://" + str);
        }
    }
}


std::unique_ptr<Connection_T> Connection_T::create(boost::asio::io_service& io, const std::string& uri_str)
{
    auto uri = parse_as_uri(uri_str);
    if(uri.get_scheme() == "tofnet")
    {
        return std::make_unique<IpConnection>(io, uri);
    }
    else if(uri.get_scheme() == "tofserial")
    {
        return std::make_unique<SerialConnection>(io, uri);
    }
    else
    {
        throw std::invalid_argument("unknown connection uri scheme: "s + uri.get_scheme());
    }
}

} //end namespace tofcore