#include <iostream>
#include "tcp_connection.hpp"

#define DBG(m,l)                                        \
     do                                                 \
     {                                                  \
         if (m_log_callback) {                          \
             std::stringstream ss {};                   \
             ss << m;                                   \
             m_log_callback(ss.str(), l);               \
         }                                              \
     } while(false)

#define ERR(m)                                          \
    do                                                  \
    {                                                   \
        if (m_log_callback) {                           \
            std::stringstream ss {};                    \
            ss << m;                                    \
            m_log_callback(ss.str(), LOG_LVL_ERROR);    \
        }                                               \
    } while(false)


namespace tofcore
{
    using namespace boost;
    using namespace boost::system;
    using namespace boost::asio;
    using boost::asio::ip::tcp;

    constexpr auto DEFAULT_PORT { 50660ul };
    constexpr auto DEFAULT_HOST {"10.10.31.180"};

    constexpr uint32_t ANSWER_START_PATTERN = 0xFFFFAA55;   ///< Pattern marking the start of an answer

TcpConnection::TcpConnection(boost::asio::io_service &ioService,
                             const uri &uri,
                             log_callback_t log_callback,
                             cmd_descr_callback_t cmd_descr_callback) :
            socket(ioService),
            resolver(ioService),
            m_response_timer(ioService),
            m_log_callback(log_callback),
            m_cmd_descr_callback(cmd_descr_callback)
    {
        auto host = uri.get_host();
        auto port = uri.get_port();
        if(host.empty())
        {
            host = DEFAULT_HOST;
        }
        if(port == 0)
        {
            port = DEFAULT_PORT;
        }
        connect(host, port);
    }

    TcpConnection::~TcpConnection()
    {
        try
        {
            disconnect();
        }
        catch (boost::system::system_error &e)
        {
            ERR(e.what());
        }
    }


    void TcpConnection::send_receive_async(const std::vector<std::byte> &data,
        std::chrono::steady_clock::duration timeout, TcpConnection::on_command_response_callback_t callback)
    {
        this->send(data);
        this->receive_async(callback);
        this->begin_response_timer(timeout);
    }


    void TcpConnection::begin_response_timer(std::chrono::steady_clock::duration timeout)
    {
        auto f = [&](const auto &error)
        {
            this->on_response_timeout(error);
        };

        this->m_response_timer.expires_after(timeout);
        this->m_response_timer.async_wait(f);
    }


    void TcpConnection::on_response_timeout(const system::error_code &error)
    {
        // If the timer expires (timeout), there is no error
        if (error && ( (error == boost::asio::error::operation_aborted) ||
                    !process_error(error, __FUNCTION__)
                    ))
        {
            return;
        }

        //The timer went off and so a "timeout" occurred while waiting for the response
        //First make sure to cancel any pending reads, 
        //TODO figure out how to make sure the canceled has completed, prior to allowing another request to be sent
        // otherwise we could end up in a race condition where the next command read is canceled.
        this->socket.cancel();
        if (this->m_on_command_response)
        {
            std::vector<std::byte> tmp;
            this->m_on_command_response(true, tmp);
            this->m_on_command_response = nullptr;
        }
    }

    void TcpConnection::send(const std::vector<std::byte> &data)
    {
        if (!isConnected())
        {
            return;
        }
        /*
         * Send the command stream
         */
        boost::system::error_code error;
        socket.write_some(boost::asio::buffer(data.data(), data.size()), error);
        if (error)
        {
            throw boost::system::system_error(error);
        }
    }

    void TcpConnection::begin_receive_prolog()
    {
        auto f = [&](const auto &error, auto)
        {
            this->on_receive_prolog(error);
        };
        constexpr size_t ANSWER_PROLOG_SIZE { 2 * sizeof(uint32_t) + sizeof(uint16_t) + 3 * sizeof(uint8_t) };

        this->m_prolog_epilog_buf.resize(ANSWER_PROLOG_SIZE);
        boost::asio::async_read(this->socket, buffer(this->m_prolog_epilog_buf, ANSWER_PROLOG_SIZE), f);
    }

    void TcpConnection::on_receive_prolog(const system::error_code &error)
    {
        if (error && !process_error(error, __FUNCTION__))
        {
            return;
        }
        const uint32_t start_marker = ::ntohl(*reinterpret_cast<const uint32_t *>(m_prolog_epilog_buf.data() + 0));
        if (start_marker != ANSWER_START_PATTERN)
        {
            ERR("Command response start pattern is incorrect");
            //Could be cruft from previous response, try again.
            this->begin_receive_prolog();
            return;
        }
        constexpr uint32_t MAX_ANSWER_PAYLOAD_EXPECTED { 64 * 1024 * 1024 }; // We should never have an answer this big
        const uint32_t answerSize = ::ntohl(*reinterpret_cast<const uint32_t *>(m_prolog_epilog_buf.data() + 6));
        if ((answerSize < 3) ||(answerSize > MAX_ANSWER_PAYLOAD_EXPECTED + 3))
        {
            ERR("Command response too big");
            //Could have been cruft from previous response, try again.
            this->begin_receive_prolog();
            return;
        }
        const uint32_t payload_size { answerSize - 3 };
        this->m_response_cid = ::ntohs(*reinterpret_cast<const uint16_t *>(m_prolog_epilog_buf.data() + 10));
        this->m_response_result = *(m_prolog_epilog_buf.data() + 12);

        // Now read the payload and final CRC bytes
        m_response_buf.resize(payload_size);
        m_prolog_epilog_buf.resize(4);
        auto f = [&](const auto &error, auto)
        {
            this->on_receive_payload(error);
        };
        auto bufs = std::array<boost::asio::mutable_buffer, 2>
            { buffer(this->m_response_buf, payload_size), buffer(m_prolog_epilog_buf, 4) };
        boost::asio::async_read(this->socket, bufs, f);
    }


    void TcpConnection::on_receive_payload(const system::error_code &error)
    {
        if (error && !process_error(error, __FUNCTION__))
        {
            return;
        }
        //RECEIVED the payload and the CRC, validate the CRC and then deliver the payload to the client
        //via the callback.
        //TODO check the crc
        if(this->m_on_command_response)
        {
            //Make sure to clear the handler `m_on_command_response` 
            // member variable and cancel the timer before calling 
            // the handler. Otherwise a race condition can ensue due
            // to the fact that the client is likely running in another
            // thread that could become unblocked and even run far enough
            // to start another command sequence prior to this thread
            // returning from the handler callback.
            auto on_command_response = this->m_on_command_response;
            this->m_on_command_response = nullptr;
            this->m_response_timer.cancel();
            auto isValid = this->is_valid_response();
            on_command_response(!isValid, this->m_response_buf);
        }
    }


    bool TcpConnection::is_valid_response()
    {
        auto isValid { true };
        if(this->m_response_result != std::byte{0})
        {
            isValid = false;
        }
        return isValid;
    }


    void TcpConnection::receive_async(TcpConnection::on_command_response_callback_t callback)
    {
        this->m_on_command_response = callback;
        this->begin_receive_prolog();
    }


    void TcpConnection::connect(const std::string& host, unsigned long port)
    {
        if (isConnected())
        {
            return;
        }

        tcp::resolver::query query(host, std::to_string(port));
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        tcp::resolver::iterator end;

        boost::system::error_code error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end)
        {
            socket.close();
            socket.connect(*endpoint_iterator++, error);
        }
        if (error)
        {
            throw ::boost::system::system_error(error);
        }
    }

    void TcpConnection::disconnect()
    {
        if (isDisconnected())
        {
            return;
        }

        boost::system::error_code error;
        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
        if (error)
        {
            throw boost::system::system_error(error);
        }
    }

    bool TcpConnection::isConnected() const
    {
        return this->socket.is_open();
    }

    bool TcpConnection::isDisconnected() const
    {
        return !this->isConnected();
    }

    bool TcpConnection::process_error(const boost::system::error_code &error, const char* where)
    {
        if (!error)
        {
            DBG("NO ERROR in " << where, LOG_LVL_DBG_LOW);
            return true;
        }
        else if((error == boost::asio::error::operation_aborted))
        {
            DBG("OPERATION aborted in " << where, LOG_LVL_DBG_HI);
            return false;
        }
        else
        {
            ERR("ERROR in " << where << ": " << error.message());
            return false;
        }
    }

} // end namespace tofcore
