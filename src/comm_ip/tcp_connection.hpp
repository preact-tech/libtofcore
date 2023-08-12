#ifndef __TOFCORE_TCPCONNECTION_H__
#define __TOFCORE_TCPCONNECTION_H__

#include "uri.hpp"
#include <boost/asio.hpp>

namespace tofcore
{
    class TcpConnection
    {
    public:
        enum State
        {
            STATE_CONNECTING,
            STATE_DISCONNECTED,
            STATE_CONNECTED,
            STATE_CLOSING,
            STATE_WAIT_ACK
        };


        TcpConnection(boost::asio::io_service &, const uri& uri);
        ~TcpConnection();

        typedef std::function<void(bool, const std::vector<std::byte>&)> on_command_response_callback_t; 
        void send_receive_async(const std::vector<std::byte> &data,
            std::chrono::steady_clock::duration timeout, on_command_response_callback_t callback);

        void send(const std::vector<std::byte> &data);

    private:
        mutable State state, previousState;
        boost::asio::ip::tcp::socket socket;
        boost::asio::ip::tcp::resolver resolver;
        on_command_response_callback_t m_on_command_response;
        boost::asio::steady_timer m_response_timer;
        std::vector<std::byte> m_prolog_epilog_buf;
        std::vector<std::byte> m_response_buf;

        //uint16_t m_response_cid { 0 };
        std::byte m_response_result { 0 };

        void connect(const std::string& host, unsigned long port);
        void disconnect();
        void updateState(State) const;
        void revertState() const;
        bool isConnected() const;
        bool isDisconnected() const;

        void begin_response_timer(std::chrono::steady_clock::duration timeout);
        void on_response_timeout(const boost::system::error_code &error);

        void receive_async(TcpConnection::on_command_response_callback_t callback);
        void begin_receive_prolog();
        void on_receive_prolog(const boost::system::error_code &error);
        void on_receive_payload(const boost::system::error_code &error);
        bool is_valid_response();

    };

} // end namespace tofcore

#endif // __TOFCORE_TCPCONNECTION_H__
