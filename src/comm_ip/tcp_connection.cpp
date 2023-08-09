#include <iostream>
#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace tofcore
{

    typedef std::vector<uint8_t> Packet;

    TcpConnection::TcpConnection(boost::asio::io_service &ioService)
        : state(STATE_DISCONNECTED), socket(ioService), resolver(ioService)
    {
        connect();
    }

    TcpConnection::~TcpConnection()
    {
        try
        {
            disconnect();
        }
        catch (boost::system::system_error &e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

    void TcpConnection::sendCommand(const std::vector<std::byte> &data)
    {
        std::vector<std::byte> answer;
        this->sendCommand(data, answer);
    }

    void TcpConnection::sendCommand(const std::vector<std::byte> &data, std::vector<std::byte> &payload)
    {
        if (!isConnected())
            return;
        /*
         * Send the command stream
         */
        boost::system::error_code error;
        socket.write_some(boost::asio::buffer(data.data(), data.size()), error);
        if (error)
        {
            throw boost::system::system_error(error);
        }
        /*
         * Read the command response
         *              ------------ ---------- ---------- ------------ ----------- -------- ------------ -----------
         * VERS. 1     | 0xFFFFAA55 | PID      | TYPE (0) | LEN (sz+3) |   CID     | Result |  Payload   | CRC32     |
         * CMD Answer: |            | (1 byte) | (1 byte) | (4 bytes)  | (2 bytes) | 1 byte | (sz bytes) | (4 bytes) |
         *              ------------ ---------- ---------- ------------ ----------- -------- ------------ -----------
         *             |<------------------- prolog --------------------------------------->|
         */
        constexpr uint32_t ANSWER_START_PATTERN = 0xFFFFAA55;   ///< Pattern marking the start of an answer
        constexpr size_t ANSWER_PROLOG_SIZE { 2 * sizeof(uint32_t) + sizeof(uint16_t) + 3 * sizeof(uint8_t) };
        Packet buf(ANSWER_PROLOG_SIZE);

        auto len = boost::asio::read(socket, boost::asio::buffer(buf));
        if (len != buf.size())
        {
            throw std::runtime_error("Failed to read command response prolog");
        }
        const uint32_t start_marker = ::ntohl(*reinterpret_cast<const uint32_t *>(buf.data() + 0));
        if (start_marker != ANSWER_START_PATTERN)
        {
            throw std::runtime_error("Command response start pattern is incorrect");
        }
//        const uint8_t pid = *reinterpret_cast<const uint8_t *>(buf.data() + 4);
//        const uint8_t type = *reinterpret_cast<const uint8_t *>(buf.data() + 5);
        constexpr uint32_t MAX_ANSWER_PAYLOAD_EXPECTED { (4 * 1024) + 256 };
        const uint32_t answerSize = ::ntohl(*reinterpret_cast<const uint32_t *>(buf.data() + 6));
        if ((answerSize < 3) ||(answerSize > MAX_ANSWER_PAYLOAD_EXPECTED + 3))
        {
            throw std::runtime_error("Command response too big");
        }
        const uint32_t payload_size { answerSize - 3 };
//        const uint16_t cid = ::ntohl(*reinterpret_cast<const uint32_t *>(buf.data() + 10));
        const uint8_t result = *reinterpret_cast<const uint8_t *>(buf.data() + 12);
        if (result != 0)
        {
            throw std::runtime_error("Command failed");
        }

        // Now read the payload
        payload.resize(payload_size);
        if (payload_size > 0)
        {
            len = boost::asio::read(socket, boost::asio::buffer(payload));
            if (payload_size != len)
            {
                throw std::runtime_error("Failed to read command response's payload");
            }
        }
        // Read the CRC
        buf.resize(4);
        len = boost::asio::read(socket, boost::asio::buffer(buf));
        if (buf.size() != len)
        {
            throw std::runtime_error("Failed to read command response's CRC");
        }
    }

    void TcpConnection::connect()
    {
        if (isConnected())
            return;

        updateState(STATE_CONNECTING);
        tcp::resolver::query query(HOST, PORT);
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
        updateState(STATE_CONNECTED);
    }

    void TcpConnection::disconnect()
    {
        if (isDisconnected())
            return;

        updateState(STATE_CLOSING);

        boost::system::error_code error;
        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
        if (error)
        {
            revertState();
            throw boost::system::system_error(error);
        }
        updateState(STATE_DISCONNECTED);
    }

    void TcpConnection::waitAck()
    {
        Packet buf(ACK_BUF_SIZE);
        boost::system::error_code error;

        this->updateState(STATE_WAIT_ACK);
        size_t len = socket.read_some(boost::asio::buffer(buf), error);
        (void)len;
        if (error)
        {
            throw boost::system::system_error(error);
        }
        this->revertState();
    }

    void TcpConnection::updateState(State state_) const
    {
        previousState = state;
        state = state_;
    }

    void TcpConnection::revertState() const
    {
        state = previousState;
    }

    bool TcpConnection::isConnected() const
    {
        return state == STATE_CONNECTED;
    }

    bool TcpConnection::isDisconnected() const
    {
        return state == STATE_DISCONNECTED;
    }

} // end namespace tofcore
