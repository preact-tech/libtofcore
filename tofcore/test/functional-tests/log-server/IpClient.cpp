/**
 * @file IpClient.cpp
 *
 * Copyright 2021 PreAct Technologies
 *
 */
#include "IpClient_T.hpp"
//#include "Logger_T.hpp"
#if defined(XILINX) || defined (HOST_UNIT_TEST_FOR_XILINX)
#   include <lwip/sockets.h>
#else
#   include <sys/socket.h>
#   include <sys/types.h>
#endif

namespace preact
{

constexpr const char *subSys = "ipclient";

class IpClient: public IpClient_T
{
public:

    IpClient(int socketFd) :
                IpClient_T(), m_socketFd(socketFd)
    {
    }
    virtual ~IpClient()
    {
        closeSocket();
    }
    virtual int closeSocket() override
    {
        int fd = m_socketFd;
        m_socketFd = -1;
        if (fd < 0)
        {
            return -1;
        }
        else
        {
#if defined(XILINX) || defined (HOST_UNIT_TEST_FOR_XILINX)
            shutdown(fd, SHUT_RDWR);
            return closesocket(fd);
#else
            ::shutdown(fd, SHUT_RDWR);
            return ::close(fd);
#endif
        }
    }
    virtual bool isOpen() override
    {
        return (m_socketFd >= 0);
    }
    virtual ssize_t recvFrom(void *buf, size_t len, int flags) override;
    virtual ssize_t recvFrom(void *buf, size_t len, int flags, struct sockaddr_in* src_addr, socklen_t* addrlen) override;
    virtual ssize_t sendTo(const void *buf, size_t len, int flags) override;
    virtual ssize_t sendTo(const void *buf, size_t len, int flags, const struct sockaddr_in* src_addr, socklen_t addrlen) override;

protected:

    int m_socketFd { -1 };
};

ssize_t IpClient::recvFrom(void *buf, size_t len, int flags)
{
    return recvFrom(buf, len, flags, nullptr, nullptr);
}

ssize_t IpClient::recvFrom(void *buf, size_t len, int flags, struct sockaddr_in* src_addr, socklen_t* addrlen)
{
#if defined(XILINX) || defined (HOST_UNIT_TEST_FOR_XILINX)
    return lwip_recvfrom(m_socketFd, buf, len, flags, (struct sockaddr*)src_addr, addrlen);
#else
    return ::recvfrom(m_socketFd, buf, len, flags, (struct sockaddr*)src_addr, addrlen);
#endif
}

ssize_t IpClient::sendTo(const void *buf, size_t len, int flags)
{
    return sendTo(buf, len, flags, nullptr, 0);
}

ssize_t IpClient::sendTo(const void *buf, size_t len, int flags, const struct sockaddr_in* src_addr, socklen_t addrlen)
{
#if defined(XILINX) || defined (HOST_UNIT_TEST_FOR_XILINX)
    return lwip_sendto(m_socketFd, buf, len, flags, (const struct sockaddr*)src_addr, addrlen);
#else
    return ::sendto(m_socketFd, buf, len, flags, (const struct sockaddr*)src_addr, addrlen);
#endif
}


std::unique_ptr<IpClient_T> IpClient_T::create(int socketFd)
{
    std::unique_ptr<IpClient_T> client = std::make_unique<IpClient>(socketFd);
    return client;
}

} // namespace preact
