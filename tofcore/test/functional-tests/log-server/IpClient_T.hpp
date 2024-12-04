#ifndef IPCLIENT_T
#   define IPCLIENT_T
/**
 * @file IpClient_T.hpp
 *
 * Copyright 2021 PreAct Technologies
 *
 */
#   include <memory>
#   include <unistd.h>
#if defined(XILINX) || defined (HOST_UNIT_TEST_FOR_XILINX)
#   include <lwip/sockets.h>
// LwIP breaks the build of a lot of C++ code by defining these MACROs:
#   undef accept
#   undef bind
#   undef read
#   undef write
#else
#   include <netinet/in.h>
#   include <sys/socket.h>
#   include <sys/types.h>
#endif

namespace preact
{

class IpClient_T
{
public:
    virtual ~IpClient_T() = default;

    virtual int closeSocket() = 0;
    virtual bool isOpen() = 0;
    virtual ssize_t recvFrom(void *buf, size_t len, int flags) = 0;
    virtual ssize_t recvFrom(void *buf, size_t len, int flags, struct sockaddr_in* src_addr, socklen_t* addrlen) = 0;
    virtual ssize_t sendTo(const void *buf, size_t len, int flags) = 0;
    virtual ssize_t sendTo(const void *buf, size_t len, int flags, const struct sockaddr_in* src_addr, socklen_t addrlen) = 0;

    /**
     * Creates a IpClient_T instance that uses the specified socket.
     * @param socketFd The already-opened TCP or UDP socket used for communication.
     * @return A unique_ptr to the IpClient_T that utilizes the socket.
     */
    static std::unique_ptr<IpClient_T> create(int socketFd);
};

} // namespace preact

#endif // IPCLIENT_T
