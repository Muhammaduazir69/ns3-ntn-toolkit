/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "e2-transport.h"

#include "ns3/log.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// libsctp — Linux SCTP user-space header.
#include <netinet/sctp.h>

namespace ns3
{
namespace oranntn
{
namespace flexric
{

NS_LOG_COMPONENT_DEFINE("E2Transport");

namespace
{

/// Common helper that frames + writes one E2 message into an open
/// SOCK_STREAM socket. Returns the number of payload bytes written
/// (excluding the 6-byte header) or -1 on error.
int
SendFramed(int fd, const E2Message& msg)
{
    if (fd < 0)
        return -1;
    const uint16_t type_n = htons(static_cast<uint16_t>(msg.type));
    const uint32_t len_n = htonl(static_cast<uint32_t>(msg.payload.size()));
    // Emit the whole frame (6-byte header + payload) in ONE send() so that, on
    // SCTP (a message-oriented transport), each frame is exactly one SCTP
    // message. The earlier two-send form put the header and payload in separate
    // SCTP messages, which desynchronised the framed reader after the first
    // exchange. A single buffer is equally correct for TCP.
    std::vector<uint8_t> frame;
    frame.reserve(6 + msg.payload.size());
    frame.resize(6);
    std::memcpy(frame.data(), &type_n, 2);
    std::memcpy(frame.data() + 2, &len_n, 4);
    frame.insert(frame.end(), msg.payload.begin(), msg.payload.end());
    size_t off = 0;
    while (off < frame.size())
    {
        ssize_t n = ::send(fd, frame.data() + off, frame.size() - off, 0);
        if (n <= 0)
            return -1;
        off += static_cast<size_t>(n);
    }
    return static_cast<int>(msg.payload.size());
}

bool
RecvAll(int fd, uint8_t* buf, size_t n, uint32_t timeout_ms)
{
    size_t got = 0;
    while (got < n)
    {
        struct pollfd p{};
        p.fd = fd;
        p.events = POLLIN;
        const int pr = ::poll(&p, 1, static_cast<int>(timeout_ms));
        if (pr <= 0)
            return false;
        ssize_t r = ::recv(fd, buf + got, n - got, 0);
        if (r <= 0)
            return false;
        got += static_cast<size_t>(r);
    }
    return true;
}

bool
RecvFramed(int fd, E2Message& out, uint32_t timeout_ms)
{
    if (fd < 0)
        return false;
    uint8_t header[6];
    if (!RecvAll(fd, header, 6, timeout_ms))
        return false;
    uint16_t type_n;
    uint32_t len_n;
    std::memcpy(&type_n, header, 2);
    std::memcpy(&len_n, header + 2, 4);
    const uint16_t type = ntohs(type_n);
    const uint32_t len = ntohl(len_n);
    if (len > 8u * 1024u * 1024u)  // 8 MB sanity cap
    {
        NS_LOG_WARN("E2 frame size " << len << " > 8 MB cap");
        return false;
    }
    out.type = static_cast<E2MessageType>(type);
    out.payload.assign(len, 0);
    if (len > 0 && !RecvAll(fd, out.payload.data(), len, timeout_ms))
    {
        return false;
    }
    return true;
}

/// Common SOCK_STREAM transport — code reused by both TCP and SCTP
/// concrete classes; only the socket() arguments differ.
class StreamTransport : public E2Transport
{
  public:
    StreamTransport(int sockType, int sockProtocol, Protocol kind)
        : m_type(sockType),
          m_proto(sockProtocol),
          m_kind(kind)
    {
    }

    ~StreamTransport() override
    {
        Close();
    }

    bool Listen(const std::string& host, uint16_t port) override
    {
        if (m_fd >= 0)
            Close();
        m_fd = ::socket(AF_INET, m_type, m_proto);
        if (m_fd < 0)
        {
            NS_LOG_WARN("socket() failed: " << std::strerror(errno));
            return false;
        }
        int yes = 1;
        ::setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        {
            NS_LOG_WARN("inet_pton failed for " << host);
            Close();
            return false;
        }
        if (::bind(m_fd, reinterpret_cast<struct sockaddr*>(&addr),
                    sizeof(addr)) != 0)
        {
            NS_LOG_WARN("bind() failed: " << std::strerror(errno));
            Close();
            return false;
        }
        if (::listen(m_fd, 4) != 0)
        {
            NS_LOG_WARN("listen() failed: " << std::strerror(errno));
            Close();
            return false;
        }
        return true;
    }

    bool Connect(const std::string& host, uint16_t port) override
    {
        if (m_fd >= 0)
            Close();
        m_fd = ::socket(AF_INET, m_type, m_proto);
        if (m_fd < 0)
        {
            NS_LOG_WARN("socket() failed: " << std::strerror(errno));
            return false;
        }
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        {
            Close();
            return false;
        }
        if (::connect(m_fd, reinterpret_cast<struct sockaddr*>(&addr),
                       sizeof(addr)) != 0)
        {
            NS_LOG_WARN("connect() failed: " << std::strerror(errno));
            Close();
            return false;
        }
        return true;
    }

    std::unique_ptr<E2Transport>
    AcceptOne(uint32_t timeout_ms) override
    {
        if (m_fd < 0)
            return nullptr;
        struct pollfd p{};
        p.fd = m_fd;
        p.events = POLLIN;
        if (::poll(&p, 1, static_cast<int>(timeout_ms)) <= 0)
        {
            return nullptr;
        }
        struct sockaddr_in peer{};
        socklen_t plen = sizeof(peer);
        int afd = ::accept(m_fd, reinterpret_cast<struct sockaddr*>(&peer),
                            &plen);
        if (afd < 0)
        {
            return nullptr;
        }
        auto out = std::make_unique<StreamTransport>(m_type, m_proto, m_kind);
        out->m_fd = afd;
        return out;
    }

    int Send(const E2Message& msg) override
    {
        return SendFramed(m_fd, msg);
    }

    bool Recv(E2Message& out, uint32_t timeout_ms) override
    {
        return RecvFramed(m_fd, out, timeout_ms);
    }

    void Close() override
    {
        if (m_fd >= 0)
        {
            ::shutdown(m_fd, SHUT_RDWR);
            ::close(m_fd);
            m_fd = -1;
        }
    }

    Protocol Kind() const override { return m_kind; }
    bool IsOpen() const override { return m_fd >= 0; }

  private:
    int m_fd{-1};
    int m_type{0};
    int m_proto{0};
    Protocol m_kind;
};

} // namespace

std::unique_ptr<E2Transport>
MakeE2Transport(E2Transport::Protocol p)
{
    if (p == E2Transport::Protocol::tcp)
    {
        return std::make_unique<StreamTransport>(SOCK_STREAM, IPPROTO_TCP,
                                                  E2Transport::Protocol::tcp);
    }
    // SCTP SOCK_STREAM. Falls back to nullptr if the kernel does not
    // support SCTP — caller can retry with TCP.
    auto sctp = std::make_unique<StreamTransport>(SOCK_STREAM, IPPROTO_SCTP,
                                                   E2Transport::Protocol::sctp);
    return sctp;
}

} // namespace flexric
} // namespace oranntn
} // namespace ns3
