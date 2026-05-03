/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_INFLUX_SINK_H
#define NTN_INFLUX_SINK_H

#include <ns3/event-id.h>
#include <ns3/nstime.h>
#include <ns3/object.h>

#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnobs
{

/// One InfluxDB line-protocol point: measurement + tags + fields + timestamp(ns).
struct Point
{
    std::string measurement;
    std::map<std::string, std::string> tags;
    std::map<std::string, double> fieldsFloat;
    std::map<std::string, std::string> fieldsString;
    std::map<std::string, long long> fieldsInt;
    Time timestamp{Seconds(0)};
};

/**
 * Encode `pts` to InfluxDB v1.x / v2.x line protocol text. Tag keys/values are
 * comma + space + equal-sign escaped per the spec; field strings are
 * double-quote escaped. Timestamps are nanoseconds since epoch (or
 * `useSimulationTime=true` to emit Simulator::Now() ns).
 *
 * Returns one '\\n'-terminated line per point.
 */
std::string EncodeLineProtocol(const std::vector<Point>& pts, bool useSimulationTime = false);

/**
 * \ingroup ntn-observability
 *
 * Buffer of `Point`s flushed periodically to an InfluxDB endpoint.
 *
 * Two transports are supported:
 *   - **UDP** (default; matches `influxdb_v1` UDP listener): low-overhead,
 *     fire-and-forget, lossy under MTU pressure.
 *   - **File**: appends line protocol to a local file. Used for offline
 *     scenarios and for the deterministic unit tests.
 *
 * HTTP/v2 transport is intentionally NOT in this module — it pulls in libcurl
 * and async event-loop concerns that fight with ns-3's discrete event
 * simulator. Users wanting v2 push: import the file dump with
 * `influx write` after the run.
 */
class NtnInfluxSink : public Object
{
  public:
    enum class Transport
    {
        Udp,
        File,
    };

    static TypeId GetTypeId();

    NtnInfluxSink();
    ~NtnInfluxSink() override;

    void SetTransport(Transport t);
    void SetUdpEndpoint(const std::string& host, uint16_t port = 8089);
    void SetFilePath(const std::string& path);
    void SetFlushPeriod(Time period);
    void SetUseSimulationTime(bool yes);

    /// Append a fully-formed Point to the buffer. The implementation is
    /// reentrant-safe across simulation time but not across threads.
    void Push(const Point& p);

    /// Force immediate flush regardless of period.
    void Flush();

    void Start();
    void Stop();

    /// Number of points emitted since `Start()`. Useful for assertions.
    uint64_t GetEmittedCount() const;

  protected:
    void DoDispose() override;

  private:
    void TickFlush();
    void SendUdp(const std::string& payload);
    void AppendToFile(const std::string& payload);

    Transport m_transport{Transport::Udp};
    std::string m_udpHost{"127.0.0.1"};
    uint16_t m_udpPort{8089};
    std::string m_filePath;
    Time m_flushPeriod{Seconds(1.0)};
    bool m_useSimulationTime{true};
    std::vector<Point> m_buffer;
    uint64_t m_emitted{0};
    bool m_running{false};
    EventId m_event;
    int m_udpSocketFd{-1};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_INFLUX_SINK_H
