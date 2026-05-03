/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_OBSERVABILITY_HELPER_H
#define NTN_OBSERVABILITY_HELPER_H

#include "ns3/ntn-influx-sink.h"
#include "ns3/ntn-netsimulyzer-exporter.h"

#include <ns3/ptr.h>

#include <string>

namespace ns3
{
namespace ntnobs
{

/**
 * \ingroup ntn-observability
 *
 * One-stop helper for the observability stack.
 *
 * Most scenarios want both a Grafana-bound InfluxDB sink and a NetSimulyzer
 * JSON trace; this helper builds both with consistent run-id tagging.
 */
class NtnObservabilityHelper
{
  public:
    NtnObservabilityHelper();

    void SetRunId(const std::string& runId);
    void SetInfluxFile(const std::string& path);
    void SetInfluxUdp(const std::string& host, uint16_t port = 8089);
    void SetNetSimulyzerOutput(const std::string& path);
    void SetFlushPeriod(Time period);

    Ptr<NtnInfluxSink> InstallInfluxSink() const;
    Ptr<NtnNetSimulyzerExporter> InstallNetSimulyzerExporter() const;

    const std::string& GetRunId() const;

  private:
    std::string m_runId{"run-1"};
    std::string m_influxFile;
    std::string m_influxUdpHost;
    uint16_t m_influxUdpPort{8089};
    bool m_useUdp{false};
    std::string m_netSimulyzerPath;
    Time m_flushPeriod{Seconds(1.0)};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_OBSERVABILITY_HELPER_H
