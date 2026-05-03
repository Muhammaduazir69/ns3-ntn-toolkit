/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-observability-helper.h"

#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnObservabilityHelper");

namespace ntnobs
{

NtnObservabilityHelper::NtnObservabilityHelper() = default;

void
NtnObservabilityHelper::SetRunId(const std::string& runId)
{
    m_runId = runId;
}

void
NtnObservabilityHelper::SetInfluxFile(const std::string& path)
{
    m_influxFile = path;
    m_useUdp = false;
}

void
NtnObservabilityHelper::SetInfluxUdp(const std::string& host, uint16_t port)
{
    m_influxUdpHost = host;
    m_influxUdpPort = port;
    m_useUdp = true;
}

void
NtnObservabilityHelper::SetNetSimulyzerOutput(const std::string& path)
{
    m_netSimulyzerPath = path;
}

void
NtnObservabilityHelper::SetFlushPeriod(Time period)
{
    m_flushPeriod = period;
}

const std::string&
NtnObservabilityHelper::GetRunId() const
{
    return m_runId;
}

Ptr<NtnInfluxSink>
NtnObservabilityHelper::InstallInfluxSink() const
{
    Ptr<NtnInfluxSink> sink = CreateObject<NtnInfluxSink>();
    if (m_useUdp)
    {
        sink->SetTransport(NtnInfluxSink::Transport::Udp);
        sink->SetUdpEndpoint(m_influxUdpHost, m_influxUdpPort);
    }
    else
    {
        sink->SetTransport(NtnInfluxSink::Transport::File);
        sink->SetFilePath(m_influxFile);
    }
    sink->SetFlushPeriod(m_flushPeriod);
    sink->SetUseSimulationTime(true);
    return sink;
}

Ptr<NtnNetSimulyzerExporter>
NtnObservabilityHelper::InstallNetSimulyzerExporter() const
{
    Ptr<NtnNetSimulyzerExporter> exp = CreateObject<NtnNetSimulyzerExporter>();
    exp->SetOutputPath(m_netSimulyzerPath);
    return exp;
}

} // namespace ntnobs
} // namespace ns3
