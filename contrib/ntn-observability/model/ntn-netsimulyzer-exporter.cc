/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-netsimulyzer-exporter.h"

#include <ns3/log.h>
#include <ns3/simulator.h>

#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnNetSimulyzerExporter");

namespace ntnobs
{

namespace
{

std::string
JsonEscape(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s)
    {
        switch (c)
        {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
            {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            }
            else
            {
                out.push_back(c);
            }
        }
    }
    return out;
}

std::string
FormatDouble(double v)
{
    std::ostringstream ss;
    ss << std::setprecision(12) << v;
    return ss.str();
}

} // namespace

NS_OBJECT_ENSURE_REGISTERED(NtnNetSimulyzerExporter);

TypeId
NtnNetSimulyzerExporter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnobs::NtnNetSimulyzerExporter")
                            .SetParent<Object>()
                            .SetGroupName("NtnObservability")
                            .AddConstructor<NtnNetSimulyzerExporter>();
    return tid;
}

NtnNetSimulyzerExporter::NtnNetSimulyzerExporter() = default;
NtnNetSimulyzerExporter::~NtnNetSimulyzerExporter() = default;

void
NtnNetSimulyzerExporter::DoDispose()
{
    if (m_running)
    {
        Stop();
    }
    Object::DoDispose();
}

void
NtnNetSimulyzerExporter::SetOutputPath(const std::string& path)
{
    m_path = path;
}

void
NtnNetSimulyzerExporter::SetUseSimulationTime(bool yes)
{
    m_useSimulationTime = yes;
}

void
NtnNetSimulyzerExporter::AddNode(uint32_t id,
                                 const std::string& model,
                                 const Vector& initialPosition,
                                 double scale)
{
    NS_ASSERT_MSG(!m_running, "AddNode must be called before Start()");
    m_nodes.push_back({id, model, initialPosition, scale});
}

uint32_t
NtnNetSimulyzerExporter::AddSeries(const std::string& name,
                                   const std::string& xLabel,
                                   const std::string& yLabel,
                                   const std::string& unit)
{
    NS_ASSERT_MSG(!m_running, "AddSeries must be called before Start()");
    const uint32_t idx = static_cast<uint32_t>(m_series.size()) + 1;
    m_series.push_back({idx, name, xLabel, yLabel, unit});
    return idx;
}

void
NtnNetSimulyzerExporter::Start()
{
    NS_ASSERT_MSG(!m_path.empty(), "output path required");
    m_out.open(m_path, std::ios::out | std::ios::trunc);
    NS_ASSERT_MSG(m_out, "could not open output");
    m_running = true;
    m_firstEvent = true;
    WriteHeader();
}

void
NtnNetSimulyzerExporter::Stop()
{
    if (!m_running)
    {
        return;
    }
    WriteFooter();
    m_out.close();
    m_running = false;
}

uint64_t
NtnNetSimulyzerExporter::GetEventCount() const
{
    return m_eventCount;
}

void
NtnNetSimulyzerExporter::NodeMove(uint32_t nodeId, double timeSec, const Vector& p)
{
    std::ostringstream ss;
    ss << R"({"type":"NodeMove","time":)" << FormatDouble(timeSec)
       << R"(,"nodeId":)" << nodeId
       << R"(,"position":[)" << FormatDouble(p.x) << "," << FormatDouble(p.y) << ","
       << FormatDouble(p.z) << "]}";
    WriteEvent(ss.str());
}

void
NtnNetSimulyzerExporter::LogMessage(uint32_t nodeId, double timeSec, const std::string& message)
{
    std::ostringstream ss;
    ss << R"({"type":"LogMessage","time":)" << FormatDouble(timeSec)
       << R"(,"nodeId":)" << nodeId
       << R"(,"message":")" << JsonEscape(message) << R"("})";
    WriteEvent(ss.str());
}

void
NtnNetSimulyzerExporter::SampleSeries(uint32_t seriesIdx, double timeSec, double value)
{
    std::ostringstream ss;
    ss << R"({"type":"SeriesSample","time":)" << FormatDouble(timeSec)
       << R"(,"seriesId":)" << seriesIdx
       << R"(,"value":)" << FormatDouble(value) << "}";
    WriteEvent(ss.str());
}

void
NtnNetSimulyzerExporter::WriteHeader()
{
    m_out << R"({"configuration":{"name":"ns3-ntn-toolkit","ms_per_frame":33,)"
             R"("module":"ntn-observability","schema":"netsimulyzer-1.0"})";

    m_out << R"(,"nodes":[)";
    for (size_t i = 0; i < m_nodes.size(); ++i)
    {
        const auto& n = m_nodes[i];
        if (i > 0)
            m_out << ",";
        m_out << R"({"id":)" << n.id << R"(,"model":")" << JsonEscape(n.model)
              << R"(","scale":)" << FormatDouble(n.scale)
              << R"(,"position":[)" << FormatDouble(n.initialPosition.x) << ","
              << FormatDouble(n.initialPosition.y) << ","
              << FormatDouble(n.initialPosition.z) << "]}";
    }
    m_out << "]";

    m_out << R"(,"series":[)";
    for (size_t i = 0; i < m_series.size(); ++i)
    {
        const auto& s = m_series[i];
        if (i > 0)
            m_out << ",";
        m_out << R"({"id":)" << s.idx << R"(,"name":")" << JsonEscape(s.name)
              << R"(","xLabel":")" << JsonEscape(s.xLabel) << R"(","yLabel":")"
              << JsonEscape(s.yLabel) << R"(","unit":")" << JsonEscape(s.unit) << R"("})";
    }
    m_out << "]";

    m_out << R"(,"events":[)";
}

void
NtnNetSimulyzerExporter::WriteFooter()
{
    m_out << "]}";
}

void
NtnNetSimulyzerExporter::WriteEvent(const std::string& json)
{
    if (!m_running)
    {
        return;
    }
    if (!m_firstEvent)
    {
        m_out << ",";
    }
    m_out << json;
    m_firstEvent = false;
    ++m_eventCount;
}

} // namespace ntnobs
} // namespace ns3
