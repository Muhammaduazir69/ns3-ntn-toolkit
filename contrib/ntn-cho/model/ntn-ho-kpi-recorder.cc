/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ntn-ho-kpi-recorder.h"

#include <ns3/application.h>
#include <ns3/callback.h>
#include <ns3/config.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/simulator.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnHoKpiRecorder");
NS_OBJECT_ENSURE_REGISTERED(NtnHoKpiRecorder);

double
NtnHoKpiRecorder::HoRecord::CplaneMs() const
{
    if (!closed || tStart < Time() || tEnd < Time())
    {
        return -1.0;
    }
    return (tEnd - tStart).GetSeconds() * 1e3;
}

double
NtnHoKpiRecorder::HoRecord::UplaneGapMs() const
{
    if (lastRxBefore < Time() || firstRxAfter < Time())
    {
        return -1.0;
    }
    return (firstRxAfter - lastRxBefore).GetSeconds() * 1e3;
}

TypeId
NtnHoKpiRecorder::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnHoKpiRecorder")
                            .SetParent<Object>()
                            .SetGroupName("NtnCho")
                            .AddConstructor<NtnHoKpiRecorder>();
    return tid;
}

NtnHoKpiRecorder::NtnHoKpiRecorder() = default;
NtnHoKpiRecorder::~NtnHoKpiRecorder() = default;

void
NtnHoKpiRecorder::DoDispose()
{
    m_ue.clear();
    m_imsiToUe.clear();
    Object::DoDispose();
}

NtnHoKpiRecorder::UeState&
NtnHoKpiRecorder::Ue(uint32_t ueIndex)
{
    return m_ue[ueIndex];
}

void
NtnHoKpiRecorder::AttachUe(uint32_t ueIndex, Ptr<Node> ueNode)
{
    NS_ABORT_MSG_IF(!ueNode, "AttachUe: null node");
    Ue(ueIndex); // create the entry so a UE without handovers still reports
    const std::string base = "/NodeList/" + std::to_string(ueNode->GetId()) + "/DeviceList/*/";
    uint32_t connected = 0;
    for (const std::string rrc : {"NrUeRrc", "LteUeRrc"})
    {
        const std::string p = base + rrc + "/";
        // TS 38.331 5.3.5.5.2: reconfigurationWithSync applied, T304 started.
        connected += Config::ConnectWithoutContextFailSafe(
                         p + "HandoverStart",
                         Callback<void, uint64_t, uint16_t, uint16_t, uint16_t>(
                             [this, ueIndex](uint64_t imsi, uint16_t cell, uint16_t, uint16_t target) {
                                 NotifyHandoverStart(ueIndex, imsi, cell, target);
                             }))
                         ? 1
                         : 0;
        // TS 38.331 5.3.5.3: random access on the target done, Complete submitted, T304 stopped.
        Config::ConnectWithoutContextFailSafe(
            p + "HandoverEndOk",
            Callback<void, uint64_t, uint16_t, uint16_t>(
                [this, ueIndex](uint64_t imsi, uint16_t cell, uint16_t) {
                    NotifyHandoverEnd(ueIndex, imsi, cell, true);
                }));
        Config::ConnectWithoutContextFailSafe(
            p + "HandoverEndError",
            Callback<void, uint64_t, uint16_t, uint16_t>(
                [this, ueIndex](uint64_t imsi, uint16_t cell, uint16_t) {
                    NotifyHandoverEnd(ueIndex, imsi, cell, false);
                }));
        Config::ConnectWithoutContextFailSafe(
            p + "ConnectionEstablished",
            Callback<void, uint64_t, uint16_t, uint16_t>(
                [this, ueIndex](uint64_t, uint16_t cell, uint16_t) { NotifyConnected(ueIndex, cell); }));
        // TS 38.331 5.3.10.3: T310 expiry.
        Config::ConnectWithoutContextFailSafe(
            p + "RadioLinkFailure",
            Callback<void, uint64_t, uint16_t, uint16_t>(
                [this, ueIndex](uint64_t, uint16_t cell, uint16_t) {
                    NotifyRadioLinkFailure(ueIndex, cell);
                }));
    }
    NS_ABORT_MSG_IF(connected == 0,
                    "NtnHoKpiRecorder: node " << ueNode->GetId()
                                              << " has no NrUeRrc/LteUeRrc to trace (build the radio first)");
    for (uint32_t a = 0; a < ueNode->GetNApplications(); ++a)
    {
        AttachRxApplication(ueIndex, ueNode->GetApplication(a));
    }
}

bool
NtnHoKpiRecorder::AttachRxApplication(uint32_t ueIndex, Ptr<Application> app)
{
    if (!app)
    {
        return false;
    }
    // Only receive traces with the (packet, address) signature; any other "Rx"
    // would be a type mismatch at connection time.
    TypeId::TraceSourceInformation info;
    if (!app->GetInstanceTypeId().LookupTraceSourceByName("Rx", &info) ||
        info.callback != "ns3::Packet::AddressTracedCallback")
    {
        return false;
    }
    return app->TraceConnectWithoutContext(
        "Rx",
        Callback<void, Ptr<const Packet>, const Address&>(
            [this, ueIndex](Ptr<const Packet>, const Address&) { NotifyRx(ueIndex); }));
}

void
NtnHoKpiRecorder::AnnotateNext(uint32_t ueIndex, double modeledInterruptionMs, const std::string& decidedBy)
{
    UeState& u = Ue(ueIndex);
    if (u.openRecord >= 0 && m_records[u.openRecord].decidedBy.empty())
    {
        m_records[u.openRecord].modeledInterruptionMs = modeledInterruptionMs;
        m_records[u.openRecord].decidedBy = decidedBy;
        return;
    }
    u.havePendingAnnotation = true;
    u.pendingModeledMs = modeledInterruptionMs;
    u.pendingDecidedBy = decidedBy;
}

void
NtnHoKpiRecorder::NotifyHandoverStart(uint32_t ueIndex, uint64_t imsi, uint16_t cellId, uint16_t targetCellId)
{
    UeState& u = Ue(ueIndex);
    m_imsiToUe[imsi] = ueIndex;
    HoRecord r;
    r.ueIndex = ueIndex;
    r.imsi = imsi;
    r.sourceCell = cellId;
    r.targetCell = targetCellId;
    r.tStart = Simulator::Now();
    r.lastRxBefore = u.lastRx;
    if (u.servingSince >= Time())
    {
        r.timeOfStayS = (r.tStart - u.servingSince).GetSeconds();
    }
    if (u.havePendingAnnotation)
    {
        r.modeledInterruptionMs = u.pendingModeledMs;
        r.decidedBy = u.pendingDecidedBy;
        u.havePendingAnnotation = false;
    }
    // A record still waiting for its first post-HO packet keeps -1: no packet
    // arrived between that handover and this one.
    u.pendingUplane = -1;
    m_records.push_back(r);
    u.openRecord = static_cast<int64_t>(m_records.size() - 1);
    NS_LOG_INFO("UE " << ueIndex << " HandoverStart " << cellId << " -> " << targetCellId << " at "
                      << r.tStart.As(Time::S));
}

void
NtnHoKpiRecorder::NotifyHandoverEnd(uint32_t ueIndex, uint64_t imsi, uint16_t cellId, bool ok)
{
    UeState& u = Ue(ueIndex);
    if (u.openRecord < 0)
    {
        NS_LOG_WARN("UE " << ueIndex << " handover end without a start (imsi " << imsi << ")");
        return;
    }
    HoRecord& r = m_records[u.openRecord];
    r.tEnd = Simulator::Now();
    r.closed = true;
    r.success = ok;
    if (ok)
    {
        r.targetCell = cellId;
        // Ping-pong: back to the cell left by the previous successful handover,
        // within the window measured from that handover's completion.
        if (u.prevEnd >= Time() && r.targetCell == u.prevSource)
        {
            const Time since = r.tStart - u.prevEnd;
            r.pingPong = since <= m_ppWindow;
            r.pingPong1s = since <= Seconds(1);
            r.pingPong5s = since <= Seconds(5);
        }
        u.prevSource = r.sourceCell;
        u.prevEnd = r.tEnd;
        u.cellId = cellId;
        u.servingSince = r.tEnd;
    }
    u.pendingUplane = u.openRecord;
    u.openRecord = -1;
}

void
NtnHoKpiRecorder::NotifyConnected(uint32_t ueIndex, uint16_t cellId)
{
    UeState& u = Ue(ueIndex);
    u.cellId = cellId;
    u.servingSince = Simulator::Now();
}

void
NtnHoKpiRecorder::NotifyRadioLinkFailure(uint32_t ueIndex, uint16_t /*cellId*/)
{
    ++Ue(ueIndex).rlf;
}

void
NtnHoKpiRecorder::NotifyRx(uint32_t ueIndex)
{
    UeState& u = Ue(ueIndex);
    const Time now = Simulator::Now();
    // Packets delivered in the same TB arrive at the same instant; a zero gap
    // says nothing about the traffic cadence, so only positive gaps are kept.
    if (u.lastRx >= Time() && now > u.lastRx && u.iatMs.size() < 200000)
    {
        u.iatMs.push_back((now - u.lastRx).GetSeconds() * 1e3);
    }
    u.lastRx = now;
    if (u.pendingUplane >= 0)
    {
        HoRecord& r = m_records[u.pendingUplane];
        if (r.firstRxAfter < Time())
        {
            r.firstRxAfter = now;
        }
        u.pendingUplane = -1;
    }
}

std::vector<NtnHoKpiRecorder::HoRecord>
NtnHoKpiRecorder::GetUeRecords(uint32_t ueIndex) const
{
    std::vector<HoRecord> out;
    for (const auto& r : m_records)
    {
        if (r.ueIndex == ueIndex)
        {
            out.push_back(r);
        }
    }
    return out;
}

uint32_t
NtnHoKpiRecorder::GetHandoverCount(bool successfulOnly) const
{
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        n += (!successfulOnly || (r.closed && r.success)) ? 1 : 0;
    }
    return n;
}

uint32_t
NtnHoKpiRecorder::GetHandoverFailureCount() const
{
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        n += (r.closed && !r.success) ? 1 : 0;
    }
    return n;
}

uint32_t
NtnHoKpiRecorder::GetRlfCount(uint32_t ueIndex) const
{
    auto it = m_ue.find(ueIndex);
    return it == m_ue.end() ? 0 : it->second.rlf;
}

uint32_t
NtnHoKpiRecorder::GetTotalRlfCount() const
{
    uint32_t n = 0;
    for (const auto& kv : m_ue)
    {
        n += kv.second.rlf;
    }
    return n;
}

bool
NtnHoKpiRecorder::IsPingPong(const HoRecord& r, const std::vector<HoRecord>& all, Time window)
{
    if (!(r.closed && r.success))
    {
        return false;
    }
    // previous successful handover of the same UE
    const HoRecord* prev = nullptr;
    for (const auto& q : all)
    {
        if (&q == &r)
        {
            break;
        }
        if (q.ueIndex == r.ueIndex && q.closed && q.success)
        {
            prev = &q;
        }
    }
    return prev && r.targetCell == prev->sourceCell && (r.tStart - prev->tEnd) <= window;
}

uint32_t
NtnHoKpiRecorder::GetPingPongCount(Time window) const
{
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        n += IsPingPong(r, m_records, window) ? 1 : 0;
    }
    return n;
}

double
NtnHoKpiRecorder::GetMeanCplaneMs() const
{
    double s = 0.0;
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        if (r.success && r.CplaneMs() >= 0.0)
        {
            s += r.CplaneMs();
            ++n;
        }
    }
    return n ? s / n : std::nan("");
}

double
NtnHoKpiRecorder::GetMeanUplaneGapMs() const
{
    double s = 0.0;
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        if (r.success && r.UplaneGapMs() >= 0.0)
        {
            s += r.UplaneGapMs();
            ++n;
        }
    }
    return n ? s / n : std::nan("");
}

double
NtnHoKpiRecorder::GetMedianInterArrivalMs(uint32_t ueIndex) const
{
    auto it = m_ue.find(ueIndex);
    if (it == m_ue.end() || it->second.iatMs.empty())
    {
        return -1.0;
    }
    std::vector<double> v = it->second.iatMs;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

double
NtnHoKpiRecorder::GetMeanTimeOfStayS() const
{
    double s = 0.0;
    uint32_t n = 0;
    for (const auto& r : m_records)
    {
        if (r.success && r.timeOfStayS >= 0.0)
        {
            s += r.timeOfStayS;
            ++n;
        }
    }
    return n ? s / n : std::nan("");
}

void
NtnHoKpiRecorder::WriteCsv(const std::string& path) const
{
    std::ofstream f(path);
    if (!f)
    {
        NS_LOG_WARN("cannot write " << path);
        return;
    }
    f << "ue,imsi,source_cell,target_cell,t_start_s,t_end_s,outcome,cplane_ms,"
         "last_rx_before_s,first_rx_after_s,uplane_gap_ms,median_iat_ms,time_of_stay_s,"
         "ping_pong_window_s,ping_pong,ping_pong_1s,ping_pong_5s,modeled_interruption_ms,"
         "decided_by,source\n";
    f << std::fixed << std::setprecision(6);
    for (const auto& r : m_records)
    {
        f << r.ueIndex << "," << r.imsi << "," << r.sourceCell << "," << r.targetCell << ","
          << r.tStart.GetSeconds() << "," << (r.closed ? r.tEnd.GetSeconds() : -1.0) << ","
          << (!r.closed ? "open" : (r.success ? "success" : "failure")) << "," << r.CplaneMs()
          << "," << r.lastRxBefore.GetSeconds() << "," << r.firstRxAfter.GetSeconds() << ","
          << r.UplaneGapMs() << "," << GetMedianInterArrivalMs(r.ueIndex) << ","
          << r.timeOfStayS << "," << m_ppWindow.GetSeconds() << "," << r.pingPong << ","
          << r.pingPong1s << "," << r.pingPong5s << "," << r.modeledInterruptionMs << ","
          << (r.decidedBy.empty() ? "unstated" : r.decidedBy)
          << ",measured:ue-rrc-trace+app-rx;modeled:caller\n";
    }
}

void
NtnHoKpiRecorder::WriteKpm(const std::string& path) const
{
    std::ofstream f(path);
    if (!f)
    {
        NS_LOG_WARN("cannot write " << path);
        return;
    }
    // TS 28.552 5.1.1.6.1 (inter-gNB handover execution): attempts, successes,
    // failures. The interruption rows are toolkit names for measured values
    // (no TS 28.552 counter exists for them); they say so in the provenance.
    const uint32_t req = GetHandoverCount(false);
    const uint32_t succ = GetHandoverCount(true);
    const uint32_t fail = GetHandoverFailureCount();
    f << "metric,value,unit,provenance\n";
    f << "MM.HoExeInterReq," << req << ",count,ts28552-5.1.1.6.1;ue-rrc HandoverStart\n";
    f << "MM.HoExeInterSucc," << succ << ",count,ts28552-5.1.1.6.1;ue-rrc HandoverEndOk\n";
    f << "MM.HoExeInterFail," << fail << ",count,ts28552-5.1.1.6.1;ue-rrc HandoverEndError\n";
    f << std::fixed << std::setprecision(4);
    // A mean over zero handovers has no value: write an empty field and say
    // so (the noValue of TS 28.552 / E2SM-KPM), never NaN or a placeholder.
    auto mean = [&f](const char* name, double v, const char* unit, const char* prov) {
        f << name << ",";
        if (std::isnan(v))
        {
            f << "," << unit << "," << prov << ";noValue (no completed handover)\n";
        }
        else
        {
            f << v << "," << unit << "," << prov << "\n";
        }
    };
    mean("ntn.HoCplaneInterruptionMean",
         GetMeanCplaneMs(),
         "ms",
         "measured: HandoverEndOk - HandoverStart (toolkit name)");
    mean("ntn.HoUplaneGapMean",
         GetMeanUplaneGapMs(),
         "ms",
         "measured: app rx gap around the handover (toolkit name)");
    f << "ntn.HoPingPong," << GetPingPongCount(m_ppWindow) << ",count,measured window "
      << m_ppWindow.GetSeconds() << " s (ts38300-15.5.2.2)\n";
    f << "ntn.Rlf," << GetTotalRlfCount() << ",count,measured: ue-rrc RadioLinkFailure (T310)\n";
    mean("ntn.TimeOfStayMean",
         GetMeanTimeOfStayS(),
         "s",
         "measured: serving-cell dwell before HO");
}

} // namespace ns3
