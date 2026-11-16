/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "oran-ntn-e2-transport.h"

#include "oran-ntn-e2-interface.h"
#include "oran-ntn-flexric-types.h"
#include "oran-ntn-service-model-kpm.h"

#include "ns3/error-model.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnE2Transport");
NS_OBJECT_ENSURE_REGISTERED(OranNtnE2Transport);

namespace
{
constexpr uint16_t kE2apPort = 36421; // IANA "e2ap" (SCTP), O-RAN.WG3.E2GAP
constexpr uint32_t kHeaderBytes = 1 + 1 + 4 + 4 + 8 + 4;
constexpr double kC = 299792458.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

void
PutU32(std::vector<uint8_t>& b, uint32_t v)
{
    for (int i = 3; i >= 0; --i)
    {
        b.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
}

void
PutU64(std::vector<uint8_t>& b, uint64_t v)
{
    for (int i = 7; i >= 0; --i)
    {
        b.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
}

void
PutF64(std::vector<uint8_t>& b, double d)
{
    uint64_t v;
    std::memcpy(&v, &d, sizeof v);
    PutU64(b, v);
}

uint32_t
GetU32(const uint8_t* p)
{
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

uint64_t
GetU64(const uint8_t* p)
{
    return (uint64_t(GetU32(p)) << 32) | GetU32(p + 4);
}

double
GetF64(const uint8_t* p)
{
    const uint64_t v = GetU64(p);
    double d;
    std::memcpy(&d, &v, sizeof d);
    return d;
}

/// Measured flags carried in CTX.Flags so the RIC restores provenance exactly.
enum Flag : uint32_t
{
    kSinr = 1u << 0,
    kThp = 1u << 1,
    kPrb = 1u << 2,
    kBler = 1u << 3,
    kRsrp = 1u << 4,
    kRsrq = 1u << 5,
    kLat = 1u << 6,
    kNtn = 1u << 7,
};
} // namespace

TypeId
OranNtnE2Transport::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnE2Transport")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnE2Transport>();
    return tid;
}

OranNtnE2Transport::OranNtnE2Transport() = default;
OranNtnE2Transport::~OranNtnE2Transport() = default;

void
OranNtnE2Transport::DoDispose()
{
    Simulator::Cancel(m_refreshEvent);
    for (auto& [id, l] : m_links)
    {
        if (l.agentSock)
        {
            l.agentSock->Close();
        }
        if (l.ricSock)
        {
            l.ricSock->Close();
        }
    }
    m_links.clear();
    m_ric = nullptr;
    if (m_trace)
    {
        m_trace->flush();
    }
    Object::DoDispose();
}

void
OranNtnE2Transport::SetTraceFile(const std::string& path)
{
    if (path.empty())
    {
        m_trace.reset();
        return;
    }
    std::error_code ec;
    const std::filesystem::path p(path);
    if (p.has_parent_path())
    {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
    m_trace = std::make_shared<std::ofstream>(path);
    *m_trace << "t_s,dir,event,procedure,message,gnb_id,seq,bytes,owd_ms\n";
}

void
OranNtnE2Transport::Trace(const char* dir,
                          uint8_t proc,
                          uint8_t type,
                          uint32_t gnb,
                          uint32_t seq,
                          uint32_t bytes,
                          double owdMs)
{
    if (!m_trace)
    {
        return;
    }
    static const char* kTypes[] = {"initiating", "successful", "unsuccessful"};
    const char* procName = proc == kE2Setup           ? "E2Setup"
                           : proc == kRicSubscription ? "RICsubscription"
                           : proc == kRicIndication   ? "RICindication"
                           : proc == kRicControl      ? "RICcontrol"
                                                      : "unknown";
    *m_trace << Simulator::Now().GetSeconds() << "," << dir << ","
             << (std::isnan(owdMs) ? "tx" : "rx") << "," << procName << ","
             << kTypes[type < 3 ? type : 0] << "," << gnb << "," << seq << "," << bytes << ",";
    if (!std::isnan(owdMs))
    {
        *m_trace << owdMs;
    }
    *m_trace << "\n";
}

Time
OranNtnE2Transport::LinkDelay(const Link& l) const
{
    if (l.sat && l.gw)
    {
        return Seconds(l.sat->GetDistanceFrom(l.gw) / kC) + m_gwToRic;
    }
    return l.e2->GetFeederLinkDelay();
}

void
OranNtnE2Transport::Attach(Ptr<OranNtnE2Node> node, Ptr<MobilityModel> sat, Ptr<MobilityModel> gw)
{
    NS_ABORT_MSG_IF(!node, "null E2 node");
    const uint32_t id = node->GetNodeId();
    NS_ABORT_MSG_IF(m_links.count(id), "E2 node " << id << " already attached");
    InternetStackHelper inet;
    if (!m_ric)
    {
        m_ric = CreateObject<Node>();
        inet.Install(m_ric);
    }
    Link l;
    l.e2 = node;
    l.sat = sat;
    l.gw = gw;
    l.agent = CreateObject<Node>();
    inet.Install(l.agent);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(m_rate));
    p2p.SetChannelAttribute("Delay", TimeValue(LinkDelay(l)));
    NetDeviceContainer d = p2p.Install(l.agent, m_ric);
    l.ch = DynamicCast<PointToPointChannel>(d.Get(0)->GetChannel());
    l.emAgent = CreateObject<RateErrorModel>();
    l.emAgent->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emAgent->SetRate(0.0);
    l.emRic = CreateObject<RateErrorModel>();
    l.emRic->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    l.emRic->SetRate(0.0);
    d.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(l.emAgent));
    d.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(l.emRic));

    // One /30 per link in 10.251.0.0/16 and up, away from the stack's 1.0.0.0,
    // 7.0.0.0 and 10.1.x nets.
    std::ostringstream base;
    base << "10." << (251 + m_nextNet / 256) << "." << (m_nextNet % 256) << ".0";
    ++m_nextNet;
    Ipv4AddressHelper ip;
    ip.SetBase(base.str().c_str(), "255.255.255.252");
    Ipv4InterfaceContainer ifc = ip.Assign(d);
    l.agentAddr = ifc.GetAddress(0).Get();
    l.ricAddr = ifc.GetAddress(1).Get();

    l.agentSock = Socket::CreateSocket(l.agent, UdpSocketFactory::GetTypeId());
    l.agentSock->Bind(InetSocketAddress(Ipv4Address(l.agentAddr), kE2apPort));
    l.agentSock->SetRecvCallback(MakeCallback(&OranNtnE2Transport::AgentRx, this));
    l.ricSock = Socket::CreateSocket(m_ric, UdpSocketFactory::GetTypeId());
    l.ricSock->Bind(InetSocketAddress(Ipv4Address(l.ricAddr), kE2apPort));
    l.ricSock->SetRecvCallback(MakeCallback(&OranNtnE2Transport::RicRx, this));
    m_links[id] = l;
    node->SetPacketTransport(this);
}

void
OranNtnE2Transport::Start(Time at)
{
    Simulator::Schedule(at, [this]() {
        for (auto& [id, l] : m_links)
        {
            // E2 Setup Request: RAN function list (E2SM-KPM, E2SM-RC) as
            // "id:name;" text.
            std::string fl = "2:E2SM-KPM v03.00;3:E2SM-RC v01.03;";
            std::vector<uint8_t> body(fl.begin(), fl.end());
            SendFrame(l, true, kE2Setup, kInitiating, body, Simulator::Now());
        }
        Refresh();
    });
}

void
OranNtnE2Transport::Refresh()
{
    for (auto& [id, l] : m_links)
    {
        if (l.ch)
        {
            l.ch->SetAttribute("Delay", TimeValue(LinkDelay(l)));
        }
    }
    m_refreshEvent = Simulator::Schedule(m_refresh, &OranNtnE2Transport::Refresh, this);
}

void
OranNtnE2Transport::SetOutage(uint32_t gnbId, bool on)
{
    auto it = m_links.find(gnbId);
    if (it == m_links.end())
    {
        return;
    }
    it->second.emAgent->SetRate(on ? 1.0 : 0.0);
    it->second.emRic->SetRate(on ? 1.0 : 0.0);
}

void
OranNtnE2Transport::ScheduleOutage(uint32_t gnbId, Time start, Time duration)
{
    Simulator::Schedule(start, &OranNtnE2Transport::SetOutage, this, gnbId, true);
    Simulator::Schedule(start + duration, &OranNtnE2Transport::SetOutage, this, gnbId, false);
}

void
OranNtnE2Transport::SendFrame(Link& l,
                              bool uplink,
                              uint8_t proc,
                              uint8_t type,
                              const std::vector<uint8_t>& body,
                              Time created)
{
    std::vector<uint8_t> b;
    b.reserve(kHeaderBytes + body.size());
    b.push_back(proc);
    b.push_back(type);
    PutU32(b, l.e2->GetNodeId());
    const uint32_t seq = uplink ? l.seqUp++ : l.seqDown++;
    PutU32(b, seq);
    PutU64(b, static_cast<uint64_t>(created.GetNanoSeconds()));
    PutU32(b, static_cast<uint32_t>(body.size()));
    b.insert(b.end(), body.begin(), body.end());
    Ptr<Packet> p = Create<Packet>(b.data(), static_cast<uint32_t>(b.size()));
    p->AddPaddingAtEnd(m_sctpPad);
    const uint32_t bytes = p->GetSize();
    if (uplink)
    {
        m_c.bytesUp += bytes;
        l.agentSock->SendTo(p, 0, InetSocketAddress(Ipv4Address(l.ricAddr), kE2apPort));
    }
    else
    {
        m_c.bytesDown += bytes;
        l.ricSock->SendTo(p, 0, InetSocketAddress(Ipv4Address(l.agentAddr), kE2apPort));
    }
    Trace(uplink ? "up" : "down", proc, type, l.e2->GetNodeId(), seq, bytes, kNaN);
}

void
OranNtnE2Transport::SendSubscriptions(Link& l)
{
    for (const auto& sub : l.e2->GetActiveSubscriptions())
    {
        if (l.subsRequested.insert(sub.subscriptionId).second)
        {
            std::vector<uint8_t> body;
            PutU32(body, sub.subscriptionId);
            PutU32(body, sub.ranFunctionId);
            PutU64(body, static_cast<uint64_t>(sub.reportingPeriod.GetMilliSeconds()));
            SendFrame(l, false, kRicSubscription, kInitiating, body, Simulator::Now());
        }
    }
}

void
OranNtnE2Transport::SendIndication(Ptr<OranNtnE2Node> node, const E2Indication& ind)
{
    auto it = m_links.find(node->GetNodeId());
    NS_ABORT_MSG_IF(it == m_links.end(), "E2 node not attached to the transport");
    Link& l = it->second;
    if (!l.setupDone || !l.subsActive.count(ind.subscriptionId))
    {
        // E2AP: a RIC Indication exists only within an accepted RIC
        // Subscription of a node that completed E2 Setup.
        ++m_c.indicationsBeforeSubscription;
        if (l.setupDone && !l.subsRequested.count(ind.subscriptionId))
        {
            SendSubscriptions(l); // a subscription created after the setup
        }
        return;
    }
    std::vector<uint8_t> body;
    PutU32(body, ind.subscriptionId);
    PutU32(body, ind.ranFunctionId);
    const std::vector<uint8_t> kpm = EncodeKpm(ind.kpmReport);
    body.insert(body.end(), kpm.begin(), kpm.end());
    m_linkDelaySum += LinkDelay(l).GetSeconds() * 1e3;
    ++m_linkDelayN;
    ++m_c.indicationsSent;
    SendFrame(l, true, kRicIndication, kInitiating, body, Simulator::Now());
}

void
OranNtnE2Transport::SendControl(Ptr<OranNtnE2Node> node, const E2RcAction& action)
{
    auto it = m_links.find(node->GetNodeId());
    NS_ABORT_MSG_IF(it == m_links.end(), "E2 node not attached to the transport");
    Link& l = it->second;
    if (!l.setupDone)
    {
        ++m_c.controlsBeforeSetup;
        return;
    }
    ++m_c.controlsSent;
    SendFrame(l, false, kRicControl, kInitiating, EncodeControl(action), Simulator::Now());
}

void
OranNtnE2Transport::AgentRx(Ptr<Socket> s)
{
    Ptr<Packet> p;
    Address from;
    while ((p = s->RecvFrom(from)))
    {
        const uint32_t bytes = p->GetSize();
        std::vector<uint8_t> buf(bytes);
        p->CopyData(buf.data(), bytes);
        if (bytes < kHeaderBytes)
        {
            ++m_c.decodeFailures;
            continue;
        }
        const uint32_t gnb = GetU32(&buf[2]);
        const uint32_t len = GetU32(&buf[18]);
        auto it = m_links.find(gnb);
        if (it == m_links.end() || kHeaderBytes + len > bytes)
        {
            ++m_c.decodeFailures;
            continue;
        }
        std::vector<uint8_t> body(buf.begin() + kHeaderBytes, buf.begin() + kHeaderBytes + len);
        HandleAtAgent(it->second, buf[0], buf[1], GetU32(&buf[6]),
                      NanoSeconds(static_cast<int64_t>(GetU64(&buf[10]))), body, bytes);
    }
}

void
OranNtnE2Transport::RicRx(Ptr<Socket> s)
{
    Ptr<Packet> p;
    Address from;
    while ((p = s->RecvFrom(from)))
    {
        const uint32_t bytes = p->GetSize();
        std::vector<uint8_t> buf(bytes);
        p->CopyData(buf.data(), bytes);
        if (bytes < kHeaderBytes)
        {
            ++m_c.decodeFailures;
            continue;
        }
        const uint32_t gnb = GetU32(&buf[2]);
        const uint32_t len = GetU32(&buf[18]);
        auto it = m_links.find(gnb);
        if (it == m_links.end() || kHeaderBytes + len > bytes)
        {
            ++m_c.decodeFailures;
            continue;
        }
        std::vector<uint8_t> body(buf.begin() + kHeaderBytes, buf.begin() + kHeaderBytes + len);
        HandleAtRic(it->second, buf[0], buf[1], GetU32(&buf[6]),
                    NanoSeconds(static_cast<int64_t>(GetU64(&buf[10]))), body, bytes);
    }
}

void
OranNtnE2Transport::HandleAtRic(Link& l,
                                uint8_t proc,
                                uint8_t type,
                                uint32_t seq,
                                Time created,
                                const std::vector<uint8_t>& body,
                                uint32_t bytes)
{
    const double owdMs = (Simulator::Now() - created).GetSeconds() * 1e3;
    Trace("up", proc, type, l.e2->GetNodeId(), seq, bytes, owdMs);
    if (proc == kE2Setup && type == kInitiating)
    {
        const std::string ok = "accepted:2;3;";
        SendFrame(l, false, kE2Setup, kSuccessful, std::vector<uint8_t>(ok.begin(), ok.end()),
                  Simulator::Now());
        SendSubscriptions(l);
    }
    else if (proc == kRicSubscription && type == kSuccessful)
    {
        ++m_c.subscriptionsDone;
    }
    else if (proc == kRicControl && type == kSuccessful)
    {
        ++m_c.controlAcks;
    }
    else if (proc == kRicIndication && body.size() >= 8)
    {
        E2Indication ind;
        ind.subscriptionId = GetU32(&body[0]);
        ind.ranFunctionId = GetU32(&body[4]);
        std::vector<uint8_t> kpm(body.begin() + 8, body.end());
        if (!DecodeKpm(kpm, ind.kpmReport))
        {
            ++m_c.decodeFailures;
            return;
        }
        ind.kpmReport.gnbId = l.e2->GetNodeId();
        ind.timestamp = created.GetSeconds();
        ind.originalTimestamp = created;
        ind.isBuffered = false;
        ind.deliveryDelay = Simulator::Now() - created;
        ++m_c.indicationsReceived;
        m_owdSum += owdMs;
        m_owdSq += owdMs * owdMs;
        ++m_owdN;
        if (m_firstIndication.IsNegative())
        {
            m_firstIndication = Simulator::Now();
        }
        if (!l.lastIndicationRx.IsNegative())
        {
            l.maxGapS = std::max(l.maxGapS, (Simulator::Now() - l.lastIndicationRx).GetSeconds());
        }
        l.lastIndicationRx = Simulator::Now();
        l.e2->DeliverFromTransport(ind);
    }
}

void
OranNtnE2Transport::HandleAtAgent(Link& l,
                                  uint8_t proc,
                                  uint8_t type,
                                  uint32_t seq,
                                  Time created,
                                  const std::vector<uint8_t>& body,
                                  uint32_t bytes)
{
    const double owdMs = (Simulator::Now() - created).GetSeconds() * 1e3;
    Trace("down", proc, type, l.e2->GetNodeId(), seq, bytes, owdMs);
    if (proc == kE2Setup && type == kSuccessful)
    {
        l.setupDone = true;
        ++m_c.setupDone;
        m_lastSetupDone = Simulator::Now();
    }
    else if (proc == kRicSubscription && type == kInitiating && body.size() >= 8)
    {
        const uint32_t subId = GetU32(&body[0]);
        l.subsActive.insert(subId);
        std::vector<uint8_t> resp;
        PutU32(resp, subId);
        SendFrame(l, true, kRicSubscription, kSuccessful, resp, Simulator::Now());
    }
    else if (proc == kRicControl && type == kInitiating)
    {
        E2RcAction a;
        if (!DecodeControl(body, a))
        {
            ++m_c.decodeFailures;
            return;
        }
        ++m_c.controlsReceived;
        std::vector<uint8_t> ack;
        PutU32(ack, seq);
        SendFrame(l, true, kRicControl, kSuccessful, ack, Simulator::Now());
        l.e2->ExecuteFromTransport(a, created);
    }
}

double
OranNtnE2Transport::GetIndicationOwdMeanMs() const
{
    return m_owdN ? m_owdSum / m_owdN : kNaN;
}

double
OranNtnE2Transport::GetIndicationOwdSdMs() const
{
    if (m_owdN < 2)
    {
        return kNaN;
    }
    const double m = m_owdSum / m_owdN;
    return std::sqrt(std::max(0.0, (m_owdSq - m_owdN * m * m) / (m_owdN - 1)));
}

double
OranNtnE2Transport::GetMeanLinkDelayMs() const
{
    return m_linkDelayN ? m_linkDelaySum / m_linkDelayN : kNaN;
}

double
OranNtnE2Transport::GetMaxIndicationGapS() const
{
    double g = 0.0;
    for (const auto& [id, l] : m_links)
    {
        g = std::max(g, l.maxGapS);
    }
    return g;
}

// ---------------------------------------------------------------------------
// E2SM-KPM v03.00 Indication Message Format 1 body of one measured report
// ---------------------------------------------------------------------------

namespace
{
struct KpmField
{
    const char* name;
    double E2KpmReport::*member;
    double scale; //!< wire value = member * scale (TS 28.552 unit)
};

// TS 28.552 names first, then the TS 38.215 L1 quantities and the NTN
// ephemeris quantities under their own (non-28.552) namespaces.
const KpmField kFields[] = {
    {"DRB.UEThpDl", &E2KpmReport::throughput_Mbps, 1e3},          // kbit/s
    {"DRB.PdcpSduVolumeDL", &E2KpmReport::pdcpVolumeDl_Mbit, 1.0}, // Mbit
    {"DRB.RlcSduDelayDl", &E2KpmReport::rlcSduDelayDl_ms, 10.0},  // 0.1 ms
    {"RRU.PrbUsedDl", &E2KpmReport::prbUsedDl, 1.0},
    {"RRU.PrbAvailDl", &E2KpmReport::prbAvailDl, 1.0},
    {"TB.TotNbrDl", &E2KpmReport::tbTotDl, 1.0},
    {"TB.ErrTotNbrDl", &E2KpmReport::tbErrDl, 1.0},
    {"L1M.SS-RSRP", &E2KpmReport::rsrp_dBm, 1.0},
    {"L1M.SS-RSRQ", &E2KpmReport::rsrq_dB, 1.0},
    {"L1M.RS-SINR", &E2KpmReport::sinr_dB, 1.0},
    {"VND.HarqBlerDl", &E2KpmReport::harqBler, 1.0},
    {"VND.PrbUtilDl", &E2KpmReport::prbUtilization, 1.0},
    {"VND.LatencyMs", &E2KpmReport::latency_ms, 1.0},
    {"VND.CellThpDlMbps", &E2KpmReport::cellThroughput_Mbps, 1.0},
    {"VND.SliceThpDlMbps", &E2KpmReport::sliceThroughput_Mbps, 1.0},
    {"VND.SliceLatencyMs", &E2KpmReport::sliceLatency_ms, 1.0},
    {"VND.SliceDeliveryRatio", &E2KpmReport::sliceReliability, 1.0},
    {"NTN.ElevationDeg", &E2KpmReport::elevation_deg, 1.0},
    {"NTN.DopplerHz", &E2KpmReport::doppler_Hz, 1.0},
    {"NTN.PropDelayMs", &E2KpmReport::propagationDelay_ms, 1.0},
    {"NTN.TteS", &E2KpmReport::tte_s, 1.0},
    {"NTN.BeamGainDb", &E2KpmReport::beamGain_dB, 1.0},
    {"VND.UePrbShareDl", &E2KpmReport::uePrbShareDl, 1.0},
};
} // namespace

std::vector<uint8_t>
OranNtnE2Transport::EncodeKpm(const E2KpmReport& r)
{
    using namespace oranntn::flexric::kpm_v3;
    kpm_ind_msg_format_1_t msg;
    msg.gran_period_ms = static_cast<uint32_t>(std::lround(r.granularityPeriod_s * 1e3));
    auto addReal = [&msg](const char* name, double v) {
        meas_info_format_1_lst_t mi;
        mi.meas_type.form = meas_type_form_t::name;
        mi.meas_type.meas_name = name;
        mi.meas_type.meas_id = 0;
        meas_record_item_t rec{};
        if (std::isfinite(v))
        {
            rec.form = meas_value_form_t::real;
            rec.real_val = v;
        }
        else
        {
            rec.form = meas_value_form_t::no_value; // E2SM-KPM noValue
        }
        mi.meas_record_lst.push_back(rec);
        msg.meas_info_lst.push_back(mi);
    };
    auto addInt = [&msg](const char* name, int64_t v) {
        meas_info_format_1_lst_t mi;
        mi.meas_type.form = meas_type_form_t::name;
        mi.meas_type.meas_name = name;
        mi.meas_type.meas_id = 0;
        meas_record_item_t rec{};
        rec.form = meas_value_form_t::integer;
        rec.int_val = v;
        mi.meas_record_lst.push_back(rec);
        msg.meas_info_lst.push_back(mi);
    };
    for (const auto& f : kFields)
    {
        double v = r.*(f.member);
        // the "unknown" sentinels of the count / size fields travel as noValue
        if ((f.member == &E2KpmReport::prbAvailDl || f.member == &E2KpmReport::tbTotDl ||
             f.member == &E2KpmReport::tbErrDl) &&
            v < 0.0)
        {
            v = kNaN;
        }
        addReal(f.name, std::isfinite(v) ? v * f.scale : kNaN);
    }
    uint32_t flags = (r.sinrMeasured ? kSinr : 0) | (r.throughputMeasured ? kThp : 0) |
                     (r.prbMeasured ? kPrb : 0) | (r.blerMeasured ? kBler : 0) |
                     (r.rsrpMeasured ? kRsrp : 0) | (r.rsrqMeasured ? kRsrq : 0) |
                     (r.latencyMeasured ? kLat : 0) | (r.isNtn ? kNtn : 0);
    addInt("CTX.UeId", r.ueId);
    addInt("CTX.SliceId", r.sliceId);
    addInt("CTX.ActiveUes", r.activeUes);
    addInt("CTX.Cqi", r.cqi);
    addInt("CTX.BeamId", r.beamId);
    addInt("CTX.Flags", flags);
    OranNtnServiceModelKpm sm;
    return sm.EncodeIndication(&msg);
}

bool
OranNtnE2Transport::DecodeKpm(const std::vector<uint8_t>& bytes, E2KpmReport& r)
{
    using namespace oranntn::flexric::kpm_v3;
    OranNtnServiceModelKpm sm;
    kpm_ind_msg_format_1_t msg;
    if (!sm.DecodeIndication(bytes, msg))
    {
        return false;
    }
    r = E2KpmReport{};
    for (const auto& f : kFields)
    {
        r.*(f.member) = kNaN;
    }
    r.prbAvailDl = -1.0;
    r.tbTotDl = -1.0;
    r.tbErrDl = -1.0;
    r.granularityPeriod_s = msg.gran_period_ms / 1e3;
    uint32_t flags = 0;
    for (const auto& mi : msg.meas_info_lst)
    {
        if (mi.meas_record_lst.empty())
        {
            continue;
        }
        const auto& rec = mi.meas_record_lst.front();
        const std::string& n = mi.meas_type.meas_name;
        if (rec.form == meas_value_form_t::integer)
        {
            if (n == "CTX.UeId")
            {
                r.ueId = static_cast<uint32_t>(rec.int_val);
            }
            else if (n == "CTX.SliceId")
            {
                r.sliceId = static_cast<uint8_t>(rec.int_val);
            }
            else if (n == "CTX.ActiveUes")
            {
                r.activeUes = static_cast<uint32_t>(rec.int_val);
            }
            else if (n == "CTX.Cqi")
            {
                r.cqi = static_cast<uint8_t>(rec.int_val);
            }
            else if (n == "CTX.BeamId")
            {
                r.beamId = static_cast<uint32_t>(rec.int_val);
            }
            else if (n == "CTX.Flags")
            {
                flags = static_cast<uint32_t>(rec.int_val);
            }
            continue;
        }
        if (rec.form != meas_value_form_t::real)
        {
            continue; // noValue: stays unknown
        }
        for (const auto& f : kFields)
        {
            if (n == f.name)
            {
                r.*(f.member) = rec.real_val / f.scale;
                break;
            }
        }
    }
    r.sinrMeasured = flags & kSinr;
    r.throughputMeasured = flags & kThp;
    r.prbMeasured = flags & kPrb;
    r.blerMeasured = flags & kBler;
    r.rsrpMeasured = flags & kRsrp;
    r.rsrqMeasured = flags & kRsrq;
    r.latencyMeasured = flags & kLat;
    r.isNtn = flags & kNtn;
    return true;
}

std::vector<uint8_t>
OranNtnE2Transport::EncodeControl(const E2RcAction& a)
{
    // RIC Control Request body: E2SM-RC header (UE id, action type) and
    // message (RAN parameters), followed by the Style-3 ControlMessage bytes
    // when the termination produced them (ORAN-10).
    std::vector<uint8_t> b;
    b.push_back(static_cast<uint8_t>(a.actionType));
    PutU32(b, a.xappId);
    PutU32(b, a.targetGnbId);
    PutU32(b, a.targetUeId);
    PutU32(b, a.targetBeamId);
    b.push_back(a.targetSliceId);
    PutF64(b, a.confidence);
    PutF64(b, a.parameter1);
    PutF64(b, a.parameter2);
    PutF64(b, a.timestamp);
    PutU32(b, static_cast<uint32_t>(a.xappName.size()));
    b.insert(b.end(), a.xappName.begin(), a.xappName.end());
    PutU32(b, static_cast<uint32_t>(a.smControlMessage.size()));
    b.insert(b.end(), a.smControlMessage.begin(), a.smControlMessage.end());
    return b;
}

bool
OranNtnE2Transport::DecodeControl(const std::vector<uint8_t>& b, E2RcAction& a)
{
    size_t i = 0;
    auto need = [&b, &i](size_t n) { return i + n <= b.size(); };
    if (!need(1 + 4 * 4 + 1 + 8 * 4 + 4))
    {
        return false;
    }
    a = E2RcAction{};
    a.actionType = static_cast<E2RcActionType>(b[i++]);
    a.xappId = GetU32(&b[i]);
    i += 4;
    a.targetGnbId = GetU32(&b[i]);
    i += 4;
    a.targetUeId = GetU32(&b[i]);
    i += 4;
    a.targetBeamId = GetU32(&b[i]);
    i += 4;
    a.targetSliceId = b[i++];
    a.confidence = GetF64(&b[i]);
    i += 8;
    a.parameter1 = GetF64(&b[i]);
    i += 8;
    a.parameter2 = GetF64(&b[i]);
    i += 8;
    a.timestamp = GetF64(&b[i]);
    i += 8;
    const uint32_t nl = GetU32(&b[i]);
    i += 4;
    if (!need(nl + 4))
    {
        return false;
    }
    a.xappName.assign(b.begin() + i, b.begin() + i + nl);
    i += nl;
    const uint32_t sl = GetU32(&b[i]);
    i += 4;
    if (!need(sl))
    {
        return false;
    }
    a.smControlMessage.assign(b.begin() + i, b.begin() + i + sl);
    return true;
}

} // namespace ns3
