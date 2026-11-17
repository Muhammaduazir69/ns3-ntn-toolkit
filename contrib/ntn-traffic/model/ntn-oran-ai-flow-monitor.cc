// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-oran-ai-flow-monitor.h"

#include "ntn-oran-application.h"

#include "../helper/ntn-real-stack-helper.h"

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnOranAiFlowMonitor");
NS_OBJECT_ENSURE_REGISTERED(NtnOranAiFlowMonitor);

namespace
{
void
SinkRxTrampoline(Ptr<NtnOranFlowProbe> probe, Ptr<const Packet> pkt, const Address&)
{
    probe->ReportRx(pkt);
}
} // namespace

// ---------------------------------------------------------------- classifier

FlowId
NtnOranFlowClassifier::Classify(const NtnOranPayloadHeader& hdr)
{
    OranFlowKey key{hdr.GetSrcId(), hdr.GetDstId(), hdr.GetFiveQi(), hdr.GetSst(),
                    hdr.GetSd()};
    auto it = m_byKey.find(key);
    if (it != m_byKey.end())
    {
        return it->second;
    }
    FlowId id = GetNewFlowId();
    m_byKey[key] = id;
    m_byId[id] = key;
    return id;
}

bool
NtnOranFlowClassifier::FindFlow(FlowId id, OranFlowKey& out) const
{
    auto it = m_byId.find(id);
    if (it == m_byId.end())
    {
        return false;
    }
    out = it->second;
    return true;
}

void
NtnOranFlowClassifier::SerializeToXmlStream(std::ostream& os, uint16_t indent) const
{
    os << std::string(indent, ' ') << "<NtnOranFlowClassifier>\n";
    for (const auto& [id, key] : m_byId)
    {
        os << std::string(indent + 2, ' ') << "<Flow flowId=\"" << id << "\" srcId=\""
           << key.srcId << "\" dstId=\"" << key.dstId << "\" fiveQi=\"" << +key.fiveQi
           << "\" sst=\"" << +key.sst << "\" sd=\"" << key.sd << "\" />\n";
    }
    os << std::string(indent, ' ') << "</NtnOranFlowClassifier>\n";
}

// --------------------------------------------------------------------- probe

TypeId
NtnOranFlowProbe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnOranFlowProbe")
                            .SetParent<FlowProbe>()
                            .SetGroupName("NtnTraffic");
    return tid;
}

NtnOranFlowProbe::NtnOranFlowProbe(Ptr<FlowMonitor> monitor,
                                   Ptr<NtnOranFlowClassifier> classifier,
                                   std::string pointName)
    : FlowProbe(monitor),
      m_classifier(classifier),
      m_pointName(std::move(pointName))
{
}

void
NtnOranFlowProbe::ReportTx(Ptr<const Packet> packet)
{
    // Guard BEFORE PeekHeader: Deserialize reads SERIALIZED_SIZE bytes
    // unconditionally (the `== 0` check is dead code), so a foreign shorter
    // packet on this port would over-read. Skip anything too small to carry the
    // header.
    if (packet->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE)
    {
        return;
    }
    NtnOranPayloadHeader hdr;
    if (packet->PeekHeader(hdr) == 0 ||
        hdr.GetVersion() != NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION)
    {
        return;
    }
    const FlowId id = m_classifier->Classify(hdr);
    m_flowMonitor->ReportFirstTx(this, id, hdr.GetSeq(), packet->GetSize());
}

void
NtnOranFlowProbe::ReportRx(Ptr<const Packet> packet)
{
    // Same size guard as ReportTx: a foreign sub-header-length packet on this
    // port must not reach PeekHeader's unconditional 24-byte deserialize.
    if (packet->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE)
    {
        return;
    }
    NtnOranPayloadHeader hdr;
    if (packet->PeekHeader(hdr) == 0 ||
        hdr.GetVersion() != NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION)
    {
        return;
    }
    const FlowId id = m_classifier->Classify(hdr);
    m_flowMonitor->ReportLastRx(this, id, hdr.GetSeq(), packet->GetSize());
}

// ------------------------------------------------------------------- monitor

TypeId
NtnOranAiFlowMonitor::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnOranAiFlowMonitor")
                            .SetParent<Object>()
                            .SetGroupName("NtnTraffic")
                            .AddConstructor<NtnOranAiFlowMonitor>();
    return tid;
}

NtnOranAiFlowMonitor::NtnOranAiFlowMonitor()
{
    m_monitor = CreateObject<FlowMonitor>();
    m_classifier = Create<NtnOranFlowClassifier>();
    m_monitor->Start(Seconds(0.0));
}

void
NtnOranAiFlowMonitor::AddSource(Ptr<NtnOranApplication> app)
{
    Ptr<NtnOranFlowProbe> probe =
        Create<NtnOranFlowProbe>(m_monitor, m_classifier, "source");
    m_probes.push_back(probe);
    app->TraceConnectWithoutContext(
        "Tx", MakeCallback(&NtnOranFlowProbe::ReportTx, PeekPointer(probe)));
}

void
NtnOranAiFlowMonitor::AddSink(Ptr<NtnOranSink> sink, int32_t ueIndex)
{
    AddSink(sink, ueIndex, Direction::Auto);
}

bool
NtnOranAiFlowMonitor::IsUplink(const SinkRef& ref) const
{
    if (ref.dir != Direction::Auto)
    {
        return ref.dir == Direction::Uplink;
    }
    // Auto: with a PHY source, a sink that is not on the UE it is mapped to sits
    // on the network side, so the flow is an uplink. Without one there is no way
    // to tell, and the historical downlink naming is kept.
    if (m_rs && ref.ueIndex >= 0 && ref.sink && ref.sink->GetNode())
    {
        const NetDeviceContainer ues = m_rs->GetUeDevices();
        if (static_cast<uint32_t>(ref.ueIndex) < ues.GetN())
        {
            return ues.Get(ref.ueIndex)->GetNode() != ref.sink->GetNode();
        }
    }
    return false;
}

void
NtnOranAiFlowMonitor::AddSink(Ptr<NtnOranSink> sink, int32_t ueIndex, Direction dir)
{
    Ptr<NtnOranFlowProbe> probe =
        Create<NtnOranFlowProbe>(m_monitor, m_classifier, "sink");
    m_probes.push_back(probe);
    // The sink's "Rx" trace fires with the full packet (header still inside).
    sink->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SinkRxTrampoline, probe));
    m_sinks.push_back({sink, ueIndex, dir});
}

void
NtnOranAiFlowMonitor::Start()
{
    if (m_started)
    {
        return;
    }
    m_started = true;
    Simulator::Schedule(m_granularity, &NtnOranAiFlowMonitor::GranularityTick, this);
}

void
NtnOranAiFlowMonitor::GranularityTick()
{
    const Time now = Simulator::Now();
    for (const auto& ref : m_sinks)
    {
        for (const auto& [sinkKey, fs] : ref.sink->GetFlowStats())
        {
            NtnOranPayloadHeader hdr;
            hdr.SetSrcId(fs.srcId);
            hdr.SetDstId(fs.dstId);
            hdr.SetFiveQi(fs.fiveQi);
            hdr.SetSnssai(fs.sst, fs.sd);
            const FlowId id = m_classifier->Classify(hdr);

            FlowCursor& cur = m_cursors[id];
            const bool firstPeriod = (cur.rxPackets == 0);
            const uint64_t dBytes = fs.rxBytes - cur.rxBytes;
            const uint64_t dPkts = fs.rxPackets - cur.rxPackets;
            // Wave 7A: the cumulative sequence-gap count can FALL when a late
            // (reordered or HARQ-delayed) packet fills a gap counted lost in an
            // earlier period. The unsigned difference then wrapped to ~1.8e19
            // and the period reported an absurd loss rate. A healed gap is not a
            // loss of this period, so the delta is clamped at zero.
            const uint64_t lostNow = fs.LostPackets();
            const uint64_t dLost = lostNow > cur.lostPackets ? lostNow - cur.lostPackets : 0;
            const double dDelaySum = fs.sumDelayMs - cur.sumDelayMs;
            cur.rxBytes = fs.rxBytes;
            cur.rxPackets = fs.rxPackets;
            cur.lostPackets = lostNow;
            cur.sumDelayMs = fs.sumDelayMs;
            const bool ul = IsUplink(ref);

            KpmSample s;
            s.time = now;
            // TS 28.552 5.1.1.3 DRB.UEThpDl / DRB.UEThpUl: data volume over the
            // time the flow was actually transferring (ThpVol / ThpTime), not
            // over the whole granularity period. In the first period of a flow
            // that starts mid-period the time base is first reception -> now;
            // dividing by the full period understated it (884.8 kbit/s against
            // a steady 2172.8 in sagin-a2g-real-stack).
            double periodS = m_granularity.GetSeconds();
            if (firstPeriod && fs.firstRx > now - m_granularity && fs.firstRx < now)
            {
                periodS = (now - fs.firstRx).GetSeconds();
            }
            // Values are all MEASURED (in-band byte/packet/delay deltas + PHY
            // trace), each in its TS 28.552 unit (see WriteCsv's units file):
            //   DRB.UEThpDl/Ul       kbit/s
            //   DRB.PdcpSduVolumeDl/Ul  Mbit (TS 28.552 5.1.2.1.1)
            //   DRB.RlcSduDelayDl, DRB.RlcDelayUl  0.1 ms (a value of 10 == 1 ms)
            //   DRB.PacketLossRateDl/Ul  1E-6 (fraction x 1e6), over the SDUs
            //                            received plus those detected lost in
            //                            the period
            const double ms = dPkts ? dDelaySum / dPkts : 0.0;
            s.metrics[ul ? "DRB.UEThpUl" : "DRB.UEThpDl"] =
                periodS > 0 ? dBytes * 8.0 / periodS / 1e3 : 0.0; // kbit/s
            s.metrics[ul ? "DRB.PdcpSduVolumeUl" : "DRB.PdcpSduVolumeDl"] =
                dBytes * 8.0 / 1e6; // Mbit
            if (ul)
            {
                s.metrics["DRB.RlcDelayUl"] = ms * 10.0; // 0.1 ms
            }
            else if (!m_ranKpm)
            {
                s.metrics["DRB.RlcSduDelayDl"] = ms * 10.0; // 0.1 ms
            }
            else
            {
                // The application probe keeps its own, honest name.
                s.metrics["app.OwdDl"] = ms; // ms
            }
            {
                const double lossFrac =
                    (dPkts + dLost) ? static_cast<double>(dLost) / (dPkts + dLost) : 0.0;
                s.metrics[ul ? "DRB.PacketLossRateUl" : "DRB.PacketLossRateDl"] = lossFrac * 1e6;
            }
            if (!ul && m_ranKpm && m_rs && ref.ueIndex >= 0)
            {
                // C6: RAN counters, deltas over this granularity period, taken
                // once per UE and per cell per tick (several flows of one UE
                // share the UE's DRB counters).
                if (m_ranTickTime != now)
                {
                    m_ranTickTime = now;
                    m_ranTick.clear();
                    m_cellPrbTick.clear();
                }
                if (m_ranTick.find(ref.ueIndex) == m_ranTick.end())
                {
                    const auto c = m_rs->GetUeRanCounters(static_cast<uint32_t>(ref.ueIndex));
                    RanCursor& rc = m_ranCursor[ref.ueIndex];
                    auto& v = m_ranTick[ref.ueIndex];
                    const uint64_t dSdu = c.rlcSdus - rc.rlcSdus;
                    // TS 28.552 5.1.3.3.1 DRB.RlcSduDelayDl, 0.1 ms units.
                    v["DRB.RlcSduDelayDl"] =
                        dSdu ? (c.rlcSduDelaySumMs - rc.rlcDelaySumMs) / dSdu * 10.0 : 0.0;
                    // TS 28.552 5.1.1.8: transport block COUNTS.
                    v["TB.TotNbrDl"] = static_cast<double>(c.dlTb - rc.dlTb);
                    v["TB.ErrTotNbrDl"] = static_cast<double>(c.dlTbErr - rc.dlTbErr);
                    rc.dlTb = c.dlTb;
                    rc.dlTbErr = c.dlTbErr;
                    rc.rlcDelaySumMs = c.rlcSduDelaySumMs;
                    rc.rlcSdus = c.rlcSdus;
                }
                for (const auto& [k, val] : m_ranTick[ref.ueIndex])
                {
                    s.metrics[k] = val;
                }
                const uint16_t cell = m_rs->GetUeServingCellId(static_cast<uint32_t>(ref.ueIndex));
                if (cell != 0)
                {
                    if (m_cellPrbTick.find(cell) == m_cellPrbTick.end())
                    {
                        // TS 28.552 5.1.1.2.x RRU.PrbUsedDl: mean number of DL
                        // PRBs in use per slot over the period (PRB x symbols/14).
                        const double tot = m_rs->GetCellDlPrbSlotsUsed(cell);
                        const double slots =
                            m_granularity.GetSeconds() / m_rs->GetSlotDuration().GetSeconds();
                        m_cellPrbTick[cell] =
                            slots > 0 ? (tot - m_cellPrbCursor[cell]) / slots : 0.0;
                        m_cellPrbCursor[cell] = tot;
                    }
                    s.metrics["RRU.PrbUsedDl"] = m_cellPrbTick[cell];
                }
            }
            if (!ul && m_rs && ref.ueIndex >= 0)
            {
                const double sinr = m_rs->GetUeRecentSinrDb(ref.ueIndex);
                const double tbler = m_rs->GetUeRecentTbler(ref.ueIndex);
                if (!std::isnan(sinr))
                {
                    // L1M.RS-SINR: DELIBERATE deviation. TS 38.133 §10.1.16
                    // defines a 0..127 RS-SINR report mapping, but this series
                    // carries the raw measured PHY SINR in dB (as the sibling
                    // oran-ntn kpm-canonical-ids CARR.AverageSINR / L1M.RS-SINR.Mean
                    // also do) so xApp/anomaly logic reads a physical dB value.
                    // Unit here is dB, not the 38.133 index.
                    s.metrics["L1M.RS-SINR"] = sinr;
                }
                if (!std::isnan(tbler))
                {
                    // TB.ErrTotNbrDl is a TB COUNT in TS 28.552 §5.1.1.8, but no
                    // integrating per-UE/per-period corrupt-TB counter is exposed
                    // by NtnRealStackHelper (only the aggregate GetPhyRxTb() and a
                    // per-UE BLER RATIO via GetUeRecentTbler()). Following the
                    // reviewed oran-ntn precedent, this series carries the measured
                    // DL HARQ BLER FRACTION (0..1); once an absolute TB counter is
                    // plumbed the canonical relation TB.ErrTotNbrDl / TB.TotNbrDl
                    // == BLER restores the count semantics. Value unit: fraction.
                    s.metrics[m_ranKpm ? "app.TbBlerDl" : "TB.ErrTotNbrDl"] = tbler;
                }
            }
            m_series[id].push_back(s);

            // EWMA z-score anomaly detection per metric.
            for (const auto& [name, value] : s.metrics)
            {
                Ewma& e = m_ewma[id][name];
                if (e.n >= 5) // warm-up before judging
                {
                    const double sd = std::sqrt(std::max(e.var, 1e-12));
                    const double z = (value - e.mean) / sd;
                    if (std::abs(z) >= m_anomalyZ)
                    {
                        AnomalyEvent ev;
                        ev.time = now;
                        ev.flowId = id;
                        m_classifier->FindFlow(id, ev.key);
                        ev.metric = name;
                        ev.value = value;
                        ev.zScore = z;
                        m_anomalies.push_back(ev);
                        if (m_anomalyCb)
                        {
                            m_anomalyCb(ev);
                        }
                    }
                }
                const double alpha = 0.2;
                const double diff = value - e.mean;
                e.mean += alpha * diff;
                e.var = (1.0 - alpha) * (e.var + alpha * diff * diff);
                ++e.n;
            }

            if (m_e2Cb)
            {
                KpmIndication ind;
                ind.collectionStart = now - m_granularity;
                ind.granularity = m_granularity;
                ind.flowId = id;
                m_classifier->FindFlow(id, ind.key);
                ind.measurements = s.metrics;
                m_e2Cb(ind);
            }
        }
    }
    Simulator::Schedule(m_granularity, &NtnOranAiFlowMonitor::GranularityTick, this);
}

NtnOranAiFlowMonitor::AiFeatures
NtnOranAiFlowMonitor::GetFeatures(FlowId id) const
{
    AiFeatures f;
    auto it = m_series.find(id);
    if (it == m_series.end() || it->second.empty())
    {
        return f;
    }
    const auto& series = it->second;
    const uint32_t n = std::min<uint32_t>(m_featureWindow, series.size());
    auto metric = [](const KpmSample& s, const char* name) {
        auto m = s.metrics.find(name);
        return (m != s.metrics.end()) ? m->second : 0.0;
    };
    // Window means + least-squares slope per granularity period.
    double sumT = 0, sumT2 = 0;
    double sumThp = 0, sumThpT = 0, sumD = 0, sumDT = 0, sumL = 0;
    double sumS = 0, sumST = 0;
    double prevD = 0;
    double jitterAccum = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        const auto& s = series[series.size() - n + i];
        const double t = static_cast<double>(i);
        // The KPM series now store TS 28.552 canonical UNITS (kbit/s, 0.1 ms,
        // per-1e6). The AiFeatures fields are documented in natural units
        // (thpMeanMbps, delayMeanMs, lossMean fraction) and feed RL/xApp logic,
        // so convert back here — export stays spec-correct, the model reads
        // physical quantities.
        // An uplink series carries the Ul names (Wave 7A).
        const bool ulS = s.metrics.count("DRB.UEThpUl") > 0;
        const double thp = metric(s, ulS ? "DRB.UEThpUl" : "DRB.UEThpDl") / 1e3; // kbit/s -> Mbps
        const double d =
            metric(s, ulS ? "DRB.RlcDelayUl" : "DRB.RlcSduDelayDl") / 10.0; // 0.1 ms -> ms
        const double l =
            metric(s, ulS ? "DRB.PacketLossRateUl" : "DRB.PacketLossRateDl") / 1e6; // 1E-6 -> fraction
        const double si = metric(s, "L1M.RS-SINR");
        sumT += t;
        sumT2 += t * t;
        sumThp += thp;
        sumThpT += thp * t;
        sumD += d;
        sumDT += d * t;
        sumL += l;
        sumS += si;
        sumST += si * t;
        if (i > 0)
        {
            jitterAccum += std::abs(d - prevD);
        }
        prevD = d;
    }
    const double denom = n * sumT2 - sumT * sumT;
    f.windowLen = n;
    f.thpMeanMbps = sumThp / n;
    f.delayMeanMs = sumD / n;
    f.lossMean = sumL / n;
    f.sinrMeanDb = sumS / n;
    f.jitterMs = (n > 1) ? jitterAccum / (n - 1) : 0.0;
    if (std::abs(denom) > 1e-12)
    {
        f.thpSlope = (n * sumThpT - sumT * sumThp) / denom;
        f.delaySlope = (n * sumDT - sumT * sumD) / denom;
        f.sinrSlope = (n * sumST - sumT * sumS) / denom;
    }
    return f;
}

void
NtnOranAiFlowMonitor::SerializeToXmlFile(const std::string& path) const
{
    std::ofstream os(path);
    os << "<?xml version=\"1.0\" ?>\n<NtnOranAiFlowMonitor>\n";
    m_monitor->SerializeToXmlStream(os, 2, /*enableHistograms=*/false,
                                    /*enableProbes=*/true);
    m_classifier->SerializeToXmlStream(os, 2);
    os << "</NtnOranAiFlowMonitor>\n";
}

void
NtnOranAiFlowMonitor::WriteCsv(const std::string& path) const
{
    std::ofstream os(path);
    os << "time_s,flow_id,src_id,dst_id,five_qi,sst,sd,DRB.UEThpDl,"
          "DRB.PdcpSduVolumeDl,DRB.RlcSduDelayDl,DRB.PacketLossRateDl,"
          "L1M.RS-SINR,TB.ErrTotNbrDl";
    // C6: the RAN-counter columns exist only in that mode, appended so the
    // legacy column layout is byte-identical.
    if (m_ranKpm)
    {
        os << ",TB.TotNbrDl,RRU.PrbUsedDl,app.OwdDl,app.TbBlerDl";
    }
    // Wave 7A: uplink flows carry the UL measurement names, appended so the
    // downlink column layout is unchanged.
    os << ",DRB.UEThpUl,DRB.PdcpSduVolumeUl,DRB.RlcDelayUl,DRB.PacketLossRateUl";
    os << "\n";
    // Units of every column (TS 28.552), next to the series.
    {
        std::string unitsPath = path;
        const auto dot = unitsPath.rfind(".csv");
        unitsPath = (dot == std::string::npos ? unitsPath : unitsPath.substr(0, dot)) +
                    "_units.csv";
        std::ofstream u(unitsPath);
        u << "column,unit,note\n"
          << "DRB.UEThpDl,kbit/s,TS 28.552 5.1.1.3: volume over the time data was transferred\n"
          << "DRB.UEThpUl,kbit/s,TS 28.552 5.1.1.3 (uplink)\n"
          << "DRB.PdcpSduVolumeDl,Mbit,TS 28.552 5.1.2.1.1\n"
          << "DRB.PdcpSduVolumeUl,Mbit,TS 28.552 5.1.2.1.1 (uplink)\n"
          << "DRB.RlcSduDelayDl,0.1 ms,TS 28.552 5.1.1.1.1: a value of 224.9 is 22.49 ms; "
             "measured in band (application one-way delay) unless RAN counters are on\n"
          << "DRB.RlcDelayUl,0.1 ms,uplink delay measured in band (application one-way delay)\n"
          << "DRB.PacketLossRateDl,1E-6,lost / (received + lost) in the period x 1e6\n"
          << "DRB.PacketLossRateUl,1E-6,TS 28.552 UL PDCP SDU loss rate x 1e6\n"
          << "L1M.RS-SINR,dB,raw measured DL SINR (not the TS 38.133 report index)\n"
          << "TB.ErrTotNbrDl,fraction or count,BLER fraction unless RAN counters are on\n"
          << "TB.TotNbrDl,count,TS 28.552 5.1.1.8 (RAN counters)\n"
          << "RRU.PrbUsedDl,PRB,mean DL PRBs in use per slot (RAN counters)\n"
          << "app.OwdDl,ms,application one-way delay (RAN counter mode)\n"
          << "app.TbBlerDl,fraction,DL BLER (RAN counter mode)\n";
    }
    // A metric the period did not measure (no UE-side PHY sample for this
    // flow, e.g. an uplink or gateway-side flow) is written as an empty field,
    // the CSV convention for a missing value, never as the text "nan".
    auto get = [](const KpmSample& s, const char* name) -> std::string {
        auto it = s.metrics.find(name);
        if (it == s.metrics.end() || !std::isfinite(it->second))
        {
            return std::string();
        }
        std::ostringstream v;
        v << it->second;
        return v.str();
    };
    for (const auto& [id, series] : m_series)
    {
        OranFlowKey key;
        m_classifier->FindFlow(id, key);
        for (const auto& s : series)
        {
            os << s.time.GetSeconds() << "," << id << "," << key.srcId << ","
               << key.dstId << "," << +key.fiveQi << "," << +key.sst << "," << key.sd
               << "," << get(s, "DRB.UEThpDl") << "," << get(s, "DRB.PdcpSduVolumeDl")
               << "," << get(s, "DRB.RlcSduDelayDl") << ","
               << get(s, "DRB.PacketLossRateDl") << "," << get(s, "L1M.RS-SINR") << ","
               << get(s, "TB.ErrTotNbrDl");
            if (m_ranKpm)
            {
                os << "," << get(s, "TB.TotNbrDl") << "," << get(s, "RRU.PrbUsedDl") << ","
                   << get(s, "app.OwdDl") << "," << get(s, "app.TbBlerDl");
            }
            os << "," << get(s, "DRB.UEThpUl") << "," << get(s, "DRB.PdcpSduVolumeUl") << ","
               << get(s, "DRB.RlcDelayUl") << "," << get(s, "DRB.PacketLossRateUl");
            os << "\n";
        }
    }
}

void
NtnOranAiFlowMonitor::WriteInfluxLp(const std::string& path) const
{
    std::ofstream os(path);
    for (const auto& [id, series] : m_series)
    {
        OranFlowKey key;
        m_classifier->FindFlow(id, key);
        for (const auto& s : series)
        {
            os << "ntn_oran_kpm,flow_id=" << id << ",five_qi=" << +key.fiveQi
               << ",sst=" << +key.sst << ",sd=" << key.sd
               << ",provenance=inband-header ";
            bool first = true;
            for (const auto& [name, value] : s.metrics)
            {
                std::string field = name;
                for (auto& c : field)
                {
                    if (c == '.' || c == '-')
                    {
                        c = '_';
                    }
                }
                os << (first ? "" : ",") << field << "=" << value;
                first = false;
            }
            os << " " << s.time.GetNanoSeconds() << "\n";
        }
    }
}

} // namespace ns3
