// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-oran-ai-flow-monitor.h"

#include "ntn-oran-application.h"

#include "../helper/ntn-real-stack-helper.h"

#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <cmath>
#include <fstream>

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
    Ptr<NtnOranFlowProbe> probe =
        Create<NtnOranFlowProbe>(m_monitor, m_classifier, "sink");
    m_probes.push_back(probe);
    // The sink's "Rx" trace fires with the full packet (header still inside).
    sink->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SinkRxTrampoline, probe));
    m_sinks.push_back({sink, ueIndex});
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
            const uint64_t dBytes = fs.rxBytes - cur.rxBytes;
            const uint64_t dPkts = fs.rxPackets - cur.rxPackets;
            const uint64_t dLost = fs.LostPackets() - cur.lostPackets;
            const double dDelaySum = fs.sumDelayMs - cur.sumDelayMs;
            cur.rxBytes = fs.rxBytes;
            cur.rxPackets = fs.rxPackets;
            cur.lostPackets = fs.LostPackets();
            cur.sumDelayMs = fs.sumDelayMs;

            KpmSample s;
            s.time = now;
            const double periodS = m_granularity.GetSeconds();
            // TS 28.552 / E2SM-KPM measurement names.
            s.metrics["DRB.UEThpDl"] = dBytes * 8.0 / periodS / 1e6; // Mbps
            s.metrics["DRB.PdcpSduVolumeDl"] = dBytes * 8.0 / 1e3;   // kbit
            s.metrics["DRB.RlcSduDelayDl"] = dPkts ? dDelaySum / dPkts : 0.0; // ms
            s.metrics["DRB.PacketLossRateDl"] =
                (dPkts + dLost) ? static_cast<double>(dLost) / (dPkts + dLost) : 0.0;
            if (m_rs && ref.ueIndex >= 0)
            {
                const double sinr = m_rs->GetUeRecentSinrDb(ref.ueIndex);
                const double tbler = m_rs->GetUeRecentTbler(ref.ueIndex);
                if (!std::isnan(sinr))
                {
                    s.metrics["L1M.RS-SINR"] = sinr;
                }
                if (!std::isnan(tbler))
                {
                    s.metrics["TB.ErrTotalNbrDl.Rate"] = tbler;
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
        const double thp = metric(s, "DRB.UEThpDl");
        const double d = metric(s, "DRB.RlcSduDelayDl");
        const double l = metric(s, "DRB.PacketLossRateDl");
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
          "L1M.RS-SINR,TB.ErrTotalNbrDl.Rate\n";
    auto get = [](const KpmSample& s, const char* name) {
        auto it = s.metrics.find(name);
        return (it != s.metrics.end()) ? it->second : std::nan("");
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
               << get(s, "TB.ErrTotalNbrDl.Rate") << "\n";
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
