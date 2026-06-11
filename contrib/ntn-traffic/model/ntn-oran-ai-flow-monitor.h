// SPDX-License-Identifier: GPL-2.0-only
//
// NtnOranAiFlowMonitor — the AI-native measurement layer of the ORAN-NTN
// adoption plan (AI-Native ORAN-NTN adoption WS2). Built on the REAL
// ns-3 flow-monitor infrastructure, deeply:
//
//   * NtnOranFlowClassifier : FlowClassifier — flows are keyed by the ORAN
//     QoS identity carried in REAL packet bytes (NtnOranPayloadHeader srcId/
//     dstId/5QI/S-NSSAI), exactly how a DRB/QoS-flow is keyed in TS 38.415 —
//     instead of the plain IP 5-tuple. Immune to GTP re-encapsulation since
//     classification happens at the application measurement points.
//   * NtnOranFlowProbe : FlowProbe — one probe per measurement point (the
//     traffic source and each NtnOranSink). Every packet is reported into a
//     real ns3::FlowMonitor (ReportFirstTx / ReportLastRx with the in-band
//     sequence number as packet id), so FlowMonitor's delay/jitter/loss
//     machinery and XML serialization operate on the ORAN flows.
//   * NtnOranAiFlowMonitor : Object — the KPM/AI layer on top:
//       - per-flow KPI time series at a KPM granularity period (default 1 s)
//         published under the OFFICIAL names (3GPP TS 28.552 / O-RAN E2SM-KPM):
//         DRB.UEThpDl, DRB.RlcSduDelayDl, DRB.PacketLossRateDl,
//         DRB.PdcpSduVolumeDl, and (when attached to NtnRealStackHelper)
//         L1M.RS-SINR, TB.TotNbrDl, TB.ErrTotalNbrDl from the PHY trace;
//       - sliding-window AI feature vectors per flow (mean/slope of
//         throughput, delay, loss, SINR) for xApps / ns3-ai-ntn / ONNX;
//       - EWMA z-score anomaly detector per flow per metric raising events
//         (the paper's zero-touch self-protection hook);
//       - exporters: FlowMonitor XML, CSV, InfluxDB line protocol, and
//         E2SM-KPM-shaped indication callbacks for the oran-ntn E2 node.
//
// Every number in the pipeline is MEASURED: app KPIs from in-band header
// primitives at the sinks, PHY KPIs from the mmwave RxPacketTraceUe trace.

#ifndef NTN_ORAN_AI_FLOW_MONITOR_H
#define NTN_ORAN_AI_FLOW_MONITOR_H

#include "ntn-oran-payload-header.h"
#include "ntn-oran-sink.h"

#include "ns3/flow-classifier.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-probe.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ns3
{

class NtnOranApplication;
class NtnRealStackHelper;

/// ORAN QoS-flow identity used as the flow key (how a DRB/QoS flow is keyed).
struct OranFlowKey
{
    uint16_t srcId{0};
    uint16_t dstId{0};
    uint8_t fiveQi{0};
    uint8_t sst{0};
    uint32_t sd{0};

    bool operator<(const OranFlowKey& o) const
    {
        return std::tie(srcId, dstId, fiveQi, sst, sd) <
               std::tie(o.srcId, o.dstId, o.fiveQi, o.sst, o.sd);
    }
};

class NtnOranFlowClassifier : public FlowClassifier
{
  public:
    /// Classify by the ORAN identity in the in-band header; allocates a new
    /// FlowId on first sight.
    FlowId Classify(const NtnOranPayloadHeader& hdr);
    /// Reverse lookup; false if unknown.
    bool FindFlow(FlowId id, OranFlowKey& out) const;
    void SerializeToXmlStream(std::ostream& os, uint16_t indent) const override;

  private:
    std::map<OranFlowKey, FlowId> m_byKey;
    std::map<FlowId, OranFlowKey> m_byId;
};

class NtnOranFlowProbe : public FlowProbe
{
  public:
    static TypeId GetTypeId();
    NtnOranFlowProbe(Ptr<FlowMonitor> monitor,
                     Ptr<NtnOranFlowClassifier> classifier,
                     std::string pointName);

    /// Report a packet leaving the traffic source (FlowMonitor first-TX).
    void ReportTx(Ptr<const Packet> packet);
    /// Report a packet arriving at a sink (FlowMonitor last-RX).
    void ReportRx(Ptr<const Packet> packet);

  private:
    Ptr<NtnOranFlowClassifier> m_classifier;
    std::string m_pointName;
};

class NtnOranAiFlowMonitor : public Object
{
  public:
    /// One KPM granularity-period sample for one flow (official metric names).
    struct KpmSample
    {
        Time time;
        std::map<std::string, double> metrics;
    };

    /// Sliding-window AI feature vector for one flow.
    struct AiFeatures
    {
        double thpMeanMbps{0}, thpSlope{0};
        double delayMeanMs{0}, delaySlope{0};
        double lossMean{0};
        double sinrMeanDb{0}, sinrSlope{0};
        double jitterMs{0};
        uint32_t windowLen{0};
        std::vector<double> AsVector() const
        {
            return {thpMeanMbps, thpSlope, delayMeanMs, delaySlope,
                    lossMean,    sinrMeanDb, sinrSlope, jitterMs};
        }
    };

    /// Anomaly event raised by the EWMA z-score detector.
    struct AnomalyEvent
    {
        Time time;
        FlowId flowId;
        OranFlowKey key;
        std::string metric;
        double value{0};
        double zScore{0};
    };

    /// E2SM-KPM-shaped indication (one granularity period, one flow), shaped
    /// for the oran-ntn E2 node (RIC Indication message payload).
    struct KpmIndication
    {
        Time collectionStart;
        Time granularity;
        FlowId flowId;
        OranFlowKey key;
        std::map<std::string, double> measurements; // TS 28.552 names
    };

    static TypeId GetTypeId();
    NtnOranAiFlowMonitor();

    // ---- wiring ----------------------------------------------------------
    void SetGranularityPeriod(Time t) { m_granularity = t; }
    void SetFeatureWindow(uint32_t n) { m_featureWindow = n; }
    void SetAnomalyZThreshold(double z) { m_anomalyZ = z; }
    /// Attach the PHY-measured KPI source (L1M.RS-SINR, TB counters). The
    /// helper outlives the monitor in every example, so a raw pointer is fine.
    void SetPhySource(const NtnRealStackHelper* rs) { m_rs = rs; }
    /// Register a traffic source: its "Tx" trace reports into the probe.
    void AddSource(Ptr<NtnOranApplication> app);
    /// Register a sink: its "Rx" trace reports into the probe, and its
    /// cumulative per-flow stats feed the KPM series. \p ueIndex links the
    /// flow to the UE for PHY metrics (-1 = no PHY mapping).
    void AddSink(Ptr<NtnOranSink> sink, int32_t ueIndex = -1);
    /// Start the granularity timer (call once after wiring; idempotent).
    void Start();

    // ---- consumers -------------------------------------------------------
    const std::map<FlowId, std::vector<KpmSample>>& GetKpmSeries() const
    {
        return m_series;
    }
    /// Feature vector over the last FeatureWindow granularity periods.
    AiFeatures GetFeatures(FlowId id) const;
    const std::vector<AnomalyEvent>& GetAnomalies() const { return m_anomalies; }
    void RegisterAnomalyCallback(std::function<void(const AnomalyEvent&)> cb)
    {
        m_anomalyCb = std::move(cb);
    }
    /// E2 consumer: called once per flow per granularity period.
    void RegisterE2Consumer(std::function<void(const KpmIndication&)> cb)
    {
        m_e2Cb = std::move(cb);
    }
    Ptr<FlowMonitor> GetFlowMonitor() const { return m_monitor; }
    Ptr<NtnOranFlowClassifier> GetClassifier() const { return m_classifier; }

    // ---- exporters -------------------------------------------------------
    /// Classic FlowMonitor XML (with the ORAN classifier records).
    void SerializeToXmlFile(const std::string& path) const;
    /// Wide CSV: one row per flow per granularity period.
    void WriteCsv(const std::string& path) const;
    /// InfluxDB line protocol (measurement "ntn_oran_kpm").
    void WriteInfluxLp(const std::string& path) const;

  private:
    void GranularityTick();

    struct SinkRef
    {
        Ptr<NtnOranSink> sink;
        int32_t ueIndex;
    };

    /// Cumulative counters at the previous tick, for windowed deltas.
    struct FlowCursor
    {
        uint64_t rxBytes{0};
        uint64_t rxPackets{0};
        uint64_t lostPackets{0};
        double sumDelayMs{0};
    };

    struct Ewma
    {
        double mean{0};
        double var{0};
        uint64_t n{0};
    };

    Time m_granularity{Seconds(1.0)};
    uint32_t m_featureWindow{10};
    double m_anomalyZ{4.0};
    const NtnRealStackHelper* m_rs{nullptr};

    Ptr<FlowMonitor> m_monitor;
    Ptr<NtnOranFlowClassifier> m_classifier;
    std::vector<Ptr<NtnOranFlowProbe>> m_probes;
    std::vector<SinkRef> m_sinks;
    bool m_started{false};

    std::map<FlowId, std::vector<KpmSample>> m_series;
    std::map<FlowId, FlowCursor> m_cursors;
    std::map<FlowId, std::map<std::string, Ewma>> m_ewma;
    std::vector<AnomalyEvent> m_anomalies;
    std::function<void(const AnomalyEvent&)> m_anomalyCb;
    std::function<void(const KpmIndication&)> m_e2Cb;
};

} // namespace ns3

#endif // NTN_ORAN_AI_FLOW_MONITOR_H
