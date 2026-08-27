// SPDX-License-Identifier: GPL-2.0-only
//
// NtnOranSink — measuring server of the AI-Native ORAN-NTN application suite
// (AI-Native ORAN-NTN adoption WS1). Receives NtnOranApplication /
// NtnCommandAndControlApp packets and computes, PER QoS FLOW (keyed by
// srcId + 5QI + S-NSSAI from the in-band NtnOranPayloadHeader):
//   * one-way delay  — RX time minus the in-band TX timestamp,
//   * jitter         — RFC 3550 interarrival-jitter estimator,
//   * loss           — sequence-number gaps,
//   * throughput     — received bytes over the flow's active span.
// All of it is measured at the application from received bytes that crossed
// the real radio + GTP tunnel — never inferred from a formula or a tag.

#ifndef NTN_ORAN_SINK_H
#define NTN_ORAN_SINK_H

#include "ntn-oran-payload-header.h"

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"

#include <cstdint>
#include <map>
#include <set>
#include <tuple>


#include <limits>
#include <vector>
namespace ns3
{

class Socket;
class Packet;

/// Platform C&C telemetry payload (paper Sec. III-C); serialized by
/// NtnCommandAndControlApp after the NtnOranPayloadHeader.
struct NtnCncTelemetry
{
    double posX{0}, posY{0}, posZ{0};    ///< m, from the REAL mobility model
    double velX{0}, velY{0}, velZ{0};    ///< m/s
    double rollDeg{0}, pitchDeg{0}, yawDeg{0}; ///< attitude from the velocity frame
    double batteryFraction{1.0};         ///< 0..1 remaining energy
    uint64_t uptimeNs{0};

    static constexpr uint32_t SERIALIZED_SIZE = 9 * 8 + 2 + 8;
    void WriteTo(uint8_t* buf) const;
    static NtnCncTelemetry ReadFrom(const uint8_t* buf);
};

class NtnOranSink : public Application
{
  public:
    struct FlowStats
    {
        uint8_t fiveQi{0};
        uint8_t sst{0};
        uint32_t sd{0};
        uint16_t srcId{0};
        uint16_t dstId{0};
        uint8_t payloadType{0};
        uint64_t rxPackets{0};
        uint64_t rxBytes{0};
        uint32_t highestSeq{0};
        uint64_t reordered{0};
        double sumDelayMs{0};
        double maxDelayMs{0};
        double jitterMs{0};       ///< RFC 3550 running estimate
        double lastTransitMs{0};
        Time firstRx{Seconds(0)};
        Time lastRx{Seconds(0)};

        /// SLICE-1: bounded one-way-delay histogram, 1 ms bins from 0 to
        /// kDelayBins-1 ms plus an overflow counter. Without it the sink kept
        /// only sums, so a caller wanting a latency PERCENTILE had nothing to
        /// work from and the slice examples stamped every packet with the mean:
        /// the reported p99 was the p99 of a constant, and no percentile SLA
        /// could ever breach. A histogram is deterministic and bounded, unlike
        /// retaining every sample.
        static constexpr uint32_t kDelayBins = 4096;
        std::vector<uint32_t> delayHistMs;  ///< lazily sized to kDelayBins
        uint64_t delayOverflow{0};          ///< samples at or beyond kDelayBins ms

        void RecordDelaySample(double delayMs)
        {
            if (delayHistMs.empty())
            {
                delayHistMs.assign(kDelayBins, 0u);
            }
            if (delayMs < 0.0)
            {
                delayMs = 0.0;
            }
            const uint32_t bin = static_cast<uint32_t>(delayMs);
            if (bin < kDelayBins)
            {
                delayHistMs[bin]++;
            }
            else
            {
                delayOverflow++;
            }
        }

        /// Delay at quantile \p q in [0,1], in ms, from the histogram. Returns
        /// the bin midpoint, so resolution is 1 ms. NaN when no samples.
        double DelayPercentileMs(double q) const
        {
            const uint64_t total = rxPackets;
            if (total == 0 || delayHistMs.empty())
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
            const uint64_t target =
                static_cast<uint64_t>(q * static_cast<double>(total));
            uint64_t cum = 0;
            for (uint32_t b = 0; b < kDelayBins; ++b)
            {
                cum += delayHistMs[b];
                if (cum >= target)
                {
                    return b + 0.5;
                }
            }
            return kDelayBins + 0.5; // everything remaining is in the overflow
        }

        double MeanDelayMs() const { return rxPackets ? sumDelayMs / rxPackets : 0.0; }
        /// Lost = expected (from seq span) minus received.
        uint64_t LostPackets() const
        {
            const uint64_t expected = static_cast<uint64_t>(highestSeq) + 1;
            return (expected > rxPackets) ? expected - rxPackets : 0;
        }
        double LossRatio() const
        {
            const uint64_t expected = static_cast<uint64_t>(highestSeq) + 1;
            return expected ? static_cast<double>(LostPackets()) / expected : 0.0;
        }
        double ThroughputMbps() const
        {
            const double span = (lastRx - firstRx).GetSeconds();
            return (span > 1e-9) ? rxBytes * 8.0 / span / 1e6 : 0.0;
        }
    };

    /// Flow key: source endpoint + QoS/slice identity from the in-band header.
    using FlowKey = std::tuple<uint16_t /*srcId*/, uint8_t /*5qi*/, uint8_t /*sst*/,
                               uint32_t /*sd*/>;

    static TypeId GetTypeId();
    NtnOranSink();
    ~NtnOranSink() override;

    uint64_t GetTotalRx() const { return m_totalRxBytes; }
    uint64_t GetRxPackets() const { return m_totalRxPackets; }
    double GetMeanDelayMs() const;
    double GetMeanJitterMs() const;
    double GetLossRatio() const;
    const std::map<FlowKey, FlowStats>& GetFlowStats() const { return m_flows; }
    /// Packets discarded because their NtnOranPayloadHeader carried an
    /// unexpected wire-format version (excluded from all flow KPIs).
    uint64_t GetVersionErrors() const { return m_versionErrors; }
    /// Latest platform telemetry seen from \p srcId; false if none yet.
    bool GetLatestTelemetry(uint16_t srcId, NtnCncTelemetry& out) const;

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;
    void HandleRead(Ptr<Socket> socket);

    Address m_local;
    Ptr<Socket> m_socket;
    std::map<FlowKey, FlowStats> m_flows;
    std::map<uint16_t, NtnCncTelemetry> m_telemetry;
    uint64_t m_totalRxBytes{0};
    uint64_t m_totalRxPackets{0};
    uint64_t m_versionErrors{0};
    std::set<FlowKey> m_versionWarnedFlows; ///< rate-limits the version warning

    TracedCallback<Ptr<const Packet>, const Address&> m_rxTrace;
};

} // namespace ns3

#endif // NTN_ORAN_SINK_H
