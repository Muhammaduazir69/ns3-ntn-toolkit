// SPDX-License-Identifier: GPL-2.0-only
//
// NtnRealStackHelper — the Phase-0 keystone of 2026-06 protocol-fidelity audit.
//
// Unlike NtnRealisticTrafficHelper (a plain PointToPoint star whose packets
// never touch any radio physics), this helper installs a REAL NR-style air
// interface — the in-tree `mmwave` module: SpectrumPhy + MAC scheduler + HARQ +
// AMC + RLC/PDCP + RRC + EPC — between caller-supplied satellite (gNB) and
// ground (UE) nodes that already carry their own ns-3 MobilityModel (SGP4,
// HAPS, OpenSky, ...). The link is NTN-ized: free-space (Friis) path loss valid
// at LEO range, S-band carrier (3GPP NR-NTN FR1), satellite EIRP via Tx power,
// HARQ off by default (terrestrial HARQ timers break over the slant), and the
// 3GPP terrestrial spatial-fading channel disabled (invalid at LEO geometry).
//
// Crucially, every headline KPI is MEASURED, not computed:
//   * DL SINR / TBLER / corrupt-fraction come from the mmwave SpectrumPhy
//     RxPacketTraceUe trace (struct ns3::mmwave::RxPacketTraceParams).
//   * Throughput / one-way delay / jitter / loss come from NtnOranSink: every
//     NtnOranApplication packet carries an in-band NtnOranPayloadHeader
//     (seq + TX timestamp + 5QI/S-NSSAI as real bytes inside the GTP tunnel),
//     so the app-layer KPIs are computed from received bytes (WS1 suite).
// WriteHealthReport() emits an HONEST sim_health.csv whose gates assert that the
// packets actually traversed the radio stack and that the KPIs have trace
// provenance — replacing the cosmetic "clock advanced over a P2P link" gates.
//
// Typical use (in any example):
//   NtnRealStackHelper rs;
//   rs.SetSimTime(Seconds(simTime));
//   rs.SetOutputDir(outputDir);
//   rs.Build(satNodes /*gNB, with MobilityModel*/, ueNodes /*ground, with MobilityModel*/);
//   rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
//                     Seconds(1.0), Seconds(simTime - 1.0));
//   Simulator::Stop(Seconds(simTime));
//   Simulator::Run();
//   rs.Collect();              // pull measured KPIs from FlowMonitor + PHY sink
//   rs.WriteHealthReport();    // honest sim_health.csv
//   Simulator::Destroy();
//
// Module logic (CHO/RIC/slice) reads measured per-UE/per-cell SINR via
// GetMeanDlSinrDb()/GetCellMeanSinrDb() instead of a closed-form formula.

#ifndef NTN_REAL_STACK_HELPER_H
#define NTN_REAL_STACK_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class Node;
class Packet;
class Address;
class MobilityModel;
class PropagationLossModel;
class NtnOranAiFlowMonitor;

namespace mmwave
{
class MmWaveHelper;
class MmWavePointToPointEpcHelper;
struct RxPacketTraceParams;
} // namespace mmwave

/**
 * \brief Installs a real mmwave (NR) NTN air interface and measures KPIs from
 *        the live data plane. See file header for rationale.
 */
class NtnRealStackHelper
{
  public:
    /// Pre-canned downlink traffic mixes (remote-host -> UE over the radio).
    /// Backed by NtnOranApplication QoS flows (AI_NATIVE_ORAN_NTN plan WS1):
    /// every packet carries an in-band NtnOranPayloadHeader (5QI/S-NSSAI/seq/
    /// timestamp), so delay/jitter/loss are measured from received bytes.
    enum class TrafficProfile : uint8_t
    {
        NbIotPeriodic,       ///< mMTC 128 B / 64 ms (5QI 9)
        EmbbStreaming,       ///< saturating UDP, 1400 B (5QI 2)
        UrllcPings,          ///< 256 B / 10 ms (5QI 82)
        ConversationalVoice, ///< vocoder 20 ms cadence (5QI 1)
        MixedBouquet,        ///< NB-IoT / eMBB / URLLC, 1/3 each across UEs
    };

    /// Satellite payload architecture (Deng 2026 Sec. II-B2; WS4). Selects
    /// where the gNB functions live and therefore which legs the user plane
    /// crosses; the extra one-way delay is computed from the LIVE feeder
    /// slant range when SetFeederGeometry() is wired:
    ///   Transparent      bent-pipe: the user plane crosses the RF feeder leg
    ///                    too -> 2 x slant/c
    ///   RegenerativeRu   O-RU on sat, O-DU/O-CU ground: Open-FH (7.2x) over
    ///                    the feeder -> slant/c + 0.25 ms lower-PHY budget
    ///   RegenerativeRuDu O-DU on sat: F1 midhaul over the feeder ->
    ///                    slant/c + 0.15 ms
    ///   FullGnb          full gNB on sat: GTP backhaul to the ground core ->
    ///                    slant/c + 0.05 ms (default option)
    enum class PayloadOption : uint8_t
    {
        Transparent,
        RegenerativeRu,
        RegenerativeRuDu,
        FullGnb,
    };

    /// Honest realism floors asserted at end of run.
    struct HealthGates
    {
        uint64_t minPhyRxTb = 50;     ///< transport blocks decoded at the UE PHY
        double minRxThroughputMbps = 0.05; ///< measured app throughput floor
        bool requireSinrProvenance = true; ///< SINR must come from a PHY trace
        bool requireErrorModelActive = true; ///< the error model must run (TBLER samples exist)
    };

    NtnRealStackHelper();
    ~NtnRealStackHelper();

    // ---- Configuration (call before Build) ------------------------------
    void SetSimTime(Time t) { m_simTime = t; }
    void SetOutputDir(std::string d) { m_outputDir = std::move(d); }
    void SetRunTag(std::string t) { m_runTag = std::move(t); }
    void SetCarrierFrequencyHz(double f) { m_freqHz = f; }
    void SetBandwidthHz(double b) { m_bwHz = b; }
    double GetBandwidthHz() const { return m_bwHz; }
    double GetCarrierFrequencyHz() const { return m_freqHz; }
    void SetSatEirpDbm(double p) { m_satEirpDbm = p; }   ///< gNB (satellite) Tx power / EIRP
    void SetUeTxPowerDbm(double p) { m_ueTxDbm = p; }
    void SetBackhaulDelay(Time t) { m_backhaulDelay = t; } ///< feeder+core one-way delay
    void SetPayloadOption(PayloadOption p) { m_payload = p; }
    PayloadOption GetPayloadOption() const { return m_payload; }
    /// One-way user-plane extra delay of the current payload option at the
    /// given feeder slant range (see PayloadOption docs).
    Time ComputePayloadExtraDelay(double slantRangeM) const;
    /**
     * \brief Drive the EPC backhaul delay LIVE from the real feeder geometry
     *        (satellite and gateway mobility models) per the selected payload
     *        option, re-evaluated every second. Call after Build().
     */
    void SetFeederGeometry(Ptr<MobilityModel> satMobility, Ptr<MobilityModel> gwMobility);
    void SetHarqEnabled(bool h) { m_harq = h; }
    /**
     * \brief Optional NTN-stretched HARQ profile (call before Build()).
     *
     * Default (and \p enable = false) keeps today's behavior: HARQ off, because
     * mmwave's terrestrial HARQ defaults (HarqDlTimeout = 20 slots,
     * NumHarqProcess = 20) assume a feedback round trip of a few slots and
     * break over a LEO slant. When enabled, HARQ is turned ON and the two
     * knobs the in-tree mmwave module actually exposes —
     * ns3::MmWavePhyMacCommon::HarqDlTimeout and
     * ns3::MmWavePhyMacCommon::NumHarqProcess — are stretched to
     * NTN-compatible values derived from the slant geometry of the nodes
     * passed to Build() (see ConfigureNtnHarqProfile() for the math; budget:
     * LEO-600 one-way ~2.2 ms at zenith, 4 HARQ rounds).
     *
     * Residual limitation: mmwave exposes no UE-side HARQ feedback-timing or
     * max-retransmission attribute (feedback rides the in-band control path
     * with a fixed L1L2 latency, and the retx count is bounded only by the
     * process timeout), and no Rel-17 K_offset scheduling-offset knob — so
     * this profile prevents premature HARQ-process recycling over the slant
     * but cannot reproduce the full TS 38.331 NTN timing relationships.
     */
    void SetNtnHarqProfile(bool enable);
    void SetRlcAmEnabled(bool a) { m_rlcAm = a; }
    void SetUplink(bool u) { m_uplink = u; }
    void SetGates(HealthGates g) { m_gates = g; }
    void SetStrictGates(bool s) { m_strictGates = s; }

    // ---- Build the real radio stack -------------------------------------
    /**
     * \brief Wire mmwave gNB devices on \p gnbNodes and UE devices on
     *        \p ueNodes, attach UEs to the closest gNB, stand up the EPC +
     *        remote host, and connect the measured-KPI PHY sink. Both node
     *        sets MUST already carry a MobilityModel.
     */
    void Build(NodeContainer gnbNodes, NodeContainer ueNodes);

    /// Install downlink (and optionally uplink) traffic over the radio link.
    void InstallTraffic(TrafficProfile profile, Time start, Time stop);

    /**
     * \brief Install one explicit NtnOranApplication QoS flow (DL: remote host
     *        -> UE \p ueIdx) with full slice/QoS identity. \p profile is an
     *        NtnOranApplication::Profile value. Returns {client, sink}.
     */
    ApplicationContainer InstallOranFlow(uint32_t ueIdx,
                                         uint8_t fiveQi,
                                         uint8_t sst,
                                         uint32_t sd,
                                         uint8_t profile,
                                         Time start,
                                         Time stop);

    /**
     * \brief Chain an extra propagation loss model onto the real radio channel
     *        (after the built-in Friis loss). This is the channel-plugin hook:
     *        a module re-homes its physics (THz molecular absorption, Sionna RT,
     *        A2G TR 38.811, ...) as a real PropagationLossModel so it actually
     *        attenuates packets and shows up in the MEASURED SINR. Call after
     *        Build().
     */
    void AddExtraPropagationLoss(Ptr<PropagationLossModel> loss);

    /// Schedule a user callback on the real event queue (e.g. CHO/KPM tick).
    void RegisterPeriodicCallback(Time period, std::function<void(Time)> cb);

    /**
     * \brief Stand up the WS2 AI-native measurement layer over every ORAN
     *        flow installed so far (call AFTER InstallTraffic/InstallOranFlow):
     *        per-flow KPM time series under TS 28.552 names, AI feature
     *        windows, EWMA anomaly events, XML/CSV/Influx/E2 export. The
     *        monitor also reads this helper's PHY trace for L1M.RS-SINR.
     */
    Ptr<NtnOranAiFlowMonitor> EnableOranFlowMonitor();

    /**
     * \brief Canonical KPM wiring (audit 2026-06-12 §4.2): stand up ONE
     *        NtnOranAiFlowMonitor over this helper's flows and auto-export
     *        its KPM series at end of simulation.
     *
     * May be called any time after Build() — before or after
     * InstallTraffic()/InstallOranFlow(). Every NtnOranApplication/NtnOranSink
     * the helper has already installed is attached immediately, and any flow
     * installed later is attached automatically. The monitor reads this
     * helper's PHY trace for L1M.RS-SINR, and at Simulator::Destroy() writes
     * `<outputPrefix>_kpm_series.csv` and `<outputPrefix>_kpm_series.lp`
     * (\p outputPrefix is used verbatim as a path prefix; parent directories
     * are created if needed).
     */
    void EnableAiFlowMonitor(const std::string& outputPrefix);
    /// The monitor created by EnableAiFlowMonitor()/EnableOranFlowMonitor(),
    /// or nullptr if neither has been called yet. (Defined out-of-line so
    /// callers need not pull in the monitor header.)
    Ptr<NtnOranAiFlowMonitor> GetAiFlowMonitor() const;

    // ---- Post-run measurement (call after Simulator::Run) ----------------
    /// Aggregate FlowMonitor + PHY-sink samples into the measured KPI set.
    void Collect();

    void WriteHealthReport();

    // ---- Measured KPI accessors (for module logic + reporting) -----------
    double GetMeanDlSinrDb() const { return m_dlSinrDbMean; }
    double GetMeanDlTbler() const { return m_dlTblerMean; }
    double GetDlCorruptFraction() const;
    uint64_t GetPhyRxTb() const { return m_phyRxTb; }
    double GetRxThroughputMbps() const { return m_rxThroughputMbps; }
    /// Measured mean one-way delay (ms) from in-band NtnOranPayloadHeader
    /// timestamps across all DL sinks (radio + GTP + backhaul, real path).
    double GetMeanDelayMs() const { return m_meanDelayMs; }
    /// Measured RFC 3550 jitter (ms) across all DL flows.
    double GetMeanJitterMs() const { return m_meanJitterMs; }
    /// Measured app-layer loss ratio (seq gaps) across all DL flows.
    double GetAppLossRatio() const { return m_appLossRatio; }
    /// Measured mean DL SINR (dB) for a given cellId, or NaN if no samples.
    double GetCellMeanSinrDb(uint16_t cellId) const;

    // ---- Per-UE measured state (for CHO / RIC / slice logic) --------------
    /// Current RNTI assigned to the UE at index \p ueIndex (0 if not attached).
    uint16_t GetUeRnti(uint32_t ueIndex) const;
    /// Most recent measured DL SINR (dB) for the UE, or NaN if no samples yet.
    double GetUeRecentSinrDb(uint32_t ueIndex) const;
    /// Run-mean measured DL SINR (dB) for the UE, or NaN if no samples.
    double GetUeMeanSinrDb(uint32_t ueIndex) const;
    /// Most recent measured DL TBLER for the UE (from the real error model),
    /// or NaN if no samples yet. Used by FAPI to drive a measured CRC outcome.
    double GetUeRecentTbler(uint32_t ueIndex) const;
    /// Total measured DL bytes delivered to the UE app (PacketSink).
    uint64_t GetUeRxBytes(uint32_t ueIndex) const;
    /// Number of UEs.
    uint32_t GetNumUes() const { return m_ue.GetN(); }

    // ---- Handles for module-specific wiring ------------------------------
    // (defined out-of-line so callers need not pull in the mmwave headers)
    Ptr<mmwave::MmWaveHelper> GetMmWaveHelper() const;
    Ptr<mmwave::MmWavePointToPointEpcHelper> GetEpcHelper() const;
    NetDeviceContainer GetUeDevices() const { return m_ueDevs; }
    NetDeviceContainer GetEnbDevices() const { return m_enbDevs; }
    Ptr<Node> GetRemoteHost() const { return m_remoteHost; }

  private:
    // Create the ORAN AI flow monitor (idempotent) and wire the PHY source.
    void EnsureOranMonitor();
    // Attach every helper-installed source/sink not yet attached to the monitor.
    void AttachInstalledFlowsToMonitor();
    // End-of-sim KPM export registered by EnableAiFlowMonitor().
    void ExportAiFlowMonitor();
    // Stretch mmwave HARQ knobs to the slant geometry (NTN HARQ profile).
    void ConfigureNtnHarqProfile();
    // PHY measured-KPI sink (connected to RxPacketTraceUe).
    void DlRxTrace(mmwave::RxPacketTraceParams params);
    // App-layer measured counters (connected to OnOff "Tx" / PacketSink "Rx").
    void DlClientTx(Ptr<const Packet> p);
    void DlSinkRx(Ptr<const Packet> p, const Address& from);
    void RunPeriodic(uint32_t idx);

    struct SinrAccum
    {
        double sumSinrDb = 0.0;
        double sumTbler = 0.0;
        uint64_t n = 0;
        uint64_t corrupt = 0;
    };

    // Config
    Time m_simTime{Seconds(30.0)};
    std::string m_outputDir{"."};
    std::string m_runTag{"run"};
    double m_freqHz{2.0e9};       // S-band (3GPP NR-NTN FR1)
    double m_bwHz{50.0e6};
    double m_satEirpDbm{55.0};    // LEO beam EIRP (Friis budget -> ~15-20 dB SINR)
    double m_ueTxDbm{33.0};
    Time m_backhaulDelay{MilliSeconds(5)};
    PayloadOption m_payload{PayloadOption::FullGnb};
    Ptr<MobilityModel> m_feederSat;
    Ptr<MobilityModel> m_feederGw;
    bool m_harq{false};
    bool m_ntnHarqProfile{false};
    bool m_rlcAm{false};
    bool m_uplink{false};
    HealthGates m_gates{};
    bool m_strictGates{false};

    // ns-3 objects
    Ptr<mmwave::MmWaveHelper> m_mmwave;
    Ptr<mmwave::MmWavePointToPointEpcHelper> m_epc;
    NodeContainer m_gnb;
    NodeContainer m_ue;
    NetDeviceContainer m_enbDevs;
    NetDeviceContainer m_ueDevs;
    Ptr<Node> m_remoteHost;
    Ptr<Object> m_backhaulCh; // PointToPointChannel of the PGW<->remote link
    Ipv4Address m_remoteHostAddr;
    std::vector<Ipv4Address> m_ueAddrs; // assigned UE IP per UE device
    ApplicationContainer m_clientApps;
    ApplicationContainer m_serverApps;
    ApplicationContainer m_dlSinks; // DL sinks on the UEs (authoritative rx)
    std::vector<uint32_t> m_dlSinkUe; // UE index of each DL sink, in order

    // Measured-KPI sink state
    SinrAccum m_dlGlobal;
    std::map<uint16_t, SinrAccum> m_dlPerCell;
    std::map<uint16_t, SinrAccum> m_dlPerRnti; // keyed by UE RNTI
    std::map<uint16_t, double> m_lastSinrDbPerRnti;
    std::map<uint16_t, double> m_lastTblerPerRnti;

    // Collected results
    double m_dlSinrDbMean{0.0};
    double m_dlTblerMean{0.0};
    uint64_t m_phyRxTb{0};
    uint64_t m_phyCorruptTb{0};
    double m_rxThroughputMbps{0.0};
    double m_meanDelayMs{0.0};
    double m_meanJitterMs{0.0};
    double m_appLossRatio{0.0};
    uint16_t m_nextDlPort{1234};
    uint16_t m_flowSeq{0}; ///< monotonic ORAN srcId allocator (never a recycled port)
    uint64_t m_appTxPackets{0};
    uint64_t m_appRxPackets{0};

    struct PeriodicEntry
    {
        Time period;
        std::function<void(Time)> cb;
    };
    std::vector<PeriodicEntry> m_periodics;
    Ptr<NtnOranAiFlowMonitor> m_oranMonitor;
    bool m_autoAttachMonitor{false};      ///< EnableAiFlowMonitor: attach later flows too
    std::string m_aiMonitorPrefix;        ///< KPM export path prefix
    bool m_aiExportScheduled{false};      ///< end-of-sim export registered once
    uint32_t m_monAttachedClients{0};     ///< m_clientApps already attached to monitor
    uint32_t m_monAttachedSinks{0};       ///< m_dlSinks already attached to monitor

    bool m_built{false};
    int64_t m_wallStartNs{0};
};

} // namespace ns3

#endif // NTN_REAL_STACK_HELPER_H
