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
// at LEO range, a mmWave-NR PHY at an S-band carrier (FR2 numerology, 60 kHz
// SCS — NOT a 3GPP NR-NTN FR1 band/numerology: 2.0 GHz has no assigned 3GPP
// band number, it sits in the n256 uplink, and the 50 MHz default BW exceeds
// the 20/30 MHz NTN-FR1 max). The "satellite EIRP" is a conducted Tx-power
// scalar on a terrestrial 8x8 UniformPlanarArray gNB with SVD beamforming
// (array gain added separately, not a reflector beam / 3 dB footprint).
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
class SpectrumPropagationLossModel;
class SpectrumChannel;
class NtnOranAiFlowMonitor;
// nr (5G-LENA) backend types — see SetRadioBackend / RadioBackend::Nr.
class NrHelper;
class NrPointToPointEpcHelper;
class NrHandoverAlgorithm;
class IdealBeamformingHelper;
struct RxPacketTraceParams; // ns3::RxPacketTraceParams (nr); != mmwave::RxPacketTraceParams

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

    /// Which in-tree NR PHY/MAC stack carries the air interface.
    ///   Mmwave (DEFAULT, zero-regression): the vendored NYU `mmwave` module,
    ///           FR2 numerology (60 kHz SCS). This is what all existing examples
    ///           get unless they opt in to Nr.
    ///   Nr:     5G-LENA `nr`, FR1 numerology (15/30 kHz SCS via SetNumerology),
    ///           the S-band NTN-FR1 regime mmwave cannot reach (closes A5(i)).
    /// Both backends feed the SAME measured-KPI accumulators (per-UE/cell SINR,
    /// TBLER, throughput, health gates, ORAN/AI flow monitor), so all downstream
    /// logic is backend-agnostic.
    enum class RadioBackend : uint8_t
    {
        Mmwave,
        Nr,
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
        /// The error model must actually RUN. Checked by probing for a
        /// strictly-positive TBLER sample: a live model always reports a finite
        /// value (~1e-8 even on a pristine link), a disabled one writes exactly
        /// 0.0 forever. (Before gap S9 this was the same predicate as
        /// requireSinrProvenance and could not detect a disabled error model.)
        bool requireErrorModelActive = true;
        /// Measured app one-way delay must respect the speed of light: at least
        /// satellite-altitude/c + backhaul. Catches a zero-delay air interface
        /// (gap S2). Disable only for non-satellite topologies.
        bool requireOwdFloor = true;
    };

    NtnRealStackHelper();
    ~NtnRealStackHelper();

    // ---- Configuration (call before Build) ------------------------------
    void SetSimTime(Time t) { m_simTime = t; }
    void SetOutputDir(std::string d) { m_outputDir = std::move(d); }
    void SetRunTag(std::string t) { m_runTag = std::move(t); }
    /// Select the radio backend (default Mmwave). Call before Build().
    void SetRadioBackend(RadioBackend b) { m_backend = b; }

    /// S2: request a REAL ConstantSpeedPropagationDelayModel on the radio
    /// channel (nr backend). DEFAULT OFF — see m_airIfaceDelayRequested: the
    /// vendored nr v3.3 lacks NTN K_offset/TA and will abort with 'Cannot TX
    /// while RX' on any uplink traffic, or assert on multi-UE UL alignment.
    /// When off (the default) the service-link slant is carried on the backhaul
    /// so the measured end-to-end OWD is still physically correct.
    void SetAirInterfaceDelay(bool enable) { m_airIfaceDelayRequested = enable; }
    RadioBackend GetRadioBackend() const { return m_backend; }
    /// FR1 numerology for the Nr backend only: 0 = 15 kHz, 1 = 30 kHz (default).
    /// Ignored by the Mmwave backend. Call before Build().
    void SetNumerology(uint16_t n) { m_numerology = n; }
    uint16_t GetNumerology() const { return m_numerology; }
    void SetCarrierFrequencyHz(double f) { m_freqHz = f; }
    void SetBandwidthHz(double b) { m_bwHz = b; }
    double GetBandwidthHz() const { return m_bwHz; }
    double GetCarrierFrequencyHz() const { return m_freqHz; }
    /// gNB (satellite) CONDUCTED Tx power in dBm.
    ///
    /// WARNING (gap S7): this value is written verbatim into the PHY's TxPower,
    /// i.e. it is the power at the array input. The UPA array gain
    /// (10*log10(rows*cols), ~18 dB for the default 8x8) and any beamforming
    /// gain are added ON TOP by the antenna model, so the radiated EIRP is
    /// HIGHER than what you pass here. Passing a TR 38.821 Set-1 EIRP figure
    /// (which already includes the 30 dBi satellite antenna) therefore
    /// double-counts the antenna by ~20 dB. Prefer SetSatEirpTotalDbm() or
    /// SetSatEirpDensityDbwMhz(), which back-compute the conducted power.
    void SetSatEirpDbm(double p) { m_satEirpDbm = p; }

    /// Set the intended TOTAL radiated EIRP in dBm (TR 38.821-style, antenna
    /// gain INCLUDED). The helper back-computes the conducted TxPower by
    /// subtracting the array gain, so the effective radiated EIRP matches \p
    /// eirpDbm. Must be called after SetMimo()/antenna config (it reads the
    /// array size) and before Build().
    void SetSatEirpTotalDbm(double eirpDbm)
    {
        m_satEirpDbm = eirpDbm - ArrayGainDb();
        m_eirpTotalDbm = eirpDbm;
    }

    /// TR 38.821 Set-1 style EIRP DENSITY (dBW/MHz). Converts to a total EIRP
    /// over the configured bandwidth then back-computes conducted power:
    ///   EIRP_dBm = density_dBW/MHz + 10log10(BW_MHz) + 30
    /// Set the bandwidth (SetBandwidthHz) before calling.
    void SetSatEirpDensityDbwMhz(double densityDbwPerMhz)
    {
        const double bwMhz = m_bwHz / 1e6;
        SetSatEirpTotalDbm(densityDbwPerMhz + 10.0 * std::log10(std::max(bwMhz, 1e-9)) + 30.0);
    }

    /// UPA array gain (dB) implied by the configured gNB antenna panel.
    double ArrayGainDb() const
    {
        const double n = static_cast<double>(m_gnbRows) * static_cast<double>(m_gnbCols);
        return 10.0 * std::log10(std::max(n, 1.0));
    }

    /// Effective radiated EIRP (dBm) the current config will actually produce:
    /// conducted TxPower + array gain, minus the per-BWP power split.
    double GetEffectiveEirpDbm() const;

    /// S8: one-way inter-gNB (X2/Xn) delay derived from the live inter-satellite
    /// geometry — a direct ISL hop for a regenerative payload, or the double
    /// feeder loop for a transparent one. Used to configure X2LinkDelay so
    /// handover preparation is not instantaneous between orbiting gNBs.
    Time ComputeX2LinkDelay() const;

    /// S2: one-way UE<->satellite (service link) propagation delay from the
    /// live geometry. On the mmwave backend (zero-delay air interface) this is
    /// folded into the backhaul so the user-plane OWD is physically right; the
    /// nr backend carries it on the air interface instead.
    Time ComputeServiceLinkDelay() const;

    /// S9 / gate 1: theoretical minimum app one-way delay (ms) for this
    /// topology = satellite altitude / c (a UE directly under the sub-satellite
    /// point — no geometry can beat it) + the configured backhaul. Returns 0 if
    /// the geometry is unavailable (gate then skipped).
    double ComputeOwdFloorMs() const;

    /// S9: true when the UE PHY's DataErrorModelEnabled attribute is set, i.e.
    /// the error model really runs. Attribute check, not a TBLER value probe:
    /// both backends report TBLER 0 when the model is OFF, and a pristine link
    /// reports 0 as well, so values cannot distinguish the two.
    bool IsErrorModelEnabled() const;
    /// Configured value written verbatim into MmWaveEnbPhy::TxPower. NOTE
    /// (gap G16): mmwave adds the antenna-array gain SEPARATELY in the spectrum
    /// model, so the effective radiated EIRP = this value + array gain; treat
    /// this as the conducted Tx power budget, not the final EIRP, when comparing
    /// against a TR 38.821 EIRP-density link budget.
    double GetSatEirpDbm() const { return m_satEirpDbm; }

    /// Enable/disable the TR 38.811 large-scale EXCESS-loss terms (atmospheric
    /// gas P.676 + scintillation P.618 + clutter + elevation-dependent shadow
    /// fading) on the MEASURED radio channel, chained after Friis (gap G1).
    /// Default ON. Call before Build().
    void SetTr38811ExcessLoss(bool e) { m_tr38811 = e; }
    /// TR 38.811 scenario for the excess-loss model: 0=DenseUrban, 1=Urban,
    /// 2=Suburban (default), 3=Rural. Call before Build().
    void SetNtnScenario(uint8_t s) { m_ntnScenario = s; }
    /// Enable the TR 38.811 §6.4.1 satellite beam pattern (off-boresight
    /// roll-off only; the radio array supplies the peak gain) — gap A5(ii).
    /// \p beamwidthDeg = 3 dB beamwidth (default 4.4127, TR 38.821 Set-1 LEO-600
    /// S-band). \p beamCenter = the fixed cell beam-centre mobility; if null the
    /// beam tracks each UE (roll-off 0). Call before Build().
    void SetSatelliteBeam(double beamwidthDeg = 4.4127,
                          Ptr<MobilityModel> beamCenter = nullptr);
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

    // =====================================================================
    // NR deep-integration infrastructure (2026-07). All OFF by default, so
    // the mmwave backend and the existing nr examples are unaffected unless
    // a setter below is called. Every knob is nr-backend only.
    // =====================================================================

    // ---- Enabler D: native NR PHY/MAC/RLC/PDCP stats --------------------
    /// Turn on 5G-LENA's native stat calculators (NrHelper::EnableTraces):
    /// per-DRB PDCP/RLC throughput+delay, per-slot MAC MCS/PRB, and the PHY
    /// RxPacketTrace, written as text files under the output dir. In addition
    /// the helper always captures measured MCS / MIMO rank / PRB usage from the
    /// NR RxPacketTrace into GetMeanDlMcs()/GetMeanDlRank()/GetMeanPrbUtil()
    /// (nr backend only). Call before Build().
    void SetNrNativeTraces(bool e) { m_nrNativeTraces = e; }
    /// Measured mean DL MCS index from the NR error model (NaN if no samples).
    double GetMeanDlMcs() const;
    /// Measured mean DL MIMO rank (streams) from the NR PHY (NaN if none).
    double GetMeanDlRank() const;
    /// Measured mean DL PRB-utilisation fraction (assigned RBs / band RBs).
    double GetMeanPrbUtil() const;
    /// Per-cell measured mean DL MCS (NaN if no samples for that cell).
    double GetCellMeanMcs(uint16_t cellId) const;

    // ---- Enabler C: scheduler selection + per-slice BWP isolation --------
    /// NR MAC scheduler. Default TdmaRR reproduces the historical behaviour.
    /// OfdmaQos is the only scheduler that differentiates 5QI/QCI priorities.
    enum class Scheduler : uint8_t
    {
        TdmaRR,   ///< TDMA round-robin (NR default, historical)
        OfdmaRR,  ///< OFDMA round-robin
        OfdmaPF,  ///< OFDMA proportional-fair
        OfdmaQos, ///< OFDMA QoS-aware (differentiates 5QI)
    };
    /// Select the NR MAC scheduler (nr backend only). Call before Build().
    void SetScheduler(Scheduler s) { m_scheduler = s; }
    Scheduler GetScheduler() const { return m_scheduler; }

    /// One network slice = a dedicated NR bandwidth part carrying one 5QI.
    struct SliceSpec
    {
        std::string label; ///< human name (eMBB / URLLC / mMTC)
        uint8_t fiveQi;    ///< 5QI carried by this slice (1,2,9,82,...)
    };
    /// Configure per-slice BWP isolation (nr backend only). Passing N>=2 slices:
    ///  (1) splits the NR band into N equal contiguous BWPs (one per slice);
    ///  (2) auto-selects the OfdmaQos scheduler unless SetScheduler() overrode it;
    ///  (3) routes each slice's 5QI to its BWP via the gNB BWP manager;
    ///  (4) activates a dedicated per-5QI EPS bearer for every matching flow.
    /// Slice isolation then EMERGES from the real MAC under contention instead
    /// of being asserted. Call before Build().
    void SetSlices(std::vector<SliceSpec> slices) { m_slices = std::move(slices); }
    /// Per-BWP (per-slice) measured mean DL SINR (dB), NaN if no samples.
    double GetBwpMeanSinrDb(uint8_t bwpId) const;
    /// Per-BWP (per-slice) transport blocks decoded at the UE PHY.
    uint64_t GetBwpRxTb(uint8_t bwpId) const;

    // ---- Enabler A: multi-gNB inter-satellite handover ------------------
    /// Enable real NR inter-cell handover across the gNBs passed to Build()
    /// (nr backend, >=2 gNBs). Installs the A3-RSRP handover algorithm + X2
    /// interfaces so a UE re-selects a real neighbour cell on measured RSRP,
    /// replacing free-space-scaled candidate SINR. Call before Build().
    void SetHandover(bool enable, double hysteresisDb = 3.0, Time ttt = MilliSeconds(256));
    /// Number of successfully completed NR handovers observed this run.
    uint32_t GetHandoverCount() const { return m_hoCount; }

    // ---- Enabler B: spectrum-level channel plugin + real MIMO ------------
    /// Install a caller-supplied SpectrumPropagationLossModel onto the NR BWP
    /// spectrum channel(s). Unlike AddExtraPropagationLoss() (a scalar dB
    /// offset applied flat across the band), this is the FREQUENCY-SELECTIVE /
    /// spatial seam: a module supplies a per-RB (and, with MIMO, per-antenna)
    /// transfer function — THz per-line molecular absorption, a Sionna
    /// ray-traced CIR, a RIS response — that drives NR AMC/BLER/rank per RB.
    /// Call after Build() (the BWP channel exists once the band is initialised).
    void AddSpectrumChannelLoss(Ptr<SpectrumPropagationLossModel> loss);
    /// Enable real NR spatial multiplexing: size the gNB/UE UniformPlanarArray
    /// and turn on NrPmSearchFull rank/PMI adaptation (nr backend only), so
    /// UM-MIMO capacity and ray-traced rank become measured PHY quantities
    /// instead of a scalar array-gain offset. Call before Build().
    void SetMimo(uint8_t gnbRows,
                 uint8_t gnbCols,
                 uint8_t ueRows,
                 uint8_t ueCols,
                 uint8_t rankLimit = 2);

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
    /// NOTE: an RNTI is only unique WITHIN a cell — pair it with
    /// GetUeServingCellId() before using it as a per-UE key.
    uint16_t GetUeRnti(uint32_t ueIndex) const;
    /// Cell id currently serving the UE at \p ueIndex (0 if not attached). This
    /// tracks handovers, unlike GetServingCellId() which reports gNB[0].
    uint16_t GetUeServingCellId(uint32_t ueIndex) const;
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

    /// Radio-agnostic serving cell id of the first gNB device: casts to
    /// mmwave::MmWaveEnbNetDevice (Mmwave backend) or ns3::NrGnbNetDevice (Nr
    /// backend) and returns GetCellId() (1 if unavailable). Defined out-of-line.
    uint16_t GetServingCellId() const;

    // ---- Handles for module-specific wiring ------------------------------
    // (defined out-of-line so callers need not pull in the mmwave headers)
    Ptr<mmwave::MmWaveHelper> GetMmWaveHelper() const;
    Ptr<mmwave::MmWavePointToPointEpcHelper> GetEpcHelper() const;
    NetDeviceContainer GetUeDevices() const { return m_ueDevs; }
    /// gNB device container for the active backend (mmwave or nr gNB devices).
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
    // Relax RLC-AM / RRC timers to the slant RTT so terrestrial defaults do not
    // fire spuriously over the NTN propagation delay (gap G4). Self-gating:
    // only ever extends a timer upward, so it is a no-op at terrestrial range.
    void ConfigureNtnRlcRrcTimers();
    // Worst-case (max) gNB<->UE slant range over the built geometry, in metres;
    // returns 600 km if no usable geometry is present.
    double WorstCaseSlantM() const;
    // Backend-specific radio install (helper + EPC + remote host + devices +
    // attach + RxPacketTraceUe wiring). Build() dispatches to one of these.
    void BuildMmwaveRadio();
    void BuildNrRadio();
    // Shared post-radio NTN channel extras (TR 38.811 excess loss + sat beam),
    // chained onto whichever backend's base loss model.
    void ApplyNtnChannelExtras();
    // PHY measured-KPI sink (connected to RxPacketTraceUe). One per backend
    // because the trace struct type differs; both feed AccumulateDl().
    void DlRxTrace(mmwave::RxPacketTraceParams params);   // mmwave backend
    void DlRxTraceNr(RxPacketTraceParams params);          // nr backend (ns3::RxPacketTraceParams)
    // Single accumulation path shared by both backends, so per-UE/cell SINR,
    // TBLER, corrupt-fraction, health gates and the AI flow monitor are
    // identical regardless of which radio produced the sample.
    void AccumulateDl(double sinrLinear,
                      double tbler,
                      bool corrupt,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint32_t tbSize,
                      double mcs = -1.0,     // nr: measured MCS index (<0 = n/a)
                      double rank = -1.0,    // nr: measured MIMO rank (<0 = n/a)
                      double rbFrac = -1.0,  // nr: assigned RBs / band RBs (<0 = n/a)
                      uint8_t bwpId = 0);    // nr: BWP id (slice)
    // App-layer measured counters (connected to OnOff "Tx" / PacketSink "Rx").
    void DlClientTx(Ptr<const Packet> p);
    void DlSinkRx(Ptr<const Packet> p, const Address& from);
    void RunPeriodic(uint32_t idx);
    // Enabler A: NrGnbRrc "HandoverEndOk" trace sink (counts completed HOs).
    void NrHandoverEndOk(std::string ctx, uint64_t imsi, uint16_t cellId, uint16_t rnti);
    // Enabler D: stretch the NR HARQ process pool to the slant RTT so a
    // process is not recycled before its ACK returns (NR analogue of the
    // mmwave ConfigureNtnHarqProfile). No-op unless SetNtnHarqProfile(true).
    void ConfigureNtnHarqProfileNr();

    struct SinrAccum
    {
        double sumSinrDb = 0.0;
        double sumTbler = 0.0;
        uint64_t n = 0;
        uint64_t corrupt = 0;
        double sumMcs = 0.0;    // measured MCS index (nr RxPacketTrace m_mcs)
        double sumRank = 0.0;   // measured MIMO rank (nr m_rank)
        double sumRbFrac = 0.0; // measured PRB fraction (m_rbAssignedNum / bandRb)
    };

    // Config
    Time m_simTime{Seconds(30.0)};
    std::string m_outputDir{"."};
    std::string m_runTag{"run"};
    double m_freqHz{2.0e9};       // S-band carrier; mmWave-NR FR2 numerology (60 kHz SCS), not a 3GPP NR-NTN FR1 band/numerology
    double m_bwHz{50.0e6};        // default exceeds the 20/30 MHz NTN-FR1 max
    double m_satEirpDbm{55.0};    // gNB conducted Tx power (UPA array gain added separately), Friis budget -> ~15-20 dB SINR
    double m_eirpTotalDbm{std::numeric_limits<double>::quiet_NaN()}; // S7: intended total EIRP if set via SetSatEirpTotalDbm/Density
    double m_ueTxDbm{33.0};
    bool m_tr38811{true};         // chain TR 38.811 excess loss on the measured plane (G1)
    uint8_t m_ntnScenario{2};     // 0 DenseUrban,1 Urban,2 Suburban,3 Rural
    bool m_satBeam{false};        // chain the TR 38.811 §6.4.1 beam pattern (A5(ii))
    double m_beamwidthDeg{4.4127};
    Ptr<MobilityModel> m_beamCenter;
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
    RadioBackend m_backend{RadioBackend::Mmwave}; // default: zero-regression mmwave
    uint16_t m_numerology{1};                     // nr backend FR1 numerology (30 kHz)

    // NR deep-integration config (all default-off / historical)
    bool m_nrNativeTraces{false};                 // D: EnableTraces() native stat files
    Scheduler m_scheduler{Scheduler::TdmaRR};     // C: NR MAC scheduler
    std::vector<SliceSpec> m_slices;              // C: per-slice BWPs (empty = 1 BWP)
    bool m_handover{false};                       // A: NR inter-cell handover
    double m_hoHystDb{3.0};
    Time m_hoTtt{MilliSeconds(256)};
    bool m_mimo{false};                           // B: real NR MIMO
    uint8_t m_gnbRows{8}, m_gnbCols{8}, m_ueRows{1}, m_ueCols{2}, m_mimoRank{1};
    std::vector<Ptr<SpectrumChannel>> m_nrBwpChannels; // B: BWP spectrum channels (seam)
    uint32_t m_nrBandRb{1};                       // total RBs (for PRB-util fraction)

    // ns-3 objects (mmwave backend)
    Ptr<mmwave::MmWaveHelper> m_mmwave;
    Ptr<mmwave::MmWavePointToPointEpcHelper> m_epc;
    // ns-3 objects (nr backend)
    Ptr<NrHelper> m_nr;
    Ptr<NrPointToPointEpcHelper> m_nrEpc;
    Ptr<IdealBeamformingHelper> m_nrBeamforming;
    Ptr<PropagationLossModel> m_nrBaseLoss; // Friis head for AddExtraPropagationLoss chaining
    /// S5: per-BWP Friis heads. Extra-loss chains MUST attach to every BWP —
    /// chaining only head[0] left sliced runs (N BWPs) with no NTN physics on
    /// slices 1..N-1, biasing the per-slice SINR comparison.
    std::vector<Ptr<PropagationLossModel>> m_nrBaseLossPerBwp;
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
    /// S3: per-UE accumulators are keyed by (cellId,RNTI), NOT by bare RNTI.
    /// RNTIs are allocated per cell and restart at each gNB, so a bare-RNTI key
    /// silently blends UEs served by different satellites in EVERY multi-gNB run
    /// (2-sat handover scenarios, constellations) and corrupts exactly the
    /// accessors CHO / RIC / slice logic and the AI flow monitor consume.
    static inline uint32_t UeKey(uint16_t cellId, uint16_t rnti)
    {
        return (static_cast<uint32_t>(cellId) << 16) | static_cast<uint32_t>(rnti);
    }

    std::map<uint32_t, SinrAccum> m_dlPerRnti; // keyed by UeKey(cellId, rnti)
    std::map<uint8_t, SinrAccum> m_dlPerBwp;   // keyed by NR BWP id (per-slice)
    std::map<uint32_t, double> m_lastSinrDbPerRnti; // keyed by UeKey(cellId, rnti)
    std::map<uint32_t, double> m_lastTblerPerRnti;  // keyed by UeKey(cellId, rnti)
    bool m_sawNonZeroTbler{false}; // S9: proves the error model actually ran
    /// S2: true when a real propagation-delay model sits on the RADIO channel.
    /// False means the service-link slant is carried on the backhaul instead
    /// (vendored stacks without NTN Timing Advance cannot align multi-UE UL
    /// under a per-distance delay). Read by ComputePayloadExtraDelay to avoid
    /// double-counting the slant.
    bool m_airIfaceDelayActive{false};
    /// S2 / R1-R3: opt-in request for a REAL propagation delay on the radio
    /// channel. OFF by default: the vendored 5G-LENA v3.3 has no NTN K_offset
    /// or Timing Advance, so a real delay makes the UE transmit while still
    /// receiving ("Cannot TX while RX") and misaligns multi-UE UL control.
    /// Until P3.16 lands K_offset, the service-link slant rides the backhaul
    /// instead — the end-to-end delay is right, the air interface just does
    /// not feel it. Safe to enable only for single-UE downlink-only studies.
    bool m_airIfaceDelayRequested{false};
    uint32_t m_hoCount{0};                      // A: completed NR handovers
    std::vector<Ptr<NrHandoverAlgorithm>> m_hoAlgos; // A: per-gNB A3 algos (kept alive)

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
