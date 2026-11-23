// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-real-stack-helper.h"
#include <cstdlib>
#include "ns3/node-list.h"

#include "ns3/ntn-spectrum-seam-model.h"

#include "ns3/abort.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/fatal-error.h"
#include "ns3/global-value.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/epc-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-ue-net-device.h"
#include "ns3/ntn-dataset-writer.h"
#include "ns3/ntn-repro-manifest.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
// nr (5G-LENA) backend
#include "ns3/antenna-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/nr-module.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/spectrum-channel.h"
#include "ns3/spectrum-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/integer.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/nr-phy-rx-trace.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-mac-scheduler-ns3.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-gnb-mac.h"

#include <tuple>
#include "ns3/seq-ts-size-header.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-sat-beam-gain-model.h"
#include "ns3/ntn-tr38811-excess-loss-model.h"
#include "ns3/ntn-earth-blockage-loss-model.h"
#include "ns3/ntn-static-extra-loss-model.h"
#include "ns3/ntn-los-channel-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/nr-common.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nr-pdcp.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/integer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <tuple>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace ns3
{

using namespace mmwave;

NS_LOG_COMPONENT_DEFINE("NtnRealStackHelper");

// Wave 2 core: opt-in per-UE serving-cell statistics trace, settable from any
// example's command line (--NtnRealStackUeStatsTrace=<csv>) without code
// changes. Empty (default) writes nothing and schedules nothing.
static GlobalValue g_ntnRealStackUeStatsTrace(
    "NtnRealStackUeStatsTrace",
    "If non-empty, NtnRealStackHelper (nr backend) writes per-UE serving-cell "
    "statistics (serving cell, RNTI, DL TB counters, recent SINR/TBLER) every "
    "second to this CSV path (see EnableUeStatsTrace)",
    StringValue(""),
    MakeStringChecker());

// Wave 5: the per-UE / per-cell KPI dataset (ue_kpi.csv, cell_kpi.csv) is on by default for every
// nr real-stack run, so every module's dataset carries the RAN and geometry metrics.
static GlobalValue g_ntnRealStackNrHarqProfile(
    "NtnRealStackNrHarqProfile",
    "Size the nr gNB HARQ process pool to the satellite RTT (16 to 32 processes, TS 38.331 "
    "nrofHARQ-ProcessesForPDSCH-v1700) for every nr real-stack run; false keeps the 16-process "
    "terrestrial default unless an example calls SetNtnHarqProfile(true)",
    BooleanValue(true),
    MakeBooleanChecker());

// Wave 6A: the default nr link budget is a documented TR 38.821 parameter set.
static GlobalValue g_ntnRealStackLinkBudget(
    "NtnRealStackLinkBudget",
    "Default nr link budget when a scenario does not declare one: Tr38821Set1 (TR 38.821 Table "
    "6.1.1.1-1/-3 Set-1: S-band handheld 1T2R 0 dBi NF 7 dB 23 dBm with a 34 dBW/MHz payload, or "
    "Ka VSAT with a 4 dBW/MHz payload) or Legacy (55 dBm conducted at the array input, UE 33 dBm, "
    "nr NF 5 dB; not a TR 38.821 set)",
    StringValue("Tr38821Set1"),
    MakeStringChecker());

// Wave 8 (nr patch 16): uplink power control with a PHR-aware PUSCH PRB limit.
static GlobalValue g_ntnRealStackUlPhrPrbLimit(
    "NtnRealStackUlPhrPrbLimit",
    "nr backend: TS 38.213 7.1.1 PUSCH power control on every UE, TS 38.321 5.4.6 power headroom "
    "reports, and a scheduler that grants no more PUSCH PRBs than the reported headroom can "
    "power at the target SNR (nr patch 16). Off under NtnRealStackLinkBudget=Legacy; false turns "
    "it off everywhere unless a scenario calls SetUlPowerHeadroomPrbLimit(true)",
    BooleanValue(true),
    MakeBooleanChecker());

// Wave 9: paired FDD for the FDD NTN bands, and TS 38.321 BSR triggering (nr patch 17).
static GlobalValue g_ntnRealStackDuplex(
    "NtnRealStackDuplex",
    "nr backend duplex mode: Auto (paired FDD when the carrier is the downlink of the FDD NTN "
    "band n255 or n256 of TS 38.101-5 under the Tr38821Set1 budget, else the single shared-slot "
    "carrier), Fdd, or Tdd (the single carrier whose slots UL and DL share, as before Wave 9)",
    StringValue("Auto"),
    MakeStringChecker());

static GlobalValue g_ntnRealStackBsrTriggering(
    "NtnRealStackBsrTriggering",
    "nr backend: TS 38.321 5.4.5 BSR triggering (Regular, Periodic with periodicBSR-Timer 20 ms, "
    "Padding) on every UE instead of a SHORT_BSR in every UL transport block (nr patch 17). Off "
    "under NtnRealStackLinkBudget=Legacy; false turns it off everywhere unless a scenario calls "
    "SetBsrTriggering(true)",
    BooleanValue(true),
    MakeBooleanChecker());

// Wave 10: after an X2/CHO handover the target gNB serves the UE on a
// UE-directed beam (registers the UE device with the target's PHY) and looks the
// scheduled RNTI up only among the UEs it currently serves (nr patch 18).
static GlobalValue g_ntnRealStackHoTargetBeam(
    "NtnRealStackHoTargetBeam",
    "nr backend: after a handover the target gNB transmits the UE's PDSCH and receives its PUSCH "
    "on the UE-directed beam (as the source did) instead of its SSB cell beam or quasi-omni, and "
    "every gNB PHY matches the scheduled RNTI only against the UEs it serves (nr patch 18). Off "
    "under NtnRealStackLinkBudget=Legacy; false restores the Wave 9 behavior unless a scenario "
    "calls SetHandoverTargetBeam(true)",
    BooleanValue(true),
    MakeBooleanChecker());

// Wave 11b: robust MCS on the handover target until its first CSI (nr patch 20).
static GlobalValue g_ntnRealStackHoRobustMcs(
    "NtnRealStackHoRobustMcs",
    "nr backend: the handover target schedules a newly admitted UE at an MCS derived from the "
    "handover-time SS-RSRP the UE reported until its first DL CQI in that cell, and the UE's MAC "
    "reset drops the source cell's PDSCH still on the air and the CQI / HARQ-ACK reports pending "
    "for the source (nr patch 20). Off under NtnRealStackLinkBudget=Legacy; false restores the "
    "Wave 11 behavior unless a scenario calls SetHandoverRobustMcs(true)",
    BooleanValue(true),
    MakeBooleanChecker());

// Wave 11: frequency reuse among the satellite cells (TR 38.821 6.1.1.1).
static GlobalValue g_ntnRealStackFrequencyReuse(
    "NtnRealStackFrequencyReuse",
    "nr backend: frequency reuse factor among the gNBs (TR 38.821 clause 6.1.1.1, Table "
    "6.1.1.1-5): Auto (3 under the TR 38.821 Set-1 budget when the run has two or more gNBs, "
    "otherwise 1), 1, 3 or 4. With FRF k > 1 every carrier is split into k sub-bands and gNB i "
    "sends its PDSCH and control region (PSS/SSB) and schedules its UEs' PUSCH only in sub-band "
    "i mod k, with its power scaled to the sub-band (constant EIRP density). Legacy is always 1 "
    "unless a scenario calls SetFrequencyReuse",
    StringValue("Auto"),
    MakeStringChecker());

// Wave 11: SSB beam grid for Earth-moving cells (TS 38.213 4.1, TS 38.331 5.5.3.3).
static GlobalValue g_ntnRealStackSsbBeamGrid(
    "NtnRealStackSsbBeamGrid",
    "nr backend, Earth-moving cells (SetSsbCellBeam(true, nullptr)): every satellite sweeps its "
    "SSB over a hexagonal grid of body-fixed beams covering its field of view, one SSB index "
    "per beam, and the UE derives the cell quality from the best SSB beam (TS 38.331 5.5.3.3, nr "
    "patch 19) instead of one nadir beam whose SS-RSRP falls into the sidelobes once the UE "
    "leaves its footprint. Off under NtnRealStackLinkBudget=Legacy; false restores the single "
    "nadir beam unless a scenario calls SetSsbBeamGrid(true)",
    BooleanValue(true),
    MakeBooleanChecker());

static GlobalValue g_ntnRealStackFddUlOfdma(
    "NtnRealStackFddUlOfdma",
    "nr backend under paired FDD: the UL BWPs run nr's OFDMA round-robin scheduler when the "
    "scenario kept the default TDMA round-robin, so PRB-limited UEs share a symbol (FDM) instead "
    "of each taking whole symbols with most of the band idle; the DL BWPs keep the TDMA scheduler",
    BooleanValue(true),
    MakeBooleanChecker());

static GlobalValue g_ntnRealStackUeKpiPeriod(
    "NtnRealStackUeKpiPeriod",
    "Sampling period (s) of the per-UE and per-cell KPI dataset NtnRealStackHelper writes to "
    "<outputDir>/ue_kpi.csv and cell_kpi.csv (nr backend); 0 disables it",
    DoubleValue(1.0),
    MakeDoubleChecker<double>(0.0));

namespace
{
// Map a 5QI to the BwpManagerAlgorithmStatic attribute that routes that QCI to a
// BWP index (Enabler C). The NrEpsBearer::Qci enum value IS the 5QI number, and
// each Qci has a same-named "route to BWP" attribute on the static BWP manager.
// Returns "" for a 5QI without a dedicated routing attribute (falls back to BWP 0).
std::string
QciAttrName(uint8_t fiveQi)
{
    switch (fiveQi)
    {
    case 1:  return "GBR_CONV_VOICE";          // conversational voice
    case 2:  return "GBR_CONV_VIDEO";          // conversational / live video (eMBB)
    case 9:  return "NGBR_VIDEO_TCP_DEFAULT";  // default non-GBR (mMTC / background)
    case 79: return "NGBR_V2X";                // non-GBR V2X
    case 80: return "NGBR_LOW_LAT_EMBB";       // low-latency eMBB
    case 82: return "DGBR_DISCRETE_AUT_SMALL"; // delay-critical GBR (URLLC)
    default: return "";
    }
}
} // namespace

NtnRealStackHelper::NtnRealStackHelper() = default;
NtnRealStackHelper::~NtnRealStackHelper()
{
    m_ueStatsEvent.Cancel();
    m_ueKpiEvent.Cancel();
    // C3: restore the process-wide elevation convention this helper switched,
    // so later scenarios in the same process (test-runner) see the default.
    if (!m_prevElevationModel.empty())
    {
        GlobalValue::Bind("GeocentricElevationModel", StringValue(m_prevElevationModel));
    }
    if (m_prevRsrpLinear >= 0)
    {
        GlobalValue::Bind("NrUeRsrpLinearAverage", BooleanValue(m_prevRsrpLinear == 1));
    }
    if (m_prevRsrqFromRssi >= 0)
    {
        GlobalValue::Bind("NrUeRsrqFromRssi", BooleanValue(m_prevRsrqFromRssi == 1));
    }
}

void
NtnRealStackHelper::SetNtnHarqProfile(bool enable)
{
    NS_ABORT_MSG_IF(m_built, "SetNtnHarqProfile must be called before Build()");
    m_ntnHarqProfile = enable;
    if (enable)
    {
        m_harq = true; // the profile is meaningless with HARQ off
    }
    // enable == false keeps today's defaults exactly (HARQ off unless the
    // caller separately opted in via SetHarqEnabled(true)).
}

void
NtnRealStackHelper::SetSatelliteBeam(double beamwidthDeg, Ptr<MobilityModel> beamCenter)
{
    m_satBeam = true;
    m_beamwidthDeg = beamwidthDeg;
    m_beamCenter = beamCenter;
}

double
NtnRealStackHelper::WorstCaseSlantM() const
{
    double slantM = 0.0;
    for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
    {
        Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
        for (uint32_t u = 0; u < m_ue.GetN(); ++u)
        {
            Ptr<MobilityModel> um = m_ue.Get(u)->GetObject<MobilityModel>();
            if (gm && um)
            {
                slantM = std::max(slantM, gm->GetDistanceFrom(um));
            }
        }
    }
    if (slantM <= 0.0)
    {
        slantM = 600e3; // no usable geometry: assume LEO-600 zenith
    }
    return slantM;
}

void
NtnRealStackHelper::ConfigureNtnRlcRrcTimers()
{
    // G4: terrestrial RLC-AM / RRC timer defaults (t-PollRetransmit 20 ms,
    // t-Reordering 10 ms, t-StatusProhibit 10 ms, T300 100 ms) are shorter than
    // a LEO feedback RTT (8-26 ms) and would fire spuriously over the slant when
    // RLC AM is in use, triggering needless retransmissions / RLF. Relax each to
    // a margin over the slant RTT, but NEVER below the terrestrial default
    // (max()), so this is a strict no-op at terrestrial range and only extends
    // timers for genuine NTN geometry. Must run before InstallUeDevice() so the
    // Config defaults are read when the RLC/RRC objects are created.
    constexpr double kC = 299792458.0;
    const double rttMs = 2.0 * WorstCaseSlantM() / kC * 1e3;
    // t-PollRetransmit must exceed one feedback RTT + a poll/status budget.
    const double pollMs = std::max(20.0, 2.0 * rttMs + 10.0);
    // t-Reordering must cover the retransmission round trip.
    const double reorderMs = std::max(10.0, 2.0 * rttMs + 10.0);
    // t-StatusProhibit must exceed one RTT so status PDUs are not over-sent.
    const double statusMs = std::max(10.0, rttMs + 5.0);
    // RRC connection (T300) must survive the setup round trip over the slant.
    //
    // Wave 2 core: clamped at 2000 ms, the largest value TS 38.331
    // UE-TimersAndConstants t300 can signal (ENUMERATED ms100 ... ms2000; the
    // same set in TS 36.331 for the mmwave/LTE class). Unclamped, a GEO at 10
    // deg (RTT ~271 ms) asked for 2166 ms and NrUeRrc's attribute checker
    // aborted the run. RTTs up to 250 ms (every LEO/MEO case) are unchanged.
    const double t300Ms = std::min(2000.0, std::max(100.0, 8.0 * rttMs));
    // WF-13: write BOTH families, because the two backends use different
    // classes and this relaxation was dead on the default one.
    //
    // The four keys below were LTE-only: ns3::LteRlcAm and ns3::LteUeRrc. The
    // mmwave backend uses those classes, so the relaxation worked there. The nr
    // backend uses ns3::NrRlcAm and ns3::NrUeRrc, and BuildNrRadio never called
    // this function at all, so on the DEFAULT backend the terrestrial 20/10/10
    // ms timers stood over a LEO slant whose feedback RTT is 8-26 ms - exactly
    // the spurious-retransmission and RLF condition the relaxation exists to
    // prevent.
    //
    // Config::SetDefault aborts on an unknown path, so each key is set through
    // a guarded helper that skips a class this build does not have rather than
    // killing a scenario that never needed it.
    auto setIfPresent = [](const std::string& path, const TimeValue& v) {
        // A TypeId that is not registered means that stack is not in this
        // build; nothing to relax there.
        const std::string cls = path.substr(0, path.rfind("::"));
        TypeId tid;
        // LookupByNameFailSafe writes through its out-parameter, so it needs a
        // real TypeId; passing nullptr dereferences it.
        if (!TypeId::LookupByNameFailSafe(cls, &tid))
        {
            return;
        }
        Config::SetDefault(path, v);
    };
    setIfPresent("ns3::LteRlcAm::PollRetransmitTimer", TimeValue(MilliSeconds(pollMs)));
    setIfPresent("ns3::LteRlcAm::ReorderingTimer", TimeValue(MilliSeconds(reorderMs)));
    setIfPresent("ns3::LteRlcAm::StatusProhibitTimer", TimeValue(MilliSeconds(statusMs)));
    setIfPresent("ns3::LteUeRrc::T300", TimeValue(MilliSeconds(t300Ms)));
    setIfPresent("ns3::NrRlcAm::PollRetransmitTimer", TimeValue(MilliSeconds(pollMs)));
    setIfPresent("ns3::NrRlcAm::ReorderingTimer", TimeValue(MilliSeconds(reorderMs)));
    setIfPresent("ns3::NrRlcAm::StatusProhibitTimer", TimeValue(MilliSeconds(statusMs)));
    setIfPresent("ns3::NrUeRrc::T300", TimeValue(MilliSeconds(t300Ms)));
    NS_LOG_INFO("NTN RLC/RRC timers: rtt=" << rttMs << " ms -> poll=" << pollMs
                                           << " reorder=" << reorderMs << " status=" << statusMs
                                           << " T300=" << t300Ms << " ms");
}

void
NtnRealStackHelper::ConfigureNtnHarqProfile()
{
    // NTN-stretched HARQ (audit issue 13, Rel-17 K_offset domain). The in-tree
    // mmwave module exposes exactly two usable HARQ timing knobs:
    //   ns3::MmWavePhyMacCommon::HarqDlTimeout  (slots a HARQ process waits
    //       for feedback before the scheduler force-recycles it; default 20)
    //   ns3::MmWavePhyMacCommon::NumHarqProcess (stop-and-wait process pool
    //       per UE; default 20)
    // Both defaults assume a terrestrial feedback round trip of a few slots.
    //
    // Math (documented per the audit):
    //   * one-way propagation at LEO-600 zenith = 600 km / c ~= 2.0 ms
    //     (~2.2 ms including payload processing); RTT = 2 * slant / c, which
    //     grows to ~12.9 ms at 10 deg elevation (slant ~1932 km).
    //   * one HARQ round = RTT + ~1 ms gNB/UE processing budget
    //     (TbDecodeLatency 100 us + L1L2 latency + scheduling).
    //   * budget 4 HARQ rounds (initial TX + 3 retransmissions), so a process
    //     must survive 4 * (2 * slant / c + 1 ms) before being recycled.
    //   * the process pool must cover at least one feedback RTT worth of
    //     slots so the stop-and-wait pipe stays full while feedback is in
    //     flight: NumHarqProcess >= ceil(RTT / slot) + margin.
    //   * slot = 0.25 ms (mmwave default numerology 2: 14 symbols, 60 kHz
    //     SCS, 4 slots per 1 ms subframe).
    // Both attributes are uint8_t, so values clamp at 255 (covers slants up
    // to ~2200 km for the 4-round budget, i.e. a LEO-600 pass down to
    // ~8 deg elevation).
    constexpr double kC = 299792458.0;
    constexpr double kSlotS = 250e-6;   // numerology-2 slot
    constexpr double kProcS = 1e-3;     // per-round processing budget
    constexpr double kHarqRounds = 4.0; // initial TX + 3 retx

    const double slantM = WorstCaseSlantM();
    const double rttS = 2.0 * slantM / kC;
    const double roundS = rttS + kProcS;
    const double timeoutSlotsExact = kHarqRounds * roundS / kSlotS;
    const auto timeoutSlots = static_cast<uint64_t>(
        std::min(255.0, std::ceil(timeoutSlotsExact)));
    // Rel-17 NTN caps the DL HARQ process pool at n32
    // (nrofHARQ-ProcessesForPDSCH-v1700, TS 38.214 §5.1); beyond that the
    // standard switches to feedback-disabled HARQ (-> RLC ARQ), which the
    // vendored mmwave PHY cannot model. Clamp the pool at 32 (gap B6: was 255).
    const double numProcNeeded =
        std::max(20.0, std::ceil(rttS / kSlotS) + kHarqRounds);
    const auto numProc = static_cast<uint64_t>(std::min(32.0, numProcNeeded));
    if (timeoutSlotsExact > 255.0)
    {
        NS_LOG_WARN("NTN HARQ profile: slant " << slantM / 1e3
                                               << " km needs more than 255 slots for "
                                               << kHarqRounds
                                               << " HARQ rounds; clamping HarqDlTimeout to 255");
    }
    if (numProcNeeded > 32.0)
    {
        NS_LOG_WARN("NTN HARQ profile: slant "
                    << slantM / 1e3 << " km needs " << numProcNeeded
                    << " HARQ processes to keep the stop-and-wait pipe full, but "
                       "Rel-17 caps DL processes at n32; clamping to 32 (a real NTN "
                       "link would disable HARQ feedback -> RLC ARQ here)");
    }
    Config::SetDefault("ns3::MmWavePhyMacCommon::HarqDlTimeout", UintegerValue(timeoutSlots));
    Config::SetDefault("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue(numProc));
    NS_LOG_INFO("NTN HARQ profile: slant=" << slantM / 1e3 << " km rtt=" << rttS * 1e3
                                           << " ms -> HarqDlTimeout=" << timeoutSlots
                                           << " slots, NumHarqProcess=" << numProc);
}

void
NtnRealStackHelper::Build(NodeContainer gnbNodes, NodeContainer ueNodes)
{
    NS_ABORT_MSG_IF(m_built, "NtnRealStackHelper::Build() called twice");
    NS_ABORT_MSG_IF(gnbNodes.GetN() == 0 || ueNodes.GetN() == 0,
                    "NtnRealStackHelper: need >=1 gNB and >=1 UE");
    m_gnb = gnbNodes;
    m_ue = ueNodes;

    // Wave 6A: default link budget (TR 38.821 Set-1) for whatever the scenario
    // did not declare. Before ResolveEirpDensity so a default density is
    // converted against the configured bandwidth like a declared one.
    ApplyDefaultLinkBudget();

    // NT-02: a declared EIRP density is converted against the bandwidth that is
    // actually configured at Build time, so SetBandwidthHz() may be called
    // before or after SetSatEirpDensityDbwMhz().
    ResolveEirpDensity();

    // Wall-clock measurement starts here, NOT in InstallTraffic: scenarios
    // that install flows only via InstallOranFlow() previously left
    // m_wallStartNs at 0 and sim_health.csv reported the steady_clock epoch
    // (time since boot) as the wall time.
    m_wallStartNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();

    if (m_backend == RadioBackend::Nr)
    {
        BuildNrRadio();
    }
    else
    {
        BuildMmwaveRadio();
    }

    m_built = true;

    // ---- Shared NTN channel extras: chain the TR 38.811 large-scale EXCESS-loss
    //      terms (and optional sat beam) onto whichever backend's base loss, so
    //      the MEASURED SINR carries the elevation-dependent physics on both. ----
    ApplyNtnChannelExtras();

    // Globe plan L4: every helper-built run records its own scene (see StartAutoScene).
    Simulator::ScheduleNow(&NtnRealStackHelper::StartAutoScene, this);
    Simulator::ScheduleDestroy(&NtnRealStackHelper::StopAutoScene, this);
}

void
NtnRealStackHelper::StartAutoScene()
{
    if (std::getenv("NTN_SCENE_OFF") || ntnobs::NtnSceneRecorder::ActiveCount() > 0 || m_autoScene)
    {
        return; // the scenario records its own scene, or recording is switched off
    }
    m_autoScene = CreateObject<ntnobs::NtnSceneRecorder>();
    double lat0 = 0, lon0 = 0, alt0 = 0;
    const bool enu = ntnobs::ConfigureSceneFrame(m_autoScene, lat0, lon0, alt0); // ECEF, or ENU about the projection origin
    m_autoScene->SetSampleInterval(Seconds(1.0));
    m_autoScene->SetFrameBudget(150);
    // Only positions that can be placed on the Earth: in the ECEF frame a node must sit
    // between 6,200 and 50,000 km from the geocentre (a local Cartesian scenario without a
    // geographic anchor records no scene rather than one at the Earth's centre).
    auto placeable = [enu](Ptr<Node> n) {
        Ptr<MobilityModel> mm = n->GetObject<MobilityModel>();
        if (!mm)
        {
            return false;
        }
        const Vector p = mm->GetPosition();
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        return enu ? r < 5.0e7 : (r > 6.2e6 && r < 5.0e7);
    };
    uint32_t sats = 0;
    for (uint32_t i = 0; i < m_gnb.GetN(); ++i)
    {
        if (placeable(m_gnb.Get(i)))
        {
            m_autoScene->TrackNode(m_gnb.Get(i), ntnobs::NtnSceneRecorder::Sat, "sat-" + std::to_string(i));
            ++sats;
        }
    }
    if (sats == 0)
    {
        m_autoScene = nullptr;
        return;
    }
    for (uint32_t i = 0; i < m_ue.GetN(); ++i)
    {
        if (placeable(m_ue.Get(i)))
        {
            m_autoScene->TrackNode(m_ue.Get(i), ntnobs::NtnSceneRecorder::Ue, "ue-" + std::to_string(i));
        }
    }
    // Serving edges from the initial attachment: every UE starts on gNB 0's cell in the
    // single-cell scenarios; handovers move the edge (OnHandover in NrHandoverEndOk).
    if (m_gnb.GetN() > 0 && placeable(m_gnb.Get(0)))
    {
        for (uint32_t i = 0; i < m_ue.GetN(); ++i)
        {
            if (placeable(m_ue.Get(i)))
            {
                m_autoScene->TrackBeam(m_gnb.Get(0)->GetId(), m_ue.Get(i)->GetId());
                m_autoServing[m_ue.Get(i)->GetId()] = m_gnb.Get(0)->GetId();
            }
        }
    }
    std::error_code ec;
    std::filesystem::create_directories(m_outputDir, ec);
    m_autoScene->EnableCzml(m_outputDir + "/scene.czml");
    const char* live = std::getenv("NTN_LIVE_SCENE");
    if (live && std::string(live) == "1")
    {
        m_autoScene->EnableLiveStdout(true);
    }
    m_autoScene->Start();
}

void
NtnRealStackHelper::StopAutoScene()
{
    if (m_autoScene)
    {
        m_autoScene->Stop();
        m_autoScene = nullptr;
    }
}

uint32_t
NtnRealStackHelper::GnbNodeIdOfCell(uint16_t cellId) const
{
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> d = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
        if (d && d->GetCellId() == cellId)
        {
            return d->GetNode()->GetId();
        }
    }
    return 0;
}

void
NtnRealStackHelper::BuildMmwaveRadio()
{
    // ---- NTN-ize the NR air interface via Config defaults (read at CreateObject
    //      time inside MmWaveHelper::DoInitialize) -------------------------------
    // NB: mmwave registers its TypeIds WITHOUT the mmwave:: namespace.
    // NT-08, mmwave half. BuildNrRadio set the spatial-channel regeneration
    // period; this path never did, so an mmwave run inherited ns-3's own
    // ThreeGppChannelModel default of 0 and froze its channel matrix at t=0.
    //
    // Measured, the consequence here is small: 15.63 dB against 14.95 dB of DL
    // SINR on the smoke scenario, which is the opposite sign and a fortieth of
    // the size of the NR backend's 17.5 dB. That asymmetry is the interesting
    // part and it is not luck. The NR spine recomputes its beam from real
    // geometry every 100 ms, so a frozen channel leaves the beam pointing at a
    // satellite the clusters no longer describe; mmwave derives its beam from
    // the channel matrix itself, so freezing both kept the two halves mutually
    // consistent and the error largely cancelled. The period is set here so the
    // two backends are configured alike rather than because this one was badly
    // wrong. The TR 38.821 calibration, which defaults to this backend, moves
    // within its own tolerance and its gate still passes.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(m_channelUpdatePeriod));
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(m_freqHz));
    Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(m_bwHz));
    Config::SetDefault("ns3::MmWaveEnbPhy::TxPower", DoubleValue(m_satEirpDbm));
    Config::SetDefault("ns3::MmWaveUePhy::TxPower", DoubleValue(m_ueTxDbm));
    if (std::isfinite(m_ueNoiseFigureDb))
    {
        Config::SetDefault("ns3::MmWaveUePhy::NoiseFigure", DoubleValue(m_ueNoiseFigureDb));
    }
    Config::SetDefault("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue(m_rlcAm));
    Config::SetDefault("ns3::MmWaveHelper::HarqEnabled", BooleanValue(m_harq));
    Config::SetDefault("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(m_harq));
    if (m_ntnHarqProfile)
    {
        ConfigureNtnHarqProfile();
    }
    // G4: relax RLC-AM/RRC timers to the slant RTT (no-op at terrestrial range).
    // Must precede InstallUeDevice so the RLC/RRC objects read the new defaults.
    ConfigureNtnRlcRrcTimers();

    m_mmwave = CreateObject<MmWaveHelper>();
    m_mmwave->SetSchedulerType("ns3::MmWaveFlexTtiMacScheduler");
    m_mmwave->SetHarqEnabled(m_harq);
    // Ideal RRC: set up the RLC bearer synchronously at attach. Real RRC's
    // contention-based RACH/connection handshake assumes terrestrial timing and
    // races/never completes over the LEO slant — a well-known NTN pitfall.
    m_mmwave->SetAttribute("UseIdealRrc", BooleanValue(true));
    // NTN LOS link: free-space (Friis) path loss is valid at LEO range, unlike the
    // terrestrial 3GPP UMa default. The 3GPP spectrum model is kept (mmwave hard-
    // requires it for antenna/beamforming array gain), with an always-LOS condition
    // model — the satellite link is line-of-sight by construction. The TR 38.811
    // large-scale EXCESS-loss terms (atmospheric gas, scintillation, clutter,
    // elevation-dependent shadow fading) are chained on top of Friis at the end of
    // Build() via the Ntn38811ExcessLossModel (gap G1), so the MEASURED SINR is
    // elevation- and scenario-dependent rather than free-space-flat.
    m_mmwave->SetPathlossModelType("ns3::FriisPropagationLossModel");
    m_mmwave->SetChannelConditionModelType("ns3::AlwaysLosChannelConditionModel");
    // G2 / S2 (propagation delay) — KNOWN, DOCUMENTED CONSTRAINT on this
    // backend: the NTN one-way slant delay (LEO ~2-13 ms, GEO ~120 ms) is NOT
    // applied on the mmwave air interface. The vendored NYU mmwave MAC/PHY
    // assumes near-zero radio propagation and has no Rel-17 K_offset
    // scheduling-offset machinery, so a ms-scale delay model on the radio
    // SpectrumChannel de-syncs DCI/UCI slot timing. That is why HARQ is off by
    // default here and why SetNtnHarqProfile only stretches the two timing knobs
    // mmwave exposes. Use SetRadioBackend(Nr) for an air interface that really
    // experiences the slant (the nr BWP channels get a
    // ConstantSpeedPropagationDelayModel in BuildNrRadio).
    //
    // CORRECTION (2026-07 audit, gap S2): this comment previously claimed the
    // END-TO-END user-plane delay was nonetheless "correct" because the slant
    // rode the feeder leg. It was not — ComputePayloadExtraDelay added only the
    // FEEDER slant, so the 2-6.4 ms UE<->satellite SERVICE leg was absent from
    // every leg of the path and all mmwave OWD/jitter/URLLC figures were short
    // by it. ComputePayloadExtraDelay now folds the live service-link slant into
    // the backhaul on this backend, and the backhaul channel below is seeded
    // with it at Build() even when SetFeederGeometry() is not wired (only a
    // couple of examples wire it). The air interface still does not experience
    // the slant — only the user-plane latency is now honest.

    m_epc = CreateObject<MmWavePointToPointEpcHelper>();
    m_mmwave->SetEpcHelper(m_epc);

    Ptr<Node> pgw = m_epc->GetPgwNode();

    // ---- Remote host behind the core (carries LEO feeder+core latency) ----
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    m_remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_backhaulDelay));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, m_remoteHost);
    m_backhaulCh = internetDevices.Get(0)->GetChannel();
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    m_remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(m_remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ---- Radio devices on the caller's mobility-carrying nodes ----
    m_enbDevs = m_mmwave->InstallEnbDevice(m_gnb);
    m_ueDevs = m_mmwave->InstallUeDevice(m_ue);

    internet.Install(m_ue);
    Ipv4InterfaceContainer ueIpIface = m_epc->AssignUeIpv4Address(NetDeviceContainer(m_ueDevs));
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(m_ue.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(m_epc->GetUeDefaultGatewayAddress(), 1);
        m_ueAddrs.push_back(ueIpIface.GetAddress(u));
    }

    m_mmwave->AttachToClosestEnb(m_ueDevs, m_enbDevs);

    // ---- Measured-KPI PHY sink: DL SINR/TBLER/corrupt from the UE SpectrumPhy ----
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
        MakeCallback(&NtnRealStackHelper::DlRxTrace, this));

    // The Friis loss is the head of the chain on CC 0 (used by AddExtraPropagationLoss).
    m_nrBaseLoss = m_mmwave->GetPathLossModel(0);
    // mmwave runs a single carrier here, so the per-BWP head vector has one entry.
    m_nrBaseLossPerBwp.assign(1, m_nrBaseLoss);

    // S2: seed the backhaul with the service-link slant now that the devices and
    // their mobility exist. Without this, a scenario that never calls
    // SetFeederGeometry() (the vast majority) reports an app OWD that omits the
    // UE<->satellite leg entirely — physically impossible, and now caught by the
    // OWD-floor health gate.
    if (m_backhaulCh)
    {
        // NT-04: and keep it tracking the geometry for the rest of the run.
        Simulator::Schedule(m_backhaulRefresh, &NtnRealStackHelper::RefreshBackhaulFold, this);
        const Time service = ComputeServiceLinkDelay();
        if (service > Seconds(0))
        {
            m_backhaulCh->SetAttribute("Delay", TimeValue(m_backhaulDelay + service));
            NS_LOG_INFO("mmwave backend: folded service-link delay "
                        << service.GetMilliSeconds() << " ms into the backhaul leg (air "
                        << "interface remains zero-delay on this backend)");
        }
    }
}

double
NtnRealStackHelper::PairedUlCarrierHz(double dlHz)
{
    // TS 38.101-5 Table 5.2-1 FDD NTN FR1 bands and their default duplex
    // spacing: n256 UL 1980-2010 MHz / DL 2170-2200 MHz (UL = DL - 190 MHz),
    // n255 UL 1626.5-1660.5 MHz / DL 1525-1559 MHz (UL = DL + 101.5 MHz).
    if (dlHz >= 2170.0e6 && dlHz <= 2200.0e6)
    {
        return dlHz - 190.0e6;
    }
    if (dlHz >= 1525.0e6 && dlHz <= 1559.0e6)
    {
        return dlHz + 101.5e6;
    }
    return 0.0;
}

NtnRealStackHelper::NtnFr1Band
NtnRealStackHelper::ClassifyNtnFr1(double carrierHz, double bandwidthHz)
{
    NtnFr1Band b;
    if (carrierHz > 7.125e9)
    {
        return b; // above FR1 the NTN band tables do not apply
    }

    // TS 38.101-5 Table 5.2-1. The names were previously swapped here: the
    // 2170-2200 MHz block was labelled n255. It is n256. The frequencies and
    // block widths were right, so nothing physical changes with the rename -
    // but a provenance row naming the wrong band is a citation a reviewer can
    // check in one lookup.
    const bool n255Dl = (carrierHz >= 1525.0e6 && carrierHz <= 1559.0e6);
    const bool n255Ul = (carrierHz >= 1626.5e6 && carrierHz <= 1660.5e6);
    const bool n256Dl = (carrierHz >= 2170.0e6 && carrierHz <= 2200.0e6);
    const bool n256Ul = (carrierHz >= 1980.0e6 && carrierHz <= 2010.0e6);

    if (n255Dl || n255Ul)
    {
        b.name = "n255";
        b.carrierInDownlink = n255Dl;
        b.carrierInUplink = n255Ul;
        b.blockWidthHz = 34.0e6;
        // Table 5.3.5-1: n255 supports 5 and 10 MHz.
        b.bandwidthSupported = (std::abs(bandwidthHz - 5.0e6) < 1.0 ||
                                std::abs(bandwidthHz - 10.0e6) < 1.0);
    }
    else if (n256Dl || n256Ul)
    {
        b.name = "n256";
        b.carrierInDownlink = n256Dl;
        b.carrierInUplink = n256Ul;
        b.blockWidthHz = 30.0e6;
        // Table 5.3.5-1: n256 supports 5, 10, 15 and 20 MHz.
        b.bandwidthSupported = (std::abs(bandwidthHz - 5.0e6) < 1.0 ||
                                std::abs(bandwidthHz - 10.0e6) < 1.0 ||
                                std::abs(bandwidthHz - 15.0e6) < 1.0 ||
                                std::abs(bandwidthHz - 20.0e6) < 1.0);
    }

    // A downlink carrier must sit in a downlink block. The helper's own
    // historical default, 2.0 GHz, sits in n256's UPLINK block and was used as
    // the downlink carrier, so it can never be conformant however the channel
    // is sized.
    // TS 38.101-5 section 5.4: the whole channel, not only its center, must
    // lie in the band's block. A 20 MHz carrier centered at 2175 MHz spans
    // 2165..2185 MHz and pokes 5 MHz below n256's 2170 MHz edge; it used to be
    // reported conformant because only the center was tested.
    if (b.carrierInDownlink)
    {
        const double lo = (std::string(b.name) == "n255") ? 1525.0e6 : 2170.0e6;
        const double hi = (std::string(b.name) == "n255") ? 1559.0e6 : 2200.0e6;
        b.channelInBlock = carrierHz - bandwidthHz / 2.0 >= lo - 1.0 &&
                           carrierHz + bandwidthHz / 2.0 <= hi + 1.0;
    }
    b.conformant = b.carrierInDownlink && b.bandwidthSupported && b.channelInBlock;
    return b;
}

void
NtnRealStackHelper::ApplyNtnChannelExtras()
{
    // ---- G1: chain the TR 38.811 large-scale EXCESS-loss terms onto the
    //      measured channel (after Friis FSPL), so the measured SINR carries the
    //      elevation-dependent atmospheric/clutter/shadow/scintillation physics
    //      that previously lived only in the ntn-measurement-model oracle. ----
    // C3: the mainline TR 38.811 channel already carries gas, scintillation,
    // clutter and shadow fading; chaining the legacy excess model would count
    // them twice.
    if (m_tr38811 && m_channelModel == ChannelModel::Legacy)
    {
        Ptr<Ntn38811ExcessLossModel> excess = CreateObject<Ntn38811ExcessLossModel>();
        excess->SetCarrierFrequencyHz(m_freqHz);
        excess->SetScenario(
            static_cast<Ntn38811ExcessLossModel::NtnScenario>(m_ntnScenario));
        AddExtraPropagationLoss(excess);
    }

    // A5(ii): TR 38.811 §6.4.1 satellite beam pattern (off-boresight roll-off
    // only; the radio array supplies the peak gain) — opt-in via SetSatelliteBeam.
    if (m_satBeam)
    {
        Ptr<NtnSatBeamGainModel> beam = CreateObject<NtnSatBeamGainModel>();
        beam->SetBeamwidth3dBDeg(m_beamwidthDeg);
        if (m_beamCenter)
        {
            beam->SetBeamCenter(m_beamCenter);
        }
        AddExtraPropagationLoss(beam);
    }

    // C4: on mmwave the UE element gain cannot be declared on the array, so it
    // rides the chain as a negative loss (reciprocal, like the element gain).
    if (m_backend == RadioBackend::Mmwave && m_ueAntennaGainDbi != 0.0)
    {
        Ptr<NtnStaticExtraLossModel> g = CreateObject<NtnStaticExtraLossModel>();
        g->SetLossDb(-m_ueAntennaGainDbi);
        g->SetFloorDb(-m_ueAntennaGainDbi);
        AddExtraPropagationLoss(g);
    }

    // C1: Earth occultation last, so the existing chain order (and its random
    // streams) is untouched; a clear link contributes exactly 0 dB.
    if (m_earthBlockage)
    {
        m_blockageModel = CreateObject<NtnEarthBlockageLossModel>();
        m_blockageModel->SetAttribute("MinElevationDeg", DoubleValue(m_minElevationDeg));
        AddExtraPropagationLoss(m_blockageModel);
    }
}

void
NtnRealStackHelper::BuildNrRadio()
{
    // WF-13: relax the RLC-AM / RRC timers on THIS backend too.
    //
    // ConfigureNtnRlcRrcTimers() was called only from BuildMmwaveRadio(), so on
    // the nr backend - which is the default for most scenarios - the
    // terrestrial 20/10/10 ms RLC-AM timers and the 100 ms T300 stood over a
    // LEO slant whose feedback round trip is 8-26 ms. That is exactly the
    // spurious-retransmission and RLF condition the relaxation exists to
    // prevent, and it was absent from the path most runs take. Must precede the
    // device installation below so the objects read the new defaults.
    ConfigureNtnRlcRrcTimers();

    // ===== 5G-LENA (nr) FR1 NTN backend (closes A5(i)) =====
    // The 5G-LENA cttc-nr-demo recipe: a single operational band ->
    // 1 CC -> 1 FR1 BWP at m_freqHz/m_bwHz, FR1 numerology m_numerology, ideal
    // beamforming, and — crucially for the real NTN mobility models (SGP4/TR
    // 38.811, which feed ECEF positions) — a Friis large-scale loss instead of
    // the 3GPP terrestrial pathloss (whose ENU height/2D split is meaningless in
    // ECEF and collapses the loss). The 3GPP spatial model is kept for the UPA
    // array gain / DirectPath beamforming.

    // Big RLC buffers so a saturating DL flow is not buffer-limited (nr RLC UM).
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    m_nrEpc = CreateObject<NrPointToPointEpcHelper>();
    m_nrBeamforming = CreateObject<IdealBeamformingHelper>();
    m_nr = CreateObject<NrHelper>();
    m_nr->SetBeamformingHelper(m_nrBeamforming);
    m_nr->SetEpcHelper(m_nrEpc);

    // ---- Enabler C: MAC scheduler ---------------------------------------
    // Multi-slice runs REQUIRE the QoS scheduler to differentiate 5QI; a
    // caller SetScheduler() overrides. Default (single slice, no override) is
    // NR's own default TdmaRR -> historical behaviour.
    Scheduler sched = m_scheduler;
    if (m_slices.size() >= 2 && sched == Scheduler::TdmaRR)
    {
        sched = Scheduler::OfdmaQos;
    }
    if (!m_schedulerTypeName.empty())
    {
        m_nr->SetSchedulerTypeId(TypeId::LookupByName(m_schedulerTypeName));
    }
    else
        switch (sched)
    {
    case Scheduler::OfdmaRR:
        m_nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaRR"));
        break;
    case Scheduler::OfdmaPF:
        m_nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaPF"));
        break;
    case Scheduler::OfdmaQos:
        m_nr->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaQos"));
        break;
    case Scheduler::TdmaRR:
    default:
        // NR default. Its TDMA allocator used to starve the whole cell once the
        // active UEs were at least as many as the data symbols (12 UEs on a
        // 12-symbol slot: one 8-byte TB each at MCS 0, below the 10-byte DCI
        // floor, so nothing was ever sent). Fixed in the vendored
        // contrib/nr/model/nr-mac-scheduler-tdma.cc (AssignRBGTDMA).
        break;
    }

    // ---- Enabler A: NR inter-cell handover algorithm --------------------
    // NOTE: the vendored 5G-LENA v3.3 NrHelper keeps a handover-algorithm
    // factory (SetHandoverAlgorithmType) but never instantiates it or wires it
    // to the gNB RRC -- the x2-handover tests are commented out of that module's
    // build, i.e. helper-driven handover is unfinished upstream. Calling
    // SetHandoverAlgorithmType here would therefore be a no-op. We instead build
    // and cross-wire the A3-RSRP algorithm per gNB ourselves, after the devices
    // exist (the RRC then knows its carrier count) and before the UEs attach
    // (so the EVENT_A3 measurement config is in place when they connect). See
    // the wiring block after InstallGnbDevice below.

    // ---- Enabler D: NTN-stretched NR HARQ process pool (before install) --
    ConfigureNtnHarqProfileNr();

    // ---- Enabler C: one BWP per slice (default 1 BWP) --------------------
    const uint8_t nBwp = static_cast<uint8_t>(std::max<size_t>(1, m_slices.size()));

    // ---- Wave 9: paired FDD ---------------------------------------------
    // n255 and n256 are FDD bands (TS 38.101-5 Table 5.2-1). nr built them as
    // one carrier whose slots UL and DL share (all slots flexible), so a
    // backlogged narrow uplink, which needs whole slots for a few bytes,
    // crowded out the downlink (ntn-v2x-canyon-relay DL OWD 11.6 ms -> 1.75 s
    // with the PHR PRB limit). Under FDD every slot of the DL carrier is a DL
    // slot and every slot of the UL carrier at the paired frequency is a UL
    // slot, 5G-LENA's own FDD recipe (cttc-nr-notching, cttc-nr-traffic-ngmn-
    // mixed): one DL BWP and one UL BWP per slice, the gNB routing UL control
    // of BWP n+k to BWP k and the UE routing the control of BWP k to BWP n+k.
    // Auto applies it only under the Set-1 budget, so the Legacy profile keeps
    // reproducing the old numbers, and not when a scenario chose a TDD pattern.
    {
        DuplexMode mode = m_duplexMode;
        if (!m_duplexModeSet)
        {
            StringValue gv;
            GlobalValue::GetValueByName("NtnRealStackDuplex", gv);
            if (gv.Get() == "Fdd")
            {
                mode = DuplexMode::Fdd;
            }
            else if (gv.Get() == "Tdd")
            {
                mode = DuplexMode::Tdd;
            }
            else
            {
                NS_ABORT_MSG_IF(gv.Get() != "Auto",
                                "NtnRealStackDuplex must be Auto, Fdd or Tdd, got " << gv.Get());
                mode = DuplexMode::Auto;
            }
        }
        const double ulHz =
            (m_ulFreqHzExplicit > 0.0) ? m_ulFreqHzExplicit : PairedUlCarrierHz(m_freqHz);
        // nr allows at most nr::MAX_NO_CC (5) BWPs per device, so a DL/UL
        // pair per slice fits up to two slices; three or more slices keep the
        // shared-slot carrier.
        const bool fitsRouting = 2 * nBwp <= static_cast<int>(nr::MAX_NO_CC);
        bool fdd = false;
        // The opt-in real air-interface delay (with K_offset consumption) was
        // validated on the shared-slot carrier only; with paired FDD the UE
        // did not complete random access in nr v3.3 (measured on the
        // RaWindowStartOffsetTest geometry: no packet delivered). Auto keeps
        // the validated carrier for that mode; open item in WAVE9_REPORT.md.
        if (mode == DuplexMode::Auto)
        {
            fdd = ulHz > 0.0 && fitsRouting && m_tddPattern.empty() && !m_airIfaceDelayRequested &&
                  GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
        }
        else if (mode == DuplexMode::Fdd)
        {
            NS_ABORT_MSG_IF(ulHz <= 0.0,
                            "SetDuplexMode(Fdd): carrier " << m_freqHz / 1e6
                                                           << " MHz is not the downlink of n255 or "
                                                              "n256; call SetUlCarrierFrequencyHz");
            NS_ABORT_MSG_IF(!fitsRouting, "SetDuplexMode(Fdd): at most 2 slices (nr allows 5 BWPs)");
            NS_ABORT_MSG_IF(!m_tddPattern.empty(),
                            "SetDuplexMode(Fdd) and SetTddPattern() exclude each other");
            NS_ABORT_MSG_IF(m_airIfaceDelayRequested,
                            "SetDuplexMode(Fdd) with SetAirInterfaceDelay(true): random access "
                            "does not complete on paired FDD with a real air-interface delay in "
                            "nr v3.3; use Tdd or Auto");
            fdd = true;
        }
        m_fddApplied = fdd;
        m_ulFreqHzApplied = fdd ? ulHz : 0.0;
    }
    // Wave 9: the UL BWP has a scheduler of its own under FDD. nr's default
    // TDMA gives each UE whole symbols; a PHR-limited UE then uses a few PRBs
    // of a symbol and the rest of the band idles, while a second UE waits
    // for the next symbol. OFDMA shares the symbols between UEs (FDM), which
    // is how a power-limited NR uplink is scheduled. The DL BWP keeps the
    // scenario's scheduler, so the downlink is scheduled exactly as before.
    m_fddUlOfdmaApplied = false;
    if (m_fddApplied && m_schedulerTypeName.empty() && sched == Scheduler::TdmaRR)
    {
        BooleanValue gv;
        GlobalValue::GetValueByName("NtnRealStackFddUlOfdma", gv);
        if (gv.Get())
        {
            for (uint8_t k = 0; k < nBwp; ++k)
            {
                m_nr->SetSchedulerTypeIdForBwp(
                    nBwp + k,
                    TypeId::LookupByName("ns3::NrMacSchedulerOfdmaRR"));
            }
            m_fddUlOfdmaApplied = true;
        }
    }
    m_nrDlBwpCount = nBwp;
    const uint8_t nBwpAll = m_fddApplied ? static_cast<uint8_t>(2 * nBwp) : nBwp;
    m_nrBwpTotal = nBwpAll;

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(m_freqHz,
                                                   m_bwHz,
                                                   nBwp,
                                                   BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    OperationBandInfo ulBand;
    if (m_fddApplied)
    {
        // The UL carrier has the DL channel bandwidth (symmetric FDD).
        CcBwpCreator::SimpleOperationBandConf ulConf(m_ulFreqHzApplied,
                                                     m_bwHz,
                                                     nBwp,
                                                     BandwidthPartInfo::UMi_StreetCanyon);
        ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulConf);
    }
    // BWP i < nBwp: DL (or TDD) of slice i; BWP nBwp + i: UL of slice i (FDD).
    std::vector<BandwidthPartInfoPtr*> bwpRefs;
    for (uint8_t cc = 0; cc < nBwp; ++cc)
    {
        bwpRefs.push_back(&band.GetBwpAt(cc, 0));
    }
    if (m_fddApplied)
    {
        for (uint8_t cc = 0; cc < nBwp; ++cc)
        {
            bwpRefs.push_back(&ulBand.GetBwpAt(cc, 0));
        }
    }

    // NT-08. ThreeGppChannelModel gates cluster regeneration on
    // `!m_updatePeriod.IsZero()`, so pinning this to 0 froze the channel matrix
    // at t=0 for the entire run. The spectrum model does compute a per-cluster
    // Doppler phase from relative velocity, but with the geometry never
    // regenerated that phase never evolves - and a comment in
    // ntn-tr38811-excess-loss-model credited the 3GPP model with applying
    // "small-scale fading with Doppler on the same link" on that basis.
    //
    // The default is now 100 ms, matching IdealBeamformingHelper's own
    // BeamformingPeriodicity. Leaving it at 0 to protect existing numbers, which
    // is what this comment used to say, meant every existing number was produced
    // by a beam that tracked the satellite against a channel matrix that did
    // not: 17.5 dB of DL SINR on the smoke scenario, and a hundredfold TBLER.
    // A scenario that genuinely wants a frozen channel can still ask for 0.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(m_channelUpdatePeriod));
    m_nr->SetChannelConditionModelAttribute("UpdatePeriod",
                                            TimeValue(m_channelUpdatePeriod));
    // A9. SRS configuration indices, not spectrum, are what caps how many UEs a
    // gNB can admit. NrGnbRrc::SrsPeriodicity defaults to 40 and the toolkit
    // never set it, so DoAllocateTemporaryCellRnti started returning 0 once the
    // indices ran out. That refusal is not honoured downstream: the MAC still
    // builds a RAR, several refused UEs collide on RNTI 0 in m_rapIdRntiMap, and
    // a UE then receives two RARs matching its own preamble in one message. The
    // second arrives at NrUeRrc in IDLE_CONNECTING, where
    // DoNotifyRandomAccessSuccessful has no case for it and calls
    // NS_FATAL_ERROR. Measured: fine at 16 UEs, aborts at 30, with the probe
    // showing IMSI 22 processing preamble 40 twice at the same instant, both
    // times with rnti=0 while its peers held 32 to 40.
    //
    // Size the periodicity to the run instead of leaving the default. The
    // allowed set is {2, 5, 10, 20, 40, 80, 160, 320} and only part of each
    // period is usable as indices, so the headroom below is measured rather than
    // assumed: 40 could not carry 30 UEs.
    {
        static const uint32_t kAllowed[] = {2, 5, 10, 20, 40, 80, 160, 320};
        // Never go BELOW NrGnbRrc's own default of 40. A first version of this
        // picked the smallest allowed value that fit, which for the three- and
        // four-UE scenarios most examples default to chose 10 or 20 instead. A
        // shorter SRS period means SRS is sent more often, so that would have
        // changed uplink behaviour, and every measured result that depends on
        // it, across scenarios that never had a capacity problem. This may only
        // add capacity, never alter a run that was already fine.
        static const uint32_t kDefaultPeriodicity = 40;
        const uint32_t needed = 2 * (m_ue.GetN() + 2);
        uint32_t srs = kAllowed[sizeof(kAllowed) / sizeof(kAllowed[0]) - 1];
        for (uint32_t cand : kAllowed)
        {
            if (cand >= needed)
            {
                srs = cand;
                break;
            }
        }
        srs = std::max(srs, kDefaultPeriodicity);
        Config::SetDefault("ns3::NrGnbRrc::SrsPeriodicity", UintegerValue(srs));
        NS_LOG_INFO("SrsPeriodicity set to " << srs << " for " << m_ue.GetN() << " UEs");
    }
    if (!m_tddPattern.empty())
    {
        Config::SetDefault("ns3::NrGnbPhy::Pattern", StringValue(m_tddPattern));
        NS_LOG_INFO("NtnRealStackHelper: TDD pattern " << m_tddPattern);
    }

    m_nr->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Friis large-scale loss on EVERY BWP (frame-independent, valid at LEO
    // range); InitializeOperationBand keeps the 3GPP spatial model for array
    // gain.
    //
    // GAP S5 FIX: keep a per-BWP Friis head (not just CC0's). The NTN excess
    // loss / beam / caller-supplied chains attach to these heads; storing only
    // head[0] meant a sliced run (N BWPs) left slices 1..N-1 on bare Friis with
    // no atmosphere, scintillation, shadowing or beam roll-off — biasing exactly
    // the per-slice SINR comparison the slicing feature exists to measure.
    m_nrBaseLossPerBwp.assign(nBwpAll, nullptr);
    if (m_channelModel == ChannelModel::Ns3Tr38811)
    {
        // C3: the ns-3 TR 38.811 NTN models need geocentric endpoints; check
        // here rather than let ThreeGppChannelModel abort mid-run.
        auto geo = [](Ptr<Node> n) {
            return DynamicCast<GeocentricConstantPositionMobilityModel>(
                       n->GetObject<MobilityModel>()) != nullptr;
        };
        for (uint32_t i = 0; i < m_gnb.GetN(); ++i)
        {
            NS_ABORT_MSG_IF(!geo(m_gnb.Get(i)),
                            "SetChannelModel(Ns3Tr38811): gNB " << i << " mobility is not a "
                            "GeocentricConstantPositionMobilityModel; use Sgp4MobilityModel or "
                            "wrap it in ntncon::NtnGeocentricMobilityAdapter");
        }
        for (uint32_t i = 0; i < m_ue.GetN(); ++i)
        {
            NS_ABORT_MSG_IF(!geo(m_ue.Get(i)),
                            "SetChannelModel(Ns3Tr38811): UE " << i << " mobility is not a "
                            "GeocentricConstantPositionMobilityModel; use NtnTr38811MobilityModel "
                            "or wrap it in ntncon::NtnGeocentricMobilityAdapter");
        }
        // src patch 0001: elevation about the WGS-84 normal at the lower end
        // (the stock ns-3.43 picks the "ground" end by ECEF z and takes |asin|).
        {
            StringValue prev;
            GlobalValue::GetValueByName("GeocentricElevationModel", prev);
            m_prevElevationModel = prev.Get();
            GlobalValue::Bind("GeocentricElevationModel", StringValue("Wgs84LowerEnd"));
        }
    }
    for (uint8_t cc = 0; cc < nBwpAll; ++cc)
    {
        BandwidthPartInfoPtr& bwp = *bwpRefs[cc];
        // Each BWP sits at its own centre frequency; using the band centre for
        // all of them mis-scales Friis across a wide multi-BWP band.
        const double bwpFreqHz = (bwp->m_centralFrequency > 0.0) ? bwp->m_centralFrequency : m_freqHz;
        if (m_channelModel == ChannelModel::Ns3Tr38811)
        {
            ApplyNs3Tr38811Channel(bwp.get());
        }
        else
        {
            Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
            friis->SetAttribute("Frequency", DoubleValue(bwpFreqHz));
            bwp->m_propagation = friis;
        }
        if (!m_smallScaleFading)
        {
            // C3: deterministic LOS plane wave (no multipath), e.g. ISLs.
            Ptr<ThreeGppSpectrumPropagationLossModel> sp =
                CreateObject<ThreeGppSpectrumPropagationLossModel>();
            Ptr<NtnLosChannelModel> los = CreateObject<NtnLosChannelModel>();
            los->SetAttribute("Frequency", DoubleValue(bwpFreqHz));
            los->SetAttribute("UpdatePeriod", TimeValue(m_channelUpdatePeriod));
            sp->SetChannelModel(los);
            bwp->m_3gppChannel = sp;
        }
        m_nrBaseLossPerBwp[cc] = bwp->m_propagation;
        if (cc == 0)
        {
            m_nrBaseLoss = bwp->m_propagation; // back-compat head
        }
    }

    m_nr->InitializeOperationBand(&band);
    if (m_fddApplied)
    {
        m_nr->InitializeOperationBand(&ulBand);
    }

    // ---- GAP S4 FIX: NTN geometry is always line-of-sight ------------------
    // InitializeOperationBand attaches a UMi-StreetCanyon *probabilistic*
    // channel-condition model (the scenario enum above only selects a parameter
    // set; 5G-LENA has no NTN scenario in v3.3). Evaluated on ECEF coordinates a
    // LEO link has d2D of hundreds-to-thousands of km, so the UMi LOS formula
    // returns NLOS essentially always, and the fading draw then comes from UMi
    // NLOS cluster tables with "antenna heights" of ~6.37e6 m. That is not
    // TR 38.811 §6.7 NTN fading in any sense, and it is why the nr backend
    // needed a physically impossible EIRP to close the link.
    //
    // A service link to a satellite above the minimum elevation is LOS by
    // construction (TR 38.811 §6.6.1: LOS probability -> 1 at high elevation;
    // the toolkit gates candidates on elevation anyway). Force it, mirroring
    // what the mmwave backend already does, until a real TR 38.811 NTN-TDL
    // spectrum model exists.
    for (uint8_t cc = 0; cc < nBwpAll; ++cc)
    {
        if (m_channelModel != ChannelModel::Legacy || !m_smallScaleFading)
        {
            break; // C3: the TR 38.811 condition model / LOS-only model own this
        }
        BandwidthPartInfoPtr& bwp = *bwpRefs[cc];
        Ptr<ThreeGppSpectrumPropagationLossModel> sp =
            DynamicCast<ThreeGppSpectrumPropagationLossModel>(bwp->m_3gppChannel);
        if (sp)
        {
            sp->SetChannelModelAttribute(
                "ChannelConditionModel",
                PointerValue(CreateObject<AlwaysLosChannelConditionModel>()));
        }
    }

    // ---- GAP S2 FIX: real propagation delay on the air interface -----------
    // 5G-LENA creates each BWP SpectrumChannel with loss models only, so
    // MultiModelSpectrumChannel leaves delay = 0: the UE "hears" the satellite
    // instantaneously. Every NTN timing conclusion (HARQ stalling, K_offset,
    // URLLC budgets, RTT) is meaningless without it, and the NTN-stretched HARQ
    // pool this helper configures was sizing for an RTT the MAC never saw.
    // TR 38.821 Table 4.2-2: LEO-600 one-way service-link delay 2.0-6.44 ms.
    //
    // HARD CONSTRAINT (measured, not assumed): the vendored 5G-LENA v3.3 has no
    // Rel-17 NTN timing machinery, and a real per-distance delay breaks it in
    // TWO independent ways. Both were reproduced, not theorised:
    //
    //  1. No Timing Advance (TS 38.213 §4.2). The gNB asserts that the UL
    //     control from EVERY UE arrives at the same instant
    //     (nr-spectrum-phy.cc:1098: NS_ASSERT(m_firstRxStart == Simulator::Now()
    //     && ...)). Aligning those arrivals is precisely TA's job. With >1 UE at
    //     different ranges the arrivals differ and the stack SIGABRTs.
    //  2. No K_offset (TS 38.213 §4.2). The delayed DL lands inside the slot the
    //     UE believes is its UL, so the half-duplex TDD check trips:
    //     "Cannot TX while RX." (nr-spectrum-phy.cc:660/711). K_offset exists
    //     exactly to push the UL grant beyond the round trip. This bites even a
    //     SINGLE UE as soon as the scenario has uplink traffic.
    //
    // So the air-interface delay is OPT-IN and OFF by default on this stack: it
    // is not a knob we can flip until K_offset/TA exist (gap R1/R3, plan phase
    // P3.16) or the toolkit moves to a stack that has them (nr v5.0/ns-3.48,
    // W-0). What we CAN do honestly today is carry the service-link slant on the
    // backhaul leg (the mmwave treatment), so the end-to-end user-plane delay is
    // physically correct on every path even though the air interface does not
    // experience it. The OWD-floor health gate then still has teeth.
    m_airIfaceDelayActive = false;
    if (m_airIfaceDelayRequested)
    {
        for (uint8_t cc = 0; cc < nBwpAll; ++cc)
        {
            BandwidthPartInfoPtr& bwp = *bwpRefs[cc];
            Ptr<SpectrumChannel> ch = bwp->m_channel;
            if (ch && !ch->GetPropagationDelayModel())
            {
                ch->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
                m_airIfaceDelayActive = true;
            }
        }
        if (m_kOffsetConsumption)
        {
            NS_LOG_WARN("nr backend: real air-interface propagation delay ACTIVE + K_offset "
                        "CONSUMPTION on. The geometry-derived K_offset is applied to N2Delay "
                        "(TS 38.213 §4.2), pushing the UL grant past the round trip — this "
                        "resolves the 'Cannot TX while RX' half-duplex conflict, so single-UE "
                        "uplink now runs. Multi-UE uplink still needs per-UE Timing Advance "
                        "(separate mechanism, not yet in nr v3.3). See gap R1/R3.");
        }
        else
        {
            NS_LOG_WARN("nr backend: real air-interface propagation delay ACTIVE by request, but "
                        "K_offset consumption is OFF. The vendored 5G-LENA v3.3 has no K_offset/TA, "
                        "so this is only safe for a single UE with downlink-only traffic; uplink "
                        "will abort with 'Cannot TX while RX'. Call SetKOffsetConsumption(true) to "
                        "consume the SIB19 K_offset and unlock single-UE uplink. See gap R1/R3.");
        }
    }

    BandwidthPartInfoPtrVector allBwps =
        m_fddApplied ? CcBwpCreator::GetAllBwps({band, ulBand}) : CcBwpCreator::GetAllBwps({band});

    // Enabler B seam: keep each BWP's spectrum channel so a caller can install
    // a frequency-selective / spatial SpectrumPropagationLossModel later.
    m_nrBwpChannels.clear();
    for (const auto& bwp : allBwps)
    {
        m_nrBwpChannels.push_back(bwp.get()->m_channel);
    }
    // Per-BWP RB count (for the measured PRB-utilisation fraction). SCS grows
    // with numerology; each BWP carries m_bwHz/nBwp of the band.
    const double scsHz = 15.0e3 * std::pow(2.0, m_numerology);
    m_nrBandRb = std::max<uint32_t>(
        1, static_cast<uint32_t>((m_bwHz / nBwp) / (12.0 * scsHz)));

    // ---- Enabler C: route each slice's 5QI to its BWP (gNB + UE mgr) ------
    // Wave 9 FDD: the gNB sends a bearer on its slice's DL BWP and the UE on
    // the paired UL BWP; every QCI the UE has goes to the UL BWP of slice 0
    // unless a slice claims it.
    if (m_fddApplied)
    {
        const TypeId algo = BwpManagerAlgorithmStatic::GetTypeId();
        for (uint32_t a = 0; a < algo.GetAttributeN(); ++a)
        {
            m_nr->SetUeBwpManagerAlgorithmAttribute(algo.GetAttribute(a).name,
                                                    UintegerValue(nBwp));
        }
    }
    for (uint8_t i = 0; i < m_slices.size(); ++i)
    {
        const std::string attr = QciAttrName(m_slices[i].fiveQi);
        if (!attr.empty())
        {
            m_nr->SetGnbBwpManagerAlgorithmAttribute(attr, UintegerValue(i));
            m_nr->SetUeBwpManagerAlgorithmAttribute(
                attr,
                UintegerValue(m_fddApplied ? static_cast<uint8_t>(nBwp + i) : i));
        }
    }

    m_nrBeamforming->SetAttribute("BeamformingMethod",
                                  TypeIdValue(DirectPathBeamforming::GetTypeId()));
    // Zero the S1-U latency; the NTN one-way delay rides the backhaul P2P leg.
    m_nrEpc->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Antennas: gNB and UE UniformPlanarArray. Defaults gNB 8x8 (64 elem) to
    // match the mmwave backend's array gain, UE 1x2. Enabler B (SetMimo) grows
    // these to enable real spatial multiplexing.
    m_nr->SetUeAntennaAttribute("NumRows", UintegerValue(m_ueRows));
    m_nr->SetUeAntennaAttribute("NumColumns", UintegerValue(m_ueCols));
    {
        // C4: declared UE element gain (reciprocal: UL too). 0 dBi is the
        // IsotropicAntennaModel default, so the legacy object is unchanged.
        Ptr<IsotropicAntennaModel> ueElem = CreateObject<IsotropicAntennaModel>();
        if (m_ueAntennaGainDbi != 0.0)
        {
            ueElem->SetAttribute("Gain", DoubleValue(m_ueAntennaGainDbi));
        }
        m_nr->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElem));
    }
    if (std::isfinite(m_ueNoiseFigureDb))
    {
        m_nr->SetUePhyAttribute("NoiseFigure", DoubleValue(m_ueNoiseFigureDb));
    }
    m_nr->SetGnbAntennaAttribute("NumRows", UintegerValue(m_gnbRows));
    m_nr->SetGnbAntennaAttribute("NumColumns", UintegerValue(m_gnbCols));
    m_nr->SetGnbAntennaAttribute("AntennaElement",
                                 PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Enabler B: real NR MIMO — NrPmSearchFull rank/PMI adaptation over the
    // (larger) arrays above. The precoding-matrix search picks the DL rank per
    // subband from the channel matrix, so UM-MIMO capacity becomes measured.
    if (m_mimo)
    {
        NrHelper::MimoPmiParams mp;
        mp.pmSearchMethod = "ns3::NrPmSearchFull";
        mp.rankLimit = m_mimoRank;
        // NR requires subbandSize 4 or 8 for BWPs of 24..72 PRBs; use 8 (valid
        // across the NTN-FR1 BWP sizes we build). subbandSize=1 (the struct
        // default) asserts on those bands.
        mp.subbandSize = 8;
        m_nr->SetupMimoPmi(mp);
    }

    // Wave 6A: Rel-17 downlinkHARQ-FeedbackDisabled-r17 bitmap (nr patch 14).
    if (m_dlHarqFbDisabled != 0)
    {
        m_nr->SetSchedulerAttribute("DlHarqFeedbackDisabled", UintegerValue(m_dlHarqFbDisabled));
        // Wave 7A (nr patch 15): the UE reads the same RRC bitmap and stops
        // reporting HARQ-ACK on PUCCH for those processes (TS 38.213 9.1.2).
        m_nr->SetUePhyAttribute("DlHarqFeedbackDisabled", UintegerValue(m_dlHarqFbDisabled));
        NS_LOG_INFO("NtnRealStackHelper: DL HARQ feedback disabled on process bitmap 0x"
                    << std::hex << m_dlHarqFbDisabled << std::dec);
    }

    // Wave 7A (nr patch 15): TS 38.321 5.4.5 retxBSR-Timer.
    {
        Time t = m_retxBsrTimer;
        if (!m_retxBsrTimerSet)
        {
            const double rttS = 2.0 * 2.0 * WorstCaseSlantM() / 299792458.0;
            t = (GetLinkBudgetProfile() == LinkBudgetProfile::Legacy)
                    ? Time(0)
                    : std::max(MilliSeconds(80), MilliSeconds(static_cast<uint64_t>(
                                                     std::ceil(rttS * 1e3))));
        }
        m_retxBsrTimerApplied = t;
        if (t.IsStrictlyPositive())
        {
            m_nr->SetUeMacAttribute("RetxBsrTimer", TimeValue(t));
        }
    }

    // Wave 8 (nr patch 16): power headroom reporting (TS 38.321 5.4.6,
    // TS 38.331 PHR-Config) and the scheduler's PHR-based PUSCH PRB limit.
    // The UE power control itself is programmed per UE after installation.
    {
        bool on = m_ulPhrLimit;
        if (!m_ulPhrLimitSet)
        {
            BooleanValue gv;
            GlobalValue::GetValueByName("NtnRealStackUlPhrPrbLimit", gv);
            on = gv.Get() && GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
        }
        m_ulPhrLimitApplied = on;
        if (on)
        {
            m_nr->SetUeMacAttribute("PhrPeriodicTimer", TimeValue(MilliSeconds(20)));
            m_nr->SetUeMacAttribute("PhrProhibitTimer", TimeValue(MilliSeconds(10)));
            m_nr->SetUeMacAttribute("PhrTxPowerFactorChange", DoubleValue(1.0));
            m_nr->SetSchedulerAttribute("PhrUlRbLimit", BooleanValue(true));
        }
    }

    // Wave 9 (nr patch 17): TS 38.321 5.4.5 BSR triggering (BSR-Config). nr
    // put a 5-byte SHORT_BSR into every UL TB; a PRB-limited handheld's grant
    // carries about 18 bytes, so that was more than a quarter of it. The gNB
    // then charges the UE's buffer net of the MAC subheader, since the
    // per-TB BSR that used to correct its estimate is gone.
    {
        bool on = m_bsrTriggering;
        if (!m_bsrTriggeringSet)
        {
            BooleanValue gv;
            GlobalValue::GetValueByName("NtnRealStackBsrTriggering", gv);
            on = gv.Get() && GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
        }
        m_bsrTriggeringApplied = on;
        if (on)
        {
            m_nr->SetUeMacAttribute("BsrTriggering", BooleanValue(true));
            m_nr->SetUeMacAttribute("PeriodicBsrTimer", TimeValue(m_periodicBsrTimer));
            m_nr->SetSchedulerAttribute("UlGrantSubheaderBytes", UintegerValue(2));
        }
    }

    // Wave 10: UE-directed beam on the handover target (see NrHandoverEndOk).
    {
        bool on = m_hoTargetBeam;
        if (!m_hoTargetBeamSet)
        {
            BooleanValue gv;
            GlobalValue::GetValueByName("NtnRealStackHoTargetBeam", gv);
            on = gv.Get() && GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
        }
        m_hoTargetBeamApplied = on;
        if (on)
        {
            m_nr->SetGnbPhyAttribute("BeamLookupServingUeOnly", BooleanValue(true));
        }
    }

    // Wave 11b (nr patch 20): robust MCS on the target until its first CSI.
    {
        bool on = m_hoRobustMcs;
        if (!m_hoRobustMcsSet)
        {
            BooleanValue gv;
            GlobalValue::GetValueByName("NtnRealStackHoRobustMcs", gv);
            on = gv.Get() && GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
        }
        m_hoRobustMcsApplied = on;
        if (on)
        {
            m_nr->SetUePhyAttribute("HandoverRxFlush", BooleanValue(true));
        }
    }

    // Install devices on the caller's mobility-carrying nodes.
    m_enbDevs = m_nr->InstallGnbDevice(m_gnb, allBwps);
    m_ueDevs = m_nr->InstallUeDevice(m_ue, allBwps);
    // RRU.PrbAvailDl (TS 28.552 5.1.1.2) is the PRB count the PHY can schedule. nr removes its RbOverhead (4 % by
    // default) before dividing by the RB width, so a 20 MHz BWP at 30 kHz has 53 PRBs, not the 55 of bandwidth /
    // (12 SCS) above; with 55 the PRB usage never exceeded 96.4 %. The PHY learns its bandwidth only when the RRC
    // configures it, so apply nr's own rule here (NrPhy::DoUpdateRbNum: the bandwidth in 100 kHz units, less the
    // installed PHY's RbOverhead, over the RB width).
    if (m_enbDevs.GetN() > 0)
    {
        const double oh = m_nr->GetGnbPhy(m_enbDevs.Get(0), 0)->GetRbOverhead();
        const double bwpHz = std::floor((m_bwHz / nBwp) / 1e5 + 1e-9) * 1e5;
        m_nrBandRb = std::max<uint32_t>(1, static_cast<uint32_t>(bwpHz * (1.0 - oh) / (12.0 * scsHz)));
    }
    if (m_fddApplied)
    {
        // Wave 9 FDD: every slot of a DL BWP is DL, every slot of a UL BWP is
        // UL, and the control of each pair is routed across it.
        const std::string kDl("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|");
        const std::string kUl("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|");
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            for (uint8_t b = 0; b < nBwpAll; ++b)
            {
                m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                    ->SetAttribute("Pattern", StringValue(b < nBwp ? kDl : kUl));
            }
            for (uint8_t k = 0; k < nBwp; ++k)
            {
                m_nr->GetBwpManagerGnb(m_enbDevs.Get(i))->SetOutputLink(nBwp + k, k);
            }
        }
        for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
        {
            Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
            ue->SetAttribute("PrimaryUlIndex", UintegerValue(nBwp));
            ue->GetRrc()->SetPrimaryUlIndex(nBwp);
            for (uint8_t k = 0; k < nBwp; ++k)
            {
                m_nr->GetBwpManagerUe(m_ueDevs.Get(i))->SetOutputLink(k, nBwp + k);
                // TS 38.213 7.1.1: the PUSCH path loss PL_b,f,c(q_d) is
                // estimated on a DL reference signal. Under FDD the UL BWP's
                // PHY receives no DL signal at all (the gNB routes the UL DCI
                // out through the DL BWP), so its power control kept nr's
                // initial 100 dB and transmitted about 50 dB too low (measured:
                // PUSCH SINR -44 dB, no PHR). Feed it the RSRP the paired DL
                // BWP measures; the SIB reference power it subtracts that from
                // is the DL BWP's (NrGnbRrc broadcasts BWP 0's).
                Ptr<NrUePhy> dlPhy = ue->GetPhy(k);
                Ptr<NrUePhy> ulPhy = ue->GetPhy(nBwp + k);
                Ptr<NrChunkProcessor> rs = Create<NrChunkProcessor>();
                rs->AddCallback(MakeCallback(&NrUePhy::ReportRsReceivedPower, ulPhy));
                dlPhy->GetSpectrumPhy()->AddRsPowerChunkProcessor(rs);
            }
        }
        NS_LOG_INFO("NtnRealStackHelper: paired FDD, DL " << m_freqHz / 1e6 << " MHz, UL "
                                                         << m_ulFreqHzApplied / 1e6 << " MHz, "
                                                         << m_bwHz / 1e6 << " MHz each, "
                                                         << +nBwp << " BWP pair(s)");
    }
    int64_t stream = 1;
    stream += m_nr->AssignStreams(m_enbDevs, stream);
    stream += m_nr->AssignStreams(m_ueDevs, stream);

    // FR1 numerology + powers per device, on EVERY BWP (else BWPs 1..N-1 keep
    // the default low TxPower and their slices see a collapsed SINR).
    //
    // GAP S7 FIX: split the conducted power across BWPs. A satellite has ONE
    // power amplifier; giving every one of N slice BWPs the full m_satEirpDbm
    // radiated N x the power budget (+10log10(N) dB of free EIRP), so a 3-slice
    // run was ~4.8 dB hot relative to a 1-slice run of the "same" satellite.
    // Wave 7A: share the amplifier in proportion to each BWP's bandwidth, so
    // the EIRP density (dBW/MHz) is constant across BWPs (TR 38.821 Table
    // 6.1.1.1-1 states the payload as a density). With equal BWPs this is the
    // old 1/N split; with unequal ones an equal split gave the narrow BWP a
    // hotter density than the wide one. A scenario may set each BWP itself.
    m_bwpPowerApplied.assign(nBwp, m_satEirpDbm);
    {
        std::vector<double> bwHz(nBwp, m_bwHz / nBwp);
        for (uint8_t b = 0; b < nBwp && b < allBwps.size(); ++b)
        {
            const double w = allBwps[b].get()->m_channelBandwidth;
            if (w > 0.0)
            {
                bwHz[b] = w;
            }
        }
        double sum = 0.0;
        for (double w : bwHz)
        {
            sum += w;
        }
        if (!m_bwpPowerExplicit.empty())
        {
            NS_ABORT_MSG_IF(m_bwpPowerExplicit.size() != nBwp,
                            "SetPerBwpConductedPowerDbm: " << m_bwpPowerExplicit.size()
                                                           << " values for " << +nBwp << " BWPs");
            m_bwpPowerApplied = m_bwpPowerExplicit;
        }
        else
        {
            for (uint8_t b = 0; b < nBwp; ++b)
            {
                m_bwpPowerApplied[b] = m_satEirpDbm + 10.0 * std::log10(bwHz[b] / sum);
            }
        }
    }
    const double bwpPowerSplitDb = m_satEirpDbm - m_bwpPowerApplied[0];
    const double conductedPerBwpDbm = m_bwpPowerApplied[0];
    // R1/R3 K_offset CONSUMPTION: TS 38.213 §4.2. When enabled, extend each gNB
    // PHY's N2Delay (the DCI->PUSCH gap, in slots) by the geometry-derived
    // K_offset. nr-gnb-phy.cc does `ulSfn.Add(GetN2Delay())`, so the UE's uplink
    // grant is pushed K_offset slots later — past the service-link round trip —
    // which is exactly what K_offset exists to do, and what stops the delayed
    // downlink from landing in the UE's uplink slot ("Cannot TX while RX").
    m_consumedKOffsetSlots = 0;
    uint32_t baseN2 = 0;
    if (m_kOffsetConsumption)
    {
        // RRC-1: prefer the value the network BROADCAST over a local
        // re-derivation, so the scheduler and SIB19 cannot disagree.
        //
        // RRC-7 (2026-09-18): the broadcast field is counted in slots of the
        // 15 kHz reference subcarrier spacing (TS 38.331), while N2Delay below
        // is counted in slots of the numerology this cell actually runs. The
        // two are the same only at numerology 0, so convert rather than adding
        // a 15 kHz slot count to a 30 kHz scheduler. ComputeKOffsetSlots()
        // already works in the configured numerology and needs no scaling.
        if (m_broadcastKOffsetSlots > 0)
        {
            m_consumedKOffsetSlots = ReferenceToCellSlots(m_broadcastKOffsetSlots);
        }
        else
        {
            m_consumedKOffsetSlots = ComputeKOffsetSlots();
        }
        // Read the stack's default N2Delay so we ADD to it, not clobber it.
        UintegerValue n2v;
        m_nr->GetGnbPhy(m_enbDevs.Get(0), 0)->GetAttribute("N2Delay", n2v);
        baseN2 = static_cast<uint32_t>(n2v.Get());
        m_baseN2Delay = baseN2;
        NS_LOG_INFO("NtnRealStackHelper: consuming SIB19 K_offset = "
                    << m_consumedKOffsetSlots << " slots (numerology " << m_numerology
                    << ", RTT " << (2.0 * ComputeServiceLinkDelay().GetSeconds() * 1e3)
                    << " ms); N2Delay " << baseN2 << " -> " << (baseN2 + m_consumedKOffsetSlots));
    }
    // RRC-4: size the RAR window for the real geometry before the cells start
    // serving. NrGnbMac hands raResponseWindowSize to the UE in its RACH
    // configuration (NrGnbMac::GetRachConfig -> NrUeCmacSapProvider), so
    // writing it here is what actually reaches the UE's timeout.
    m_rachVerdict = NtnRachWindowVerdict{};
    if (m_ntnRachWindow)
    {
        const Time rtt = Seconds(2.0 * ComputeServiceLinkDelay().GetSeconds());
        const Time slot = NtnRachWindow::SlotPeriodForNumerology(m_numerology);
        // Wave 6A: with the Rel-17 window start offset in force (air-interface
        // delay on), the UE opens the window one RTT late, so only the
        // processing allowance has to fit in the window itself.
        const Time offset = (m_ntnRaWindowOffset && m_airIfaceDelayActive) ? rtt : Time();
        m_rachVerdict = NtnRachWindow::Evaluate(rtt, slot, Time(), offset);
        if (m_rachVerdict.fits)
        {
            NS_LOG_INFO("NtnRealStackHelper: RAR window sized for "
                        << rtt.GetMilliSeconds() << " ms round trip -> "
                        << "RaResponseWindowSize=" << +m_rachVerdict.appliedWindowSize
                        << " (" << m_rachVerdict.configuredWindow.GetMilliSeconds() << " ms)");
        }
        else
        {
            // Do not pretend. The stack cannot express this window, so access
            // will fail no matter what is written, and saying so here is the
            // difference between a documented limit and a mystery.
            NS_LOG_WARN("NtnRealStackHelper: this geometry needs a RAR window of "
                        << m_rachVerdict.requiredWindow.GetMilliSeconds()
                        << " ms (RaResponseWindowSize=" << m_rachVerdict.requiredWindowSize
                        << ") but nr caps the attribute at "
                        << +NtnRachWindow::kNrWindowSizeMax << ", buying only "
                        << m_rachVerdict.configuredWindow.GetMilliSeconds()
                        << " ms. Random access CANNOT complete on this stack at this "
                        << "geometry; shortfall "
                        << m_rachVerdict.shortfall.GetMilliSeconds() << " ms. TR 38.821 "
                        << "section 7.3 resolves this by offsetting the window start with "
                        << "ta-Common, which nr v3.3 does not implement.");
        }
    }

    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        if (m_ntnRachWindow && m_rachVerdict.appliedWindowSize > 0)
        {
            for (uint8_t b = 0; b < nBwpAll; ++b)
            {
                Ptr<NrGnbMac> gm = NrHelper::GetGnbMac(m_enbDevs.Get(i), b);
                if (gm)
                {
                    gm->SetAttribute("RaResponseWindowSize",
                                     UintegerValue(m_rachVerdict.appliedWindowSize));
                }
            }
        }
        for (uint8_t b = 0; b < nBwpAll; ++b)
        {
            m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                ->SetAttribute("Numerology", UintegerValue(m_numerology));
            // Wave 9 FDD: the UL BWP's gNB PHY carries no DL signal of its
            // own (its DCIs leave through the DL BWP); it gets the DL BWP's
            // power so nothing it might send is out of budget.
            m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                ->SetAttribute("TxPower", DoubleValue(m_bwpPowerApplied[b % nBwp]));
            if (m_kOffsetConsumption)
            {
                m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                    ->SetAttribute("N2Delay",
                                   UintegerValue(baseN2 + m_consumedKOffsetSlots));
            }
        }
    }

    // S7: state the radiated result so a physically impossible operating point
    // cannot hide behind an innocuous-looking "TxPower" number.
    NS_LOG_INFO("NtnRealStackHelper power budget: conducted "
                << conductedPerBwpDbm << " dBm/BWP (+" << bwpPowerSplitDb << " dB split over "
                << static_cast<uint32_t>(nBwp) << " BWP) + array gain " << ArrayGainDb()
                << " dB (" << static_cast<uint32_t>(m_gnbRows) << "x"
                << static_cast<uint32_t>(m_gnbCols) << " UPA) => effective EIRP "
                << GetEffectiveEirpDbm() << " dBm");
    if (GetEffectiveEirpDbm() > 90.0)
    {
        NS_LOG_WARN("effective satellite EIRP "
                    << GetEffectiveEirpDbm()
                    << " dBm exceeds any TR 38.821 Set-1/Set-2 figure (Set-1 LEO S-band is "
                       "~78.8 dBm total, antenna INCLUDED). Use SetSatEirpTotalDbm() or "
                       "SetSatEirpDensityDbwMhz() so the array gain is not double-counted.");
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < nBwpAll; ++b)
        {
            m_nr->GetUePhy(m_ueDevs.Get(i), b)->SetAttribute("TxPower", DoubleValue(m_ueTxDbm));
        }
    }
    // Wave 8 (nr patch 16): TS 38.213 7.1.1 PUSCH power control,
    //   P = min(P_CMAX, P_O_PUSCH + 10log10(2^mu M_RB) + alpha PL + deltaTF + f),
    // with P_CMAX the UE power class, alpha = 1 (full path-loss compensation)
    // and P_O_NOMINAL_PUSCH the per-180-kHz receive level that gives the target
    // SNR over the gNB's thermal noise. PL is the UE's own estimate,
    // ss-PBCH-BlockPower (the gNB conducted power nr broadcasts) minus the
    // filtered RSRP. nr measures that RSRP on the control channel, which goes
    // out on the quasi-omni beam unless the SSB cell beam is set, while the
    // PUSCH is received on the UE-directed beam: the gNB knows this gain
    // difference of its own antenna and subtracts it from P0, as an operator
    // tunes P0 to the receive beam.
    m_ulP0Applied = std::numeric_limits<double>::quiet_NaN();
    if (m_ulPhrLimitApplied && m_ueDevs.GetN() > 0 && m_enbDevs.GetN() > 0)
    {
        DoubleValue gnbNf;
        m_nr->GetGnbPhy(m_enbDevs.Get(0), 0)->GetAttribute("NoiseFigure", gnbNf);
        const double noise180kDbm = -174.0 + 10.0 * std::log10(180.0e3) + gnbNf.Get();
        const double rxBeamOverRsBeamDb = m_ssbCellBeam ? 0.0 : ArrayGainDb();
        const double p0 = std::round(noise180kDbm + m_ulTargetSnrDb - rxBeamOverRsBeamDb);
        m_ulP0Applied = std::clamp(p0, -202.0, 24.0); // TS 38.331 p0-NominalWithGrant
        for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
        {
            for (uint8_t b = 0; b < nBwpAll; ++b)
            {
                Ptr<NrUePhy> phy = m_nr->GetUePhy(m_ueDevs.Get(i), b);
                Ptr<NrUePowerControl> pc = phy->GetUplinkPowerControl();
                pc->SetAttribute("TSpec", EnumValue(NrUePowerControl::TS_38_213));
                pc->SetAttribute("Pcmax", DoubleValue(m_ueTxDbm));
                pc->SetAttribute("Alpha", DoubleValue(1.0));
                pc->SetAttribute("PoNominalPusch",
                                 IntegerValue(static_cast<int16_t>(m_ulP0Applied)));
                pc->SetAttribute("PoUePusch", IntegerValue(0));
                phy->SetAttribute("EnableUplinkPowerControl", BooleanValue(true));
            }
        }
        NS_LOG_INFO("NtnRealStackHelper: UL power control (TS 38.213 7.1.1) P_CMAX "
                    << m_ueTxDbm << " dBm, alpha 1, P_O_NOMINAL_PUSCH " << m_ulP0Applied
                    << " dBm (target " << m_ulTargetSnrDb << " dB over " << noise180kDbm
                    << " dBm/180 kHz, receive beam " << rxBeamOverRsBeamDb
                    << " dB over the RS beam); PHR 20/10 ms, 1 dB; PHR PRB limit on");
    }
    m_nr->UpdateDeviceConfigs(m_enbDevs);
    m_nr->UpdateDeviceConfigs(m_ueDevs);
    // Wave 11: frequency reuse among the cells. After the device configuration,
    // which gives each PHY its RB count and each MAC its RBG count, and after
    // the per-BWP powers above, which it scales to the cell's sub-band.
    ApplyFrequencyReuse();

    // ---- Enabler A: instantiate + cross-wire the A3-RSRP handover algorithm.
    // This is the wiring the vendored NrHelper omits (see the note above). It
    // must run AFTER InstallGnbDevice -- the RRC now knows its component-carrier
    // count, which AddUeMeasReportConfig needs -- and BEFORE AttachToClosestGnb,
    // so the EVENT_A3 report config that the algorithm's DoInitialize() registers
    // on the gNB RRC is delivered to the UE in its connection reconfiguration.
    // Without this, SetHandoverAlgorithmType alone leaves the RRC on the no-op
    // SAP and the UE never emits neighbour measurement reports (handover count
    // stays 0 regardless of geometry).
    if (m_handover && m_hoAutonomous && m_gnb.GetN() >= 2)
    {
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
            Ptr<NrGnbRrc> rrc = gnbDev->GetRrc();
            Ptr<NrA3RsrpHandoverAlgorithm> algo = CreateObject<NrA3RsrpHandoverAlgorithm>();
            algo->SetAttribute("Hysteresis", DoubleValue(m_hoHystDb));
            algo->SetAttribute("TimeToTrigger", TimeValue(m_hoTtt));
            // Cross-wire the handover-management SAP (RRC <-> algorithm), exactly
            // as LteHelper::InstallSingleEnbDevice does for the LTE stack.
            rrc->SetNrHandoverManagementSapProvider(algo->GetNrHandoverManagementSapProvider());
            algo->SetNrHandoverManagementSapUser(rrc->GetNrHandoverManagementSapUser());
            // Initialize() -> DoInitialize() registers the EVENT_A3 RSRP report
            // config on this gNB's RRC now (before any UE connects).
            algo->Initialize();
            m_hoAlgos.push_back(algo);
        }
    }

    // C2: measurement plane without the A3 decision-maker. A scenario that owns
    // every handover decision (SetAutonomousHandover(false)) still needs the
    // UE's measurement reports, so configure a TS 38.331 5.5.4.1 PERIODICAL
    // report (RSRP and RSRQ) on every gNB before the UEs attach; the RRC then
    // ships it in the connection reconfiguration (nr patch 08 implements the
    // periodical trigger in NrUeRrc).
    if (!m_periodicMeasExplicit && m_handover && !m_hoAutonomous)
    {
        m_periodicMeas = true;
    }
    if (m_periodicMeas)
    {
        // nr patch 08: real SS-RSRQ (TS 38.215 5.1.3) in the reports.
        BooleanValue prevRsrq;
        GlobalValue::GetValueByName("NrUeRsrqFromRssi", prevRsrq);
        m_prevRsrqFromRssi = prevRsrq.Get() ? 1 : 0;
        GlobalValue::Bind("NrUeRsrqFromRssi", BooleanValue(true));
        NrRrcSap::ReportConfigEutra rc;
        rc.triggerType = NrRrcSap::ReportConfigEutra::PERIODICAL;
        rc.purpose = NrRrcSap::ReportConfigEutra::REPORT_STRONGEST_CELLS;
        rc.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
        rc.reportQuantity = NrRrcSap::ReportConfigEutra::BOTH;
        const int64_t ms = m_periodicMeasInterval.GetMilliSeconds();
        rc.reportInterval = (ms == 120)    ? NrRrcSap::ReportConfigEutra::MS120
                            : (ms == 240)  ? NrRrcSap::ReportConfigEutra::MS240
                            : (ms == 640)  ? NrRrcSap::ReportConfigEutra::MS640
                            : (ms == 1024) ? NrRrcSap::ReportConfigEutra::MS1024
                            : (ms == 2048) ? NrRrcSap::ReportConfigEutra::MS2048
                            : (ms == 5120) ? NrRrcSap::ReportConfigEutra::MS5120
                            : (ms == 10240) ? NrRrcSap::ReportConfigEutra::MS10240
                                            : NrRrcSap::ReportConfigEutra::MS480;
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
            gnbDev->GetRrc()->AddUeMeasReportConfig(rc);
        }
    }

    // Enabler D: native 5G-LENA stat calculators (per-DRB PDCP/RLC
    // throughput+delay, per-slot MAC MCS/PRB, PHY RxPacketTrace) written under
    // the output dir. The measured MCS/rank/PRB the helper exposes are captured
    // separately in DlRxTraceNr(); EnableTraces() adds the authoritative file
    // dump an O-RAN KPM / observability sink can also read.
    if (m_nrNativeTraces)
    {
        // Point 5G-LENA's stat calculators at the run's output directory. The
        // comment above has always said these files land under the output dir;
        // NrPhyRxTrace::m_resultsFolder is a static that defaults to empty, so
        // without this they were written to the process working directory
        // instead, which put UlPathlossTrace.txt and its siblings in the source
        // tree whenever an example was launched from the repo root.
        std::string folder = m_outputDir.empty() ? std::string("./") : m_outputDir;
        if (folder.back() != '/')
        {
            folder += '/';
        }
        m_nr->GetPhyRxTrace()->SetResultsFolder(folder);
        m_nr->EnableTraces();
    }

    // NOTE: the mmwave NTN RLC-RRC slant-timer relaxation is mmwave-attribute
    // specific; nr runs RLC UM with a large buffer here. The NR HARQ process
    // pool IS now stretched to the slant (ConfigureNtnHarqProfileNr, above)
    // when SetNtnHarqProfile(true). NR K_offset timing remains a follow-on.

    // ---- Remote host behind the core (carries the LEO feeder+core latency) ----
    Ptr<Node> pgw = m_nrEpc->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    m_remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_backhaulDelay));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, m_remoteHost);
    m_backhaulCh = internetDevices.Get(0)->GetChannel();
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    m_remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(m_remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(m_ue);
    Ipv4InterfaceContainer ueIpIface = m_nrEpc->AssignUeIpv4Address(NetDeviceContainer(m_ueDevs));
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(m_ue.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(m_nrEpc->GetUeDefaultGatewayAddress(), 1);
        m_ueAddrs.push_back(ueIpIface.GetAddress(u));
    }

    m_nr->AttachToClosestGnb(m_ueDevs, m_enbDevs);
    ApplyRaWindowOffsets();
    if (m_nrBeamforming)
    {
        // Record the (cell, IMSI) pairs AttachToGnb registered, then keep each
        // UE's beam on its serving cell after every beamforming pass (offset by
        // 1 us so it runs right after IdealBeamformingHelper::Run).
        for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
        {
            Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
            if (ue && ue->GetTargetGnb())
            {
                m_nrBfPairs.insert({ue->GetTargetGnb()->GetCellId(), ue->GetImsi()});
            }
        }
        TimeValue period;
        m_nrBeamforming->GetAttribute("BeamformingPeriodicity", period);
        if (period.Get().IsStrictlyPositive())
        {
            Simulator::Schedule(period.Get() + MicroSeconds(1),
                                &NtnRealStackHelper::NrReassertUeBeams,
                                this);
        }
    }

    // ---- Enabler A: X2 interfaces + handover-completion trace ------------
    // With >=2 gNBs (e.g. several satellites in view) the A3-RSRP algorithm set
    // above now moves a UE to a real neighbour cell on measured RSRP, over the
    // X2 the following call stands up. GetHandoverCount() reports completions.
    if (m_handover && m_gnb.GetN() >= 2)
    {
        // GAP S8 FIX: the X2 between two satellites is NOT a zero-delay wire.
        // NrNoBackhaulEpcHelper defaults X2LinkDelay to 0 s, so handover
        // preparation (HANDOVER REQUEST/ACK, SN status transfer) completed
        // instantaneously between orbiting gNBs and every handover-interruption
        // figure was structurally optimistic. Derive the one-way delay from the
        // real inter-satellite geometry: either a direct ISL (range/c) or, for a
        // transparent payload, the ground loop via both feeder links.
        // TR 38.821 §8.3 counts this leg in the HO interruption budget.
        //
        // NT-01 FIX (2026-08-24): this used to call
        //   Config::SetDefault("ns3::NrNoBackhaulEpcHelper::X2LinkDelay", ...)
        // which is a no-op here. X2LinkDelay is a *member* attribute
        // (MakeTimeAccessor(&NrNoBackhaulEpcHelper::m_x2LinkDelay)) read once
        // during ConstructSelf, and m_nrEpc was already constructed several
        // hundred lines above. AddX2Interface then read the stale member, so
        // the log below printed the geometric delay while the wire stayed at
        // 0 s and every handover still prepared instantaneously. Setting the
        // attribute on the live object is what actually reaches the channel.
        const Time x2Delay = ComputeX2LinkDelay();
        NS_ASSERT_MSG(m_nrEpc, "X2 setup requires the nr EPC helper");
        m_nrEpc->SetAttribute("X2LinkDelay", TimeValue(x2Delay));
        NS_LOG_INFO("NtnRealStackHelper X2 (inter-satellite) one-way delay: "
                    << x2Delay.GetMilliSeconds() << " ms");
        m_nr->AddX2Interface(m_gnb);
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                        MakeCallback(&NtnRealStackHelper::NrHandoverEndOk, this));
        if (m_hoRobustMcsApplied)
        {
            Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/NewUeContext",
                            MakeCallback(&NtnRealStackHelper::NrNewUeContext, this));
        }
        // CHO-6: neighbour RSRP from the UE's own RRC measurement reports.
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/RecvMeasurementReport",
                        MakeCallback(&NtnRealStackHelper::NrMeasurementReport, this));
    }
    else if (m_periodicMeas)
    {
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/RecvMeasurementReport",
                        MakeCallback(&NtnRealStackHelper::NrMeasurementReport, this));
    }
    // C2/C6: initial serving cell of each UE (for handover records), and the
    // UE RRC events after which DRB PDCP entities exist (for RLC SDU delay).
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
        if (ue && ue->GetTargetGnb())
        {
            m_lastCellOfImsi[ue->GetImsi()] = ue->GetTargetGnb()->GetCellId();
        }
    }
    Config::ConnectFailSafe("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionReconfiguration",
                            MakeCallback(&NtnRealStackHelper::NrUeRrcConnTrace, this));
    Config::ConnectFailSafe("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                            MakeCallback(&NtnRealStackHelper::NrUeRrcConnTrace, this));

    // ---- Measured-KPI PHY sink: DL SINR/TBLER from the UE NrSpectrumPhy ----
    // Feeds the SAME accumulators as the mmwave path via AccumulateDl(). Connect
    // the trace on EVERY BWP the UE carries (nBwp) so per-slice (per-BWP) TBs on
    // BWPs 1..N-1 are measured too — not only the primary BWP 0.
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < m_nrBwpTotal; ++b)
        {
            Ptr<NrSpectrumPhy> sp = m_nr->GetUePhy(m_ueDevs.Get(i), b)->GetSpectrumPhy();
            sp->TraceConnectWithoutContext("RxPacketTraceUe",
                                           MakeCallback(&NtnRealStackHelper::DlRxTraceNr, this));
        }
    }
    {
        StringValue ueStatsPath;
        GlobalValue::GetValueByName("NtnRealStackUeStatsTrace", ueStatsPath);
        if (!ueStatsPath.Get().empty())
        {
            EnableUeStatsTrace(ueStatsPath.Get(), Seconds(1));
        }
        DoubleValue kpiPeriod;
        GlobalValue::GetValueByName("NtnRealStackUeKpiPeriod", kpiPeriod);
        if (kpiPeriod.Get() > 0.0)
        {
            EnableUeKpiDataset(Seconds(kpiPeriod.Get()));
        }
    }

    // S2: when the air interface could not take the slant (multi-UE without TA),
    // seed the backhaul with it so the measured user-plane OWD is still correct
    // and the OWD-floor health gate stays meaningful. Mirrors the mmwave path.
    if (!m_airIfaceDelayActive && m_backhaulCh)
    {
        // NT-04: and keep it tracking the geometry for the rest of the run.
        Simulator::Schedule(m_backhaulRefresh, &NtnRealStackHelper::RefreshBackhaulFold, this);
        const Time service = ComputeServiceLinkDelay();
        if (service > Seconds(0))
        {
            m_backhaulCh->SetAttribute("Delay", TimeValue(m_backhaulDelay + service));
            NS_LOG_INFO("nr backend: folded service-link delay " << service.GetMilliSeconds()
                                                                 << " ms into the backhaul leg");
        }
    }

    if (m_ssbCellBeam)
    {
        // C2: cell (SSB) beam per gNB, re-steered as the satellites move.
        if (m_prevRsrpLinear < 0) // Wave 11: reuse may have saved it already
        {
            BooleanValue prev;
            GlobalValue::GetValueByName("NrUeRsrpLinearAverage", prev);
            m_prevRsrpLinear = prev.Get() ? 1 : 0;
        }
        GlobalValue::Bind("NrUeRsrpLinearAverage", BooleanValue(true));
        // Wave 11: Earth-moving cells sweep their SSB over a beam grid.
        BuildSsbBeamGrid();
        RefreshCellBeams();
    }

    NS_LOG_INFO("NtnRealStackHelper nr backend: " << m_enbDevs.GetN() << " gNB, " << m_ueDevs.GetN()
                                                  << " UE, fc=" << m_freqHz / 1e9 << " GHz, BW="
                                                  << m_bwHz / 1e6 << " MHz, numerology="
                                                  << m_numerology
                                                  << ", air-interface delay="
                                                  << (m_airIfaceDelayActive ? "REAL" : "on-backhaul"));
}

Ptr<PropagationLossModel>
NtnRealStackHelper::GetBaseLossHead() const
{
    return m_nrBaseLossPerBwp.empty() ? nullptr : m_nrBaseLossPerBwp[0];
}

std::vector<std::string>
NtnRealStackHelper::GetExtraPropagationLossChain() const
{
    std::vector<std::string> names;
    if (!m_built || m_nrBaseLossPerBwp.empty() || !m_nrBaseLossPerBwp[0])
    {
        return names;
    }
    // The head is the Friis loss the backend installed; report only what was
    // chained after it, which is exactly what AddExtraPropagationLoss appends.
    for (Ptr<PropagationLossModel> m = m_nrBaseLossPerBwp[0]->GetNext(); m; m = m->GetNext())
    {
        names.push_back(m->GetInstanceTypeId().GetName());
    }
    return names;
}

void
NtnRealStackHelper::AddExtraPropagationLoss(Ptr<PropagationLossModel> loss)
{
    NS_ABORT_MSG_IF(!m_built, "AddExtraPropagationLoss before Build()");
    if (!loss)
    {
        return;
    }
    // Chain after the built-in Friis loss head on EVERY BWP. Both backends fill
    // m_nrBaseLossPerBwp (mmwave: the single CC-0 head; nr: one Friis per BWP),
    // so this is radio-agnostic.
    //
    // GAP S5 FIX: previously this appended only to m_nrBaseLoss (BWP 0), so with
    // SetSlices() the URLLC/mMTC BWPs saw pure Friis — no atmosphere, no
    // scintillation, no shadow fading, no beam roll-off — which biased the very
    // per-slice SINR comparison slicing exists to produce. The loss instance is
    // shared across chains deliberately: stateful models (e.g. correlated
    // shadowing keyed by node pair) SHOULD see all BWPs of the same link.
    NS_ABORT_MSG_IF(m_nrBaseLossPerBwp.empty(),
                    "no base propagation loss model on the radio channel");
    for (auto& head : m_nrBaseLossPerBwp)
    {
        if (!head)
        {
            continue;
        }
        Ptr<PropagationLossModel> tail = head;
        while (tail->GetNext())
        {
            if (tail->GetNext() == loss)
            {
                break; // already chained here
            }
            tail = tail->GetNext();
        }
        if (tail->GetNext() != loss)
        {
            tail->SetNext(loss);
        }
    }
}

// ---- Enabler A: handover config -----------------------------------------
void
NtnRealStackHelper::SetHandover(bool enable, double hysteresisDb, Time ttt)
{
    NS_ABORT_MSG_IF(m_built, "SetHandover must be called before Build()");
    m_handover = enable;
    m_hoHystDb = hysteresisDb;
    m_hoTtt = ttt;
}

void
NtnRealStackHelper::NrNewUeContext(std::string /*ctx*/, uint16_t cellId, uint16_t rnti)
{
    // The target RRC stores the IMSI of a handover UE right after it fires
    // NewUeContext, so read the context once this event is done.
    Simulator::ScheduleNow(&NtnRealStackHelper::ApplyHoRobustMcs, this, cellId, rnti);
}

void
NtnRealStackHelper::ApplyHoRobustMcs(uint16_t cellId, uint16_t rnti)
{
    // Wave 11b: a UE admitted by a handover has sent this cell no CSI yet. Its
    // first MCS comes from the SS-RSRP it reported for this cell before the
    // handover (the A3/CHO report the decision used), not from the source's CQI.
    Ptr<NrGnbNetDevice> gnb;
    uint32_t gnbIdx = 0;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> g = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
        if (g && g->GetCellId() == cellId)
        {
            gnb = g;
            gnbIdx = i;
            break;
        }
    }
    if (!gnb || !gnb->GetRrc()->HasUeManager(rnti))
    {
        return;
    }
    Ptr<NrUeManager> um = gnb->GetRrc()->GetUeManager(rnti);
    if (um->GetState() != NrUeManager::HANDOVER_JOINING)
    {
        return; // initial access: nr's StartingMcsDl
    }
    const int32_t ue = UeIndexOfImsi(um->GetImsi());
    if (ue < 0)
    {
        return;
    }
    const double rsrpT = GetUeNeighbourRsrpDbm(static_cast<uint32_t>(ue), cellId);
    if (!std::isfinite(rsrpT))
    {
        return; // nothing reported for this cell: StartingMcsDl
    }
    // Co-channel cells: every other reported cell on the same reuse color.
    const uint8_t myColor = gnbIdx < m_cellColor.size() ? m_cellColor[gnbIdx] : 0;
    double iMw = 0.0;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> g = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
        if (!g || i == gnbIdx)
        {
            continue;
        }
        const uint8_t c = i < m_cellColor.size() ? m_cellColor[i] : 0;
        if (m_frfApplied > 1 && c != myColor)
        {
            continue;
        }
        const double r = GetUeNeighbourRsrpDbm(static_cast<uint32_t>(ue), g->GetCellId());
        if (std::isfinite(r))
        {
            iMw += std::pow(10.0, r / 10.0);
        }
    }
    // Thermal noise per RE (SS-RSRP is a per-RE power, TS 38.215 5.1.1).
    Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(static_cast<uint32_t>(ue)));
    DoubleValue nf(5.0);
    if (ueDev)
    {
        ueDev->GetPhy(0)->GetAttribute("NoiseFigure", nf);
    }
    const double nDbm = -174.0 + nf.Get() + 10.0 * std::log10(15e3 * std::pow(2.0, m_numerology));
    const double sinr = std::pow(10.0, rsrpT / 10.0) / (iMw + std::pow(10.0, nDbm / 10.0));
    // nr's AMC Shannon gap for the NR error models (NrAmc::GetBer, BER 1e-5).
    const double gap = -std::log(5.0 * 1e-5) / 1.5;
    const double se = std::log2(1.0 + sinr / gap);
    const uint8_t nBwp = GetNrBwpCount();
    uint8_t mcs = 0;
    bool set = false;
    for (uint8_t b = 0; b < nBwp && b < gnb->GetCcMapSize(); ++b)
    {
        Ptr<NrMacSchedulerNs3> sched = DynamicCast<NrMacSchedulerNs3>(gnb->GetScheduler(b));
        if (!sched || !sched->GetDlAmc())
        {
            continue;
        }
        mcs = sched->GetDlAmc()->GetMcsFromSpectralEfficiency(se);
        set = sched->SetUeDlMcsUntilCsi(rnti, mcs) || set;
    }
    if (set)
    {
        ++m_hoRobustMcsCount;
        m_hoRobustMcsList.push_back(mcs);
    }
}

void
NtnRealStackHelper::NrHandoverEndOk(std::string /*ctx*/,
                                    uint64_t imsi,
                                    uint16_t cellId,
                                    uint16_t /*rnti*/)
{
    ++m_hoCount;
    {
        // C2: measured handover record (source = the cell the UE last
        // completed on, target = this cell).
        const int32_t ue = UeIndexOfImsi(imsi);
        auto it = m_lastCellOfImsi.find(imsi);
        const uint16_t src = (it == m_lastCellOfImsi.end()) ? 0 : it->second;
        m_hoRecords.push_back({Simulator::Now(), static_cast<uint32_t>(std::max(ue, 0)), src, cellId});
        m_lastCellOfImsi[imsi] = cellId;
        if (m_autoScene && ue >= 0 && static_cast<uint32_t>(ue) < m_ue.GetN())
        {
            // Source = the node this UE was last drawn on (not cell 0 -> node 0, which
            // collides with the first satellite's node id).
            const uint32_t ueNode = m_ue.Get(ue)->GetId();
            const uint32_t to = GnbNodeIdOfCell(cellId);
            auto sv = m_autoServing.find(ueNode);
            const uint32_t from = sv != m_autoServing.end() ? sv->second : GnbNodeIdOfCell(src);
            if (from != to)
            {
                m_autoScene->OnHandover(ueNode, from, to);
            }
            m_autoServing[ueNode] = to;
        }
    }
    // 5G-LENA registers an ideal-beamforming task only at attach
    // (NrHelper::AttachToGnb), so after an X2 handover the TARGET gNB had no
    // beam toward the UE and served it with an unsteered vector: its RSRP
    // collapsed and A3 sent the UE straight back (ping-pong, measured 8 to 31
    // handovers on one pass). Register the (target, UE) pair once and point the
    // UE's receive beam at the new serving cell.
    if (m_backend != RadioBackend::Nr || !m_nrBeamforming)
    {
        return;
    }
    Ptr<NrGnbNetDevice> gnb;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> g = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
        if (g && g->GetCellId() == cellId)
        {
            gnb = g;
            break;
        }
    }
    Ptr<NrUeNetDevice> ue;
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        Ptr<NrUeNetDevice> d = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
        if (d && d->GetImsi() == imsi)
        {
            ue = d;
            break;
        }
    }
    if (!gnb || !ue)
    {
        return;
    }
    if (m_nrBfPairs.insert({cellId, imsi}).second)
    {
        m_nrBeamforming->AddBeamformingTask(gnb, ue); // also points the UE beam at gnb
        if (m_hoTargetBeamApplied)
        {
            // Wave 10: the beamforming task above stores the target's
            // UE-directed vector, but NrGnbPhy picks the PDSCH/PUSCH beam by
            // looking the scheduled RNTI up among the UE devices registered
            // with NrGnbPhy::RegisterUe, which only NrHelper::AttachToGnb
            // calls. After a reconfiguration with sync (TS 38.331 5.3.5.5.2)
            // the target found no device, fell back to its predefined (SSB
            // cell) beam or quasi-omni, and sent the UE's data without the
            // array gain the source had given it (measured: data path loss
            // 161 dB on the target vs 134 dB on the source, PDSCH SINR -8 dB).
            // TS 38.214 5.1.6.1 / TS 38.300 10.9: after the handover the
            // target serves the UE on the beam it acquired the UE on, i.e. a
            // UE-specific beam, as the source did.
            for (std::size_t cc = 0; cc < gnb->GetCcMapSize(); ++cc)
            {
                gnb->GetPhy(cc)->RegisterUe(imsi, ue);
            }
            ++m_hoTargetBeamRegistrations;
        }
    }
    else
    {
        for (std::size_t cc = 0; cc < ue->GetCcMapSize(); ++cc)
        {
            ue->GetPhy(cc)->GetSpectrumPhy()->GetBeamManager()->ChangeBeamformingVector(gnb);
        }
    }
}

void
NtnRealStackHelper::NrReassertUeBeams()
{
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
        if (!ue)
        {
            continue;
        }
        const uint16_t cellId = GetUeServingCellId(u);
        if (m_nrBfPairs.find({cellId, ue->GetImsi()}) == m_nrBfPairs.end())
        {
            continue; // no stored beam for this pair yet
        }
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            Ptr<NrGnbNetDevice> g = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(i));
            if (g && g->GetCellId() == cellId)
            {
                for (std::size_t cc = 0; cc < ue->GetCcMapSize(); ++cc)
                {
                    ue->GetPhy(cc)->GetSpectrumPhy()->GetBeamManager()->ChangeBeamformingVector(g);
                }
                break;
            }
        }
    }
    TimeValue period;
    m_nrBeamforming->GetAttribute("BeamformingPeriodicity", period);
    if (period.Get().IsStrictlyPositive())
    {
        Simulator::Schedule(period.Get(), &NtnRealStackHelper::NrReassertUeBeams, this);
    }
}

// ---- Enabler B: MIMO + spectrum-level channel plugin --------------------
void
NtnRealStackHelper::SetMimo(uint8_t gnbRows,
                            uint8_t gnbCols,
                            uint8_t ueRows,
                            uint8_t ueCols,
                            uint8_t rankLimit)
{
    NS_ABORT_MSG_IF(m_built, "SetMimo must be called before Build()");
    m_mimo = true;
    m_gnbRows = gnbRows;
    m_gnbCols = gnbCols;
    m_ueRows = ueRows;
    m_ueCols = ueCols;
    m_mimoRank = rankLimit;
}

void
NtnRealStackHelper::AddSpectrumChannelLoss(Ptr<SpectrumPropagationLossModel> loss)
{
    NS_ABORT_MSG_IF(!m_built, "AddSpectrumChannelLoss before Build()");
    if (!loss)
    {
        return;
    }
    NS_ABORT_MSG_IF(m_backend != RadioBackend::Nr,
                    "AddSpectrumChannelLoss is nr-backend only (SetRadioBackend(Nr))");
    NS_ABORT_MSG_IF(m_nrBwpChannels.empty(), "no NR BWP spectrum channels to plug into");

    // GAP S1 FIX: compose, do NOT replace.
    //
    // MultiModelSpectrumChannel applies spectrum-loss ELSE-IF phased-array-loss.
    // 5G-LENA installs the 3GPP channel (array gain + fading + the MIMO
    // spectrumChannelMatrix) as the PHASED-ARRAY model, so the old code path
    // here — ch->AddSpectrumPropagationLossModel(loss) — silently switched the
    // entire 3GPP spatial channel OFF the moment a module plugged in a THz /
    // Sionna / RIS transfer function. Enabler B's two halves (seam and MIMO)
    // destroyed each other.
    //
    // Instead wrap the plugin in a phased-array composite that runs the 3GPP
    // model first and then multiplies the plugin's per-RB gain into the result
    // (rescaling the MIMO matrix to match), and install THAT on the
    // phased-array slot so both survive.
    for (const auto& ch : m_nrBwpChannels)
    {
        if (!ch)
        {
            continue;
        }
        Ptr<PhasedArraySpectrumPropagationLossModel> installed =
            ch->GetPhasedArraySpectrumPropagationLossModel();
        Ptr<NtnSpectrumSeamModel> seam = DynamicCast<NtnSpectrumSeamModel>(installed);
        if (!seam)
        {
            // First plugin on this channel: build the composite around whatever
            // 3GPP model 5G-LENA already installed and take over the slot.
            seam = CreateObject<NtnSpectrumSeamModel>();
            seam->SetInnerModel(installed); // may be null (then the composite is pass-through)
            ch->AddPhasedArraySpectrumPropagationLossModel(seam);
            // Wave 2 core: cut the chain link the channel just made. ns-3.43
            // SpectrumChannel::AddPhasedArraySpectrumPropagationLossModel sets
            // the previously installed model (the 3GPP one, already the seam's
            // INNER model) as the seam's m_next, and
            // PhasedArraySpectrumPropagationLossModel::CalcRxPowerSpectralDensity
            // then REPLACES the seam's result with m_next's, computed from the
            // original params: every plugin was evaluated and discarded and the
            // 3GPP model ran twice. The seam already runs the inner model, so it
            // must be the only link. (Wave 3: the interim per-example unchain
            // work-arounds in thz-ntn, ntn-sionna and ntn-v2x were removed.)
            seam->SetNext(nullptr);
            if (!installed)
            {
                NS_LOG_WARN("no 3GPP phased-array model on this BWP channel; the spectrum "
                            "seam will carry the plugin alone (no array gain / fading)");
            }
        }
        seam->AddPlugin(loss);
    }
}

// ---- Enabler D: NR HARQ process pool stretched to the slant RTT ----------
void
NtnRealStackHelper::ConfigureNtnHarqProfileNr()
{
    // Only when the caller opted into the NTN HARQ profile. NR runs HARQ by
    // default; the NTN hazard is recycling a stop-and-wait process before its
    // feedback returns over the LEO slant. Size the pool to cover the RTT.
    // Wave 5: on by default for the nr backend (GlobalValue NtnRealStackNrHarqProfile), since
    // every nr run here is a satellite link; SetNtnHarqProfile(true) still forces it.
    BooleanValue byDefault;
    GlobalValue::GetValueByName("NtnRealStackNrHarqProfile", byDefault);
    if (!m_ntnHarqProfile && !byDefault.Get())
    {
        return;
    }
    constexpr double kC = 299792458.0;
    const double slantM = WorstCaseSlantM();       // metres (>=600 km fallback)
    const double rttS = 2.0 * slantM / kC;         // round trip
    const double slotS = 1.0e-3 / std::pow(2.0, m_numerology); // NR slot duration
    constexpr double kRounds = 4.0;                // initial TX + 3 retx in flight
    double n = std::ceil(rttS / slotS) + kRounds;
    // TS 38.331 PDSCH-ServingCellConfig nrofHARQ-ProcessesForPDSCH-v1700 allows up to n32 (Rel-17
    // NTN, 5-bit HARQ process ID in DCI per TS 38.212); 16 is the legacy pool. Beyond 32 the
    // standard relies on HARQ feedback disabling, so the pool is capped at 32, not stretched.
    if (n > 32.0)
    {
        NS_LOG_WARN("NtnRealStackHelper nr HARQ: RTT needs " << n
                    << " processes; capped at the TS 38.331 maximum of 32");
    }
    n = std::max(16.0, std::min(32.0, n));
    m_nr->SetGnbMacAttribute("NumHarqProcess",
                             UintegerValue(static_cast<uint32_t>(n)));
    // Wave 7A: the UE MAC needs the same pool (TS 38.331 nrofHARQ-ProcessesForPUSCH-r17,
    // up to 32). The gNB issues UL grants on process IDs up to n - 1, and the UE kept
    // nr's 16-entry UL HARQ buffer, so any UL grant on process 16 or above threw
    // std::out_of_range in NrUeMac::SendNewData (found with a sustained uplink at
    // 30 deg elevation, where the pool is 18).
    m_nr->SetUeMacAttribute("NumHarqProcess", UintegerValue(static_cast<uint32_t>(n)));
    NS_LOG_INFO("NtnRealStackHelper nr HARQ profile: slant=" << slantM / 1e3 << " km, RTT="
                                                             << rttS * 1e3 << " ms -> NumHarqProcess="
                                                             << static_cast<uint32_t>(n));
}

// ---- Enabler D/C: measured MCS / rank / PRB / per-slice accessors --------
double
NtnRealStackHelper::GetMeanDlMcs() const
{
    return (m_dlGlobal.n > 0) ? m_dlGlobal.sumMcs / m_dlGlobal.n : NAN;
}

double
NtnRealStackHelper::GetMeanDlRank() const
{
    return (m_dlGlobal.n > 0) ? m_dlGlobal.sumRank / m_dlGlobal.n : NAN;
}

double
NtnRealStackHelper::GetMeanPrbUtil() const
{
    return (m_dlGlobal.n > 0) ? m_dlGlobal.sumRbFrac / m_dlGlobal.n : NAN;
}

double
NtnRealStackHelper::GetCellMeanMcs(uint16_t cellId) const
{
    auto it = m_dlPerCell.find(cellId);
    if (it == m_dlPerCell.end() || it->second.n == 0)
    {
        return NAN;
    }
    return it->second.sumMcs / it->second.n;
}

double
NtnRealStackHelper::GetBwpMeanSinrDb(uint8_t bwpId) const
{
    auto it = m_dlPerBwp.find(bwpId);
    if (it == m_dlPerBwp.end() || it->second.n == 0)
    {
        return NAN;
    }
    return it->second.sumSinrDb / it->second.n;
}

uint64_t
NtnRealStackHelper::GetBwpRxTb(uint8_t bwpId) const
{
    auto it = m_dlPerBwp.find(bwpId);
    return (it == m_dlPerBwp.end()) ? 0 : it->second.n;
}

uint64_t
NtnRealStackHelper::GetUeRxPackets(uint32_t ueIndex) const
{
    // P3 (M5): measured delivered-packet COUNT for a UE, summed over its flows.
    // Use this instead of GetUeRxBytes()/1400 — that estimate is wrong for any
    // non-1400 B packet (128 B mMTC, 227 B URLLC).
    uint64_t pkts = 0;
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        if (i >= m_dlSinkUe.size() || m_dlSinkUe[i] != ueIndex)
        {
            continue;
        }
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (!sink)
        {
            continue;
        }
        for (const auto& [key, fs] : sink->GetFlowStats())
        {
            pkts += fs.rxPackets;
        }
    }
    for (const auto& f : m_extFlows)
    {
        pkts += (f.ue == ueIndex) ? f.rxPkts : 0;
    }
    return pkts;
}

uint64_t
NtnRealStackHelper::GetUeLostPackets(uint32_t ueIndex) const
{
    // P3 (L3): measured sequence-gap losses for a UE (expected-from-highest-seq
    // minus received), summed over its flows. This is a real loss count, not
    // delivered*TBLER/(1-TBLER) from a single last-TB sample.
    uint64_t lost = 0;
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        if (i >= m_dlSinkUe.size() || m_dlSinkUe[i] != ueIndex)
        {
            continue;
        }
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (!sink)
        {
            continue;
        }
        for (const auto& [key, fs] : sink->GetFlowStats())
        {
            lost += fs.LostPackets();
        }
    }
    // External flows: sequence-gap losses where the packets carry a
    // SeqTsSizeHeader (expected from the highest sequence minus received).
    for (const auto& f : m_extFlows)
    {
        if (f.ue == ueIndex && f.anySeq && f.highestSeq + 1 > f.seqRx)
        {
            lost += f.highestSeq + 1 - f.seqRx;
        }
    }
    return lost;
}

std::vector<std::pair<uint32_t, uint64_t>>
NtnRealStackHelper::GetSliceDelayHistogram(uint8_t fiveQi) const
{
    std::vector<uint64_t> pooled(NtnOranSink::FlowStats::kDelayBins, 0u);
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (!sink)
        {
            continue;
        }
        for (const auto& [key, fs] : sink->GetFlowStats())
        {
            if (fs.fiveQi != fiveQi || fs.delayHistMs.empty())
            {
                continue;
            }
            for (uint32_t b = 0; b < NtnOranSink::FlowStats::kDelayBins; ++b)
            {
                pooled[b] += fs.delayHistMs[b];
            }
        }
    }
    std::vector<std::pair<uint32_t, uint64_t>> out;
    for (uint32_t b = 0; b < NtnOranSink::FlowStats::kDelayBins; ++b)
    {
        if (pooled[b] > 0)
        {
            out.emplace_back(b, pooled[b]);
        }
    }
    return out;
}

NtnRealStackHelper::SliceMeasuredStats
NtnRealStackHelper::GetSliceMeasuredStats(uint8_t fiveQi) const
{
    // P3 (L1-L3): aggregate the MEASURED in-band flow stats for this 5QI. Every
    // number here is from the real data plane — per-packet TX timestamps and
    // sequence numbers carried inside the GTP tunnel — so a slice example never
    // needs to re-derive latency from geometry or losses from a TBLER sample.
    SliceMeasuredStats out;
    double sumDelayMs = 0.0;
    double thrMbps = 0.0;
    // SLICE-1: pool the per-flow delay histograms so the percentiles describe
    // the slice, not one flow within it.
    std::vector<uint64_t> pooledHist(NtnOranSink::FlowStats::kDelayBins, 0u);
    uint64_t pooledOverflow = 0;
    // Percentiles come from the 0.1 ms histogram (TS 28.552 DRB.RlcSduDelayDl
    // is reported in 0.1 ms units); with 1 ms bins a LEO slice whose delays
    // sit inside one millisecond reported p50 = p95 = p99.
    std::vector<uint64_t> pooledFine(NtnOranSink::FlowStats::kFineBins, 0u);
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (!sink)
        {
            continue;
        }
        for (const auto& [key, fs] : sink->GetFlowStats())
        {
            if (fs.fiveQi != fiveQi)
            {
                continue;
            }
            out.flows += 1;
            out.rxPackets += fs.rxPackets;
            out.lostPackets += fs.LostPackets();
            sumDelayMs += fs.sumDelayMs;
            out.maxOwdMs = std::max(out.maxOwdMs, fs.maxDelayMs);
            thrMbps += fs.ThroughputMbps();
            if (!fs.delayHistMs.empty())
            {
                for (uint32_t b = 0; b < NtnOranSink::FlowStats::kDelayBins; ++b)
                {
                    pooledHist[b] += fs.delayHistMs[b];
                }
            }
            for (size_t b = 0; b < fs.delayHistFine.size() && b < pooledFine.size(); ++b)
            {
                pooledFine[b] += fs.delayHistFine[b];
            }
            pooledOverflow += fs.delayOverflow;
        }
    }
    out.meanOwdMs = (out.rxPackets > 0) ? sumDelayMs / out.rxPackets : 0.0;

    // SLICE-1: percentiles from the pooled histogram.
    auto pct = [&](double q) -> double {
        if (out.rxPackets == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const uint64_t target = static_cast<uint64_t>(q * static_cast<double>(out.rxPackets));
        uint64_t cum = 0;
        for (uint32_t b = 0; b < NtnOranSink::FlowStats::kFineBins; ++b)
        {
            cum += pooledFine[b];
            if (cum >= target)
            {
                return (b + 0.5) * NtnOranSink::FlowStats::kFineBinMs;
            }
        }
        // Past the fine range (1 s): continue on the 1 ms bins above it.
        const uint32_t firstCoarse = static_cast<uint32_t>(NtnOranSink::FlowStats::kFineBins *
                                                           NtnOranSink::FlowStats::kFineBinMs);
        for (uint32_t b = firstCoarse; b < NtnOranSink::FlowStats::kDelayBins; ++b)
        {
            cum += pooledHist[b];
            if (cum >= target)
            {
                return b + 0.5;
            }
        }
        return (pooledOverflow > 0) ? out.maxOwdMs
                                    : static_cast<double>(NtnOranSink::FlowStats::kDelayBins);
    };
    out.p50OwdMs = pct(0.50);
    out.p95OwdMs = pct(0.95);
    out.p99OwdMs = pct(0.99);
    const uint64_t expected = out.rxPackets + out.lostPackets;
    out.lossRatio = (expected > 0) ? static_cast<double>(out.lostPackets) / expected : 0.0;
    out.thrMbps = thrMbps;
    return out;
}

void
NtnRealStackHelper::InstallTraffic(TrafficProfile profile, Time start, Time stop)
{
    // NT-12: remember the window so throughput is measured over it.
    m_trafficStart = start;
    m_trafficStop = stop;
    NS_ABORT_MSG_IF(!m_built, "InstallTraffic before Build()");
    if (m_backend == RadioBackend::Nr)
    {
        Simulator::Schedule(start, &NtnRealStackHelper::ConnectUePdcpTraces, this);
    }

    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        // Choose per-UE profile params (NtnOranApplication QoS flows).
        TrafficProfile p = profile;
        if (profile == TrafficProfile::MixedBouquet)
        {
            uint32_t b = u % 3;
            p = (b == 0) ? TrafficProfile::NbIotPeriodic
                         : (b == 1) ? TrafficProfile::EmbbStreaming
                                    : TrafficProfile::UrllcPings;
        }
        NtnOranApplication::Profile oranProfile;
        uint8_t fiveQi;
        switch (p)
        {
        case TrafficProfile::NbIotPeriodic:
            oranProfile = NtnOranApplication::MMTC_PERIODIC;
            fiveQi = 9;
            break;
        case TrafficProfile::UrllcPings:
            oranProfile = NtnOranApplication::URLLC_PERIODIC;
            fiveQi = 82;
            break;
        case TrafficProfile::ConversationalVoice:
            oranProfile = NtnOranApplication::CONVERSATIONAL_VOICE;
            fiveQi = 1;
            break;
        case TrafficProfile::EmbbStreaming:
        default:
            oranProfile = NtnOranApplication::CBR_SATURATING;
            fiveQi = 2;
            break;
        }
        ApplicationContainer flow =
            InstallOranFlow(u, fiveQi, 1, 0x000001, oranProfile, start, stop);

        if (m_uplink)
        {
            uint16_t ulPort = 2000 + u;
            Ptr<NtnOranSink> ulSink = CreateObject<NtnOranSink>();
            ulSink->SetAttribute("Local",
                                 AddressValue(InetSocketAddress(Ipv4Address::GetAny(), ulPort)));
            m_remoteHost->AddApplication(ulSink);
            m_serverApps.Add(ulSink);

            Ptr<NtnOranApplication> ulClient = CreateObject<NtnOranApplication>();
            ulClient->SetRemote(InetSocketAddress(m_remoteHostAddr, ulPort));
            ulClient->SetProfile(NtnOranApplication::MMTC_PERIODIC);
            ulClient->SetAttribute("PacketSize", UintegerValue(256));
            ulClient->SetAttribute("Period", TimeValue(MilliSeconds(16))); // 128 kbps
            ulClient->SetFlowIdentity(9, 1, 0x000001, 1000 + u, 0);
            m_ue.Get(u)->AddApplication(ulClient);
            m_clientApps.Add(ulClient);
        }
    }

    // Pick up the uplink clients (added after each InstallOranFlow call) too.
    if (m_oranMonitor && m_autoAttachMonitor)
    {
        AttachInstalledFlowsToMonitor();
    }

    m_serverApps.Start(Seconds(0.0));
    m_serverApps.Stop(m_simTime);
    m_clientApps.Start(start);
    m_clientApps.Stop(stop);

    // NB: FlowMonitor is intentionally NOT used. Over the LTE/mmwave EPC the GTP
    // tunnel re-encapsulates DL packets and strips the flow byte-tag, so
    // FlowMonitor under-counts DL rx. The NtnOranSink is the authoritative DL
    // measurement (delivery, in-band one-way delay, jitter, seq-gap loss) —
    // its primitives ride INSIDE the GTP tunnel as real payload bytes.

    // Periodic callbacks are scheduled at registration time (see
    // RegisterPeriodicCallback), so registration order vs InstallTraffic does
    // not matter.

    m_wallStartNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
}

ApplicationContainer
NtnRealStackHelper::InstallOranFlow(uint32_t ueIdx,
                                    uint8_t fiveQi,
                                    uint8_t sst,
                                    uint32_t sd,
                                    uint8_t profile,
                                    Time start,
                                    Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "InstallOranFlow before Build()");
    NS_ABORT_MSG_IF(ueIdx >= m_ue.GetN(), "InstallOranFlow: UE index out of range");
    const auto oranProfile = static_cast<NtnOranApplication::Profile>(profile);
    const uint16_t dlPort = m_nextDlPort++;

    Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
    sink->SetAttribute("Local",
                       AddressValue(InetSocketAddress(Ipv4Address::GetAny(), dlPort)));
    m_ue.Get(ueIdx)->AddApplication(sink);
    m_serverApps.Add(sink);
    m_dlSinks.Add(sink);
    m_dlSinkUe.push_back(ueIdx);
    sink->TraceConnectWithoutContext("Rx",
                                     MakeCallback(&NtnRealStackHelper::DlSinkRx, this));
    sink->TraceConnectWithoutContext("Rx",
                                     MakeCallback(&NtnRealStackHelper::DlOranSinkRx, this));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(m_ueAddrs[ueIdx], dlPort));
    client->SetProfile(oranProfile);
    // Cadence/size presets matching the helper's historical OnOff rates, so
    // the 30+ examples already gated on these profiles keep their budgets.
    switch (oranProfile)
    {
    case NtnOranApplication::MMTC_PERIODIC:
        client->SetAttribute("PacketSize", UintegerValue(128));
        client->SetAttribute("Period", TimeValue(MilliSeconds(64))); // ~16 kbps
        break;
    case NtnOranApplication::URLLC_PERIODIC:
        // 5QI 82: MDBV 255 bytes per 10 ms PDB (TS 23.501 Table 5.7.4-1). One
        // packet per period, so the IP PDU must fit: 227 B payload + 28 B
        // IPv4/UDP = 255 B. It was 256 B of payload (284 B PDU), over budget.
        client->SetAttribute("PacketSize", UintegerValue(227));
        client->SetAttribute("Period", TimeValue(MilliSeconds(10))); // ~204 kbps on the wire
        break;
    case NtnOranApplication::CONVERSATIONAL_VOICE:
        client->SetAttribute("PacketSize", UintegerValue(92)); // AMR-WB + RTP-ish
        break;
    case NtnOranApplication::EMBB_VIDEO:
    case NtnOranApplication::POISSON_BACKGROUND:
    case NtnOranApplication::CBR_SATURATING:
    default:
        // Wave 2 core: the rate used to be hard-coded to 5 Mb/s here, so a
        // Config::SetDefault("ns3::NtnOranApplication::DataRate", ...) did
        // nothing. Now SetEmbbRate() wins, else the attribute's (Config)
        // default applies; that default is 5 Mb/s, so nothing changes unless
        // a caller asks.
        if (m_embbRateSet)
        {
            client->SetAttribute("DataRate", DataRateValue(m_embbRate));
        }
        client->SetAttribute("PacketSize", UintegerValue(1400));
        break;
    }
    // srcId is a helper-scoped monotonically increasing flow counter, NOT the
    // recycled DL port number (audit issue 15): port allocation restarts at
    // 1234 per helper/run, so port-derived srcIds could collide across
    // scenarios in any tooling that merges flow keys. dstId stays the UE
    // index. The UDP port allocation itself is unchanged.
    const uint16_t srcId = ++m_flowSeq;
    client->SetFlowIdentity(fiveQi, sst, sd, srcId, /*dstId=*/ueIdx);
    m_remoteHost->AddApplication(client);
    m_clientApps.Add(client);
    client->TraceConnectWithoutContext("Tx",
                                       MakeCallback(&NtnRealStackHelper::DlClientTx, this));
    client->TraceConnectWithoutContext("Tx",
                                       MakeCallback(&NtnRealStackHelper::DlOranClientTx, this));

    // Enabler C: activate a dedicated per-5QI EPS bearer so the QoS scheduler /
    // BWP manager actually differentiate this flow (instead of everything
    // riding one default bearer). Only on the nr backend when slices or the
    // QoS scheduler are configured — otherwise the historical default bearer
    // is kept for zero regression.
    const bool perQosBearers =
        (m_backend == RadioBackend::Nr) &&
        (!m_slices.empty() || m_scheduler == Scheduler::OfdmaQos);
    if (perQosBearers && !QciAttrName(fiveQi).empty())
    {
        NrEpsBearer bearer(static_cast<NrEpsBearer::Qci>(fiveQi));
        Ptr<NrEpcTft> tft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter pf;
        pf.localPortStart = dlPort;
        pf.localPortEnd = dlPort;
        tft->Add(pf);
        m_nr->ActivateDedicatedEpsBearer(m_ueDevs.Get(ueIdx), bearer, tft);
    }

    sink->SetStartTime(Seconds(0.0));
    sink->SetStopTime(m_simTime);
    client->SetStartTime(start);
    client->SetStopTime(stop);

    // Flows installed after EnableAiFlowMonitor() attach automatically.
    if (m_oranMonitor && m_autoAttachMonitor)
    {
        AttachInstalledFlowsToMonitor();
    }

    ApplicationContainer apps;
    apps.Add(client);
    apps.Add(sink);
    return apps;
}

uint16_t
NtnRealStackHelper::GetGnbCellId(uint32_t gnbIndex) const
{
    if (gnbIndex >= m_enbDevs.GetN())
    {
        return 0;
    }
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(gnbIndex));
        return gnb ? gnb->GetCellId() : 0;
    }
    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(m_enbDevs.Get(gnbIndex));
    return enb ? enb->GetCellId() : 0;
}


void
NtnRealStackHelper::RecordHandoverRefusal(const char* reason)
{
    // WF-11: the honesty channel must survive the release build.
    //
    // Every one of these refusal paths used to be an NS_LOG_WARN and nothing
    // else. The shipped build sets NS3_LOG=OFF (and NS3_ASSERT=OFF), so those
    // warnings are compiled out entirely: a decision module could ask for a
    // handover, be refused for a structural reason - wrong backend, no X2, one
    // gNB - and the run would say nothing at all. "Never silently accept" is
    // only true if the refusal is audible, and in the profile users actually
    // build it was not.
    //
    // The counter and the reason are members, so they reach sim_health.csv and
    // an accessor regardless of how the tree was configured. The NS_LOG_WARN is
    // kept for the debug build, where it carries more context.
    ++m_handoverRefusals;
    m_lastHandoverRefusal = reason ? reason : "";
    NS_LOG_WARN("TriggerHandover NOT actuated: " << m_lastHandoverRefusal);
}


bool
NtnRealStackHelper::TriggerHandover(uint32_t ueIndex, uint16_t targetCellId, Time when)
{
    // P1 / gap H2: the actuation bridge. Every failure path below WARNs with the
    // reason — a decision module must never believe it actuated when it did not
    // (that silent-success pattern is exactly what this fix exists to kill).
    if (!m_built)
    {
        RecordHandoverRefusal("before Build()");
        return false;
    }
    if (m_backend != RadioBackend::Nr)
    {
        RecordHandoverRefusal("mmwave backend has no X2 handover request");
        return false;
    }
    if (!m_handover)
    {
        RecordHandoverRefusal("handover never enabled (SetHandover)");
        return false;
    }
    if (m_enbDevs.GetN() < 2)
    {
        RecordHandoverRefusal("fewer than 2 gNBs");
        return false;
    }
    if (ueIndex >= m_ueDevs.GetN())
    {
        RecordHandoverRefusal("UE index out of range");
        return false;
    }
    const uint16_t servingCell = GetUeServingCellId(ueIndex);
    if (servingCell == 0)
    {
        RecordHandoverRefusal("UE not attached yet");
        return false;
    }
    if (servingCell == targetCellId)
    {
        NS_LOG_INFO("TriggerHandover: UE " << ueIndex << " already served by cell " << targetCellId
                                           << " — nothing to do");
        return false;
    }
    // Resolve the source gNB device from the UE's CURRENT serving cell, not from
    // gNB[0]: after an earlier handover the source has moved.
    Ptr<NetDevice> srcDev;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        if (GetGnbCellId(i) == servingCell)
        {
            srcDev = m_enbDevs.Get(i);
            break;
        }
    }
    if (!srcDev)
    {
        NS_LOG_WARN("TriggerHandover: no gNB device for serving cell " << servingCell
                                                                       << " — not actuated");
        RecordHandoverRefusal("no gNB device for the serving cell");
        return false;
    }
    bool targetKnown = false;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        if (GetGnbCellId(i) == targetCellId)
        {
            targetKnown = true;
            break;
        }
    }
    if (!targetKnown)
    {
        NS_LOG_WARN("TriggerHandover: target cell " << targetCellId
                                                    << " is not a gNB in this scenario — not "
                                                       "actuated");
        // Counted like every other refusal (WF-11): NS_LOG_WARN is compiled out
        // of the optimized build, so an xApp or twin naming a cell that does not
        // exist was silently dropped.
        RecordHandoverRefusal("target cell is not a gNB in this scenario");
        return false;
    }

    NS_LOG_INFO("TriggerHandover: UE " << ueIndex << " cell " << servingCell << " -> "
                                       << targetCellId << " in " << when.GetMilliSeconds()
                                       << " ms (X2 one-way "
                                       << ComputeX2LinkDelay().GetMilliSeconds() << " ms)");
    m_nr->HandoverRequest(when, m_ueDevs.Get(ueIndex), srcDev, targetCellId);
    ++m_hoRequested;
    return true;
}

Time
NtnRealStackHelper::ComputeFeederLinkDelay() const
{
    // CVC-14: the satellite-to-gateway leg, from the same live mobility models
    // SetFeederGeometry() already drives the backhaul fold with. Returns zero
    // when the geometry was never wired, so a caller can tell "no feeder
    // modelled" from "a very short one" instead of silently getting 0 ms.
    constexpr double kC = 299792458.0;
    if (!m_feederSat || !m_feederGw)
    {
        return Seconds(0);
    }
    return Seconds(m_feederSat->GetDistanceFrom(m_feederGw) / kC);
}

Time
NtnRealStackHelper::ComputeServiceLinkDelay() const
{
    // GAP S2: one-way UE<->satellite (service link) propagation.
    // TR 38.821 Table 4.2-2: LEO-600 one-way service-link delay 2.0-6.44 ms.
    constexpr double kC = 299792458.0;
    if (m_gnb.GetN() == 0 || m_ue.GetN() == 0)
    {
        return Seconds(0);
    }
    // NT-04: average over every gNB x UE pair rather than sampling index 0.
    // A single pair made the folded delay a property of one terminal, so a
    // scenario with terminals spread across a beam gave them all the slant of
    // whichever happened to be first.
    //
    // Each UE contributes the slant to its OWN serving gNB only. Averaging over
    // every gNB x UE pair let non-serving (even below-horizon) satellites of a
    // multi-satellite scene inflate the folded delay by milliseconds. Before
    // attachment (no serving cell yet) the nearest gNB stands in for it.
    double sumM = 0.0;
    uint32_t pairs = 0;
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        Ptr<MobilityModel> um = m_ue.Get(u)->GetObject<MobilityModel>();
        if (!um)
        {
            continue;
        }
        const uint16_t servingCell = GetUeServingCellId(u);
        double servingM = -1.0;
        double nearestM = std::numeric_limits<double>::infinity();
        for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
        {
            Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
            if (!gm)
            {
                continue;
            }
            const double d = gm->GetDistanceFrom(um);
            nearestM = std::min(nearestM, d);
            if (servingCell != 0 && GetGnbCellId(g) == servingCell)
            {
                servingM = d;
            }
        }
        const double useM = (servingM >= 0.0) ? servingM : nearestM;
        if (std::isfinite(useM))
        {
            sumM += useM;
            ++pairs;
        }
    }
    if (pairs == 0)
    {
        return Seconds(0);
    }
    return Seconds((sumM / pairs) / kC);
}

void
NtnRealStackHelper::RefreshBackhaulFold()
{
    // NT-04. The service-link delay folded into the backhaul was computed once,
    // at Build(), from the geometry at t=0, and never revisited. Over a LEO pass
    // the slant sweeps from a few hundred to a couple of thousand kilometres, so
    // a delay pinned at the first instant is wrong by several milliseconds for
    // the entire run - and the application one-way delay the toolkit reports as
    // measured inherits exactly that error.
    //
    // This is the same defect SAGIN-1 had in the contact graph: a quantity taken
    // once at the start of a pass and treated as constant. Refresh it on a timer
    // for as long as the fold is the mechanism in use.
    //
    // When the real air interface carries the delay (SetAirInterfaceDelay) there
    // is nothing to fold and this does not run.
    if (m_airIfaceDelayActive || !m_backhaulCh)
    {
        return;
    }
    // SetFeederGeometry() owns the channel once wired: its callback writes
    // feeder + payload processing + service. Writing backhaul + service here
    // as well made the delay flip between two models every refresh.
    if (m_feederSat && m_feederGw)
    {
        return;
    }
    const Time service = ComputeServiceLinkDelay();
    if (service > Seconds(0))
    {
        m_backhaulCh->SetAttribute("Delay", TimeValue(m_backhaulDelay + service));
    }
    Simulator::Schedule(m_backhaulRefresh, &NtnRealStackHelper::RefreshBackhaulFold, this);
}

uint32_t
NtnRealStackHelper::GetGnbN2Delay(uint32_t gnbIdx, uint8_t bwp) const
{
    if (m_backend != RadioBackend::Nr || !m_nr || gnbIdx >= m_enbDevs.GetN())
    {
        return 0;
    }
    UintegerValue v;
    m_nr->GetGnbPhy(m_enbDevs.Get(gnbIdx), bwp)->GetAttribute("N2Delay", v);
    return static_cast<uint32_t>(v.Get());
}

uint32_t
NtnRealStackHelper::ReferenceToCellSlots(uint32_t referenceSlots) const
{
    // TS 38.331 cellSpecificKoffset is counted in slots of the 15 kHz reference
    // SCS (mu = 0); a slot of numerology mu is 2^-mu as long (TS 38.211 4.3.2),
    // so the same time span is 2^mu cell slots.
    return referenceSlots * (1u << m_numerology);
}

void
NtnRealStackHelper::SetBroadcastKOffsetSlots(uint32_t referenceSlots)
{
    m_broadcastKOffsetSlots = referenceSlots;
    if (!m_built || !m_kOffsetConsumption || m_backend != RadioBackend::Nr ||
        referenceSlots == 0)
    {
        return;
    }
    // Already running: reprogram the live gNB PHYs so a SIB19 refresh mid-pass
    // actually changes how the network schedules, instead of the broadcast
    // value and the scheduler drifting apart for the rest of the run.
    //
    // Wave 2 core fix: the argument is in 15 kHz reference slots on BOTH paths.
    // This path used to write the raw reference value into N2Delay (cell slots)
    // and compare it against the converted value Build() stored, so at mu = 1 a
    // SIB19 refresh halved the UL DCI->PUSCH gap. Convert exactly as Build()
    // does; m_consumedKOffsetSlots is always in cell slots.
    const uint32_t slots = ReferenceToCellSlots(referenceSlots);
    if (slots == m_consumedKOffsetSlots)
    {
        return;
    }
    // Apply the change at a drained boundary rather than instantaneously.
    //
    // N2Delay is the gap between a UL grant and the UE's transmission. Writing
    // it onto live PHYs the instant the broadcast changes re-times a
    // transmission the UE has already been told about, which is wrong
    // independently of whether anything downstream notices. TS 38.331 does not
    // switch instantaneously either: a SIB19 change takes effect at a boundary
    // both ends can compute. Deferring by the larger of the two offsets plus a
    // slot is long enough for every grant issued under the old value to have
    // been served.
    //
    // Stated plainly because it was tried as a fix and is not one: this does
    // NOT resolve the "Cannot TX while RX" abort that ntn-cho-full-constellation
    // hits at 8 or more UEs. That fault survives this change, and it survives
    // removing the live reprogram entirely; what clears it is disabling K_offset
    // consumption altogether, so its trigger is the SIZE of the offset, not the
    // moment it is applied. See boundary A9 for the measured diagnosis.
    const uint32_t oldSlots = m_consumedKOffsetSlots;
    m_consumedKOffsetSlots = slots;
    const uint8_t nBwp = GetNrBwpCount();
    const Time slotPeriod = MilliSeconds(1) / (1 << m_numerology);
    const Time guard = slotPeriod * static_cast<int64_t>(std::max(oldSlots, slots) + 1);
    Simulator::Schedule(guard, [this, slots, nBwp]() {
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            for (uint8_t b = 0; b < nBwp; ++b)
            {
                m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                    ->SetAttribute("N2Delay", UintegerValue(m_baseN2Delay + slots));
            }
        }
    });
    NS_LOG_INFO("NtnRealStackHelper: SIB19 refresh -> K_offset "
                << referenceSlots << " reference slots = " << slots
                << " cell slots (numerology " << m_numerology << "); N2Delay " << m_baseN2Delay << " -> "
                << (m_baseN2Delay + slots) << " applied after "
                << guard.GetMilliSeconds() << " ms so no in-flight grant straddles it");
}

uint32_t
NtnRealStackHelper::ComputeKOffsetSlots() const
{
    // TS 38.213 §4.2 cell-specific K_offset: enough slots to cover the service-
    // link ROUND TRIP so a UL grant issued now is transmitted only after the DL
    // has propagated and been decoded. Matches the SIB19 cellSpecificKoffset
    // derivation (ceil(RTT / slot) + 1). Slot = 1 ms / 2^numerology.
    // NT-11: the cell-specific K_offset must cover the cell's WORST round trip,
    // not its average one.
    //
    // This used ComputeServiceLinkDelay(), which averages over every gNB-UE
    // pair. An average under-provisions the farthest terminal by construction,
    // and under-provisioning K_offset produces exactly the failure it exists to
    // prevent: the delayed downlink lands inside the slot the UE believes is
    // its uplink and the stack trips "Cannot TX while RX". A value that is too
    // large only costs latency; one that is too small breaks the link, so the
    // asymmetry says to bound rather than to average.
    constexpr double kC = 299792458.0;
    const double worstSlantM = WorstCaseSlantM();
    const double oneWayS = (worstSlantM > 0.0) ? (worstSlantM / kC)
                                               : ComputeServiceLinkDelay().GetSeconds();
    const double rttS = 2.0 * oneWayS;
    const double slotS = 1.0e-3 / std::pow(2.0, static_cast<double>(m_numerology));
    if (slotS <= 0.0)
    {
        return 0;
    }
    return static_cast<uint32_t>(std::ceil(rttS / slotS)) + 1;
}

Time
NtnRealStackHelper::ComputePayloadExtraDelay(double slantRangeM) const
{
    constexpr double kC = 299792458.0;
    const Time prop = Seconds(slantRangeM / kC);

    // GAP S2 FIX: fold the UE<->satellite SERVICE link into the user-plane delay
    // whenever the air interface itself is NOT carrying it.
    //
    // The old code added only the FEEDER slant here, and the header claimed the
    // end-to-end OWD was therefore "correct" — it was not: the 2-6.4 ms service
    // leg was missing from every leg of the path, so every OWD / jitter / URLLC
    // figure was short by it.
    //
    // m_airIfaceDelayActive is true only when a real ConstantSpeedPropagation-
    // DelayModel sits on the radio channel (nr backend, single UE — see
    // BuildNrRadio for why multi-UE cannot have it without TA). When it is
    // true the slant is already in the path and adding it here would
    // double-count.
    const Time service = m_airIfaceDelayActive ? Seconds(0) : ComputeServiceLinkDelay();

    switch (m_payload)
    {
    case PayloadOption::Transparent:
        // Bent pipe: the user plane crosses the feeder ONCE (gateway gNB ->
        // satellite -> UE), TR 38.821 Table 4.2-2 (one-way = feeder + service).
        // This returned prop + prop + service, counting the feeder twice. The
        // transparent penalty is in the HARQ / K_offset round trip, not in the
        // one-way delivery of a packet. No on-board processing is added.
        return prop + service;
    case PayloadOption::RegenerativeRu:
        // Open-FH (split 7.2x) over the feeder; 0.25 ms lower-PHY budget.
        return prop + MicroSeconds(250) + service;
    case PayloadOption::RegenerativeRuDu:
        // F1 midhaul over the feeder.
        return prop + MicroSeconds(150) + service;
    case PayloadOption::FullGnb:
    default:
        // GTP backhaul to the ground core.
        return prop + MicroSeconds(50) + service;
    }
}

void
NtnRealStackHelper::SetFeederGeometry(Ptr<MobilityModel> satMobility,
                                      Ptr<MobilityModel> gwMobility)
{
    NS_ABORT_MSG_IF(!m_built, "SetFeederGeometry before Build()");
    NS_ABORT_MSG_IF(!satMobility || !gwMobility, "SetFeederGeometry: null mobility");
    m_feederSat = satMobility;
    m_feederGw = gwMobility;
    // Live update: the EPC backhaul channel delay tracks the real slant.
    RegisterPeriodicCallback(Seconds(1.0), [this](Time) {
        const double slantM = m_feederSat->GetDistanceFrom(m_feederGw);
        m_backhaulCh->SetAttribute("Delay", TimeValue(ComputePayloadExtraDelay(slantM)));
    });
    const double slantM = m_feederSat->GetDistanceFrom(m_feederGw);
    m_backhaulCh->SetAttribute("Delay", TimeValue(ComputePayloadExtraDelay(slantM)));
}

void
NtnRealStackHelper::EnsureOranMonitor()
{
    if (m_oranMonitor)
    {
        return;
    }
    m_oranMonitor = CreateObject<NtnOranAiFlowMonitor>();
    m_oranMonitor->SetPhySource(this);
    m_oranMonitor->SetRanCounterKpm(m_kpmFromRan);
    m_oranMonitor->Start();
}

void
NtnRealStackHelper::AttachInstalledFlowsToMonitor()
{
    for (uint32_t i = m_monAttachedClients; i < m_clientApps.GetN(); ++i)
    {
        Ptr<NtnOranApplication> app = DynamicCast<NtnOranApplication>(m_clientApps.Get(i));
        if (app)
        {
            m_oranMonitor->AddSource(app);
        }
    }
    m_monAttachedClients = m_clientApps.GetN();
    for (uint32_t i = m_monAttachedSinks; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (sink)
        {
            // m_dlSinkUe maps the flow to its UE for PHY (L1M.RS-SINR) metrics.
            const int32_t ue =
                (i < m_dlSinkUe.size()) ? static_cast<int32_t>(m_dlSinkUe[i]) : -1;
            m_oranMonitor->AddSink(sink, ue);
        }
    }
    m_monAttachedSinks = m_dlSinks.GetN();
}

Ptr<NtnOranAiFlowMonitor>
NtnRealStackHelper::EnableOranFlowMonitor()
{
    NS_ABORT_MSG_IF(!m_built, "EnableOranFlowMonitor before Build()");
    NS_ABORT_MSG_IF(m_dlSinks.GetN() == 0,
                    "EnableOranFlowMonitor before InstallTraffic/InstallOranFlow");
    EnsureOranMonitor();
    AttachInstalledFlowsToMonitor();
    return m_oranMonitor;
}

void
NtnRealStackHelper::EnableAiFlowMonitor(const std::string& outputPrefix)
{
    NS_ABORT_MSG_IF(!m_built, "EnableAiFlowMonitor before Build()");
    EnsureOranMonitor();
    // Attach everything installed so far; later InstallTraffic/InstallOranFlow
    // calls auto-attach via this flag.
    m_autoAttachMonitor = true;
    AttachInstalledFlowsToMonitor();
    m_aiMonitorPrefix = outputPrefix;
    if (!m_aiExportScheduled)
    {
        // Same export mechanics as the reference wiring (ntn-oran-qos-flows
        // used to call WriteCsv/WriteInfluxLp after Simulator::Run()):
        // ScheduleDestroy fires inside Simulator::Destroy(), after the last
        // granularity tick, while the helper is still alive in main().
        Simulator::ScheduleDestroy(&NtnRealStackHelper::ExportAiFlowMonitor, this);
        m_aiExportScheduled = true;
    }
}

Ptr<NtnOranAiFlowMonitor>
NtnRealStackHelper::GetAiFlowMonitor() const
{
    return m_oranMonitor;
}

void
NtnRealStackHelper::ExportAiFlowMonitor()
{
    if (!m_oranMonitor || m_aiMonitorPrefix.empty())
    {
        return;
    }
    // A bare prefix ("sagin-aeronautical") used to land in the process working
    // directory, outside the run's output directory, so the KPM series never
    // reached the dataset. A prefix without a directory part now goes into the
    // helper's output directory, like sim_health.csv.
    std::filesystem::path prefix(m_aiMonitorPrefix);
    if (!prefix.has_parent_path() && !m_outputDir.empty())
    {
        prefix = std::filesystem::path(m_outputDir) / prefix;
        m_aiMonitorPrefix = prefix.string();
    }
    if (prefix.has_parent_path())
    {
        std::error_code ec;
        std::filesystem::create_directories(prefix.parent_path(), ec);
    }
    m_oranMonitor->WriteCsv(m_aiMonitorPrefix + "_kpm_series.csv");
    m_oranMonitor->WriteInfluxLp(m_aiMonitorPrefix + "_kpm_series.lp");
    NS_LOG_INFO("KPM series exported to " << m_aiMonitorPrefix << "_kpm_series.{csv,lp}");
}

void
NtnRealStackHelper::RegisterPeriodicCallback(Time period, std::function<void(Time)> cb)
{
    uint32_t idx = m_periodics.size();
    m_periodics.push_back({period, std::move(cb)});
    // Schedule immediately so it fires regardless of call order vs InstallTraffic.
    Simulator::Schedule(period, &NtnRealStackHelper::RunPeriodic, this, idx);
}

void
NtnRealStackHelper::RunPeriodic(uint32_t idx)
{
    if (Simulator::Now() >= m_simTime)
    {
        return;
    }
    m_periodics[idx].cb(Simulator::Now());
    Simulator::Schedule(m_periodics[idx].period, &NtnRealStackHelper::RunPeriodic, this, idx);
}

void
NtnRealStackHelper::AccumulateDl(double sinrLinear,
                                 double tbler,
                                 bool corrupt,
                                 uint16_t cellId,
                                 uint16_t rnti,
                                 uint32_t tbSize,
                                 double mcs,
                                 double rank,
                                 double rbFrac,
                                 uint8_t bwpId)
{
    // Only count data TBs that actually carry a transport block.
    if (tbSize == 0)
    {
        return;
    }
    const double sinrDb = 10.0 * std::log10(std::max(sinrLinear, 1e-12));

    // Fold one measured sample into an accumulator (Enabler D adds MCS/rank/PRB).
    auto fold = [&](SinrAccum& a) {
        a.sumSinrDb += sinrDb;
        a.sumSinrLin += std::max(sinrLinear, 1e-12);
        a.sumTbler += tbler;
        a.n += 1;
        if (corrupt)
        {
            a.corrupt += 1;
        }
        if (mcs >= 0.0)
        {
            a.sumMcs += mcs;
        }
        if (rank >= 0.0)
        {
            a.sumRank += rank;
        }
        if (rbFrac >= 0.0)
        {
            a.sumRbFrac += rbFrac;
        }
    };

    fold(m_dlGlobal);
    fold(m_dlPerCell[cellId]);
    fold(m_dlPerRnti[UeKey(cellId, rnti)]);
    if (mcs >= 0.0 || rank >= 0.0 || rbFrac >= 0.0)
    {
        fold(m_dlPerBwp[bwpId]); // Enabler C: per-slice (per-BWP) breakdown
    }
    m_lastSinrDbPerRnti[UeKey(cellId, rnti)] = sinrDb;
    m_lastTblerPerRnti[UeKey(cellId, rnti)] = tbler;
    // S9: a live error model always reports a finite TBLER (~1e-8 even on a
    // pristine link); a disabled one writes exactly 0.0. One positive sample is
    // therefore proof the model ran — see the gateErrorModel probe.
    if (tbler > 0.0)
    {
        m_sawNonZeroTbler = true;
    }
}

void
NtnRealStackHelper::DlRxTrace(mmwave::RxPacketTraceParams params)
{
    AccumulateDl(params.m_sinr,
                 params.m_tbler,
                 params.m_corrupt,
                 static_cast<uint16_t>(params.m_cellId),
                 params.m_rnti,
                 params.m_tbSize);
}

void
NtnRealStackHelper::DlRxTraceNr(::ns3::RxPacketTraceParams params)
{
    // ns3::RxPacketTraceParams (nr): same field family as mmwave; m_sinr is the
    // linear average SINR. Feeds the identical accumulators as the mmwave path,
    // and additionally the measured MCS / MIMO rank / PRB-utilisation / per-BWP
    // (per-slice) breakdown (Enablers C/D). Fully qualified (::ns3::) to
    // disambiguate from mmwave::RxPacketTraceParams (in scope via using).
    const double rbFrac =
        static_cast<double>(params.m_rbAssignedNum) / static_cast<double>(m_nrBandRb);
    if (params.m_tbSize > 0)
    {
        // C6: RAN-side TB and PRB counters (TS 28.552 5.1.1.8, 5.1.1.2).
        const double prbSlots =
            static_cast<double>(params.m_rbAssignedNum) * static_cast<double>(params.m_numSym) / 14.0;
        m_cellPrbSlots[static_cast<uint16_t>(params.m_cellId)] += prbSlots;
        const int32_t ue = UeIndexOfCellRnti(static_cast<uint16_t>(params.m_cellId), params.m_rnti);
        if (ue >= 0)
        {
            UeRanCounters& c = m_ueRan[static_cast<uint32_t>(ue)];
            c.dlTb += 1;
            c.dlTbErr += params.m_corrupt ? 1 : 0;
            c.prbSymSum += prbSlots;
            c.sumMcs += params.m_mcs;
            c.sumRank += params.m_rank;
            c.harqRetx += (params.m_rv > 0 && params.m_rv != std::numeric_limits<uint8_t>::max()) ? 1 : 0;
            c.okBits += params.m_corrupt ? 0.0 : 8.0 * params.m_tbSize;
            c.sinrLinSum += std::max(params.m_sinr, 1e-30);
            // Per-UE decision window (TakeUeSinrWindowMeanDb): keyed by the UE
            // index, so it is continuous across handovers.
            UeSinrWindow& w = m_ueSinrWin[static_cast<uint32_t>(ue)];
            w.sumLin += std::max(params.m_sinr, 1e-30);
            w.n += 1;
            w.lastDb = 10.0 * std::log10(std::max(params.m_sinr, 1e-30));
        }
    }
    AccumulateDl(params.m_sinr,
                 params.m_tbler,
                 params.m_corrupt,
                 static_cast<uint16_t>(params.m_cellId),
                 params.m_rnti,
                 params.m_tbSize,
                 static_cast<double>(params.m_mcs),
                 static_cast<double>(params.m_rank),
                 rbFrac,
                 params.m_bwpId);
}

uint16_t
NtnRealStackHelper::GetUeRnti(uint32_t ueIndex) const
{
    if (ueIndex >= m_ueDevs.GetN())
    {
        return 0;
    }
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(ueIndex));
        if (!dev || !dev->GetPhy(0))
        {
            return 0;
        }
        return dev->GetPhy(0)->GetRnti();
    }
    Ptr<MmWaveUeNetDevice> dev = DynamicCast<MmWaveUeNetDevice>(m_ueDevs.Get(ueIndex));
    if (!dev || !dev->GetRrc())
    {
        return 0;
    }
    return dev->GetRrc()->GetRnti();
}

Time
NtnRealStackHelper::GetAppliedX2LinkDelay() const
{
    if (!m_nrEpc)
    {
        return Time(0);
    }
    TimeValue tv;
    m_nrEpc->GetAttribute("X2LinkDelay", tv);
    return tv.Get();
}

double
NtnRealStackHelper::GetNeighbourRsrpDbm(uint16_t cellId) const
{
    auto it = m_neighbourRsrpDbm.find(cellId);
    return (it == m_neighbourRsrpDbm.end()) ? std::numeric_limits<double>::quiet_NaN()
                                            : it->second;
}

double
NtnRealStackHelper::GetServingRsrpDbm() const
{
    return m_servingRsrpDbm;
}

double
NtnRealStackHelper::GetUeNoiseFigureDb() const
{
    // OBS-09: read the figure off the live PHY instead of assuming it. Both
    // backends expose the same attribute name on the UE PHY, and a scenario
    // that changes it must not silently bias anything derived from it.
    if (m_ueDevs.GetN() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    DoubleValue nf;
    if (m_backend == RadioBackend::Nr && m_nr)
    {
        m_nr->GetUePhy(m_ueDevs.Get(0), 0)->GetAttribute("NoiseFigure", nf);
        return nf.Get();
    }
    if (m_backend == RadioBackend::Mmwave)
    {
        Ptr<MmWaveUeNetDevice> ue = m_ueDevs.Get(0)->GetObject<MmWaveUeNetDevice>();
        if (ue && ue->GetPhy())
        {
            ue->GetPhy()->GetAttribute("NoiseFigure", nf);
            return nf.Get();
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

uint32_t
NtnRealStackHelper::GetSignalResourceElements() const
{
    // OBS-09. SS-RSRP is defined per resource element (TS 38.215 Sec. 5.1.1),
    // so anything reconstructed from a wideband SINR has to be divided by the
    // number of REs the SINR was averaged over.
    //
    // SCS follows the backend's numerology: the mmwave path runs numerology 2
    // (60 kHz) and the nr path runs the configured FR1 numerology.
    if (m_bwHz <= 0.0)
    {
        return 0;
    }
    const uint16_t mu = (m_backend == RadioBackend::Mmwave) ? 2 : m_numerology;
    const double scsHz = 15.0e3 * std::pow(2.0, static_cast<double>(mu));
    const double rb = std::floor(m_bwHz / (12.0 * scsHz));
    if (rb < 1.0)
    {
        return 0;
    }
    return static_cast<uint32_t>(rb) * 12u;
}

void
NtnRealStackHelper::NrMeasurementReport(std::string /*ctx*/,
                                        uint64_t imsi,
                                        uint16_t cellId,
                                        uint16_t /*rnti*/,
                                        NrRrcSap::MeasurementReport report)
{
    // C2: per-UE store. The vendored stack ENCODES with the TS 36.133 9.1.4 /
    // 9.1.7 mappings (nr::EutranMeasurementMapping: RSRP = index - 141 dBm,
    // RSRQ = (index - 40) / 2 dB), so decode with the same functions. The
    // legacy getters below used the TS 38.133 SS-RSRP offset (index - 156),
    // 15 dB low; absolute values were wrong, differences were not.
    {
        const int32_t ue = UeIndexOfImsi(imsi);
        if (ue >= 0)
        {
            const Time now = Simulator::Now();
            auto& cells = m_ueMeas[static_cast<uint32_t>(ue)];
            ++m_ueMeasReports[static_cast<uint32_t>(ue)];
            auto store = [&](uint16_t c, uint8_t rsrpIdx, uint8_t rsrqIdx, bool haveRsrq, bool serving) {
                UeCellMeasurement& m = cells[c];
                m.rsrpDbm = nr::EutranMeasurementMapping::RsrpRange2Dbm(rsrpIdx);
                m.rsrqDb = haveRsrq ? nr::EutranMeasurementMapping::RsrqRange2Db(rsrqIdx)
                                    : std::numeric_limits<double>::quiet_NaN();
                m.time = now;
                m.serving = serving;
                ++m.reports;
                if (m_measTrace)
                {
                    *m_measTrace << now.GetSeconds() << "," << ue << "," << imsi << "," << c << ","
                                 << (serving ? 1 : 0) << ",rrc-report," << m.rsrpDbm << ","
                                 << m.rsrqDb << "," << SlantKmToCell(ue, c) << "\n";
                }
            };
            for (auto& [c, m] : cells)
            {
                m.serving = false;
            }
            store(cellId,
                  report.measResults.measResultPCell.rsrpResult,
                  report.measResults.measResultPCell.rsrqResult,
                  true,
                  true);
            if (report.measResults.haveMeasResultNeighCells)
            {
                for (const auto& n : report.measResults.measResultListEutra)
                {
                    if (n.haveRsrpResult)
                    {
                        store(n.physCellId, n.rsrpResult, n.rsrqResult, n.haveRsrqResult, false);
                    }
                }
            }
        }
    }
    // CHO-6: harvest the NEIGHBOUR measurements a UE actually reports, which is
    // the only standards-defined source of candidate-cell quality. TS 38.133
    // maps the reported RSRP index to dBm as RSRP = index - 156 over the range
    // 0..127, so index 0 is -156 dBm and 127 is -29 dBm.
    ++m_measReportCount;
    // OBS-09: the serving cell reports on every measurement report, whether or
    // not neighbours are present, so harvest it before the neighbour guard.
    // TS 38.331 measResultPCell carries the same TS 38.133 index encoding as
    // the neighbour entries.
    m_servingRsrpDbm =
        nr::EutranMeasurementMapping::RsrpRange2Dbm(report.measResults.measResultPCell.rsrpResult);
    if (!report.measResults.haveMeasResultNeighCells)
    {
        return;
    }
    for (const auto& n : report.measResults.measResultListEutra)
    {
        if (!n.haveRsrpResult)
        {
            continue;
        }
        m_neighbourRsrpDbm[n.physCellId] = nr::EutranMeasurementMapping::RsrpRange2Dbm(n.rsrpResult);
    }
}

int
NtnRealStackHelper::EvaluateEirpGate(double& budgetOut, double& toleranceOut) const
{
    // NT-02. Establish the budget the run is supposed to respect, in order of
    // decreasing directness:
    //   1. an explicit SetEirpBudgetDbm() from the scenario;
    //   2. the total EIRP the scenario declared via SetSatEirpTotalDbm() or
    //      SetSatEirpDensityDbwMhz() (the gate then catches an array-gain
    //      double-count, which is the defect that motivated it);
    //   3. for an S-band NTN carrier, the TR 38.821 Set-1 density over the
    //      configured bandwidth.
    // If none applies the gate reports "not asserted" rather than a silent pass.
    // A scenario that explicitly declared CONDUCTED power is exempt: it is not
    // claiming to model a standardized satellite payload at all.
    toleranceOut = m_eirpToleranceDb;

    if (std::isfinite(m_eirpBudgetDbm))
    {
        budgetOut = m_eirpBudgetDbm;
    }
    else if (std::isfinite(m_eirpTotalDbm))
    {
        budgetOut = m_eirpTotalDbm;
    }
    else if (!m_eirpDeclaredConducted && m_freqHz > 1.0e9 && m_freqHz <= 7.125e9 &&
             m_bwHz > 0.0)
    {
        const double bwMhz = m_bwHz / 1e6;
        budgetOut = kTr38821Set1SBandEirpDensityDbwMhz + 10.0 * std::log10(bwMhz) + 30.0;
    }
    else
    {
        budgetOut = std::numeric_limits<double>::quiet_NaN();
        return -1;
    }

    const double eff = GetEffectiveEirpDbm();
    if (!std::isfinite(eff) || !std::isfinite(budgetOut))
    {
        return -1;
    }
    // One-sided: being under budget is a scenario choice, being over it is the
    // double-count this gate exists to catch.
    return (eff <= budgetOut + toleranceOut) ? 1 : 0;
}

int
NtnRealStackHelper::EvaluateEirpPlausibility(double& referenceDbm, double& excessDb) const
{
    // WF-08 gate 3. Independent of what the scenario declared: the reference is
    // the standard's, so a hotter declaration cannot move it.
    referenceDbm = std::numeric_limits<double>::quiet_NaN();
    excessDb = std::numeric_limits<double>::quiet_NaN();

    if (m_eirpDeclaredConducted)
    {
        return -1; // not claiming to model a standardized payload
    }
    if (!(m_freqHz > 1.0e9 && m_freqHz <= 7.125e9) || m_bwHz <= 0.0)
    {
        return -1; // TR 38.821 Set-1 is an S-band FR1 reference
    }

    const double bwMhz = m_bwHz / 1e6;
    referenceDbm = kTr38821Set1SBandEirpDensityDbwMhz + 10.0 * std::log10(bwMhz) + 30.0;
    const double eff = GetEffectiveEirpDbm();
    if (!std::isfinite(eff) || !std::isfinite(referenceDbm))
    {
        return -1;
    }
    excessDb = eff - referenceDbm;
    // One-sided, like the budget gate: a quieter satellite is a scenario
    // choice, a louder one is a claim the standard does not support.
    return (excessDb <= m_eirpPlausibilityTolDb) ? 1 : 0;
}

Time
NtnRealStackHelper::ComputeX2LinkDelay() const
{
    // S8: one-way inter-gNB (inter-satellite) delay from the live geometry.
    //  - FullGnb (regenerative, Rel-19): the Xn/X2 rides a direct ISL, so the
    //    delay is the inter-satellite range / c.
    //  - Transparent payload: both "gNBs" are on the ground behind their feeder
    //    links, so the X2 loop is 2 x feeder slant / c plus the core hop.
    // Falls back to m_backhaulDelay when the geometry is unavailable.
    //
    // NT-01 FIX (2026-08-24): AddX2Interface() stands up a full mesh over every
    // gNB pair, but nr exposes X2LinkDelay as a single scalar attribute shared
    // by every X2 channel, so a genuinely per-pair delay is not expressible
    // without patching nr. Sampling only the (0,1) pair, as this did, made the
    // mesh delay an accident of node ordering. Average over all pairs instead:
    // that is the representative value the scalar attribute can actually carry,
    // and it is stable under reordering. Per-pair delay is tracked as a future
    // nr patch (see rd-audit-2026-08-24/GAP_REGISTER.md, NT-01).
    if (m_gnb.GetN() < 2)
    {
        return m_backhaulDelay;
    }
    if (m_payload != PayloadOption::FullGnb)
    {
        // Transparent: both "gNBs" sit on the ground behind their feeder links,
        // so the inter-gNB path goes down one feeder and up the other. The
        // geometry of the space segment does not enter.
        return m_backhaulDelay + m_backhaulDelay;
    }

    // Regenerative (Rel-19): the Xn/X2 rides a direct ISL, so the delay is the
    // inter-satellite range over c, averaged across the mesh.
    constexpr double c = 299792458.0;
    double sumRangeM = 0.0;
    uint32_t pairs = 0;
    for (uint32_t i = 0; i < m_gnb.GetN(); ++i)
    {
        Ptr<MobilityModel> a = m_gnb.Get(i)->GetObject<MobilityModel>();
        if (!a)
        {
            continue;
        }
        for (uint32_t j = i + 1; j < m_gnb.GetN(); ++j)
        {
            Ptr<MobilityModel> b = m_gnb.Get(j)->GetObject<MobilityModel>();
            if (!b)
            {
                continue;
            }
            sumRangeM += a->GetDistanceFrom(b);
            ++pairs;
        }
    }
    if (pairs == 0)
    {
        return m_backhaulDelay;
    }
    return Seconds((sumRangeM / pairs) / c);
}

bool
NtnRealStackHelper::IsErrorModelEnabled() const
{
    // S9: prefer the PHY's own DataErrorModelEnabled attribute — both backends
    // zero the reported TBLER when it is false, so a value probe alone cannot
    // tell "model off" from "no errors on a pristine link".
    //
    // CAVEAT: mmwave registers the attribute with a MEMBER accessor (readable),
    // but nr registers it with a SETTER-ONLY accessor
    // (MakeBooleanAccessor(&NrSpectrumPhy::SetDataErrorModelEnabled)), so it is
    // not gettable and a plain GetAttribute() aborts the run. Use the fail-safe
    // read and fall back to the TBLER-value probe when the attribute is
    // write-only.
    if (m_ueDevs.GetN() == 0)
    {
        return false;
    }
    BooleanValue enabled(false);
    Ptr<Object> phyObj;
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(0));
        if (dev && dev->GetPhy(0))
        {
            phyObj = dev->GetPhy(0)->GetSpectrumPhy();
        }
    }
    else
    {
        Ptr<MmWaveUeNetDevice> dev = DynamicCast<MmWaveUeNetDevice>(m_ueDevs.Get(0));
        if (dev && dev->GetPhy())
        {
            phyObj = dev->GetPhy()->GetDlSpectrumPhy();
        }
    }
    if (phyObj && phyObj->GetAttributeFailSafe("DataErrorModelEnabled", enabled))
    {
        return enabled.Get();
    }
    // S9 follow-up: ask the nr PHY directly.
    //
    // The value probe below was a heuristic forced by the setter-only attribute,
    // and its own comment admitted it can false-fail an extremely clean link.
    // It did: thz-ntn-leo-ground-downlink-traffic decodes 2906 transport blocks
    // at 41.8 dB with zero block errors, which is what a healthy link looks
    // like, and the probe reported the error model as disabled. NrSpectrumPhy
    // now carries IsDataErrorModelEnabled(), so the question is answered rather
    // than inferred.
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(0));
        if (dev && dev->GetPhy(0) && dev->GetPhy(0)->GetSpectrumPhy())
        {
            return dev->GetPhy(0)->GetSpectrumPhy()->IsDataErrorModelEnabled();
        }
    }

    // Other backends with an unreadable attribute keep the value probe: a live
    // model reports a finite TBLER even on a clean link, a disabled one writes
    // exactly 0.0 for every TB.
    return m_sawNonZeroTbler || (m_dlGlobal.n == 0);
}

double
NtnRealStackHelper::ComputeOwdFloorMs() const
{
    // S9 / gate 1. A packet cannot reach the UE faster than the satellite's
    // ALTITUDE / c (the UE directly under the sub-satellite point) plus the
    // configured backhaul. Using the altitude rather than the instantaneous
    // slant keeps this a true lower bound for every geometry in the run, so the
    // gate never false-fails as the satellite rises and sets.
    // NT-11: the LOWEST satellite in the deployment, not satellite 0.
    //
    // This is a lower bound on the one-way delay, and a bound sampled from one
    // arbitrary node is not a bound. In a mixed-altitude shell, if node 0
    // happens to be the highest satellite the floor comes out too large and the
    // gate FALSE-FAILS a run whose traffic was served by a lower one. The
    // minimum altitude is the only value that is a true floor for every
    // geometry in the run.
    if (m_gnb.GetN() == 0)
    {
        return 0.0;
    }
    // WGS-84 equatorial radius, the largest Earth radius: r - a never exceeds
    // the true altitude, so the floor stays a lower bound at every latitude.
    // The mean radius (6371 km) overstated the altitude by up to ~7 km at the
    // equator, putting the "floor" ~23 us above the true zenith delay there.
    const double kEarthR = 6378137.0;
    double altM = std::numeric_limits<double>::infinity();
    for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
    {
        Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
        if (!gm)
        {
            continue;
        }
        const Vector p = gm->GetPosition();
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        // ECEF (|r| ~ Earth radius + altitude) vs a local/ENU frame (z = altitude).
        const double a = (r > 6.0e6) ? (r - kEarthR) : p.z;
        if (a > 0.0 && std::isfinite(a))
        {
            altM = std::min(altM, a);
        }
    }
    if (!std::isfinite(altM) || altM <= 0.0)
    {
        return 0.0;
    }
    const double c = 299792458.0;
    // Once SetFeederGeometry() is wired the configured backhaul is no longer in
    // the path (the feeder model replaced it), so it must not be in the floor
    // either; the service leg alone is then the bound.
    const double backhaulMs =
        (m_feederSat && m_feederGw) ? 0.0 : m_backhaulDelay.GetSeconds() * 1000.0;
    return (altM / c) * 1000.0 + backhaulMs;
}

double
NtnRealStackHelper::GetEffectiveEirpDbm() const
{
    // Wave 7A: the TOTAL radiated EIRP over the carrier, i.e. the sum over
    // BWPs of what BuildNrRadio programs, plus the array gain. It used to be
    // the per-BWP value, so the budget gate (a total over m_bwHz) read
    // 10log10(N) low with N slices (-4.77 dB at three BWPs). Before Build() the
    // proportional split sums to the amplifier power by construction.
    if (m_bwpPowerApplied.empty())
    {
        if (!m_bwpPowerExplicit.empty())
        {
            double mw = 0.0;
            for (double p : m_bwpPowerExplicit)
            {
                mw += std::pow(10.0, p / 10.0);
            }
            return 10.0 * std::log10(mw) + ArrayGainDb();
        }
        return m_satEirpDbm + ArrayGainDb();
    }
    double mw = 0.0;
    for (double p : m_bwpPowerApplied)
    {
        mw += std::pow(10.0, p / 10.0);
    }
    return 10.0 * std::log10(mw) + ArrayGainDb();
}

double
NtnRealStackHelper::GetBwpConductedPowerDbm(uint8_t b) const
{
    return b < m_bwpPowerApplied.size() ? m_bwpPowerApplied[b]
                                        : std::numeric_limits<double>::quiet_NaN();
}

uint16_t
NtnRealStackHelper::GetUeServingCellId(uint32_t ueIndex) const
{
    // S3: the UE's OWN serving cell — not gNB[0]'s. Needed to key per-UE stats
    // by (cellId,RNTI) and, after a handover, to read the accumulator of the
    // cell the UE actually moved to (its old-cell history is a separate key).
    if (ueIndex >= m_ueDevs.GetN())
    {
        return 0;
    }
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(ueIndex));
        if (!dev || !dev->GetPhy(0))
        {
            return 0;
        }
        return dev->GetPhy(0)->GetCellId();
    }
    Ptr<MmWaveUeNetDevice> dev = DynamicCast<MmWaveUeNetDevice>(m_ueDevs.Get(ueIndex));
    if (!dev || !dev->GetRrc())
    {
        return 0;
    }
    return dev->GetRrc()->GetCellId();
}

uint16_t
NtnRealStackHelper::GetServingCellId() const
{
    if (m_enbDevs.GetN() == 0)
    {
        return 1;
    }
    if (m_backend == RadioBackend::Nr)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(m_enbDevs.Get(0));
        return gnb ? gnb->GetCellId() : 1;
    }
    Ptr<mmwave::MmWaveEnbNetDevice> enb =
        DynamicCast<mmwave::MmWaveEnbNetDevice>(m_enbDevs.Get(0));
    return enb ? enb->GetCellId() : 1;
}

double
NtnRealStackHelper::GetUeRecentSinrDb(uint32_t ueIndex) const
{
    const uint32_t key = UeKey(GetUeServingCellId(ueIndex), GetUeRnti(ueIndex));
    auto it = m_lastSinrDbPerRnti.find(key);
    return (it != m_lastSinrDbPerRnti.end()) ? it->second
                                             : std::numeric_limits<double>::quiet_NaN();
}

double
NtnRealStackHelper::TakeUeSinrWindowMeanDb(uint32_t ueIndex)
{
    auto it = m_ueSinrWin.find(ueIndex);
    if (it == m_ueSinrWin.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    UeSinrWindow& w = it->second;
    if (w.n == 0)
    {
        return w.lastDb;
    }
    const double db = 10.0 * std::log10(std::max(w.sumLin / static_cast<double>(w.n), 1e-30));
    w.sumLin = 0.0;
    w.n = 0;
    return db;
}

double
NtnRealStackHelper::GetUeMeanSinrDb(uint32_t ueIndex) const
{
    const uint32_t key = UeKey(GetUeServingCellId(ueIndex), GetUeRnti(ueIndex));
    auto it = m_dlPerRnti.find(key);
    if (it == m_dlPerRnti.end() || it->second.n == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return it->second.sumSinrDb / it->second.n;
}

double
NtnRealStackHelper::GetUeRecentTbler(uint32_t ueIndex) const
{
    const uint32_t key = UeKey(GetUeServingCellId(ueIndex), GetUeRnti(ueIndex));
    auto it = m_lastTblerPerRnti.find(key);
    return (it != m_lastTblerPerRnti.end()) ? it->second
                                            : std::numeric_limits<double>::quiet_NaN();
}

void
NtnRealStackHelper::Collect()
{
    // PHY-measured aggregates.
    m_phyRxTb = m_dlGlobal.n;
    m_phyCorruptTb = m_dlGlobal.corrupt;
    m_dlSinrDbMean = (m_dlGlobal.n > 0) ? m_dlGlobal.sumSinrDb / m_dlGlobal.n
                                        : std::numeric_limits<double>::quiet_NaN();
    m_dlTblerMean = (m_dlGlobal.n > 0) ? m_dlGlobal.sumTbler / m_dlGlobal.n : 0.0;

    // App-measured KPIs from the NtnOranSinks (authoritative for the DL data
    // plane): delivery from received bytes; one-way delay / jitter / loss
    // from the in-band NtnOranPayloadHeader primitives.
    uint64_t dlRxBytes = 0;
    double sumDelayMs = 0.0;
    uint64_t delaySamples = 0;
    double sumJitterMs = 0.0;
    uint64_t jitterFlows = 0;
    uint64_t lostPkts = 0;
    uint64_t expectedPkts = 0;
    // Merged 0.1 ms one-way-delay histogram over all DL flows, for percentiles.
    std::vector<uint64_t> owdHist(NtnOranSink::FlowStats::kFineBins, 0);
    uint64_t owdOverflow = 0;
    uint64_t owdN = 0;
    double owdMax = 0.0;
    // C6: RFC 3393 IPDV (consecutive-sequence pairs) merged over all DL flows.
    std::vector<uint64_t> ipdvHist(NtnOranSink::FlowStats::kIpdvBins, 0);
    uint64_t ipdvN = 0;
    uint64_t ipdvOverflow = 0;
    double ipdvSumAbs = 0.0;
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (oranSink)
        {
            dlRxBytes += oranSink->GetTotalRx();
            for (const auto& [key, fs] : oranSink->GetFlowStats())
            {
                ipdvN += fs.ipdvN;
                ipdvSumAbs += fs.ipdvSumAbsMs;
                ipdvOverflow += fs.ipdvOverflow;
                for (size_t b = 0; b < fs.ipdvHistAbs.size() && b < ipdvHist.size(); ++b)
                {
                    ipdvHist[b] += fs.ipdvHistAbs[b];
                }
                sumDelayMs += fs.sumDelayMs;
                delaySamples += fs.rxPackets;
                if (fs.rxPackets > 1)
                {
                    sumJitterMs += fs.jitterMs;
                    ++jitterFlows;
                }
                lostPkts += fs.LostPackets();
                expectedPkts += static_cast<uint64_t>(fs.highestSeq) + 1;
                for (size_t b = 0; b < fs.delayHistFine.size() && b < owdHist.size(); ++b)
                {
                    owdHist[b] += fs.delayHistFine[b];
                    owdN += fs.delayHistFine[b];
                }
                owdOverflow += fs.delayFineOverflow;
                owdN += fs.delayFineOverflow;
                owdMax = std::max(owdMax, fs.maxDelayMs);
            }
            continue;
        }
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(i));
        if (sink)
        {
            dlRxBytes += sink->GetTotalRx();
        }
        else
        {
            dlRxBytes += ExtRxBytesOfSink(i);
        }
    }
    // Wave 7A: caller-installed flows whose packets carry a SeqTsSizeHeader
    // contribute their in-band one-way delay, IPDV and sequence-gap loss.
    for (const auto& f : m_extFlows)
    {
        if (!f.anySeq)
        {
            continue;
        }
        sumDelayMs += f.sumDelayMs;
        delaySamples += f.seqRx;
        ipdvN += f.ipdvN;
        ipdvSumAbs += f.ipdvSumAbsMs;
        ipdvOverflow += f.ipdvOverflow;
        for (size_t b = 0; b < f.ipdvHistAbs.size() && b < ipdvHist.size(); ++b)
        {
            ipdvHist[b] += f.ipdvHistAbs[b];
        }
        for (size_t b = 0; b < f.owdHistFine.size() && b < owdHist.size(); ++b)
        {
            owdHist[b] += f.owdHistFine[b];
            owdN += f.owdHistFine[b];
        }
        owdOverflow += f.owdFineOverflow;
        owdN += f.owdFineOverflow;
        owdMax = std::max(owdMax, f.maxDelayMs);
        lostPkts += (f.highestSeq + 1 > f.seqRx) ? f.highestSeq + 1 - f.seqRx : 0;
        expectedPkts += f.highestSeq + 1;
        if (f.ipdvN > 0)
        {
            sumJitterMs += f.ipdvSumAbsMs / static_cast<double>(f.ipdvN);
            ++jitterFlows;
        }
    }
    // NT-12: divide by the window traffic was actually OFFERED in, not by the
    // whole simulation.
    //
    // Callers routinely start at 1.0 s and stop at simTime-0.5, so a 10 s run
    // had an 8.5 s emission window and this divided by 10: a 15 percent
    // understatement that grows as the guard bands grow. The per-flow path
    // already did it correctly, over each flow's measured lifetime, so the two
    // accessors disagreed about the same run.
    double activeS = std::max(1e-6, m_simTime.GetSeconds());
    if (m_trafficStop > m_trafficStart)
    {
        activeS = std::max(1e-6, (m_trafficStop - m_trafficStart).GetSeconds());
    }
    m_rxThroughputMbps = (dlRxBytes * 8.0) / activeS / 1e6;
    // m_appTxPackets / m_appRxPackets are accumulated live by the app traces.
    m_meanDelayMs = delaySamples ? sumDelayMs / delaySamples : 0.0;
    m_meanJitterMs = jitterFlows ? sumJitterMs / jitterFlows : 0.0;
    m_appSeqGapLossRatio =
        expectedPkts ? static_cast<double>(lostPkts) / expectedPkts : 0.0;

    // RFC 7680 one-way packet loss with waiting time Tmax (see GetAppLossRatio).
    // Every term is keyed by the packet's SEND time, so the population and the
    // on-time arrivals are counted over exactly the same packets. A bin enters
    // the population only when all of it lies at or before Tend - Tmax.
    {
        Time tEnd = Simulator::Now();
        if (tEnd.IsZero() || tEnd > m_simTime)
        {
            tEnd = m_simTime; // after Destroy(), or past the sinks' stop time
        }
        const double cutoffS = (tEnd - m_lossTmax).GetSeconds();
        const int64_t nFull =
            (cutoffS > 0.0) ? static_cast<int64_t>(std::floor(cutoffS / kLossBinS + 1e-9)) : 0;
        auto tally = [&](const std::vector<uint32_t>& sent,
                         const std::vector<uint32_t>& rx,
                         int64_t nBins,
                         uint64_t& pop,
                         uint64_t& ok,
                         uint64_t& pending) {
            for (size_t b = 0; b < sent.size(); ++b)
            {
                const uint64_t r = (b < rx.size()) ? std::min(rx[b], sent[b]) : 0;
                if (static_cast<int64_t>(b) < nBins)
                {
                    pop += sent[b];
                    ok += r;
                }
                else
                {
                    pending += sent[b];
                }
            }
        };
        auto tallyAll = [&](int64_t nBins, uint64_t& pop, uint64_t& ok, uint64_t& pending) {
            tally(m_lossSentBins, m_lossRxBins, nBins, pop, ok, pending);
            for (const auto& f : m_extFlows)
            {
                // A flow without an in-band timestamp cannot say when its packets
                // were sent or whether they were on time; it stays out, as before.
                if (f.anySeq)
                {
                    tally(f.lossSentBins, f.lossRxBins, nBins, pop, ok, pending);
                }
            }
        };
        uint64_t pop = 0;
        uint64_t ok = 0;
        uint64_t pending = 0;
        tallyAll(nFull, pop, ok, pending);
        m_lossCensored = false;
        if (pop == 0 && pending > 0)
        {
            // No packet had a full Tmax window before Tend: take every packet and
            // Tend as the deadline, and say so.
            pop = ok = pending = 0;
            tallyAll(std::numeric_limits<int64_t>::max(), pop, ok, pending);
            m_lossCensored = true;
            NS_LOG_WARN("GetAppLossRatio: run shorter than Tmax = "
                        << m_lossTmax.GetSeconds()
                        << " s after the first packet; loss measured with deadline Tend");
        }
        m_lossSent = pop;
        m_lossDelivered = ok;
        m_lossPending = pending;
        m_appLossRatio = pop ? static_cast<double>(pop - ok) / static_cast<double>(pop) : 0.0;
    }
    if (ipdvN > 0)
    {
        m_ipdvMeanAbsMs = ipdvSumAbs / static_cast<double>(ipdvN);
        const uint64_t target = static_cast<uint64_t>(std::ceil(0.95 * static_cast<double>(ipdvN)));
        uint64_t acc = 0;
        m_ipdvP95AbsMs = std::numeric_limits<double>::quiet_NaN();
        for (size_t b = 0; b < ipdvHist.size(); ++b)
        {
            acc += ipdvHist[b];
            if (acc >= target)
            {
                m_ipdvP95AbsMs = (static_cast<double>(b) + 0.5) * NtnOranSink::FlowStats::kIpdvBinMs;
                break;
            }
        }
    }

    // Delay percentiles (bin midpoint, 0.1 ms resolution) and the exact maximum.
    auto quantile = [&](double q) {
        const uint64_t target = static_cast<uint64_t>(std::ceil(q * static_cast<double>(owdN)));
        uint64_t cum = 0;
        for (size_t b = 0; b < owdHist.size(); ++b)
        {
            cum += owdHist[b];
            if (cum >= std::max<uint64_t>(target, 1))
            {
                return (static_cast<double>(b) + 0.5) * NtnOranSink::FlowStats::kFineBinMs;
            }
        }
        return owdMax; // the quantile lies in the overflow (beyond 1 s): report the max
    };
    if (owdN > 0)
    {
        m_owdP50Ms = quantile(0.50);
        m_owdP95Ms = quantile(0.95);
        m_owdP99Ms = quantile(0.99);
        m_owdMaxMs = owdMax;
    }
}

void
NtnRealStackHelper::DlClientTx(Ptr<const Packet>)
{
    ++m_appTxPackets;
}

void
NtnRealStackHelper::DlSinkRx(Ptr<const Packet>, const Address&)
{
    ++m_appRxPackets;
}

void
NtnRealStackHelper::SetAppLossWaitTime(Time tmax)
{
    NS_ABORT_MSG_IF(m_lossRxJudged && tmax != m_lossTmax,
                    "SetAppLossWaitTime: arrivals were already judged against Tmax = "
                        << m_lossTmax.GetSeconds() << " s; set it before the run");
    NS_ABORT_MSG_IF(!tmax.IsStrictlyPositive(), "SetAppLossWaitTime: Tmax must be > 0");
    m_lossTmax = tmax;
}

void
NtnRealStackHelper::LossBinAdd(std::vector<uint32_t>& bins, Time tx)
{
    const double t = std::max(0.0, tx.GetSeconds());
    const size_t b = static_cast<size_t>(t / kLossBinS);
    if (b >= bins.size())
    {
        bins.resize(b + 1, 0u);
    }
    ++bins[b];
}

void
NtnRealStackHelper::LossRecordRx(std::vector<uint32_t>& rx, Time tx, Time delay)
{
    m_lossRxJudged = true;
    // RFC 7680 section 2.4: a packet that arrives after Tmax is lost.
    if (delay <= m_lossTmax)
    {
        LossBinAdd(rx, tx);
    }
}

void
NtnRealStackHelper::DlOranClientTx(Ptr<const Packet>)
{
    // NtnOranApplication fires "Tx" in the event that stamps the in-band
    // header with Simulator::Now(), so this send time equals the timestamp
    // the sink reads back.
    LossBinAdd(m_lossSentBins, Simulator::Now());
}

void
NtnRealStackHelper::DlOranSinkRx(Ptr<const Packet> p, const Address&)
{
    if (!p || p->GetSize() < NtnOranPayloadHeader::SERIALIZED_SIZE)
    {
        return;
    }
    NtnOranPayloadHeader hdr;
    p->PeekHeader(hdr);
    if (hdr.GetVersion() != NtnOranPayloadHeader::NTN_ORAN_PAYLOAD_VERSION)
    {
        return; // the sink discards it too
    }
    const Time tx = NanoSeconds(static_cast<int64_t>(hdr.GetTxTimestampNs()));
    LossRecordRx(m_lossRxBins, tx, Simulator::Now() - tx);
}

void
NtnRealStackHelper::ExtClientTx(uint32_t flow, Ptr<const Packet>)
{
    LossBinAdd(m_extFlows[flow].lossSentBins, Simulator::Now());
}

double
NtnRealStackHelper::GetDlCorruptFraction() const
{
    return (m_phyRxTb > 0) ? static_cast<double>(m_phyCorruptTb) / m_phyRxTb : 0.0;
}

Ptr<MmWaveHelper>
NtnRealStackHelper::GetMmWaveHelper() const
{
    return m_mmwave;
}

Ptr<MmWavePointToPointEpcHelper>
NtnRealStackHelper::GetEpcHelper() const
{
    return m_epc;
}

uint64_t
NtnRealStackHelper::GetUeRxBytes(uint32_t ueIndex) const
{
    // GAP M6 FIX: m_dlSinks is a per-FLOW container, not per-UE. Indexing it by
    // UE index was correct only by coincidence — when InstallTraffic happens to
    // create exactly one DL flow per UE, in order. Any InstallOranFlow() call or
    // reordering silently returned another UE's bytes. m_dlSinkUe maps sink
    // index -> UE index precisely for this; use it and sum every flow of the UE.
    uint64_t total = 0;
    bool matched = false;
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        if (i >= m_dlSinkUe.size() || m_dlSinkUe[i] != ueIndex)
        {
            continue;
        }
        matched = true;
        Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (oranSink)
        {
            total += oranSink->GetTotalRx();
            continue;
        }
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(i));
        if (sink)
        {
            total += sink->GetTotalRx();
        }
        else
        {
            total += ExtRxBytesOfSink(i);
        }
    }
    if (matched)
    {
        return total;
    }
    // No mapping recorded (e.g. sinks installed by a caller directly): fall back
    // to the legacy positional lookup rather than silently reporting zero.
    if (ueIndex >= m_dlSinks.GetN())
    {
        return 0;
    }
    Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(ueIndex));
    if (oranSink)
    {
        return oranSink->GetTotalRx();
    }
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(ueIndex));
    return sink ? sink->GetTotalRx() : 0;
}

double
NtnRealStackHelper::GetCellMeanSinrDb(uint16_t cellId) const
{
    auto it = m_dlPerCell.find(cellId);
    if (it == m_dlPerCell.end() || it->second.n == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return it->second.sumSinrDb / it->second.n;
}

void
NtnRealStackHelper::WriteHealthReport()
{
    int64_t wallEndNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    double wallSec = (wallEndNs - m_wallStartNs) / 1e9;

    // ---- HONEST gates (2026-06 protocol-fidelity audit §8; hardened by the
    //      2026-07 standards audit, gap S9) ----
    //
    // S9: the previous gate set could not fail a physically wrong radio. It
    // passed a stack with +36 dB of EIRP, a zero-delay air interface, and
    // UMi-NLOS fading at 600 km, because:
    //   * gateProvenance and gateErrorModel were the SAME predicate (n > 0);
    //   * "error model active" never checked the error model (nr writes
    //     m_tbler = 0 when it is disabled, and the sample still counts);
    //   * every delay / jitter / loss row hard-coded pass=1.
    // The gates below add physical plausibility: an OWD floor from the geometry
    // (no packet can beat altitude/c) and a real error-model probe.
    bool gateStackDepth = (m_phyRxTb >= m_gates.minPhyRxTb);          // packets crossed the radio PHY
    bool gateThroughput = (m_rxThroughputMbps >= m_gates.minRxThroughputMbps); // measured app KPI
    bool gateProvenance = (!m_gates.requireSinrProvenance) || (m_dlGlobal.n > 0); // SINR from PHY trace
    // Ask the PHY whether the error model is actually switched on, rather than
    // inferring it from the reported TBLER. Both backends zero the TBLER field
    // when the model is disabled (nr-phy-mac-common.h:585:
    //   m_tbler(errorModelEnabled ? ... : 0)
    // ), but a genuinely pristine link ALSO reports exactly 0 — so a value probe
    // cannot tell "model off" from "no errors" and would false-fail a strong
    // link. The attribute is unambiguous.
    bool gateErrorModel = (!m_gates.requireErrorModelActive) || IsErrorModelEnabled();
    bool channelInPath = (m_ueDevs.GetN() > 0 && m_enbDevs.GetN() > 0); // packets rode mmwave devs

    // GATE 1 (plan P0.1): measured one-way delay must respect the speed of
    // light. The floor uses the satellite ALTITUDE as the minimum possible
    // slant (a UE directly under the sub-satellite point) plus the configured
    // backhaul, so it is a true lower bound for any geometry in the run. Before
    // gap S2 was fixed, the air interface had zero delay and this gate fails.
    const double owdFloorMs = ComputeOwdFloorMs();
    const bool haveDelay = (m_meanDelayMs > 0.0);
    bool gateOwdFloor = true;
    if (m_gates.requireOwdFloor && haveDelay && owdFloorMs > 0.0)
    {
        gateOwdFloor = (m_meanDelayMs >= owdFloorMs);
    }

    // NT-02: EIRP budget gate. -1 (not asserted) does not fail the run.
    double eirpBudgetPre = 0.0;
    double eirpTolPre = 0.0;
    const int eirpGatePre = EvaluateEirpGate(eirpBudgetPre, eirpTolPre);
    const bool gateEirp = !m_gates.requireEirpBudget || (eirpGatePre != 0);

    // NT-05. Give the application-KPI rows floors they can actually fail on.
    // They previously wrote pass=1 unconditionally with a "-" floor, so a run
    // that delivered nothing still reported a healthy sim_health.csv.
    // Only assert delivery when the scenario actually sent something: a run with
    // no traffic has no delivery ratio to judge.
    const double deliveryRatio =
        (m_appTxPackets > 0)
            ? (static_cast<double>(m_appRxPackets) / static_cast<double>(m_appTxPackets))
            : -1.0;
    const bool gateDelivery = (m_gates.minAppDeliveryRatio <= 0.0) || (deliveryRatio < 0.0) ||
                              (deliveryRatio >= m_gates.minAppDeliveryRatio);
    const double corruptFrac = GetDlCorruptFraction();
    const bool gateCorrupt = (m_gates.maxDlCorruptFraction >= 1.0) ||
                             !std::isfinite(corruptFrac) ||
                             (corruptFrac <= m_gates.maxDlCorruptFraction);

    bool allOk = gateStackDepth && gateThroughput && gateProvenance && gateErrorModel &&
                 channelInPath && gateOwdFloor && gateEirp && gateDelivery && gateCorrupt;

    // Label the air interface by the ACTUAL carrier, not a hardcoded "fr1": a THz
    // example at 100 GHz on the Nr data plane must not report "nr-fr1-ntn". FR1 is
    // <= 7.125 GHz, FR2 24.25-71 GHz (TS 38.104); above that we tag "thz".
    const char* backendTag = (m_backend == RadioBackend::Nr) ? "nr" : "mmwave";
    const char* frTag = (m_freqHz <= 7.125e9)   ? "fr1"
                        : (m_freqHz <= 71.0e9)   ? "fr2"
                                                 : "thz";
    // NT-09. The "-ntn" suffix asserted a 3GPP NTN band on any carrier under
    // 7.125 GHz, which is a claim about the CHANNEL, not just the frequency
    // range. TS 38.101-5 Table 5.2-1 defines the NTN FR1 bands narrowly: n256
    // pairs 1980-2010 MHz uplink with 2170-2200 MHz downlink, and n255 pairs
    // 1626.5-1660.5 MHz uplink with 1525-1559 MHz downlink. The helper's former
    // default carrier, 2.0 GHz, sits in n256's UPLINK block, and its former
    // default 30 MHz was documented as "the TS 38.101-5 NTN-FR1 maximum channel
    // bandwidth" when 30 MHz is the total WIDTH of the n256 downlink block, not
    // a channel bandwidth that fits inside it alongside guard bands.
    //
    // Rather than silently relabel physics, the tag now states what is true.
    // A run whose carrier does not fall in an NTN downlink block, or whose
    // channel cannot fit inside that block, is tagged "-ntnband-nonconformant"
    // and the row's pass flag reports it, so nothing downstream can read an
    // NTN band conformance claim off a run that does not have one.
    // SLICE-4 / NT-09. Classified against the real TS 38.101-5 tables rather
    // than an inline approximation, so the tag names the band correctly and
    // checks the channel against that band's supported set instead of merely
    // against the block width.
    NtnFr1Band band = ClassifyNtnFr1(m_freqHz, m_bwHz);
    // Wave 11: under frequency reuse k the cells may also be k carriers of
    // m_bwHz / k each inside the block (TR 38.821 Set-1 S band: 30 MHz shared by
    // three 10 MHz beams). That is conformant when each cell channel is a
    // supported bandwidth and the k of them fit in the block; otherwise the
    // sub-bands are BWPs of the configured carrier and the carrier is judged.
    if (m_frfApplied > 1 && !band.conformant)
    {
        const NtnFr1Band cell = ClassifyNtnFr1(m_freqHz, m_bwHz / m_frfApplied);
        if (cell.conformant && m_bwHz <= cell.blockWidthHz + 1.0)
        {
            band = cell;
        }
    }
    bool ntnBandOk = true;
    std::string bandTag;
    if (frTag == std::string("fr1"))
    {
        ntnBandOk = band.conformant;
        if (!ntnBandOk)
        {
            // Say WHICH way it fails. "non-conformant" alone leaves the reader
            // to guess between a carrier in the wrong half of the band and a
            // channel width that does not exist.
            if (band.name == std::string("none"))
            {
                bandTag = "-ntnband-none";
            }
            else if (!band.carrierInDownlink)
            {
                bandTag = std::string("-") + band.name + "-uplinkcarrier";
            }
            else
            {
                bandTag = std::string("-") + band.name + "-bwunsupported";
            }
        }
        else
        {
            bandTag = std::string("-ntn-") + band.name;
        }
    }
    else
    {
        // Above FR1 the NTN band tables do not apply; do not assert
        // conformance either way.
        bandTag = "";
    }

    if (!ntnBandOk && m_calibrationCarrier)
    {
        // TR 38.821 calibration carrier declared by the scenario: report, do not fail.
        bandTag += "-calibration";
        ntnBandOk = true;
    }
    const std::string airTag = std::string(backendTag) + "-" + frTag + bandTag;

    std::error_code ec;
    std::filesystem::create_directories(m_outputDir, ec);
    std::ofstream out(m_outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass,provenance\n";
    out << "sim_time_s," << m_simTime.GetSeconds() << "," << m_simTime.GetSeconds()
        << ",1,config\n";
    out << "wall_clock_s," << wallSec << ",-,-,measured\n";
    out << "air_interface," << airTag << ",-," << (ntnBandOk ? 1 : 0) << ",config\n";
    if (m_backend == RadioBackend::Nr)
    {
        // Subcarrier spacing vs frequency range (TS 38.101-1/-2 SCS tables,
        // TS 38.211 4.2): FR1 mu 0..2, FR2-1 (24.25-52.6 GHz) mu 2..3, FR2-2
        // (52.6-71 GHz) mu 3, 5, 6; nothing is defined above 71 GHz. The helper
        // runs whatever the scenario set; this row says whether it is legal.
        // Rel-18 also defines Ka-band NTN bands (TS 38.101-5, n510-n512, roughly
        // 17.3 to 30 GHz) with 60 or 120 kHz SCS. Carriers in no defined range
        // (for example Ku-band, 10.7 to 12.75 GHz, or above 71 GHz) have no
        // 3GPP numerology at all: the row is then informational ("-"), not a
        // failure, because "illegal" would claim a rule that does not exist.
        const uint16_t mu = m_numerology;
        std::string muPass = "-";
        if (m_freqHz <= 7.125e9)
        {
            muPass = (mu <= 2) ? "1" : "0";
        }
        else if (m_freqHz >= 17.3e9 && m_freqHz < 24.25e9)
        {
            muPass = (mu == 2 || mu == 3) ? "1" : "0"; // FR2-NTN Ka (TS 38.101-5)
        }
        else if (m_freqHz >= 24.25e9 && m_freqHz <= 52.6e9)
        {
            muPass = (mu == 2 || mu == 3) ? "1" : "0";
        }
        else if (m_freqHz > 52.6e9 && m_freqHz <= 71.0e9)
        {
            muPass = (mu == 3 || mu == 5 || mu == 6) ? "1" : "0";
        }
        out << "numerology," << mu << ",-," << muPass << ",ts38211-4.2+ts38101-scs\n";
    }

    // Half-duplex drops, so they can never be silent.
    //
    // The vendored NR PHY used to NS_FATAL_ERROR whenever a TDD node was asked
    // to transmit while receiving, or to receive an SRS while transmitting. That
    // is not a physical impossibility, it is the absence of a scheduling rule,
    // and a real half-duplex radio simply does not perform the transmission. The
    // patch makes it drop instead of abort, which would be a silent degradation
    // if the count were not reported. It is reported here.
    //
    // Non-zero is not automatically wrong: over NTN a large cell-specific
    // K_offset widens the grant-to-transmission gap enough that occasional
    // collisions are expected. A LARGE count means the node is over-subscribed
    // and its throughput figure means something different, so read it alongside
    // phy_rx_tb rather than on its own.
    if (m_backend == RadioBackend::Nr)
    {
        uint64_t ctrlDrops = 0;
        uint64_t dataDrops = 0;
        uint64_t srsDrops = 0;
        uint64_t dlCtrlDrops = 0;
        const uint8_t nBwpHealth = GetNrBwpCount();
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            for (uint8_t b = 0; b < nBwpHealth; ++b)
            {
                if (Ptr<NrSpectrumPhy> sp = m_nr->GetGnbPhy(m_enbDevs.Get(i), b)->GetSpectrumPhy())
                {
                    ctrlDrops += sp->GetUlCtrlDropCount();
                    dataDrops += sp->GetDataDropCount();
                    srsDrops += sp->GetSrsDropCount();
                    dlCtrlDrops += sp->GetDlCtrlDropCount();
                }
            }
        }
        for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
        {
            for (uint8_t b = 0; b < nBwpHealth; ++b)
            {
                if (Ptr<NrSpectrumPhy> sp = m_nr->GetUePhy(m_ueDevs.Get(i), b)->GetSpectrumPhy())
                {
                    ctrlDrops += sp->GetUlCtrlDropCount();
                    dataDrops += sp->GetDataDropCount();
                    srsDrops += sp->GetSrsDropCount();
                    dlCtrlDrops += sp->GetDlCtrlDropCount();
                }
            }
        }
        out << "halfduplex_ulctrl_drops," << ctrlDrops << ",-,-,measured\n";
        out << "halfduplex_data_drops," << dataDrops << ",-,-,measured\n";
        out << "halfduplex_srs_rx_drops," << srsDrops << ",-,-,measured\n";
        out << "halfduplex_dlctrl_drops," << dlCtrlDrops << ",-,-,measured\n";
    }
    out << "carrier_hz," << m_freqHz << ",-,-,config\n";
    // Wave 9: the duplex the nr carrier ran, and the paired UL carrier.
    out << "duplex," << (m_fddApplied ? "fdd" : "tdd") << ",-,-,"
        << (m_fddApplied ? "ts38101-5-table-5.2-1 (paired UL BWP)" : "config (shared-slot carrier)")
        << "\n";
    if (m_fddApplied)
    {
        out << "ul_carrier_hz," << m_ulFreqHzApplied << ",-,-,ts38101-5-duplex-spacing\n";
    }
    out << "ul_bsr_triggering," << (m_bsrTriggeringApplied ? 1 : 0) << ",-,-,"
        << (m_bsrTriggeringApplied ? "ts38321-5.4.5" : "nr-bsr-in-every-tb") << "\n";
    out << "ho_target_ue_beam," << (m_hoTargetBeamApplied ? 1 : 0) << ",-,-,"
        << (m_hoTargetBeamApplied ? "registered-on-handover" : "nr-attach-only") << "\n";
    // Wave 11b: source downlink flushed at the handover's MAC reset.
    out << "ho_robust_mcs," << (m_hoRobustMcsApplied ? 1 : 0) << ",-,-,"
        << (m_hoRobustMcsApplied ? "ts38214-5.2.2.5 (rsrp-derived mcs until first csi)"
                                 : "nr-default")
        << "\n";
    if (m_hoRobustMcsApplied)
    {
        double sum = 0.0;
        for (uint8_t m : m_hoRobustMcsList)
        {
            sum += m;
        }
        // Count always; the mean only when a target admitted a UE (no NaN row in
        // runs without handovers).
        out << "ho_robust_mcs_admissions," << m_hoRobustMcsList.size() << ",-,-,nr-patch-20\n";
        if (!m_hoRobustMcsList.empty())
        {
            out << "ho_robust_mcs_mean," << sum / m_hoRobustMcsList.size() << ",-,"
                << m_hoRobustMcsList.size() << ",nr-patch-20\n";
        }
        out << "ho_aborted_dl_rx," << GetHandoverAbortedDlRxCount() << ",-,-,nr-patch-20\n";
        out << "ho_flushed_reports," << GetHandoverFlushedReportCount() << ",-,-,nr-patch-20\n";
    }
    // Wave 11: reuse factor and SSB beams per satellite.
    out << "frequency_reuse," << +m_frfApplied << ",-,-,"
        << (m_frfApplied > 1 ? "tr38821-6.1.1.1 (sub-band per cell)" : "reuse-1") << "\n";
    out << "ssb_beam_grid," << m_ssbGridSize << ",-,-,"
        << (m_ssbGridApplied ? "ts38331-5.5.3.3 (best SSB beam)" : "single-cell-beam") << "\n";
    // WF-11: refusals reach the health file, so they are visible in the
    // release build where NS_LOG_WARN is compiled out. pass=0 when a decision
    // module asked for a handover the stack could not perform.
    out << "handover_refusals," << m_handoverRefusals << ",0,"
        << (m_handoverRefusals == 0 ? 1 : 0) << ",measured"
        << (m_handoverRefusals ? (" (last: " + m_lastHandoverRefusal + ")") : std::string())
        << "\n";
    // Informational, not gated: air_interface above already carries the single
    // pass/fail verdict for band conformance. Three failing rows for one issue
    // would dilute the gate rather than sharpen it.
    out << "channel_bw_hz," << m_bwHz << ",-,-,ts38101-5-table-5.3.5-1"
        << (band.bandwidthSupported ? "" : " (UNSUPPORTED for this band)") << "\n";
    if (m_frfApplied > 1)
    {
        out << "cell_bw_hz," << m_bwHz / m_frfApplied << ",-,-,tr38821-6.1.1.1 (carrier / FRF)\n";
        // Per-beam EIRP at the carrier's EIRP density over the cell's sub-band.
        out << "cell_eirp_dbm," << GetEffectiveEirpDbm() - 10.0 * std::log10(m_frfApplied)
            << ",-,-,tr38821-table-6.1.1.1-1 (constant density)\n";
    }
    out << "ntn_fr1_band," << band.name << ",-,-,ts38101-5-table-5.2-1"
        << (band.carrierInDownlink ? " (downlink)" : " (carrier NOT in the downlink block)")
        << "\n";
    out << "channel_in_path," << (channelInPath ? 1 : 0) << ",1,"
        << (channelInPath ? 1 : 0) << ",topology\n";
    // TS 38.213 4.2. The toolkit advertises "cellSpecificKoffset consumed by
    // the scheduler" as a capability, and it is real: the broadcast offset is
    // added to the gNB N2Delay so a UL grant is issued the right number of slots
    // early for the round trip. It left no trace in this artifact, though, so a
    // reader could not tell whether a given run had it. It is off by default and
    // one shipped example turns it on, which is exactly the kind of thing a
    // provenance row exists to make visible.
    out << "koffset_slots_consumed," << (m_kOffsetConsumption ? m_consumedKOffsetSlots : 0)
        << ",-,-,"
        << (m_kOffsetConsumption ? "ts38213-4.2 (added to gNB N2Delay)"
                                 : "not-consumed (SetKOffsetConsumption is off)")
        << "\n";
    out << "phy_rx_tb," << m_phyRxTb << "," << m_gates.minPhyRxTb << ","
        << (gateStackDepth ? 1 : 0) << ",phy-trace\n";
    out << "dl_sinr_db," << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << ",-,"
        << (gateProvenance ? 1 : 0) << ",phy-trace\n";
    // The row above is the arithmetic mean of the per-TB SINRs IN DECIBELS,
    // which is the geometric mean of the linear values. On a bimodal sample set
    // it collapses toward the 1e-12 floor and stops describing the link the
    // decoder saw: thz-ntn-isac-coexist-traffic reported -85.19 dB while 21.5%
    // of its transport blocks decoded and the sink measured 4.27 Mbps, a
    // throughput far above the Shannon bound for -85 dB. Both statistics ship,
    // so the discrepancy is visible instead of being a single misleading number.
    const double sinrLinMean = (m_dlGlobal.n > 0) ? m_dlGlobal.sumSinrLin / m_dlGlobal.n : 0.0;
    const double sinrLinMeanDb = (sinrLinMean > 0.0) ? 10.0 * std::log10(sinrLinMean) : 0.0;
    out << "dl_sinr_db_linear_mean," << sinrLinMeanDb << ",-,-,phy-trace\n";
    out << "dl_tbler_mean," << m_dlTblerMean << ",-," << (gateErrorModel ? 1 : 0)
        << ",phy-trace\n";
    out << "dl_corrupt_frac," << corruptFrac << "," << m_gates.maxDlCorruptFraction << ","
        << (gateCorrupt ? 1 : 0) << ",phy-trace\n";
    out << "rx_throughput_mbps," << m_rxThroughputMbps << "," << m_gates.minRxThroughputMbps
        << "," << (gateThroughput ? 1 : 0) << ",packetsink\n";
    double delivery = (m_appTxPackets > 0)
                          ? static_cast<double>(m_appRxPackets) / m_appTxPackets
                          : 0.0;
    out << "app_delivery_ratio," << delivery << "," << m_gates.minAppDeliveryRatio << ","
        << (gateDelivery ? 1 : 0) << ",app-trace\n";
    // S9: real floor + real pass flag (was: floor "-", pass hard-coded 1).
    out << "app_owd_ms," << m_meanDelayMs << "," << owdFloorMs << "," << (gateOwdFloor ? 1 : 0)
        << ",inband-timestamp\n";
    // NT-05: these are reported quantities, not gates. Writing pass=1 on a row
    // with no floor asserted health that was never checked; "-" says plainly
    // that nothing is being judged.
    //
    // app_loss_ratio is NOT the complement of the delivery ratio above. They
    // have different populations. Delivery is rx/tx over every packet the
    // application sent, whenever it arrived. Loss follows RFC 7680: over the
    // packets sent by Tend - Tmax, the share not received within Tmax, so a
    // late packet counts as lost and the last Tmax of sends is excluded.
    //
    // Until Nov 2026 the loss row counted sequence gaps among received packets
    // (now app_seq_gap_loss_ratio), which could not see a tail that never
    // arrived: an overloaded run delivering less than half its offered load
    // reported 0 % loss, and delivery + loss summed to 0.65 on
    // sagin-a2g-real-stack. Gating loss separately would need its own floor
    // rather than inheriting the delivery gate's.
    // CVC-07: say WHERE the service-link flight time lives.
    //
    // The end-to-end user-plane OWD above is physically right on every path,
    // but the 1.8 ms one-way flight to a 550 km satellite is carried on the
    // BACKHAUL leg by default, not by the air interface, because a real
    // per-distance air delay SIGABRTs the nr stack until K_offset and Timing
    // Advance exist (see the comment at BuildNrRadio). That is a defensible
    // treatment and it is not the same thing as modelling the delay on the air
    // interface. A reader of this artifact had no way to tell which they had,
    // and the manuscript criticizes a comparator on exactly that axis, so the
    // distinction belongs in the file rather than in the source.
    {
        const Time svc = ComputeServiceLinkDelay();
        out << "service_link_delay_ms," << svc.GetSeconds() * 1e3 << ",-,-,"
            << (m_airIfaceDelayActive ? "air-interface (real per-distance propagation)"
                                      : "folded-onto-backhaul (air interface is zero-delay)")
            << "\n";
    }
    out << "app_jitter_ms," << m_meanJitterMs << ",-,-,inband-timestamp\n";
    // One-way-delay distribution (empty value when nothing was received).
    auto owdField = [](double v) {
        std::ostringstream f;
        if (std::isfinite(v))
        {
            f << v;
        }
        return f.str();
    };
    out << "app_owd_p50_ms," << owdField(m_owdP50Ms) << ",-,-,inband-timestamp-hist0.1ms\n";
    out << "app_owd_p95_ms," << owdField(m_owdP95Ms) << ",-,-,inband-timestamp-hist0.1ms\n";
    out << "app_owd_p99_ms," << owdField(m_owdP99Ms) << ",-,-,inband-timestamp-hist0.1ms\n";
    out << "app_owd_max_ms," << owdField(m_owdMaxMs) << ",-,-,inband-timestamp\n";
    out << "app_loss_ratio," << m_appLossRatio << ",-,-,rfc7680-tmax-"
        << m_lossTmax.GetSeconds() << "s" << (m_lossCensored ? "-censored" : "") << "\n";
    // NT-02: real budget + real pass flag (was: floor "-", pass hard-coded 1,
    // which made a 20 dB link-budget error invisible to every gate).
    double eirpBudget = 0.0;
    double eirpTol = 0.0;
    const int eirpGate = EvaluateEirpGate(eirpBudget, eirpTol);
    if (eirpGate < 0)
    {
        out << "effective_eirp_dbm," << GetEffectiveEirpDbm()
            << ",not-asserted,-,derived\n";
    }
    else
    {
        out << "effective_eirp_dbm," << GetEffectiveEirpDbm() << "," << (eirpBudget + eirpTol)
            << "," << eirpGate << ",derived\n";
    }
    // WF-08 gate 3: plausibility against the STANDARD, reported separately from
    // the budget gate above. The budget gate cannot catch an implausible
    // declaration, because declaring a hotter satellite moves the budget too.
    {
        double eirpRef = 0.0;
        double eirpExcess = 0.0;
        const int plaus = EvaluateEirpPlausibility(eirpRef, eirpExcess);
        if (plaus < 0)
        {
            out << "eirp_vs_tr38821_set1_db,not-asserted,-,-,ts38821-table-6.1.1.1-1\n";
        }
        else
        {
            out << "eirp_vs_tr38821_set1_db," << eirpExcess << ","
                << m_eirpPlausibilityTolDb << "," << plaus
                << ",ts38821-table-6.1.1.1-1\n";
        }
    }

    // NT-05: counts and configuration are facts about the run, not assertions
    // about its health.
    out << "app_tx_pkts," << m_appTxPackets << ",-,-,app-trace\n";
    out << "app_rx_pkts," << m_appRxPackets << ",-,-,app-trace\n";
    out << "ues," << m_ue.GetN() << ",-,-,config\n";
    out << "gnbs," << m_gnb.GetN() << ",-,-,config\n";
    out << "run_tag," << m_runTag << ",-,-,config\n";
    // ---- Wave 1 rows (appended so existing row order is unchanged) ----
    // Undefined when no flow carries consecutive sequence numbers (e.g. NR-TV video): an
    // empty value with a "not-measured" provenance, not a NaN that reads as a broken number.
    out << "app_ipdv_mean_abs_ms," << (std::isnan(m_ipdvMeanAbsMs) ? std::string("") : std::to_string(m_ipdvMeanAbsMs))
        << ",-,-," << (std::isnan(m_ipdvMeanAbsMs) ? "not-measured (no consecutive-sequence flows)" : "rfc3393-consecutive-seq") << "\n";
    out << "app_ipdv_p95_abs_ms," << (std::isnan(m_ipdvP95AbsMs) ? std::string("") : std::to_string(m_ipdvP95AbsMs))
        << ",-,-," << (std::isnan(m_ipdvP95AbsMs) ? "not-measured (no consecutive-sequence flows)" : "rfc3393-consecutive-seq-hist0.01ms") << "\n";
    out << "channel_model,"
        << (m_channelModel == ChannelModel::Ns3Tr38811 ? "ns3-tr38811-ntn" : "legacy-friis+excess")
        << ",-,-," << (m_smallScaleFading ? "fading-on" : "los-only-no-fading") << "\n";
    if (m_blockageModel)
    {
        out << "earth_blockage_blocked_evals," << m_blockageModel->GetBlockedCount() << ",-,-,"
            << "geometry-wgs84 (clear " << m_blockageModel->GetClearCount() << "; non-geocentric "
            << m_blockageModel->GetNonGeocentricCount() << ")\n";
    }
    else
    {
        out << "earth_blockage_blocked_evals,off,-,-,config\n";
    }
    {
        const double nf = GetUeNoiseFigureDb();
        const double gUe = m_ueAntennaGainDbi;
        out << "ue_terminal," << (m_set1Applied ? "tr38821-set1" : "custom") << ",-,-,"
            << "ue-panel " << +m_ueRows << "x" << +m_ueCols << "\n";
        out << "ue_element_gain_dbi," << gUe << ",-,-,config\n";
        // G/T of one branch with T_ant = 290 K (the stack's noise model).
        out << "ue_g_over_t_dbk," << (gUe - nf - 10.0 * std::log10(290.0)) << ",-,-,"
            << "derived (NF " << nf << " dB; T0 290 K)\n";
    }
    if (m_feederGw && m_feederSat)
    {
        out << "backhaul_delay_ms," << ComputeFeederLinkDelay().GetSeconds() * 1e3 << ",-,-,"
            << "feeder-geometry (gateway elevation " << GetFeederElevationDeg() << " deg)\n";
    }
    else
    {
        out << "backhaul_delay_ms," << m_backhaulDelay.GetSeconds() * 1e3 << ",-,-,assumed-constant\n";
    }
    if (m_periodicMeas || m_handover)
    {
        uint32_t reports = 0;
        for (const auto& [u, n] : m_ueMeasReports)
        {
            reports += n;
        }
        out << "meas_reports_received," << reports << ",-,-,"
            << (m_periodicMeas ? "rrc-periodical" : "rrc-event") << "\n";
        out << "handover_ping_pong_5s," << GetPingPongCount(Seconds(5)) << ",-,-,rrc-trace\n";
    }
    // RFC 7680 loss breakdown (appended so the earlier row order is unchanged):
    // the population sent by Tend - Tmax, its on-time arrivals, the packets
    // sent later and excluded, and the former sequence-gap figure for
    // comparison.
    out << "app_loss_sent_pkts," << m_lossSent << ",-,-,rfc7680-population\n";
    out << "app_loss_delivered_pkts," << m_lossDelivered << ",-,-,rfc7680-within-tmax\n";
    out << "app_loss_pending_pkts," << m_lossPending << ",-,-,rfc7680-sent-after-cutoff\n";
    out << "app_seq_gap_loss_ratio," << m_appSeqGapLossRatio << ",-,-,inband-seq\n";
    // Caller-supplied provenance rows, emitted last so they cannot displace a
    // gated row if a module picks a colliding name.
    // Caller text is free-form: keep the file at exactly five fields per row (ntn-dataset@1).
    auto cell = [](std::string v) {
        std::replace(v.begin(), v.end(), ',', ';');
        std::replace(v.begin(), v.end(), '\n', ' ');
        return v;
    };
    for (const auto& [metric, value, prov] : m_extraHealthRows)
    {
        out << cell(metric) << "," << cell(value) << ",-,-," << cell(prov) << "\n";
    }
    out.close();

    // ---- OBS-07: reproducibility manifest, written for EVERY real-stack run --
    //
    // The manifest was a decision island: one example out of roughly 87 wrote
    // one, and the single manifest the repo shipped had an empty git SHA, an
    // empty TLE epoch, no NORAD ids and empty sionna/hitran/itu fields, even
    // though the producing example held the values it could have supplied. A
    // provenance record that exists for one scenario and is blank when it does
    // exist cannot do the job it was added for.
    //
    // Writing it here means every example built on this helper gets one beside
    // its sim_health.csv without opting in. Fields the helper genuinely knows
    // are filled; ones it does not are left to the scenario rather than
    // guessed.
    {
        ntnobs::NtnReproManifest manifest;
        // OBS-07: TLE provenance. SetTleEpoch and AddNoradId had no
        // production callers anywhere, so the one manifest the repo shipped
        // recorded an empty epoch and an empty NORAD list even though the
        // producing scenario held both. Reading it off the satellites directly
        // would make this helper depend on ntn-constellation, which closes a
        // build cycle, so the scenario declares it through
        // SetTleProvenance() and the helper carries it into the manifest.
        // ntn-dataset@1: when the scenario did not declare its element sets, read them
        // off every node's SGP4 mobility through its read-only attributes (FromRealTle,
        // NoradId, ElementEpochUnix), so every example records them without code changes.
        // A generated (Walker) shell is recorded as such.
        std::string tleSource = m_tleEpochUtc.empty() ? "none" : "declared";
        if (m_tleEpochUtc.empty() && m_noradIds.empty())
        {
            std::vector<double> epochs;
            uint32_t generated = 0;
            for (uint32_t n = 0; n < NodeList::GetNNodes(); ++n)
            {
                Ptr<MobilityModel> mob = NodeList::GetNode(n)->GetObject<MobilityModel>();
                if (!mob)
                {
                    continue;
                }
                BooleanValue real;
                if (!mob->GetAttributeFailSafe("FromRealTle", real))
                {
                    continue;
                }
                if (!real.Get())
                {
                    ++generated;
                    continue;
                }
                UintegerValue id;
                DoubleValue ep;
                mob->GetAttributeFailSafe("NoradId", id);
                mob->GetAttributeFailSafe("ElementEpochUnix", ep);
                m_noradIds.push_back(static_cast<uint32_t>(id.Get()));
                epochs.push_back(ep.Get());
            }
            if (!epochs.empty())
            {
                std::sort(epochs.begin(), epochs.end());
                const auto iso = [](double u) {
                    std::time_t t = static_cast<std::time_t>(u);
                    char b[32];
                    std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
                    return std::string(b);
                };
                m_tleEpochUtc = iso(epochs[epochs.size() / 2]);
                tleSource = "sgp4-mobility (median of " + std::to_string(epochs.size()) +
                            " element sets, " + iso(epochs.front()) + " to " + iso(epochs.back()) + ")";
            }
            else if (generated > 0)
            {
                tleSource = "generated shell (" + std::to_string(generated) + " satellites, no real TLE)";
            }
        }
        if (!m_tleEpochUtc.empty())
        {
            manifest.SetTleEpoch(m_tleEpochUtc);
        }
        manifest.AddExtra("element_sets", tleSource);
        for (uint32_t id : m_noradIds)
        {
            manifest.AddNoradId(id);
        }
        manifest.SetScenarioName(m_runTag.empty() ? "ntn-real-stack" : m_runTag)
            .SetScenarioDuration(Simulator::Now().GetSeconds())
            .SetRng(RngSeedManager::GetSeed(), RngSeedManager::GetRun())
            .AddExtra("radio_backend",
                      (m_backend == RadioBackend::Nr) ? "nr" : "mmwave")
            .AddExtra("carrier_hz", std::to_string(m_freqHz))
            .AddExtra("duplex", m_fddApplied ? "fdd" : "tdd")
            .AddExtra("bandwidth_hz", std::to_string(m_bwHz))
            .AddExtra("numerology", std::to_string(m_numerology))
            .AddExtra("gnbs", std::to_string(m_gnb.GetN()))
            .AddExtra("ues", std::to_string(m_ue.GetN()))
            .AddExtra("sat_eirp_dbm", std::to_string(m_satEirpDbm))
            .AddExtra("effective_eirp_dbm", std::to_string(GetEffectiveEirpDbm()))
            .AddExtra("mean_dl_sinr_db",
                      std::isnan(m_dlSinrDbMean) ? std::string("nan")
                                                 : std::to_string(m_dlSinrDbMean))
            .AddExtra("mean_dl_tbler", std::to_string(m_dlTblerMean))
            .AddExtra("phy_rx_tb", std::to_string(m_phyRxTb))
            .AddExtra("air_interface", airTag)
            // ntn-dataset@1 (plan/MASTER_PLAN_GLOBE_DATASET_E2E_2026-11.md): the standard
            // release and the specifications the data plane follows, recorded per run.
            .AddExtra("dataset_schema", "ntn-dataset@1")
            .AddExtra("threegpp_release", "Rel-17 NR-NTN")
            .AddExtra("standards",
                      "TS 38.300; TS 38.211; TS 38.213; TS 38.214; TS 38.321; TS 38.331; "
                      "TR 38.811; TR 38.821; RFC 7680")
            .AddExtra("kpm_series",
                      m_aiMonitorPrefix.empty()
                          ? std::string("none")
                          : (std::filesystem::path(m_aiMonitorPrefix).has_parent_path()
                                 ? m_aiMonitorPrefix
                                 : (std::filesystem::path(m_outputDir) / m_aiMonitorPrefix).string()) +
                                "_kpm_series.csv");
        if (!manifest.WriteJson(m_outputDir + "/repro-manifest.json"))
        {
            NS_LOG_WARN("could not write " << m_outputDir << "/repro-manifest.json");
        }
    }

    std::cout << "[sim_health/honest] air=" << airTag << " fc=" << (m_freqHz / 1e9) << "GHz | phy_rx_tb="
              << m_phyRxTb << " (floor " << m_gates.minPhyRxTb << ") | dl_sinr="
              << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << "dB | tbler="
              << m_dlTblerMean << " | corrupt=" << GetDlCorruptFraction() << " | thr="
              << m_rxThroughputMbps << "Mbps | delay=" << m_meanDelayMs << "ms -> "
              << (allOk ? "PASS" : "FAIL") << "\n";

    m_lastGateVerdict = allOk;

    if (m_strictGates && !allOk)
    {
        NS_FATAL_ERROR("Honest fidelity gates failed; see " << m_outputDir << "/sim_health.csv");
    }
}

// ===================== Wave 1 (16 Nov 2026) =====================

void
NtnRealStackHelper::ApplyNs3Tr38811Channel(BandwidthPartInfo* bwp)
{
    // C3: ns-3 mainline TR 38.811 channel on one BWP. One condition model is
    // shared by the path loss and the fast fading so both see the same
    // LOS/NLOS state (TR 38.811 Table 6.6.1-1 elevation-dependent probability).
    const double f = (bwp->m_centralFrequency > 0.0) ? bwp->m_centralFrequency : m_freqHz;
    Ptr<ThreeGppPropagationLossModel> pl;
    Ptr<ThreeGppChannelConditionModel> cond;
    std::string scen;
    switch (m_ntnScenario)
    {
    case 0:
        pl = CreateObject<ThreeGppNTNDenseUrbanPropagationLossModel>();
        cond = CreateObject<ThreeGppNTNDenseUrbanChannelConditionModel>();
        scen = "NTN-DenseUrban";
        break;
    case 1:
        pl = CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
        cond = CreateObject<ThreeGppNTNUrbanChannelConditionModel>();
        scen = "NTN-Urban";
        break;
    case 3:
        pl = CreateObject<ThreeGppNTNRuralPropagationLossModel>();
        cond = CreateObject<ThreeGppNTNRuralChannelConditionModel>();
        scen = "NTN-Rural";
        break;
    case 2:
    default:
        pl = CreateObject<ThreeGppNTNSuburbanPropagationLossModel>();
        cond = CreateObject<ThreeGppNTNSuburbanChannelConditionModel>();
        scen = "NTN-Suburban";
        break;
    }
    cond->SetAttribute("UpdatePeriod", TimeValue(m_channelUpdatePeriod));
    pl->SetAttribute("Frequency", DoubleValue(f));
    pl->SetChannelConditionModel(cond);
    bwp->m_propagation = pl;
    if (m_smallScaleFading)
    {
        Ptr<ThreeGppSpectrumPropagationLossModel> sp =
            CreateObject<ThreeGppSpectrumPropagationLossModel>();
        sp->SetChannelModelAttribute("Frequency", DoubleValue(f));
        sp->SetChannelModelAttribute("Scenario", StringValue(scen));
        sp->SetChannelModelAttribute("ChannelConditionModel", PointerValue(cond));
        sp->SetChannelModelAttribute("UpdatePeriod", TimeValue(m_channelUpdatePeriod));
        bwp->m_3gppChannel = sp;
    }
}

void
NtnRealStackHelper::ApplyTr38821Set1Defaults()
{
    NS_ABORT_MSG_IF(m_built, "ApplyTr38821Set1Defaults must be called before Build()");
    m_set1Applied = true;
    if (m_freqHz <= 7.125e9)
    {
        // TR 38.821 Table 6.1.1.1-2, handheld: 1T2R, 0 dBi, NF 7 dB, 23 dBm.
        m_ueRows = 1;
        m_ueCols = 2;
        m_ueAntennaGainDbi = 0.0;
        m_ueNoiseFigureDb = 7.0;
        m_ueTxDbm = 23.0;
        // Table 6.1.1.1-1, Set-1 LEO-600 S band: 34 dBW/MHz.
        SetSatEirpDensityDbwMhz(kTr38821Set1SBandEirpDensityDbwMhz);
    }
    else
    {
        // Table 6.1.1.1-2, VSAT: 39.7 dBi, 33 dBm, G/T 15.9 dB/K. The stack's
        // noise is k T0 F with T0 = 290 K, so the declared G/T is carried by
        // the equivalent figure F = T_sys / T0 with T_sys = G - G/T.
        m_ueRows = 1;
        m_ueCols = 1;
        m_ueAntennaGainDbi = 39.7;
        const double tSysDbK = 39.7 - 15.9;
        m_ueNoiseFigureDb = tSysDbK - 10.0 * std::log10(290.0);
        m_ueTxDbm = 33.0;
        // Table 6.1.1.1-1, Set-1 LEO-600 Ka band: 4 dBW/MHz.
        SetSatEirpDensityDbwMhz(4.0);
    }
}

Vector
NtnRealStackHelper::GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    // WGS-84 (NIMA TR8350.2): a = 6378137 m, f = 1/298.257223563.
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    return Vector((n + altM) * std::cos(lat) * std::cos(lon),
                  (n + altM) * std::cos(lat) * std::sin(lon),
                  (n * (1.0 - e2) + altM) * std::sin(lat));
}

Ptr<MobilityModel>
NtnRealStackHelper::SetFeederGatewayGeodetic(double latDeg, double lonDeg, double altM)
{
    Ptr<ConstantPositionMobilityModel> gw = CreateObject<ConstantPositionMobilityModel>();
    gw->SetPosition(GeodeticToEcef(latDeg, lonDeg, altM));
    SetFeederGateway(gw);
    return gw;
}

void
NtnRealStackHelper::SetFeederGateway(Ptr<MobilityModel> gwMobility)
{
    NS_ABORT_MSG_IF(!m_built, "SetFeederGateway before Build()");
    NS_ABORT_MSG_IF(!gwMobility, "SetFeederGateway: null mobility");
    m_feederGwTracked = gwMobility;
    m_feederGw = gwMobility;
    RefreshFeederFromServing();
    const Time period = m_backhaulRefresh.IsStrictlyPositive() ? m_backhaulRefresh : Seconds(1);
    RegisterPeriodicCallback(period, [this](Time) { RefreshFeederFromServing(); });
}

void
NtnRealStackHelper::RefreshFeederFromServing()
{
    // C5: feeder = serving satellite of the UEs to the real gateway. With
    // several UEs on different satellites the single PGW-to-remote-host channel
    // carries the mean feeder slant (TR 38.821 Table 4.2-2 one-way = feeder +
    // service); the service leg is folded by ComputePayloadExtraDelay.
    if (!m_feederGwTracked || !m_backhaulCh)
    {
        return;
    }
    double sumM = 0.0;
    uint32_t n = 0;
    Ptr<MobilityModel> firstSat;
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        const uint16_t cell = GetUeServingCellId(u);
        for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
        {
            if (cell != 0 && GetGnbCellId(g) != cell)
            {
                continue;
            }
            Ptr<MobilityModel> sm = m_gnb.Get(g)->GetObject<MobilityModel>();
            if (sm)
            {
                if (!firstSat)
                {
                    firstSat = sm;
                }
                const Vector satE = NtnEarthBlockageLossModel::EcefOf(sm);
                const Vector gwE = NtnEarthBlockageLossModel::EcefOf(m_feederGwTracked);
                sumM += CalculateDistance(satE, gwE);
                ++n;
            }
            break;
        }
    }
    if (n == 0)
    {
        return;
    }
    m_feederSat = firstSat ? firstSat : m_feederSat;
    m_backhaulCh->SetAttribute("Delay", TimeValue(ComputePayloadExtraDelay(sumM / n)));
}

double
NtnRealStackHelper::GetFeederElevationDeg() const
{
    if (!m_feederSat || !m_feederGw)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return NtnEarthBlockageLossModel::GeodeticElevationDeg(
        NtnEarthBlockageLossModel::EcefOf(m_feederGw),
        NtnEarthBlockageLossModel::EcefOf(m_feederSat));
}

void
NtnRealStackHelper::SetPeriodicMeasurementReports(bool enable, Time interval)
{
    NS_ABORT_MSG_IF(m_built, "SetPeriodicMeasurementReports must be called before Build()");
    static const int64_t kAllowedMs[] = {120, 240, 480, 640, 1024, 2048, 5120, 10240};
    bool ok = false;
    for (int64_t v : kAllowedMs)
    {
        ok = ok || (interval == MilliSeconds(v));
    }
    NS_ABORT_MSG_IF(enable && !ok,
                    "SetPeriodicMeasurementReports: " << interval.GetMilliSeconds()
                                                      << " ms is not a TS 38.331 ReportInterval");
    m_periodicMeas = enable;
    m_periodicMeasExplicit = true;
    m_periodicMeasInterval = interval;
}

bool
NtnRealStackHelper::GetPeriodicMeasurementReports() const
{
    return m_periodicMeas;
}

int32_t
NtnRealStackHelper::UeIndexOfImsi(uint64_t imsi) const
{
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        Ptr<NrUeNetDevice> d = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
        if (d && d->GetImsi() == imsi)
        {
            return static_cast<int32_t>(u);
        }
    }
    return -1;
}

int32_t
NtnRealStackHelper::UeIndexOfCellRnti(uint16_t cellId, uint16_t rnti) const
{
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        if (GetUeRnti(u) == rnti && GetUeServingCellId(u) == cellId)
        {
            return static_cast<int32_t>(u);
        }
    }
    return -1;
}

double
NtnRealStackHelper::GetUeNeighbourRsrpDbm(uint32_t ueIndex, uint16_t cellId) const
{
    auto it = m_ueMeas.find(ueIndex);
    if (it == m_ueMeas.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    auto c = it->second.find(cellId);
    return (c == it->second.end()) ? std::numeric_limits<double>::quiet_NaN() : c->second.rsrpDbm;
}

double
NtnRealStackHelper::GetUeNeighbourRsrqDb(uint32_t ueIndex, uint16_t cellId) const
{
    auto it = m_ueMeas.find(ueIndex);
    if (it == m_ueMeas.end())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    auto c = it->second.find(cellId);
    return (c == it->second.end()) ? std::numeric_limits<double>::quiet_NaN() : c->second.rsrqDb;
}

double
NtnRealStackHelper::GetUeServingRsrpDbm(uint32_t ueIndex) const
{
    return GetUeNeighbourRsrpDbm(ueIndex, GetUeServingCellId(ueIndex));
}

double
NtnRealStackHelper::GetUeServingRsrqDb(uint32_t ueIndex) const
{
    return GetUeNeighbourRsrqDb(ueIndex, GetUeServingCellId(ueIndex));
}

Time
NtnRealStackHelper::GetUeMeasReportAge(uint32_t ueIndex, uint16_t cellId) const
{
    auto it = m_ueMeas.find(ueIndex);
    if (it == m_ueMeas.end())
    {
        return Seconds(-1);
    }
    auto c = it->second.find(cellId);
    return (c == it->second.end()) ? Seconds(-1) : Simulator::Now() - c->second.time;
}

uint32_t
NtnRealStackHelper::GetUeMeasurementReportCount(uint32_t ueIndex) const
{
    auto it = m_ueMeasReports.find(ueIndex);
    return (it == m_ueMeasReports.end()) ? 0 : it->second;
}

std::map<uint16_t, NtnRealStackHelper::UeCellMeasurement>
NtnRealStackHelper::GetUeMeasurements(uint32_t ueIndex) const
{
    auto it = m_ueMeas.find(ueIndex);
    return (it == m_ueMeas.end()) ? std::map<uint16_t, UeCellMeasurement>{} : it->second;
}

void
NtnRealStackHelper::EnableUeStatsTrace(const std::string& csvPath, Time period)
{
    NS_ABORT_MSG_IF(period <= Seconds(0), "EnableUeStatsTrace: period must be positive");
    std::error_code ec;
    const std::filesystem::path p(csvPath);
    if (p.has_parent_path())
    {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
    m_ueStatsTrace = std::make_shared<std::ofstream>(csvPath);
    *m_ueStatsTrace << "time_s,ue,serving_cell,rnti,dl_tb,dl_tb_err,recent_sinr_db,"
                       "cell_mean_sinr_db,recent_tbler\n";
    m_ueStatsPeriod = period;
    if (!m_ueStatsEvent.IsPending())
    {
        m_ueStatsEvent = Simulator::Schedule(period, &NtnRealStackHelper::SampleUeStats, this);
    }
}

void
NtnRealStackHelper::SampleUeStats()
{
    if (!m_ueStatsTrace)
    {
        return;
    }
    // Read-only: no window is consumed (TakeUeSinrWindowMeanDb is left to the
    // caller's own decision loop), so enabling the trace cannot change a run.
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        const UeRanCounters c = GetUeRanCounters(u);
        *m_ueStatsTrace << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds()
                        << "," << u << "," << GetUeServingCellId(u) << "," << GetUeRnti(u)
                        << "," << c.dlTb << "," << c.dlTbErr << "," << GetUeRecentSinrDb(u)
                        << "," << GetUeMeanSinrDb(u) << "," << std::setprecision(6)
                        << GetUeRecentTbler(u) << "\n";
    }
    m_ueStatsTrace->flush();
    m_ueStatsEvent = Simulator::Schedule(m_ueStatsPeriod, &NtnRealStackHelper::SampleUeStats, this);
}

void
NtnRealStackHelper::EnableUeKpiDataset(Time period)
{
    NS_ABORT_MSG_IF(period <= Seconds(0), "EnableUeKpiDataset: period must be positive");
    m_ueKpiPeriod = period;
    if (!m_ueKpiEvent.IsPending())
    {
        m_ueKpiEvent = Simulator::Schedule(period, &NtnRealStackHelper::SampleUeKpi, this);
    }
}

void
NtnRealStackHelper::SampleUeKpi()
{
    constexpr double kC = 299792458.0;
    if (Simulator::Now() > m_simTime)
    {
        return;
    }
    if (!m_ueKpiCsv)
    {
        // Opened at the first sample, so the example's SetOutputDir (any time before Run) applies.
        std::error_code ec;
        std::filesystem::create_directories(m_outputDir, ec);
        m_ueKpiCsv = std::make_shared<std::ofstream>(m_outputDir + "/ue_kpi.csv");
        // prb_used_dl_pct is TS 28.552 5.1.1.2.1 RRU.PrbDl (DL PRB usage in
        // percent), i.e. 100 x RRU.PrbUsedDl / RRU.PrbAvailDl (both PRB
        // counts). The column name is kept because dataset consumers map it.
        *m_ueKpiCsv << "time_s,ue,rnti,serving_cell,ss_rsrp_dbm,ss_rsrq_db,sinr_db,mcs_mean,rank_mean,"
                       "prb_used_dl_pct,dl_tb,dl_tb_err,bler,harq_retx,dl_mac_thp_mbps,app_dl_thp_mbps,"
                       "app_rx_pkts,app_lost_pkts,elevation_deg,slant_range_km,rtt_ms,doppler_hz\n";
        m_cellKpiCsv = std::make_shared<std::ofstream>(m_outputDir + "/cell_kpi.csv");
        *m_cellKpiCsv << "time_s,cell_id,active_ues,prb_used_dl_pct,dl_mac_thp_mbps,sinr_db,bler\n";
    }
    const double dt = m_ueKpiPeriod.GetSeconds();
    // Available DL PRB-slots in one period: PRBs of the BWP x slots per second x period.
    const double prbCapacity =
        static_cast<double>(m_nrBandRb) * (1000.0 * (1 << m_numerology)) * dt;
    auto field = [](double v, int prec = 3) {
        std::ostringstream f;
        if (std::isfinite(v))
        {
            f << std::fixed << std::setprecision(prec) << v;
        }
        return f.str();
    };
    struct CellAgg
    {
        uint32_t ues{0};
        double bits{0.0};
        double sinrLin{0.0};
        uint64_t tb{0};
        uint64_t tbErr{0};
    };
    std::map<uint16_t, CellAgg> cells;
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        const UeRanCounters c = GetUeRanCounters(u);
        const UeRanCounters p = m_ueKpiPrev[u];
        const uint64_t tb = c.dlTb - p.dlTb;
        const uint64_t tbErr = c.dlTbErr - p.dlTbErr;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double sinrDb = tb ? 10.0 * std::log10((c.sinrLinSum - p.sinrLinSum) / tb) : nan;
        const double mcs = tb ? (c.sumMcs - p.sumMcs) / tb : nan;
        const double rank = tb ? (c.sumRank - p.sumRank) / tb : nan;
        const double bler = tb ? static_cast<double>(tbErr) / tb : nan;
        const double prbPct = prbCapacity > 0 ? 100.0 * (c.prbSymSum - p.prbSymSum) / prbCapacity : nan;
        const double macMbps = (c.okBits - p.okBits) / dt / 1e6;
        const uint64_t rxB = GetUeRxBytes(u), rxP = GetUeRxPackets(u), lost = GetUeLostPackets(u);
        const double appMbps = 8.0 * (rxB - m_ueKpiPrevRxBytes[u]) / dt / 1e6;
        const uint16_t cell = GetUeServingCellId(u);
        // Geometry to the serving satellite (the gNB whose cell serves this UE).
        double elev = nan, rangeM = nan, doppler = nan;
        Ptr<MobilityModel> um = m_ue.Get(u)->GetObject<MobilityModel>();
        for (uint32_t g = 0; g < m_gnb.GetN() && um; ++g)
        {
            if (cell == 0 || GetGnbCellId(g) != cell)
            {
                continue;
            }
            Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
            if (!gm)
            {
                break;
            }
            const Vector ue = NtnEarthBlockageLossModel::EcefOf(um);
            const Vector sat = NtnEarthBlockageLossModel::EcefOf(gm);
            elev = NtnEarthBlockageLossModel::GeodeticElevationDeg(ue, sat);
            rangeM = CalculateDistance(ue, sat);
            break;
        }
        // The range rate is only defined against the same satellite: across a handover the previous
        // range belongs to the old serving satellite, so the Doppler of that interval stays empty.
        const bool sameSat = m_ueKpiPrevCell.count(u) && m_ueKpiPrevCell[u] == cell;
        if (sameSat && std::isfinite(rangeM) && m_ueKpiPrevRangeM.count(u) && std::isfinite(m_ueKpiPrevRangeM[u]))
        {
            // Doppler of the carrier from the measured range rate: positive while the satellite approaches.
            doppler = -(rangeM - m_ueKpiPrevRangeM[u]) / dt * m_freqHz / kC;
        }
        const double rssp = GetUeServingRsrpDbm(u), rsrq = GetUeServingRsrqDb(u);
        *m_ueKpiCsv << field(Simulator::Now().GetSeconds()) << "," << u << "," << GetUeRnti(u) << ","
                    << cell << "," << field(rssp, 1) << "," << field(rsrq, 1) << "," << field(sinrDb, 2)
                    << "," << field(mcs, 2) << "," << field(rank, 2) << "," << field(prbPct, 2) << ","
                    << tb << "," << tbErr << "," << field(bler, 5) << "," << (c.harqRetx - p.harqRetx)
                    << "," << field(macMbps, 4) << "," << field(appMbps, 4) << ","
                    << (rxP - m_ueKpiPrevRxPkts[u]) << "," << (lost - m_ueKpiPrevLostPkts[u]) << ","
                    << field(elev, 3) << "," << field(rangeM / 1e3, 3) << ","
                    << field(2.0 * rangeM / kC * 1e3, 4) << "," << field(doppler, 1) << "\n";
        if (cell != 0)
        {
            CellAgg& a = cells[cell];
            a.ues += 1;
            a.bits += c.okBits - p.okBits;
            a.sinrLin += c.sinrLinSum - p.sinrLinSum;
            a.tb += tb;
            a.tbErr += tbErr;
        }
        m_ueKpiPrev[u] = c;
        m_ueKpiPrevRxBytes[u] = rxB;
        m_ueKpiPrevRxPkts[u] = rxP;
        m_ueKpiPrevLostPkts[u] = lost;
        m_ueKpiPrevRangeM[u] = rangeM;
        m_ueKpiPrevCell[u] = cell;
    }
    for (const auto& [cell, a] : cells)
    {
        const double used = GetCellDlPrbSlotsUsed(cell);
        const double prbPct = prbCapacity > 0 ? 100.0 * (used - m_ueKpiPrevCellPrb[cell]) / prbCapacity
                                              : std::numeric_limits<double>::quiet_NaN();
        m_ueKpiPrevCellPrb[cell] = used;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        *m_cellKpiCsv << field(Simulator::Now().GetSeconds()) << "," << cell << "," << a.ues << ","
                      << field(prbPct, 2) << "," << field(a.bits / dt / 1e6, 4) << ","
                      << field(a.tb ? 10.0 * std::log10(a.sinrLin / a.tb) : nan, 2) << ","
                      << field(a.tb ? static_cast<double>(a.tbErr) / a.tb : nan, 5) << "\n";
    }
    m_ueKpiCsv->flush();
    m_cellKpiCsv->flush();
    m_ueKpiEvent = Simulator::Schedule(m_ueKpiPeriod, &NtnRealStackHelper::SampleUeKpi, this);
}

void
NtnRealStackHelper::EnableMeasurementTrace(const std::string& csvPath)
{
    m_measTracePath = csvPath;
    std::error_code ec;
    const std::filesystem::path p(csvPath);
    if (p.has_parent_path())
    {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
    m_measTrace = std::make_shared<std::ofstream>(csvPath);
    *m_measTrace << "time_s,ue,imsi,cell,serving,source,rsrp_dbm,rsrq_db,slant_km\n";
    if (m_built && m_backend == RadioBackend::Nr)
    {
        for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
        {
            Ptr<NrUeNetDevice> d = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(u));
            if (!d)
            {
                continue;
            }
            d->GetPhy(0)->TraceConnectWithoutContext(
                "ReportUeMeasurements",
                MakeCallback(&NtnRealStackHelper::NrUePhyMeas, this).Bind(u));
        }
    }
}

void
NtnRealStackHelper::NrUePhyMeas(uint32_t ueIndex,
                                uint16_t /*rnti*/,
                                uint16_t cellId,
                                double rsrp,
                                double rsrq,
                                bool isServing,
                                uint8_t /*ccId*/)
{
    if (!m_measTrace)
    {
        return;
    }
    Ptr<NrUeNetDevice> d = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(ueIndex));
    *m_measTrace << Simulator::Now().GetSeconds() << "," << ueIndex << ","
                 << (d ? d->GetImsi() : 0) << "," << cellId << "," << (isServing ? 1 : 0)
                 << ",l1," << rsrp << "," << rsrq << "," << SlantKmToCell(ueIndex, cellId) << "\n";
}

double
NtnRealStackHelper::SlantKmToCell(uint32_t ueIndex, uint16_t cellId) const
{
    if (ueIndex >= m_ue.GetN())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    Ptr<MobilityModel> um = m_ue.Get(ueIndex)->GetObject<MobilityModel>();
    for (uint32_t g = 0; g < m_gnb.GetN(); ++g)
    {
        if (GetGnbCellId(g) == cellId)
        {
            Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
            return (um && gm) ? gm->GetDistanceFrom(um) / 1e3
                              : std::numeric_limits<double>::quiet_NaN();
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

uint32_t
NtnRealStackHelper::GetPingPongCount(Time window) const
{
    // TS 38.300 15.5.2.2 (MRO): a handover back to the cell the UE just left,
    // within the window after the previous handover of the same UE.
    uint32_t n = 0;
    std::map<uint32_t, const HoRecord*> last;
    for (const auto& r : m_hoRecords)
    {
        auto it = last.find(r.ue);
        if (it != last.end() && r.target == it->second->source && r.t - it->second->t <= window)
        {
            ++n;
        }
        last[r.ue] = &r;
    }
    return n;
}

NtnRealStackHelper::UeRanCounters
NtnRealStackHelper::GetUeRanCounters(uint32_t ueIndex) const
{
    auto it = m_ueRan.find(ueIndex);
    return (it == m_ueRan.end()) ? UeRanCounters{} : it->second;
}

double
NtnRealStackHelper::GetCellDlPrbSlotsUsed(uint16_t cellId) const
{
    auto it = m_cellPrbSlots.find(cellId);
    return (it == m_cellPrbSlots.end()) ? 0.0 : it->second;
}

void
NtnRealStackHelper::NrUeRrcConnTrace(std::string /*ctx*/,
                                     uint64_t /*imsi*/,
                                     uint16_t /*cellId*/,
                                     uint16_t /*rnti*/)
{
    // DRBs exist only after the RRC (re)configuration; hook any new PDCP.
    Simulator::ScheduleNow(&NtnRealStackHelper::ConnectUePdcpTraces, this);
}

void
NtnRealStackHelper::ConnectUePdcpTraces()
{
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        std::ostringstream path;
        path << "/NodeList/" << m_ue.Get(u)->GetId()
             << "/DeviceList/*/NrUeRrc/DataRadioBearerMap/*/NrPdcp";
        Config::MatchContainer mc = Config::LookupMatches(path.str());
        for (uint32_t i = 0; i < mc.GetN(); ++i)
        {
            Ptr<Object> pdcp = mc.Get(i);
            if (m_pdcpConnected.insert(PeekPointer(pdcp)).second)
            {
                pdcp->TraceConnectWithoutContext(
                    "RxPDU",
                    MakeCallback(&NtnRealStackHelper::UePdcpRx, this).Bind(u));
            }
        }
    }
}

void
NtnRealStackHelper::UePdcpRx(uint32_t ueIndex,
                             uint16_t /*rnti*/,
                             uint8_t /*lcid*/,
                             uint32_t /*size*/,
                             uint64_t delayNs)
{
    // The NrPdcpTag is stamped when the gNB PDCP hands the PDU (the RLC SDU)
    // to its RLC, so this is the gNB RLC ingress to UE delivery time.
    UeRanCounters& c = m_ueRan[ueIndex];
    c.rlcSduDelaySumMs += static_cast<double>(delayNs) / 1e6;
    c.rlcSdus += 1;
}

void
NtnRealStackHelper::SetSsbCellBeam(bool enable, Ptr<MobilityModel> cellCenter, Time refresh)
{
    NS_ABORT_MSG_IF(m_built, "SetSsbCellBeam must be called before Build()");
    m_ssbCellBeam = enable;
    m_cellBeamCenter = cellCenter;
    m_cellBeamRefresh = refresh;
}

void
NtnRealStackHelper::RefreshCellBeams()
{
    if (m_backend != RadioBackend::Nr || !m_nr)
    {
        return;
    }
    const uint8_t nBwp = GetNrBwpCount();
    if (m_nadirCenters.size() != m_enbDevs.GetN())
    {
        m_nadirCenters.clear();
        for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
        {
            m_nadirCenters.push_back(CreateObject<ConstantPositionMobilityModel>());
        }
    }
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        Ptr<MobilityModel> gm = m_gnb.Get(i)->GetObject<MobilityModel>();
        if (!gm)
        {
            continue;
        }
        Ptr<MobilityModel> target = m_cellBeamCenter;
        if (!target)
        {
            // Earth-moving beam: the sub-satellite point on the WGS-84 surface
            // (the radial projection is within 0.2 deg of the geodetic nadir).
            const Vector p = gm->GetPosition();
            const double r = p.GetLength();
            if (r < 1.0)
            {
                continue;
            }
            const double lat = std::asin(p.z / r) * 180.0 / M_PI;
            const double lon = std::atan2(p.y, p.x) * 180.0 / M_PI;
            m_nadirCenters[i]->SetPosition(GeodeticToEcef(lat, lon, 0.0));
            target = m_nadirCenters[i];
        }
        // Wave 11: the SSB grid's body frame. nadir n (towards the Earth's
        // center), e1 along-track (velocity without its radial part), e2 = n x e1.
        std::vector<Ptr<MobilityModel>>* gridTargets = nullptr;
        if (m_ssbGridApplied && !m_cellBeamCenter && i < m_ssbGridTargets.size())
        {
            const Vector p = gm->GetPosition();
            const double r = p.GetLength();
            const Vector n(-p.x / r, -p.y / r, -p.z / r);
            Vector v = gm->GetVelocity();
            const double vn = v.x * n.x + v.y * n.y + v.z * n.z;
            Vector e1(v.x - vn * n.x, v.y - vn * n.y, v.z - vn * n.z);
            double l1 = e1.GetLength();
            if (l1 < 1e-9)
            {
                // No velocity (a static gNB): any direction normal to nadir.
                e1 = (std::abs(n.z) < 0.9) ? Vector(-n.y, n.x, 0.0) : Vector(0.0, -n.z, n.y);
                l1 = e1.GetLength();
            }
            e1 = Vector(e1.x / l1, e1.y / l1, e1.z / l1);
            const Vector e2(n.y * e1.z - n.z * e1.y, n.z * e1.x - n.x * e1.z, n.x * e1.y - n.y * e1.x);
            const double reach = 1.0e6; // any point on the ray: only the direction matters
            gridTargets = &m_ssbGridTargets[i];
            for (size_t k = 0; k < m_ssbGridUv.size(); ++k)
            {
                const double u = m_ssbGridUv[k].first;
                const double w = m_ssbGridUv[k].second;
                const double c = std::sqrt(std::max(0.0, 1.0 - u * u - w * w));
                const Vector d(u * e1.x + w * e2.x + c * n.x,
                               u * e1.y + w * e2.y + c * n.y,
                               u * e1.z + w * e2.z + c * n.z);
                (*gridTargets)[k]->SetPosition(
                    Vector(p.x + reach * d.x, p.y + reach * d.y, p.z + reach * d.z));
            }
        }
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrGnbPhy> gphy = m_nr->GetGnbPhy(m_enbDevs.Get(i), b);
            Ptr<NrSpectrumPhy> sp = gphy->GetSpectrumPhy();
            Ptr<const UniformPlanarArray> upa = sp->GetAntenna()->GetObject<UniformPlanarArray>();
            if (!upa)
            {
                continue;
            }
            sp->GetBeamManager()->SetPredefinedBeam(CreateDirectPathBfv(gm, target, upa));
            if (gridTargets && b < m_nrDlBwpCount)
            {
                // nr patch 19: SSB index k on beam k (index 0 is the nadir beam,
                // which also stays the predefined beam for UL control and the
                // data fallback).
                std::vector<PhasedArrayModel::ComplexVector> sweep;
                sweep.reserve(gridTargets->size());
                for (const auto& t : *gridTargets)
                {
                    sweep.push_back(CreateDirectPathBfv(gm, t, upa));
                }
                gphy->SetSsbBeamSweep(sweep);
            }
        }
    }
    if (m_cellBeamRefresh.IsStrictlyPositive())
    {
        Simulator::Schedule(m_cellBeamRefresh, &NtnRealStackHelper::RefreshCellBeams, this);
    }
}

// ---- Wave 11: frequency reuse and the SSB beam grid --------------------------

uint8_t
NtnRealStackHelper::ResolveFrequencyReuse(uint32_t nGnb) const
{
    if (m_backend != RadioBackend::Nr)
    {
        return 1;
    }
    uint8_t k = m_frf;
    if (!m_frfSet)
    {
        StringValue gv;
        GlobalValue::GetValueByName("NtnRealStackFrequencyReuse", gv);
        const std::string v = gv.Get();
        NS_ABORT_MSG_IF(v != "Auto" && v != "1" && v != "3" && v != "4",
                        "NtnRealStackFrequencyReuse must be Auto, 1, 3 or 4, got " << v);
        k = (v == "Auto") ? 0 : static_cast<uint8_t>(std::stoi(v));
        if (GetLinkBudgetProfile() == LinkBudgetProfile::Legacy)
        {
            k = 1; // Legacy reproduces the reuse-1 runs
        }
    }
    if (k == 0)
    {
        // TR 38.821 Table 6.1.1.1-5 evaluates FRF 1 and FRF 3; Table 6.1.1.2-1
        // gives FRF 3 the better geometry SINR (SC10 median 8.2 dB against
        // -1.1 dB for FRF 1, SC9). A single cell has no co-channel neighbor
        // in the run, so it keeps the whole carrier.
        k = (GetLinkBudgetProfile() == LinkBudgetProfile::Tr38821Set1 && nGnb >= 2) ? 3 : 1;
    }
    NS_ABORT_MSG_IF(k != 1 && k != 3 && k != 4,
                    "SetFrequencyReuse: FRF must be 1, 3 or 4 (TR 38.821 6.1.1.1), got " << +k);
    return k;
}

void
NtnRealStackHelper::ApplyFrequencyReuse()
{
    const uint32_t nGnb = m_enbDevs.GetN();
    m_frfApplied = 1;
    m_cellColor.assign(nGnb, 0);
    m_cellRbRange.assign(nGnb, {});
    const uint8_t k = ResolveFrequencyReuse(nGnb);
    if (k == 1)
    {
        return;
    }
    m_frfApplied = k;
    const uint8_t nBwpAll = m_nrBwpTotal;
    for (uint32_t i = 0; i < nGnb; ++i)
    {
        auto it = m_cellColorExplicit.find(i);
        const uint8_t c = (it != m_cellColorExplicit.end()) ? it->second
                                                            : static_cast<uint8_t>(i % k);
        NS_ABORT_MSG_IF(c >= k, "SetCellReuseColor: color " << +c << " >= FRF " << +k);
        m_cellColor[i] = c;
        m_cellRbRange[i].assign(m_nrDlBwpCount, {0, 0});
        for (uint8_t b = 0; b < nBwpAll; ++b)
        {
            Ptr<NrGnbPhy> phy = m_nr->GetGnbPhy(m_enbDevs.Get(i), b);
            Ptr<NrGnbMac> mac = NrHelper::GetGnbMac(m_enbDevs.Get(i), b);
            Ptr<NrMacSchedulerNs3> sched =
                DynamicCast<NrMacSchedulerNs3>(m_nr->GetScheduler(m_enbDevs.Get(i), b));
            NS_ABORT_MSG_IF(!phy || !mac || !sched, "frequency reuse needs an nr ns-3 scheduler");
            const uint32_t nRb = phy->GetRbNum();
            const uint32_t rbPerRbg = std::max<uint32_t>(1, mac->GetNumRbPerRbg());
            const uint32_t nRbg = nRb / rbPerRbg;
            NS_ABORT_MSG_IF(nRbg < k,
                            "frequency reuse " << +k << ": BWP " << +b << " has only " << nRbg
                                               << " RBGs");
            // Equal RBG shares, contiguous: sub-band c = RBGs [c G/k, (c+1) G/k).
            const uint32_t g0 = c * nRbg / k;
            const uint32_t g1 = (c + 1) * nRbg / k;
            std::vector<uint8_t> mask(nRbg, 0);
            for (uint32_t g = g0; g < g1; ++g)
            {
                mask[g] = 1;
            }
            const uint32_t rbFirst = g0 * rbPerRbg;
            const uint32_t rbCount = (g1 - g0) * rbPerRbg;
            const bool dl = !m_fddApplied || b < m_nrDlBwpCount;
            const bool ul = !m_fddApplied || b >= m_nrDlBwpCount;
            if (dl)
            {
                // PDSCH only in the sub-band (nr's notched-RBG scheduling) and
                // the control region, PSS/SSB included, only there (nr patch 19).
                sched->SetDlNotchedRbgMask(mask);
                phy->SetCellRbRange(rbFirst, rbCount);
                m_cellRbRange[i][b] = {rbFirst, rbCount};
            }
            if (ul)
            {
                // The cell grants PUSCH only in its sub-band.
                sched->SetUlNotchedRbgMask(mask);
            }
            // Constant EIRP density (TR 38.821 Table 6.1.1.1-1 states the
            // payload as dBW/MHz): the cell radiates the carrier's power times
            // its share of the RBs. With nr's power over the used RBs this keeps
            // the energy per RE of the PDSCH and of the SSB equal to FRF 1.
            const double share = static_cast<double>(rbCount) / static_cast<double>(nRb);
            phy->SetAttribute("TxPower",
                              DoubleValue(m_bwpPowerApplied[b % m_nrDlBwpCount] +
                                          10.0 * std::log10(share)));
        }
    }
    // TS 38.215 5.1.1: SS-RSRP is the LINEAR average of the power of the REs
    // that carry the SSB. nr's default L1 average is taken in dB over a uint8_t
    // sample count (patch 08 notes): over the 17 RBs of a sub-band the per-sample
    // spread is larger than over the whole carrier, and that dB average put the
    // serving cell 6 dB under its neighbors often enough for A3 (6 dB, 1024 ms)
    // to cycle three cells 94 times in ntn-cho-full-constellation (none with the
    // linear average, measured). So reuse turns the linear average on, as the
    // SSB cell beam already does.
    if (m_prevRsrpLinear < 0)
    {
        BooleanValue prev;
        GlobalValue::GetValueByName("NrUeRsrpLinearAverage", prev);
        m_prevRsrpLinear = prev.Get() ? 1 : 0;
        GlobalValue::Bind("NrUeRsrpLinearAverage", BooleanValue(true));
    }
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        for (uint8_t b = 0; b < nBwpAll; ++b)
        {
            m_nr->GetUePhy(m_ueDevs.Get(u), b)
                ->SetAttribute("SsRsrpOnOccupiedRbs", BooleanValue(true));
        }
    }
    // The PRBs a cell can use (RRU.PrbAvailDl) are its sub-band.
    m_nrBandRb = std::max<uint32_t>(1, m_nrBandRb / k);
    NS_LOG_INFO("NtnRealStackHelper: frequency reuse " << +k << " over " << nGnb
                                                      << " cells (TR 38.821 6.1.1.1)");
}

std::pair<uint32_t, uint32_t>
NtnRealStackHelper::GetCellRbRange(uint32_t gnbIndex, uint8_t bwp) const
{
    if (gnbIndex >= m_cellRbRange.size() || bwp >= m_cellRbRange[gnbIndex].size())
    {
        return {0, 0};
    }
    return m_cellRbRange[gnbIndex][bwp];
}

void
NtnRealStackHelper::BuildSsbBeamGrid()
{
    m_ssbGridApplied = false;
    m_ssbGridSize = 0;
    m_ssbGridUv.clear();
    m_ssbGridTargets.clear();
    if (m_cellBeamCenter || m_backend != RadioBackend::Nr)
    {
        return; // Earth-fixed cell: one beam steered at the cell center
    }
    bool on = m_ssbGrid;
    if (!m_ssbGridSet)
    {
        BooleanValue gv;
        GlobalValue::GetValueByName("NtnRealStackSsbBeamGrid", gv);
        on = gv.Get() && GetLinkBudgetProfile() != LinkBudgetProfile::Legacy;
    }
    if (!on || m_enbDevs.GetN() == 0)
    {
        return;
    }
    // Half-power beamwidth of an N-element half-wavelength array in
    // direction-cosine space, about 0.886 x 2 / N; hexagonal spacing sqrt(3)/2 of
    // it makes adjacent 3 dB contours touch (no coverage hole on the grid).
    const double nEl = std::max<double>(1.0, std::max(m_gnbRows, m_gnbCols));
    double delta = std::sqrt(3.0) / 2.0 * 0.886 * 2.0 / nEl;
    // Field of view down to the horizon of the lowest satellite.
    double sinLimb = 0.0;
    for (uint32_t i = 0; i < m_gnb.GetN(); ++i)
    {
        Ptr<MobilityModel> gm = m_gnb.Get(i)->GetObject<MobilityModel>();
        const double r = gm ? gm->GetPosition().GetLength() : 0.0;
        if (r > 6371.0e3)
        {
            sinLimb = std::max(sinLimb, 6371.0e3 / r);
        }
    }
    if (sinLimb <= 0.0)
    {
        return; // not an orbital geometry
    }
    std::vector<std::tuple<int, double, double, double>> pts; // ring, angle, u, v
    for (;;)
    {
        pts.clear();
        const int rings = static_cast<int>(std::ceil(sinLimb / delta));
        for (int b = -rings; b <= rings; ++b)
        {
            for (int a = -rings; a <= rings; ++a)
            {
                const int ring = std::max({std::abs(a), std::abs(b), std::abs(a + b)});
                if (ring > rings)
                {
                    continue;
                }
                const double u = delta * (a + 0.5 * b);
                const double v = delta * (std::sqrt(3.0) / 2.0 * b);
                if (u * u + v * v > sinLimb * sinLimb)
                {
                    continue;
                }
                pts.emplace_back(ring, std::atan2(v, u), u, v);
            }
        }
        if (pts.size() <= 254)
        {
            break;
        }
        delta *= 1.1; // nr patch 19 carries at most 255 SSB indices
    }
    std::sort(pts.begin(), pts.end());
    for (const auto& p : pts)
    {
        m_ssbGridUv.emplace_back(std::get<2>(p), std::get<3>(p));
    }
    m_ssbGridSize = static_cast<uint32_t>(m_ssbGridUv.size());
    m_ssbGridTargets.resize(m_enbDevs.GetN());
    for (auto& t : m_ssbGridTargets)
    {
        for (uint32_t k = 0; k < m_ssbGridSize; ++k)
        {
            t.push_back(CreateObject<ConstantPositionMobilityModel>());
        }
    }
    // The UE takes its uplink path-loss reference from the best SSB of a burst.
    // nr patch 21: the sweep visits one SSB index per downlink control region,
    // so a grid of a few hundred beams takes longer than the 200 ms layer-1
    // period to come round, and periods that missed the best beam reported the
    // cell 6 to 8 dB low (seen as dips about once a second on a fixed UE).
    // Each beam's last result is therefore kept for two sweeps (at least two
    // layer-1 periods) until the beam is measured again.
    const Time slot = MilliSeconds(1) / (1 << m_numerology);
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        for (uint8_t b = 0; b < m_nrBwpTotal; ++b)
        {
            Ptr<NrUePhy> uphy = m_nr->GetUePhy(m_ueDevs.Get(u), b);
            uphy->SetAttribute("SsbBeamsPerBurst", UintegerValue(m_ssbGridSize));
            TimeValue l1;
            uphy->GetAttribute("UeMeasurementsFilterPeriod", l1);
            uphy->SetAttribute("SsbBeamResultLifetime",
                               TimeValue(std::max(2 * l1.Get(), 2 * m_ssbGridSize * slot)));
        }
    }
    m_ssbGridApplied = true;
    NS_LOG_INFO("NtnRealStackHelper: Earth-moving SSB beam grid, " << m_ssbGridSize
                                                                   << " beams per satellite, spacing "
                                                                   << delta << " in u/v");
}

// ---- Wave 6A ---------------------------------------------------------------

NtnRealStackHelper::LinkBudgetProfile
NtnRealStackHelper::GetLinkBudgetProfile() const
{
    if (m_linkBudgetProfileExplicit)
    {
        return m_linkBudgetProfile;
    }
    StringValue v;
    GlobalValue::GetValueByName("NtnRealStackLinkBudget", v);
    if (v.Get() == "Legacy")
    {
        return LinkBudgetProfile::Legacy;
    }
    NS_ABORT_MSG_IF(v.Get() != "Tr38821Set1",
                    "NtnRealStackLinkBudget must be Tr38821Set1 or Legacy, got " << v.Get());
    return LinkBudgetProfile::Tr38821Set1;
}

void
NtnRealStackHelper::ApplyDefaultLinkBudget()
{
    // Root cause of the 29-30 dB DL SINR in the published LEO datasets: runs
    // that took the helper/example defaults radiated 70 dBm conducted + 18 dB
    // UPA gain = 88 dBm over 20 MHz (75 dBm/MHz = 45 dBW/MHz, 11 dB above the
    // TR 38.821 Set-1 S-band payload) into nr's 5 dB noise figure (the
    // handheld is 7 dB). Hand budget at nadir, LEO-600, 2.185 GHz:
    //   FSPL 154.8 dB; noise -174 + 60 + NF dBm/MHz; UE 1x2 array +3 dB.
    //   legacy 70 dBm: 75 - 154.8 + 3 + 109 = 32.2 dB (29.5 dB after the
    //   TR 38.811 excess terms), Set-1: 64 - 154.8 + 3 + 107 = 19.2 dB, and
    //   at 30 deg elevation (FSPL 159.1 dB) with the TR 38.821 Table
    //   6.1.3.3-1 margins (shadow 3, scintillation 2.2, gas 0.1) about 9 dB.
    if (m_backend != RadioBackend::Nr || m_set1Applied ||
        GetLinkBudgetProfile() != LinkBudgetProfile::Tr38821Set1)
    {
        return;
    }
    const bool sBand = m_freqHz <= 7.125e9;
    if (sBand)
    {
        // TR 38.821 Table 6.1.1.1-3 handheld; m_ueRows x m_ueCols is already the
        // 1x2 (two receive branches) panel unless the scenario changed it.
        if (!std::isfinite(m_ueNoiseFigureDb))
        {
            m_ueNoiseFigureDb = 7.0;
        }
        if (!m_ueTxExplicit)
        {
            m_ueTxDbm = 23.0;
        }
        if (!m_satPowerExplicit)
        {
            SetSatEirpDensityDbwMhz(kTr38821Set1SBandEirpDensityDbwMhz);
            m_defaultSet1Payload = true;
        }
    }
    else if (!m_satPowerExplicit)
    {
        // Ka: the VSAT terminal only together with the Set-1 Ka payload.
        if (!m_ueGainExplicit)
        {
            m_ueRows = 1;
            m_ueCols = 1;
            m_ueAntennaGainDbi = 39.7;
        }
        if (!std::isfinite(m_ueNoiseFigureDb))
        {
            m_ueNoiseFigureDb = (39.7 - 15.9) - 10.0 * std::log10(290.0);
        }
        if (!m_ueTxExplicit)
        {
            m_ueTxDbm = 33.0;
        }
        SetSatEirpDensityDbwMhz(4.0);
        m_defaultSet1Payload = true;
    }
    NS_LOG_INFO("NtnRealStackHelper: default TR 38.821 Set-1 link budget ("
                << (sBand ? "S-band handheld" : "Ka VSAT") << "), payload "
                << (m_defaultSet1Payload ? "Set-1 density" : "declared by the scenario")
                << ", UE NF " << m_ueNoiseFigureDb << " dB, UE Tx " << m_ueTxDbm << " dBm");
}

Time
NtnRealStackHelper::ComputeUeGnbRtt(uint32_t ueIndex) const
{
    // TS 38.321 3: UE-gNB RTT = T_TA + k_mac. The UE's full TA pre-compensates
    // the service-link round trip; k_mac is 0 because the gNB is on board
    // (DL and UL frames aligned at the satellite in this model).
    constexpr double kC = 299792458.0;
    double slantKm = SlantKmToCell(ueIndex, GetUeServingCellId(ueIndex));
    if (!std::isfinite(slantKm))
    {
        Ptr<MobilityModel> um = m_ue.Get(ueIndex)->GetObject<MobilityModel>();
        double best = std::numeric_limits<double>::infinity();
        for (uint32_t g = 0; g < m_gnb.GetN() && um; ++g)
        {
            Ptr<MobilityModel> gm = m_gnb.Get(g)->GetObject<MobilityModel>();
            if (gm)
            {
                best = std::min(best, gm->GetDistanceFrom(um) / 1e3);
            }
        }
        slantKm = best;
    }
    if (!std::isfinite(slantKm))
    {
        return Time();
    }
    return Seconds(2.0 * slantKm * 1e3 / kC);
}

void
NtnRealStackHelper::ApplyRaWindowOffsets()
{
    if (m_backend != RadioBackend::Nr)
    {
        return;
    }
    const uint8_t nBwp = GetNrBwpCount();
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrUeMac> mac = NrHelper::GetUeMac(m_ueDevs.Get(u), b);
            if (!mac)
            {
                continue;
            }
            mac->TraceConnectWithoutContext(
                "RarOutsideWindow",
                MakeCallback(+[](uint32_t* n, uint64_t) { ++(*n); }).Bind(&m_rarOutsideWindow));
            mac->TraceConnectWithoutContext(
                "RaResponseTimeout",
                MakeCallback(+[](uint32_t* n, uint64_t, bool, uint8_t, uint8_t) { ++(*n); })
                    .Bind(&m_raResponseTimeouts));
        }
    }
    if (!m_ntnRaWindowOffset || !m_airIfaceDelayActive)
    {
        return; // the MAC experiences no round trip: offset stays 0
    }
    RefreshRaWindowOffsets();
}

void
NtnRealStackHelper::RefreshRaWindowOffsets()
{
    const uint8_t nBwp = GetNrBwpCount();
    for (uint32_t u = 0; u < m_ueDevs.GetN(); ++u)
    {
        const Time rtt = ComputeUeGnbRtt(u);
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrUeMac> mac = NrHelper::GetUeMac(m_ueDevs.Get(u), b);
            if (mac)
            {
                mac->SetAttribute("RaResponseWindowStartOffset", TimeValue(rtt));
            }
        }
    }
    Simulator::Schedule(Seconds(1.0), &NtnRealStackHelper::RefreshRaWindowOffsets, this);
}

Time
NtnRealStackHelper::GetRaWindowStartOffset(uint32_t ueIndex) const
{
    if (m_backend != RadioBackend::Nr || ueIndex >= m_ueDevs.GetN())
    {
        return Time();
    }
    Ptr<NrUeMac> mac = NrHelper::GetUeMac(m_ueDevs.Get(ueIndex), GetUlBwpIndex());
    if (!mac)
    {
        return Time();
    }
    TimeValue v;
    mac->GetAttribute("RaResponseWindowStartOffset", v);
    return v.Get();
}

uint64_t
NtnRealStackHelper::GetDlHarqFeedbackDisabledTxCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    const uint8_t nBwp = GetNrBwpCount();
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrMacSchedulerNs3> sch =
                DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(m_enbDevs.Get(i), b));
            if (sch)
            {
                n += sch->GetDlHarqFeedbackDisabledTxCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetDlHarqFeedbackSuppressedCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        if (!dev)
        {
            continue;
        }
        for (uint32_t b = 0; b < dev->GetCcMapSize(); ++b)
        {
            Ptr<NrUePhy> phy = dev->GetPhy(b);
            if (phy)
            {
                n += phy->GetDlHarqFeedbackSuppressedCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetDlSoftBufferFlushCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        if (!dev)
        {
            continue;
        }
        for (uint32_t b = 0; b < dev->GetCcMapSize(); ++b)
        {
            Ptr<NrUePhy> phy = dev->GetPhy(b);
            if (phy && phy->GetSpectrumPhy())
            {
                n += phy->GetSpectrumPhy()->GetDlSoftBufferFlushCount();
            }
        }
    }
    return n;
}

Ipv4Address
NtnRealStackHelper::GetUeIpv4Address(uint32_t ueIndex) const
{
    NS_ABORT_MSG_IF(ueIndex >= m_ueAddrs.size(), "GetUeIpv4Address: UE index out of range");
    return m_ueAddrs[ueIndex];
}

bool
NtnRealStackHelper::AttachExternalDlFlow(uint32_t ueIdx,
                                         Ptr<Application> client,
                                         Ptr<Application> sink,
                                         Time start,
                                         Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "AttachExternalDlFlow before Build()");
    NS_ABORT_MSG_IF(ueIdx >= m_ue.GetN(), "AttachExternalDlFlow: UE index out of range");
    NS_ABORT_MSG_IF(!sink, "AttachExternalDlFlow: null sink");
    // Throughput window: the union of every flow's emission window.
    if (m_trafficStop <= m_trafficStart)
    {
        m_trafficStart = start;
        m_trafficStop = stop;
    }
    else
    {
        m_trafficStart = std::min(m_trafficStart, start);
        m_trafficStop = std::max(m_trafficStop, stop);
    }
    m_dlSinks.Add(sink);
    m_dlSinkUe.push_back(ueIdx);
    const bool rxOk =
        sink->TraceConnectWithoutContext("Rx", MakeCallback(&NtnRealStackHelper::DlSinkRx, this));
    const uint32_t flow = static_cast<uint32_t>(m_extFlows.size());
    ExtFlowState st;
    st.ue = ueIdx;
    st.sinkIndex = m_dlSinks.GetN() - 1;
    m_extFlows.push_back(st);
    sink->TraceConnectWithoutContext("Rx",
                                     MakeCallback(&NtnRealStackHelper::ExtSinkRx, this).Bind(flow));
    // PacketSink fires this only when it parses an in-band SeqTsSizeHeader.
    sink->TraceConnectWithoutContext(
        "RxWithSeqTsSize",
        MakeCallback(&NtnRealStackHelper::ExtSinkRxSeq, this).Bind(flow));
    bool txOk = false;
    if (client)
    {
        txOk = client->TraceConnectWithoutContext(
            "Tx",
            MakeCallback(&NtnRealStackHelper::DlClientTx, this));
        if (txOk)
        {
            client->TraceConnectWithoutContext(
                "Tx",
                MakeCallback(&NtnRealStackHelper::ExtClientTx, this).Bind(flow));
        }
    }
    if (m_backend == RadioBackend::Nr)
    {
        // Idempotent (m_pdcpConnected), so several flows may schedule it.
        Simulator::Schedule(std::max(Seconds(0), start - Simulator::Now()),
                            &NtnRealStackHelper::ConnectUePdcpTraces,
                            this);
    }
    if (m_wallStartNs == 0)
    {
        m_wallStartNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    }
    return rxOk && txOk;
}

void
NtnRealStackHelper::ExtSinkRx(uint32_t flow, Ptr<const Packet> p, const Address&)
{
    ++m_extFlows[flow].rxPkts;
    m_extFlows[flow].rxBytes += p->GetSize();
}

uint64_t
NtnRealStackHelper::ExtRxBytesOfSink(uint32_t sinkIndex) const
{
    for (const auto& f : m_extFlows)
    {
        if (f.sinkIndex == sinkIndex)
        {
            return f.rxBytes;
        }
    }
    return 0;
}

void
NtnRealStackHelper::ExtSinkRxSeq(uint32_t flow,
                                 Ptr<const Packet>,
                                 const Address&,
                                 const Address&,
                                 const SeqTsSizeHeader& h)
{
    ExtFlowState& f = m_extFlows[flow];
    const double d = (Simulator::Now() - h.GetTs()).GetSeconds() * 1e3;
    f.sumDelayMs += d;
    f.maxDelayMs = std::max(f.maxDelayMs, d);
    if (f.owdHistFine.empty())
    {
        f.owdHistFine.assign(NtnOranSink::FlowStats::kFineBins, 0u);
    }
    const uint32_t fbin = static_cast<uint32_t>(std::max(0.0, d) / NtnOranSink::FlowStats::kFineBinMs);
    if (fbin < NtnOranSink::FlowStats::kFineBins)
    {
        f.owdHistFine[fbin]++;
    }
    else
    {
        f.owdFineOverflow++;
    }
    if (f.anySeq && h.GetSeq() == f.lastSeq + 1)
    {
        const double absIpdv = std::fabs(d - f.lastDelayMs);
        f.ipdvSumAbsMs += absIpdv;
        ++f.ipdvN;
        if (f.ipdvHistAbs.empty())
        {
            f.ipdvHistAbs.assign(NtnOranSink::FlowStats::kIpdvBins, 0u);
        }
        const uint32_t ibin = static_cast<uint32_t>(absIpdv / NtnOranSink::FlowStats::kIpdvBinMs);
        if (ibin < NtnOranSink::FlowStats::kIpdvBins)
        {
            f.ipdvHistAbs[ibin]++;
        }
        else
        {
            f.ipdvOverflow++;
        }
    }
    LossRecordRx(f.lossRxBins, h.GetTs(), Simulator::Now() - h.GetTs());
    f.lastSeq = h.GetSeq();
    f.lastDelayMs = d;
    ++f.seqRx;
    f.highestSeq = f.anySeq ? std::max<uint64_t>(f.highestSeq, h.GetSeq()) : h.GetSeq();
    f.anySeq = true;
}

uint64_t
NtnRealStackHelper::GetPhrSentCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        if (!dev)
        {
            continue;
        }
        for (uint32_t b = 0; b < dev->GetCcMapSize(); ++b)
        {
            Ptr<NrUeMac> mac = DynamicCast<NrUeMac>(dev->GetMac(b));
            if (mac)
            {
                n += mac->GetPhrSentCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetPhrReceivedCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    const uint8_t nBwp = GetNrBwpCount();
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrMacSchedulerNs3> sch =
                DynamicCast<NrMacSchedulerNs3>(NrHelper::GetScheduler(m_enbDevs.Get(i), b));
            if (sch)
            {
                n += sch->GetPhrReceivedCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetBsrRetriggerCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        if (!dev)
        {
            continue;
        }
        for (uint32_t b = 0; b < dev->GetCcMapSize(); ++b)
        {
            Ptr<NrUeMac> mac = DynamicCast<NrUeMac>(dev->GetMac(b));
            if (mac)
            {
                n += mac->GetBsrRetriggerCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetHandoverAbortedDlRxCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr || !m_nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        for (uint32_t b = 0; dev && b < dev->GetCcMapSize(); ++b)
        {
            if (Ptr<NrUePhy> phy = dev->GetPhy(b))
            {
                n += phy->GetSpectrumPhy()->GetDlDataRxAbortedCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetHandoverFlushedReportCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr || !m_nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        for (uint32_t b = 0; dev && b < dev->GetCcMapSize(); ++b)
        {
            if (Ptr<NrUePhy> phy = dev->GetPhy(b))
            {
                n += phy->GetHandoverFlushedReportCount();
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetBsrSentCount(uint8_t type) const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        for (uint32_t b = 0; dev && b < dev->GetCcMapSize(); ++b)
        {
            if (Ptr<NrUeMac> mac = DynamicCast<NrUeMac>(dev->GetMac(b)))
            {
                n += mac->GetBsrSentCount(type);
            }
        }
    }
    return n;
}

uint64_t
NtnRealStackHelper::GetUlNewTxGrantCount() const
{
    uint64_t n = 0;
    if (m_backend != RadioBackend::Nr)
    {
        return 0;
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> dev = DynamicCast<NrUeNetDevice>(m_ueDevs.Get(i));
        for (uint32_t b = 0; dev && b < dev->GetCcMapSize(); ++b)
        {
            if (Ptr<NrUeMac> mac = DynamicCast<NrUeMac>(dev->GetMac(b)))
            {
                n += mac->GetUlNewTxGrantCount();
            }
        }
    }
    return n;
}

} // namespace ns3
