// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-real-stack-helper.h"

#include "ns3/ntn-spectrum-seam-model.h"

#include "ns3/abort.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/fatal-error.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/epc-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-ue-net-device.h"
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
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-phy-mac-common.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-sat-beam-gain-model.h"
#include "ns3/ntn-tr38811-excess-loss-model.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace ns3
{

using namespace mmwave;

NS_LOG_COMPONENT_DEFINE("NtnRealStackHelper");

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
NtnRealStackHelper::~NtnRealStackHelper() = default;

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
    const double t300Ms = std::max(100.0, 8.0 * rttMs);
    Config::SetDefault("ns3::LteRlcAm::PollRetransmitTimer", TimeValue(MilliSeconds(pollMs)));
    Config::SetDefault("ns3::LteRlcAm::ReorderingTimer", TimeValue(MilliSeconds(reorderMs)));
    Config::SetDefault("ns3::LteRlcAm::StatusProhibitTimer", TimeValue(MilliSeconds(statusMs)));
    Config::SetDefault("ns3::LteUeRrc::T300", TimeValue(MilliSeconds(t300Ms)));
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
}

void
NtnRealStackHelper::BuildMmwaveRadio()
{
    // ---- NTN-ize the NR air interface via Config defaults (read at CreateObject
    //      time inside MmWaveHelper::DoInitialize) -------------------------------
    // NB: mmwave registers its TypeIds WITHOUT the mmwave:: namespace.
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(m_freqHz));
    Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(m_bwHz));
    Config::SetDefault("ns3::MmWaveEnbPhy::TxPower", DoubleValue(m_satEirpDbm));
    Config::SetDefault("ns3::MmWaveUePhy::TxPower", DoubleValue(m_ueTxDbm));
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

void
NtnRealStackHelper::ApplyNtnChannelExtras()
{
    // ---- G1: chain the TR 38.811 large-scale EXCESS-loss terms onto the
    //      measured channel (after Friis FSPL), so the measured SINR carries the
    //      elevation-dependent atmospheric/clutter/shadow/scintillation physics
    //      that previously lived only in the ntn-measurement-model oracle. ----
    if (m_tr38811)
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
}

void
NtnRealStackHelper::BuildNrRadio()
{
    // ===== 5G-LENA (nr) FR1 NTN backend (closes A5(i)) =====
    // Mirrors the validated NtnNrStackHelper recipe: a single operational band ->
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
        break; // NR default
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
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(m_freqHz,
                                                   m_bwHz,
                                                   nBwp,
                                                   BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Quasi-static channel, no shadowing -> clean link over the slant.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    m_nr->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
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
    m_nrBaseLossPerBwp.assign(nBwp, nullptr);
    for (uint8_t cc = 0; cc < nBwp; ++cc)
    {
        BandwidthPartInfoPtr& bwp = band.GetBwpAt(cc, 0);
        Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
        // Each BWP sits at its own centre frequency; using the band centre for
        // all of them mis-scales Friis across a wide multi-BWP band.
        const double bwpFreqHz = (bwp->m_centralFrequency > 0.0) ? bwp->m_centralFrequency : m_freqHz;
        friis->SetAttribute("Frequency", DoubleValue(bwpFreqHz));
        bwp->m_propagation = friis;
        m_nrBaseLossPerBwp[cc] = friis;
        if (cc == 0)
        {
            m_nrBaseLoss = friis; // back-compat head
        }
    }

    m_nr->InitializeOperationBand(&band);

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
    for (uint8_t cc = 0; cc < nBwp; ++cc)
    {
        BandwidthPartInfoPtr& bwp = band.GetBwpAt(cc, 0);
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
        for (uint8_t cc = 0; cc < nBwp; ++cc)
        {
            BandwidthPartInfoPtr& bwp = band.GetBwpAt(cc, 0);
            Ptr<SpectrumChannel> ch = bwp->m_channel;
            if (ch && !ch->GetPropagationDelayModel())
            {
                ch->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
                m_airIfaceDelayActive = true;
            }
        }
        NS_LOG_WARN("nr backend: real air-interface propagation delay ACTIVE by request. The "
                    "vendored 5G-LENA v3.3 has no K_offset/TA, so this is only safe for a "
                    "single UE with downlink-only traffic; anything else will abort with "
                    "'Cannot TX while RX' or a UL-alignment assert. See gap R1/R3.");
    }

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

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
    for (uint8_t i = 0; i < m_slices.size(); ++i)
    {
        const std::string attr = QciAttrName(m_slices[i].fiveQi);
        if (!attr.empty())
        {
            m_nr->SetGnbBwpManagerAlgorithmAttribute(attr, UintegerValue(i));
            m_nr->SetUeBwpManagerAlgorithmAttribute(attr, UintegerValue(i));
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
    m_nr->SetUeAntennaAttribute("AntennaElement",
                                PointerValue(CreateObject<IsotropicAntennaModel>()));
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

    // Install devices on the caller's mobility-carrying nodes.
    m_enbDevs = m_nr->InstallGnbDevice(m_gnb, allBwps);
    m_ueDevs = m_nr->InstallUeDevice(m_ue, allBwps);
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
    const double bwpPowerSplitDb = 10.0 * std::log10(static_cast<double>(nBwp));
    const double conductedPerBwpDbm = m_satEirpDbm - bwpPowerSplitDb;
    for (uint32_t i = 0; i < m_enbDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                ->SetAttribute("Numerology", UintegerValue(m_numerology));
            m_nr->GetGnbPhy(m_enbDevs.Get(i), b)
                ->SetAttribute("TxPower", DoubleValue(conductedPerBwpDbm));
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
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            m_nr->GetUePhy(m_ueDevs.Get(i), b)->SetAttribute("TxPower", DoubleValue(m_ueTxDbm));
        }
    }
    m_nr->UpdateDeviceConfigs(m_enbDevs);
    m_nr->UpdateDeviceConfigs(m_ueDevs);

    // ---- Enabler A: instantiate + cross-wire the A3-RSRP handover algorithm.
    // This is the wiring the vendored NrHelper omits (see the note above). It
    // must run AFTER InstallGnbDevice -- the RRC now knows its component-carrier
    // count, which AddUeMeasReportConfig needs -- and BEFORE AttachToClosestGnb,
    // so the EVENT_A3 report config that the algorithm's DoInitialize() registers
    // on the gNB RRC is delivered to the UE in its connection reconfiguration.
    // Without this, SetHandoverAlgorithmType alone leaves the RRC on the no-op
    // SAP and the UE never emits neighbour measurement reports (handover count
    // stays 0 regardless of geometry).
    if (m_handover && m_gnb.GetN() >= 2)
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

    // Enabler D: native 5G-LENA stat calculators (per-DRB PDCP/RLC
    // throughput+delay, per-slot MAC MCS/PRB, PHY RxPacketTrace) written under
    // the output dir. The measured MCS/rank/PRB the helper exposes are captured
    // separately in DlRxTraceNr(); EnableTraces() adds the authoritative file
    // dump an O-RAN KPM / observability sink can also read.
    if (m_nrNativeTraces)
    {
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
        const Time x2Delay = ComputeX2LinkDelay();
        Config::SetDefault("ns3::NrNoBackhaulEpcHelper::X2LinkDelay", TimeValue(x2Delay));
        NS_LOG_INFO("NtnRealStackHelper X2 (inter-satellite) one-way delay: "
                    << x2Delay.GetMilliSeconds() << " ms");
        m_nr->AddX2Interface(m_gnb);
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                        MakeCallback(&NtnRealStackHelper::NrHandoverEndOk, this));
    }

    // ---- Measured-KPI PHY sink: DL SINR/TBLER from the UE NrSpectrumPhy ----
    // Feeds the SAME accumulators as the mmwave path via AccumulateDl(). Connect
    // the trace on EVERY BWP the UE carries (nBwp) so per-slice (per-BWP) TBs on
    // BWPs 1..N-1 are measured too — not only the primary BWP 0.
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        for (uint8_t b = 0; b < nBwp; ++b)
        {
            Ptr<NrSpectrumPhy> sp = m_nr->GetUePhy(m_ueDevs.Get(i), b)->GetSpectrumPhy();
            sp->TraceConnectWithoutContext("RxPacketTraceUe",
                                           MakeCallback(&NtnRealStackHelper::DlRxTraceNr, this));
        }
    }

    // S2: when the air interface could not take the slant (multi-UE without TA),
    // seed the backhaul with it so the measured user-plane OWD is still correct
    // and the OWD-floor health gate stays meaningful. Mirrors the mmwave path.
    if (!m_airIfaceDelayActive && m_backhaulCh)
    {
        const Time service = ComputeServiceLinkDelay();
        if (service > Seconds(0))
        {
            m_backhaulCh->SetAttribute("Delay", TimeValue(m_backhaulDelay + service));
            NS_LOG_INFO("nr backend: folded service-link delay " << service.GetMilliSeconds()
                                                                 << " ms into the backhaul leg");
        }
    }

    NS_LOG_INFO("NtnRealStackHelper nr backend: " << m_enbDevs.GetN() << " gNB, " << m_ueDevs.GetN()
                                                  << " UE, fc=" << m_freqHz / 1e9 << " GHz, BW="
                                                  << m_bwHz / 1e6 << " MHz, numerology="
                                                  << m_numerology
                                                  << ", air-interface delay="
                                                  << (m_airIfaceDelayActive ? "REAL" : "on-backhaul"));
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
NtnRealStackHelper::NrHandoverEndOk(std::string /*ctx*/,
                                    uint64_t /*imsi*/,
                                    uint16_t /*cellId*/,
                                    uint16_t /*rnti*/)
{
    ++m_hoCount;
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
    if (!m_ntnHarqProfile)
    {
        return;
    }
    constexpr double kC = 299792458.0;
    const double slantM = WorstCaseSlantM();       // metres (>=600 km fallback)
    const double rttS = 2.0 * slantM / kC;         // round trip
    const double slotS = 1.0e-3 / std::pow(2.0, m_numerology); // NR slot duration
    constexpr double kRounds = 4.0;                // initial TX + 3 retx in flight
    double n = std::ceil(rttS / slotS) + kRounds;
    n = std::max(20.0, std::min(255.0, n));        // NumHarqProcess is uint8_t
    m_nr->SetGnbMacAttribute("NumHarqProcess",
                             UintegerValue(static_cast<uint32_t>(n)));
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

void
NtnRealStackHelper::InstallTraffic(TrafficProfile profile, Time start, Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "InstallTraffic before Build()");

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
        client->SetAttribute("PacketSize", UintegerValue(256));
        client->SetAttribute("Period", TimeValue(MilliSeconds(10))); // ~205 kbps
        break;
    case NtnOranApplication::CONVERSATIONAL_VOICE:
        client->SetAttribute("PacketSize", UintegerValue(92)); // AMR-WB + RTP-ish
        break;
    case NtnOranApplication::EMBB_VIDEO:
    case NtnOranApplication::POISSON_BACKGROUND:
    case NtnOranApplication::CBR_SATURATING:
    default:
        client->SetAttribute("DataRate", DataRateValue(DataRate("5Mb/s")));
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

bool
NtnRealStackHelper::TriggerHandover(uint32_t ueIndex, uint16_t targetCellId, Time when)
{
    // P1 / gap H2: the actuation bridge. Every failure path below WARNs with the
    // reason — a decision module must never believe it actuated when it did not
    // (that silent-success pattern is exactly what this fix exists to kill).
    if (!m_built)
    {
        NS_LOG_WARN("TriggerHandover before Build() — ignored");
        return false;
    }
    if (m_backend != RadioBackend::Nr)
    {
        NS_LOG_WARN("TriggerHandover requires the nr backend (SetRadioBackend(Nr)); the vendored "
                    "mmwave path has no equivalent X2 handover request — not actuated");
        return false;
    }
    if (!m_handover)
    {
        NS_LOG_WARN("TriggerHandover called but handover was never enabled (SetHandover(true) "
                    "before Build() stands up the X2) — not actuated");
        return false;
    }
    if (m_enbDevs.GetN() < 2)
    {
        NS_LOG_WARN("TriggerHandover needs >= 2 gNBs (have " << m_enbDevs.GetN()
                                                             << ") — not actuated");
        return false;
    }
    if (ueIndex >= m_ueDevs.GetN())
    {
        NS_LOG_WARN("TriggerHandover: UE index " << ueIndex << " out of range — not actuated");
        return false;
    }
    const uint16_t servingCell = GetUeServingCellId(ueIndex);
    if (servingCell == 0)
    {
        NS_LOG_WARN("TriggerHandover: UE " << ueIndex << " is not attached yet — not actuated");
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
NtnRealStackHelper::ComputeServiceLinkDelay() const
{
    // GAP S2: one-way UE<->satellite (service link) propagation.
    // TR 38.821 Table 4.2-2: LEO-600 one-way service-link delay 2.0-6.44 ms.
    constexpr double kC = 299792458.0;
    if (m_gnb.GetN() == 0 || m_ue.GetN() == 0)
    {
        return Seconds(0);
    }
    Ptr<MobilityModel> gm = m_gnb.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> um = m_ue.Get(0)->GetObject<MobilityModel>();
    if (!gm || !um)
    {
        return Seconds(0);
    }
    return Seconds(gm->GetDistanceFrom(um) / kC);
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
        // Bent-pipe: the user plane rides the RF feeder leg too.
        return prop + prop + service;
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
    const std::filesystem::path prefix(m_aiMonitorPrefix);
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
NtnRealStackHelper::ComputeX2LinkDelay() const
{
    // S8: one-way inter-gNB (inter-satellite) delay from the live geometry.
    //  - FullGnb (regenerative, Rel-19): the Xn/X2 rides a direct ISL, so the
    //    delay is the inter-satellite range / c.
    //  - Transparent payload: both "gNBs" are on the ground behind their feeder
    //    links, so the X2 loop is 2 x feeder slant / c plus the core hop.
    // Falls back to m_backhaulDelay when the geometry is unavailable.
    if (m_gnb.GetN() < 2)
    {
        return m_backhaulDelay;
    }
    Ptr<MobilityModel> a = m_gnb.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> b = m_gnb.Get(1)->GetObject<MobilityModel>();
    if (!a || !b)
    {
        return m_backhaulDelay;
    }
    const double islRangeM = a->GetDistanceFrom(b);
    const double c = 299792458.0;
    if (m_payload == PayloadOption::FullGnb)
    {
        return Seconds(islRangeM / c);
    }
    // Transparent: the inter-gNB path goes down one feeder and up the other.
    return m_backhaulDelay + m_backhaulDelay;
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
    // Attribute not readable on this backend (nr): fall back to the value probe.
    // A live model reports a finite TBLER (~1e-8 even on a clean link); a
    // disabled one writes exactly 0.0 for every TB. This can false-fail an
    // extremely clean link, so treat "no TB samples at all" as inconclusive and
    // let the separate provenance gate judge that case.
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
    if (m_gnb.GetN() == 0)
    {
        return 0.0;
    }
    Ptr<MobilityModel> gm = m_gnb.Get(0)->GetObject<MobilityModel>();
    if (!gm)
    {
        return 0.0;
    }
    const Vector p = gm->GetPosition();
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    // ECEF (|r| ~ Earth radius + altitude) vs a local/ENU frame (z = altitude).
    const double kEarthR = 6371000.0;
    double altM = (r > 6.0e6) ? (r - kEarthR) : p.z;
    if (altM <= 0.0 || !std::isfinite(altM))
    {
        return 0.0;
    }
    const double c = 299792458.0;
    return (altM / c) * 1000.0 + m_backhaulDelay.GetSeconds() * 1000.0;
}

double
NtnRealStackHelper::GetEffectiveEirpDbm() const
{
    // Radiated EIRP = conducted power at the array input + array gain, less the
    // per-BWP power split. Mirrors exactly what BuildNrRadio programs.
    const double nBwp = static_cast<double>(std::max<size_t>(1, m_slices.size()));
    return m_satEirpDbm - 10.0 * std::log10(nBwp) + ArrayGainDb();
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
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (oranSink)
        {
            dlRxBytes += oranSink->GetTotalRx();
            for (const auto& [key, fs] : oranSink->GetFlowStats())
            {
                sumDelayMs += fs.sumDelayMs;
                delaySamples += fs.rxPackets;
                if (fs.rxPackets > 1)
                {
                    sumJitterMs += fs.jitterMs;
                    ++jitterFlows;
                }
                lostPkts += fs.LostPackets();
                expectedPkts += static_cast<uint64_t>(fs.highestSeq) + 1;
            }
            continue;
        }
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(i));
        if (sink)
        {
            dlRxBytes += sink->GetTotalRx();
        }
    }
    double activeS = std::max(1e-6, m_simTime.GetSeconds());
    m_rxThroughputMbps = (dlRxBytes * 8.0) / activeS / 1e6;
    // m_appTxPackets / m_appRxPackets are accumulated live by the app traces.
    m_meanDelayMs = delaySamples ? sumDelayMs / delaySamples : 0.0;
    m_meanJitterMs = jitterFlows ? sumJitterMs / jitterFlows : 0.0;
    m_appLossRatio =
        expectedPkts ? static_cast<double>(lostPkts) / expectedPkts : 0.0;
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

    bool allOk = gateStackDepth && gateThroughput && gateProvenance && gateErrorModel &&
                 channelInPath && gateOwdFloor;

    const char* airTag = (m_backend == RadioBackend::Nr) ? "nr-fr1-ntn" : "mmwave-ntn";

    std::error_code ec;
    std::filesystem::create_directories(m_outputDir, ec);
    std::ofstream out(m_outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass,provenance\n";
    out << "sim_time_s," << m_simTime.GetSeconds() << "," << m_simTime.GetSeconds()
        << ",1,config\n";
    out << "wall_clock_s," << wallSec << ",-,1,measured\n";
    out << "air_interface," << airTag << ",-,1,config\n";
    out << "carrier_hz," << m_freqHz << ",-,1,config\n";
    out << "channel_in_path," << (channelInPath ? 1 : 0) << ",1,"
        << (channelInPath ? 1 : 0) << ",topology\n";
    out << "phy_rx_tb," << m_phyRxTb << "," << m_gates.minPhyRxTb << ","
        << (gateStackDepth ? 1 : 0) << ",phy-trace\n";
    out << "dl_sinr_db," << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << ",-,"
        << (gateProvenance ? 1 : 0) << ",phy-trace\n";
    out << "dl_tbler_mean," << m_dlTblerMean << ",-," << (gateErrorModel ? 1 : 0)
        << ",phy-trace\n";
    out << "dl_corrupt_frac," << GetDlCorruptFraction() << ",-,1,phy-trace\n";
    out << "rx_throughput_mbps," << m_rxThroughputMbps << "," << m_gates.minRxThroughputMbps
        << "," << (gateThroughput ? 1 : 0) << ",packetsink\n";
    double delivery = (m_appTxPackets > 0)
                          ? static_cast<double>(m_appRxPackets) / m_appTxPackets
                          : 0.0;
    out << "app_delivery_ratio," << delivery << ",-,1,app-trace\n";
    // S9: real floor + real pass flag (was: floor "-", pass hard-coded 1).
    out << "app_owd_ms," << m_meanDelayMs << "," << owdFloorMs << "," << (gateOwdFloor ? 1 : 0)
        << ",inband-timestamp\n";
    out << "app_jitter_ms," << m_meanJitterMs << ",-,1,inband-timestamp\n";
    out << "app_loss_ratio," << m_appLossRatio << ",-,1,inband-seq\n";
    out << "effective_eirp_dbm," << GetEffectiveEirpDbm() << ",-,1,derived\n";
    out << "app_tx_pkts," << m_appTxPackets << ",-,1,app-trace\n";
    out << "app_rx_pkts," << m_appRxPackets << ",-,1,app-trace\n";
    out << "ues," << m_ue.GetN() << ",-,1,config\n";
    out << "gnbs," << m_gnb.GetN() << ",-,1,config\n";
    out << "run_tag," << m_runTag << ",-,1,config\n";
    out.close();

    std::cout << "[sim_health/honest] air=" << airTag << " fc=" << (m_freqHz / 1e9) << "GHz | phy_rx_tb="
              << m_phyRxTb << " (floor " << m_gates.minPhyRxTb << ") | dl_sinr="
              << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << "dB | tbler="
              << m_dlTblerMean << " | corrupt=" << GetDlCorruptFraction() << " | thr="
              << m_rxThroughputMbps << "Mbps | delay=" << m_meanDelayMs << "ms -> "
              << (allOk ? "PASS" : "FAIL") << "\n";

    if (m_strictGates && !allOk)
    {
        NS_FATAL_ERROR("Honest fidelity gates failed; see " << m_outputDir << "/sim_health.csv");
    }
}

} // namespace ns3
