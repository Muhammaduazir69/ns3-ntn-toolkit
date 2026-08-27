// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-scalability-load-regimes — reviewer-response experiment R2.6.
//
// WHY THIS EXAMPLE EXISTS
// -----------------------
// The toolkit paper reports that "1,584 satellites completed a 300-second
// simulation within 32 seconds" while also stating that the platform is
// single-process and that a constellation-scale packet plane with more than a
// few thousand active streams would need distributed simulation. Read together,
// those two sentences invite an obvious question: what was the OFFERED LOAD in
// the 1,584-satellite run? If it was one lightweight telemetry report per
// terminal per second, the 32 s figure says nothing about an eMBB regime.
//
// This example replaces the unbounded headline with a CHARACTERISED OPERATING
// ENVELOPE. Every configuration reports its own offered load, measured from the
// application byte/packet counters that actually ran, plus wall clock, peak RSS
// and a measured decision about which plane the run was bound by.
//
// WHAT IS MEASURED (nothing here is asserted from a formula)
// ---------------------------------------------------------
//   offered load   NtnOranApplication::GetTxPackets()/GetTxBytes() — the real
//                  emission counters of the real 3GPP traffic profiles. The
//                  per-terminal packet rate, mean packet size and offered bit
//                  rate are all divisions of those counters by the active
//                  window; none of them is read back from the configuration.
//   delivered      NtnOranSink::GetRxPackets()/GetFlowStats() — bytes that
//                  crossed the whole stack, with the one-way delay taken from
//                  the in-band NtnOranPayloadHeader TX timestamp.
//   wall clock     std::chrono::steady_clock around scenario construction and
//                  Simulator::Run().
//   peak RSS       /proc/self/status VmHWM (the kernel's own high-water mark).
//   bound class    the measured ratio of two runs of the SAME scenario, one
//                  with the measured NR/mmwave cell attached and one without.
//
// THE RUNTIME REGIME SPLIT (the crux of the reviewer's question)
// -------------------------------------------------------------
// A constellation-scale NTN run has two cost centres that scale with different
// knobs:
//   * the GEOMETRY / CONTROL plane — propagating N orbits and re-deciding the
//     serving satellite for every terminal on every control tick. This is what
//     "1,584 satellites" stresses. It is O(sats x terminals x ticks) and it is
//     completely independent of how many bits the terminals send.
//   * the RADIO / PACKET plane — the SpectrumPhy, MAC scheduler, error model
//     and per-packet stack traversal. This is what "eMBB" stresses. It is
//     O(packets) and completely independent of how many satellites exist.
// A single wall-clock number cannot be generalised across both. So this example
// runs the same scenario twice in one invocation:
//     phase A (radio_attached = 0): the N-satellite geometry/control plane plus
//         an identical application load carried on a plain point-to-point
//         access plane (no air interface). NOTE: phase A is a RUNTIME REFERENCE
//         only — its delivery ratio and OWD are properties of a wired reference
//         link, not radio measurements.
//     phase B (radio_attached = 1): the identical geometry/control plane with
//         the measured NR/mmwave cell (NtnRealStackHelper) underneath.
// bound_class then follows from the measured split
//     radio_share = (wall_B - wall_A) / wall_B
// and is "radio-bound" when radio_share >= --boundThreshold (default 0.5),
// "geometry-control-bound" otherwise. Both wall-clock numbers are written to
// the CSV as separate rows, so the crossover is visible as data rather than as
// a claim.
//
// USAGE
// -----
//   ./ns3 run "ntn-scalability-load-regimes --regime=telemetry --sats=1584 \
//              --ues=30 --duration=300 --outputDir=scal-out"
//   ./ns3 run "ntn-scalability-load-regimes --regime=embb --sats=66 --ues=30 \
//              --duration=60 --outputDir=scal-out"
//
// Each invocation appends its rows to <outputDir>/scalability_regimes.csv, so a
// sweep is just a shell loop over --regime / --sats / --ues / --duration.
//
// CAVEAT ON peak_rss_mb IN --phase=both. VmHWM is a per-PROCESS high-water mark,
// so the phase-B row inherits whatever phase A had already touched. For a clean
// per-configuration RSS, run the phases as separate processes:
//     ./ns3 run "... --phase=control"                  -> reports wall_A
//     ./ns3 run "... --phase=radio --controlWallRef=<wall_A>"
// --controlWallRef feeds the externally measured phase-A wall clock back in, so
// bound_class is still decided by a measured ratio and never by assumption. With
// only one phase and no reference, bound_class is written as "undetermined".
//
// The traffic regimes are the EXISTING NtnOranApplication profiles with the
// EXISTING NtnRealStackHelper presets — no new traffic model is introduced:
//   telemetry  MMTC_PERIODIC   5QI 9   128 B / 64 ms   (mMTC class)
//   cbr        CBR_SATURATING  5QI 2  1400 B @ 1 Mb/s  (moderate CBR)
//   embb       CBR_SATURATING  5QI 2  1400 B @ 5 Mb/s  (saturating, the
//                                                       helper's own eMBB rate)
//   urllc      URLLC_PERIODIC  5QI 82  256 B / 10 ms
//   voice      CONVERSATIONAL_VOICE 5QI 1  92 B / 20 ms
//   mixed      telemetry / embb / urllc round-robin across terminals

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NtnScalabilityLoadRegimes");

namespace
{

constexpr double kC = 299792458.0;

// ---------------------------------------------------------------------------
// Real process measurements (never estimated)
// ---------------------------------------------------------------------------

/// Peak resident set size in MB, read from the kernel's own high-water mark
/// (/proc/self/status VmHWM). Returns -1 where /proc is unavailable, which is
/// then written verbatim to the CSV rather than replaced by a guess.
double
PeakRssMb()
{
    std::ifstream f("/proc/self/status");
    if (!f.is_open())
    {
        return -1.0;
    }
    std::string line;
    while (std::getline(f, line))
    {
        if (line.rfind("VmHWM:", 0) == 0)
        {
            std::istringstream is(line.substr(6));
            double kb = 0.0;
            is >> kb;
            return kb / 1024.0;
        }
    }
    return -1.0;
}

/// Monotonic wall clock in seconds.
double
WallNow()
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// Traffic regimes — thin selectors over the EXISTING NtnOranApplication
// profiles and the EXISTING NtnRealStackHelper::InstallOranFlow presets.
// ---------------------------------------------------------------------------

struct RegimeSpec
{
    std::string name;
    NtnOranApplication::Profile profile;
    uint8_t fiveQi;
    uint32_t pktSize;   ///< bytes, incl. the 24 B in-band NtnOranPayloadHeader
    Time period;        ///< Seconds(0) = rate-driven (profile preset applies)
    std::string dataRate; ///< empty = not a rate-driven profile
};

RegimeSpec
TelemetrySpec()
{
    // NtnRealStackHelper::InstallOranFlow preset for MMTC_PERIODIC.
    return {"telemetry", NtnOranApplication::MMTC_PERIODIC, 9, 128, MilliSeconds(64), ""};
}

RegimeSpec
UrllcSpec()
{
    return {"urllc", NtnOranApplication::URLLC_PERIODIC, 82, 256, MilliSeconds(10), ""};
}

RegimeSpec
VoiceSpec()
{
    // The 20 ms vocoder cadence is the profile preset; Period stays 0.
    return {"voice", NtnOranApplication::CONVERSATIONAL_VOICE, 1, 92, Seconds(0), ""};
}

RegimeSpec
CbrSpec()
{
    return {"cbr", NtnOranApplication::CBR_SATURATING, 2, 1400, Seconds(0), "1Mb/s"};
}

RegimeSpec
EmbbSpec()
{
    // The helper's own eMBB / saturating rate (InstallOranFlow default).
    return {"embb", NtnOranApplication::CBR_SATURATING, 2, 1400, Seconds(0), "5Mb/s"};
}

/// Regime for terminal \p ueIdx. Only "mixed" varies across terminals.
RegimeSpec
LookupRegime(const std::string& regime, uint32_t ueIdx)
{
    if (regime == "telemetry")
    {
        return TelemetrySpec();
    }
    if (regime == "cbr")
    {
        return CbrSpec();
    }
    if (regime == "embb")
    {
        return EmbbSpec();
    }
    if (regime == "urllc")
    {
        return UrllcSpec();
    }
    if (regime == "voice")
    {
        return VoiceSpec();
    }
    if (regime == "mixed")
    {
        switch (ueIdx % 3)
        {
        case 0:
            return TelemetrySpec();
        case 1:
            return EmbbSpec();
        default:
            return UrllcSpec();
        }
    }
    NS_ABORT_MSG("unknown --regime '" << regime
                                      << "' (telemetry|cbr|embb|urllc|voice|mixed)");
    return TelemetrySpec();
}

// ---------------------------------------------------------------------------
// Scenario configuration
// ---------------------------------------------------------------------------

struct Cfg
{
    std::string regime{"telemetry"};
    uint32_t sats{66};
    uint32_t ues{30};
    double duration{60.0};
    std::string radio{"mmwave"}; ///< mmwave | nr | none
    std::string phase{"both"};   ///< both | control | radio
    uint32_t planes{0};          ///< 0 = auto (largest divisor <= 72)
    double altKm{780.0};
    // R1.2: the remaining Walker-Delta (T/P/F) degrees of freedom, exposed so a
    // cross-platform comparison can pin an IDENTICAL shell on every simulator.
    double inclinationDeg{53.0}; ///< Walker inclination
    uint32_t phasingF{1};        ///< Walker phasing factor F in T/P/F
    double eccentricity{0.0};    ///< 0 = circular shell
    double argPerigeeDeg{0.0};   ///< argument of perigee
    /// Reference epoch (Unix seconds). Default 2025-01-01T00:00:00Z. Set this
    /// explicitly when matching another platform — e.g. Hypatia pins its
    /// generated TLE epoch to 2000-01-01T00:00:00Z (946684800).
    double epochUnix{1735689600.0};
    double controlHz{1.0};
    double freqGhz{2.0};
    double satEirpDbm{-1.0}; ///< -1 = backend default (mmwave 55, nr 70)
    uint32_t rngRun{1};
    // Offered-load overrides (0 / <0 / "" = keep the regime preset).
    uint32_t pktSizeOverride{0};
    double periodMsOverride{-1.0};
    std::string dataRateOverride;
    std::string outputDir{"ntn-scalability-out"};
    std::string csvName{"scalability_regimes.csv"};
    bool append{true};
    double boundThreshold{0.5};
    double controlWallRef{-1.0}; ///< externally supplied phase-A wall clock (s)
};

/// Plane count for a Walker-Delta shell of \p sats: WalkerConstellation requires
/// T divisible by P, so pick the largest divisor <= 72 (Starlink shell 1 is
/// 1584 = 72 x 22).
uint32_t
AutoPlanes(uint32_t sats)
{
    for (uint32_t p = std::min<uint32_t>(72, sats); p >= 1; --p)
    {
        if (sats % p == 0)
        {
            return p;
        }
    }
    return 1;
}

// ---------------------------------------------------------------------------
// Geometry / control plane — the cost centre that "1,584 satellites" stresses.
//
// Every control tick propagates all N orbits and re-decides the serving
// satellite for every terminal from the real ECEF geometry (elevation angle).
// This is deliberately the same shape of work an NTN control loop (CHO
// candidate evaluation, beam/cell selection, RIC handover xApp) performs, and
// it runs IDENTICALLY in both phases so that the phase difference isolates the
// radio.
// ---------------------------------------------------------------------------
class GeometryControlPlane
{
  public:
    void Init(std::vector<Ptr<MobilityModel>> sats,
              std::vector<Ptr<MobilityModel>> ues,
              Time period,
              Time stop)
    {
        m_sats = std::move(sats);
        m_ues = std::move(ues);
        m_period = period;
        m_stop = stop;
        m_serving.assign(m_ues.size(), 0);
        m_satEcef.resize(m_sats.size());
    }

    void Start() { Tick(); }

    uint64_t GetEvalCount() const { return m_evals; }
    uint64_t GetTickCount() const { return m_ticks; }
    uint32_t GetServing(uint32_t ueIdx) const
    {
        return (ueIdx < m_serving.size()) ? m_serving[ueIdx] : 0;
    }

    /// R1.2 cross-platform "reaction" metric: how many times a terminal's
    /// best-elevation serving satellite CHANGED over the run, summed across
    /// terminals. The first assignment on tick 0 is not a reassignment.
    ///
    /// This is deliberately NOT called a "handover": it is the serving-link
    /// reassignment event, which is the largest honest common denominator
    /// across platforms that have no 3GPP handover procedure (e.g. Hypatia
    /// switches ground-station links with no RAN state machine). Comparing it
    /// against a 3GPP handover count would compare two different physical
    /// events; see R1_2_COMPARISON_PLAN.md.
    uint64_t GetServingReassignments() const { return m_reassigns; }

  private:
    void Tick()
    {
        // 1) Propagate every orbit once for this tick (ECEF, real Kepler+J2).
        for (std::size_t s = 0; s < m_sats.size(); ++s)
        {
            m_satEcef[s] = m_sats[s]->GetPosition();
        }
        // 2) Re-decide the serving satellite of every terminal from the real
        //    elevation geometry.
        for (std::size_t u = 0; u < m_ues.size(); ++u)
        {
            const Vector ue = m_ues[u]->GetPosition();
            double bestEl = -1e9;
            std::size_t best = 0;
            for (std::size_t s = 0; s < m_satEcef.size(); ++s)
            {
                const double el = ntngeo::ElevationDeg(ue, m_satEcef[s]);
                if (el > bestEl)
                {
                    bestEl = el;
                    best = s;
                }
            }
            const auto chosen = static_cast<uint32_t>(best);
            // Count a serving-link reassignment only after the initial
            // assignment (tick 0 establishes the link, it does not change it).
            if (m_ticks > 0 && chosen != m_serving[u])
            {
                ++m_reassigns;
            }
            m_serving[u] = chosen;
        }
        m_evals += static_cast<uint64_t>(m_sats.size()) * m_ues.size();
        ++m_ticks;

        if (Simulator::Now() + m_period < m_stop)
        {
            Simulator::Schedule(m_period, &GeometryControlPlane::Tick, this);
        }
    }

    std::vector<Ptr<MobilityModel>> m_sats;
    std::vector<Ptr<MobilityModel>> m_ues;
    std::vector<Vector> m_satEcef;
    std::vector<uint32_t> m_serving;
    uint64_t m_reassigns{0}; ///< serving-link changes after the first assignment
    Time m_period{Seconds(1.0)};
    Time m_stop{Seconds(0)};
    uint64_t m_evals{0};
    uint64_t m_ticks{0};
};

// ---------------------------------------------------------------------------
// Per-phase measured result
// ---------------------------------------------------------------------------
struct PhaseResult
{
    bool valid{false};
    bool radioAttached{false};
    double setupS{0.0};
    double runS{0.0};
    double wallS{0.0};
    double peakRssMb{-1.0};
    double activeWindowS{0.0};
    uint64_t txPkts{0};
    uint64_t txBytes{0};
    uint64_t rxPkts{0};
    uint64_t rxBytes{0};
    double meanOwdMs{0.0};
    uint64_t geomEvals{0};
    uint64_t geomTicks{0};
    /// R1.2 cross-platform reaction metric (see GeometryControlPlane).
    uint64_t servingReassigns{0};
    // Radio-plane extras (phase B only; NaN when no radio ran).
    double meanSinrDb{std::numeric_limits<double>::quiet_NaN()};
    uint64_t phyRxTb{0};

    double MeanPktSizeB() const
    {
        return txPkts ? static_cast<double>(txBytes) / txPkts : 0.0;
    }
    double PerUePktPerS(uint32_t ues) const
    {
        return (ues && activeWindowS > 0.0)
                   ? static_cast<double>(txPkts) / ues / activeWindowS
                   : 0.0;
    }
    double PerUeOfferedBps(uint32_t ues) const
    {
        return (ues && activeWindowS > 0.0)
                   ? static_cast<double>(txBytes) * 8.0 / ues / activeWindowS
                   : 0.0;
    }
    double AggregateOfferedBps() const
    {
        return (activeWindowS > 0.0)
                   ? static_cast<double>(txBytes) * 8.0 / activeWindowS
                   : 0.0;
    }
    double DeliveryRatio() const
    {
        return txPkts ? static_cast<double>(rxPkts) / txPkts : 0.0;
    }
    double WallPerSimS(double duration) const
    {
        return (duration > 0.0) ? wallS / duration : 0.0;
    }
};

/// Apply the regime's offered-load parameters (and any CLI override) to a
/// client. Safe to call after InstallOranFlow(): NtnOranApplication resolves its
/// profile at StartApplication(), so attributes set before the start time are
/// the ones that run. Both phases call this, which is what makes their offered
/// load byte-for-byte identical.
void
ApplyRegimeAttributes(Ptr<NtnOranApplication> app, const RegimeSpec& spec, const Cfg& cfg)
{
    const uint32_t pkt = cfg.pktSizeOverride ? cfg.pktSizeOverride : spec.pktSize;
    app->SetAttribute("PacketSize", UintegerValue(pkt));

    const Time period = (cfg.periodMsOverride >= 0.0)
                            ? Seconds(cfg.periodMsOverride / 1000.0)
                            : spec.period;
    app->SetAttribute("Period", TimeValue(period));

    const std::string rate =
        cfg.dataRateOverride.empty() ? spec.dataRate : cfg.dataRateOverride;
    if (!rate.empty())
    {
        app->SetAttribute("DataRate", DataRateValue(DataRate(rate)));
    }
}

/// Packet-weighted mean one-way delay (ms) over a set of NtnOranSinks, taken
/// from the in-band TX timestamps. Identical code path in both phases.
void
CollectSinks(const std::vector<Ptr<NtnOranSink>>& sinks,
             uint64_t& rxPkts,
             uint64_t& rxBytes,
             double& meanOwdMs)
{
    rxPkts = 0;
    rxBytes = 0;
    double sumDelayMs = 0.0;
    uint64_t delaySamples = 0;
    for (const auto& s : sinks)
    {
        if (s == nullptr)
        {
            continue;
        }
        rxPkts += s->GetRxPackets();
        rxBytes += s->GetTotalRx();
        for (const auto& [key, fs] : s->GetFlowStats())
        {
            sumDelayMs += fs.sumDelayMs;
            delaySamples += fs.rxPackets;
        }
    }
    meanOwdMs = delaySamples ? sumDelayMs / delaySamples : 0.0;
}

/// Build the shell of real orbiting satellite nodes (one ns-3 Node with a real
/// Kepler+J2 orbit each) and the TR 38.811 terminals under the t=0 sub-point of
/// satellite 0. Shared verbatim by both phases.
struct Scenario
{
    NodeContainer satNodes;
    NodeContainer ueNodes;
    std::vector<Ptr<MobilityModel>> satMobs;
    std::vector<Ptr<MobilityModel>> ueMobs;
    double servingSlantM{0.0}; ///< measured sat0 <-> ue0 slant at t = 0
};

Scenario
BuildScenario(const Cfg& cfg)
{
    Scenario sc;

    const uint32_t planes = cfg.planes ? cfg.planes : AutoPlanes(cfg.sats);
    ns3::ntncon::WalkerConfig wcfg;
    wcfg.num_planes = planes;
    wcfg.total_sats = cfg.sats;
    wcfg.altitude_km = cfg.altKm;
    wcfg.inclination_deg = cfg.inclinationDeg;
    wcfg.phasing_f = cfg.phasingF;
    wcfg.eccentricity = cfg.eccentricity;
    wcfg.arg_perigee_deg = cfg.argPerigeeDeg;
    wcfg.epoch_unix_s = cfg.epochUnix;
    const auto elements = ns3::ntncon::WalkerConstellation::BuildDelta(wcfg);
    NS_ABORT_MSG_IF(elements.empty(),
                    "Walker shell empty: --sats=" << cfg.sats << " is not divisible by "
                                                  << planes << " planes (use --planes)");

    sc.satNodes.Create(static_cast<uint32_t>(elements.size()));
    sc.satMobs.reserve(elements.size());
    for (std::size_t i = 0; i < elements.size(); ++i)
    {
        Ptr<ns3::ntncon::Sgp4MobilityModel> mob =
            CreateObject<ns3::ntncon::Sgp4MobilityModel>();
        mob->SetElements(elements[i]);
        sc.satNodes.Get(i)->AggregateObject(mob);
        sc.satMobs.push_back(mob);
    }

    double subLat = 0.0;
    double subLon = 0.0;
    double subAlt = 0.0;
    DynamicCast<ns3::ntncon::Sgp4MobilityModel>(sc.satMobs[0])
        ->GetGeodetic(subLat, subLon, subAlt);

    sc.ueNodes.Create(cfg.ues);
    NtnTr38811MobilityHelper ueMobility(static_cast<uint64_t>(cfg.rngRun));
    auto profile = NtnMobilityScenarios::MixedContinental();
    auto ueModels = ueMobility.Install(sc.ueNodes,
                                       profile,
                                       subLat - 0.03,
                                       subLat + 0.03,
                                       subLon - 0.03,
                                       subLon + 0.03);
    sc.ueMobs.reserve(ueModels.size());
    for (const auto& m : ueModels)
    {
        sc.ueMobs.push_back(m);
    }

    if (!sc.ueMobs.empty())
    {
        sc.servingSlantM =
            ntngeo::SlantRangeM(sc.ueMobs[0]->GetPosition(), sc.satMobs[0]->GetPosition());
    }
    return sc;
}

// ---------------------------------------------------------------------------
// PHASE A — geometry/control plane + identical offered load, NO air interface.
//
// The access plane is a plain point-to-point star whose one-way delay is taken
// from the MEASURED t=0 slant geometry, so the reference run has a physically
// sane latency without pretending to be a radio measurement. Its only job is to
// price the geometry/control plane and the application plane on their own.
// ---------------------------------------------------------------------------
PhaseResult
RunControlPhase(const Cfg& cfg)
{
    const double wall0 = WallNow();
    PhaseResult r;
    r.radioAttached = false;

    Scenario sc = BuildScenario(cfg);

    // Access plane: remote host -> one P2P link per terminal.
    NodeContainer remote;
    remote.Create(1);
    InternetStackHelper internet;
    internet.Install(remote);
    internet.Install(sc.ueNodes);

    const double owdS = sc.servingSlantM / kC + 5e-3; // slant + 5 ms backhaul
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(owdS)));

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    std::vector<Ipv4Address> ueAddrs(cfg.ues);
    for (uint32_t u = 0; u < cfg.ues; ++u)
    {
        NetDeviceContainer devs = p2p.Install(remote.Get(0), sc.ueNodes.Get(u));
        Ipv4InterfaceContainer ifs = addr.Assign(devs);
        ueAddrs[u] = ifs.GetAddress(1);
        addr.NewNetwork();
    }
    // No routing setup is needed: every terminal is one directly connected hop
    // from the remote host, so the connected routes suffice.

    const Time start = Seconds(1.0);
    const Time stop = Seconds(std::max(1.5, cfg.duration - 0.5));
    r.activeWindowS = (stop - start).GetSeconds();

    std::vector<Ptr<NtnOranApplication>> clients;
    std::vector<Ptr<NtnOranSink>> sinks;
    uint16_t port = 1234;
    for (uint32_t u = 0; u < cfg.ues; ++u)
    {
        const RegimeSpec spec = LookupRegime(cfg.regime, u);

        Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
        sink->SetAttribute("Local",
                           AddressValue(InetSocketAddress(Ipv4Address::GetAny(), port)));
        sc.ueNodes.Get(u)->AddApplication(sink);
        sink->SetStartTime(Seconds(0.0));
        sink->SetStopTime(Seconds(cfg.duration));
        sinks.push_back(sink);

        Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
        client->SetRemote(InetSocketAddress(ueAddrs[u], port));
        client->SetProfile(spec.profile);
        ApplyRegimeAttributes(client, spec, cfg);
        client->SetFlowIdentity(spec.fiveQi, 1, 0x000001, static_cast<uint16_t>(u + 1), u);
        remote.Get(0)->AddApplication(client);
        client->SetStartTime(start);
        client->SetStopTime(stop);
        clients.push_back(client);

        ++port;
    }

    GeometryControlPlane gcp;
    gcp.Init(sc.satMobs,
             sc.ueMobs,
             Seconds(1.0 / std::max(1e-6, cfg.controlHz)),
             Seconds(cfg.duration));
    Simulator::ScheduleNow(&GeometryControlPlane::Start, &gcp);

    const double wallSetupEnd = WallNow();
    r.setupS = wallSetupEnd - wall0;

    Simulator::Stop(Seconds(cfg.duration));
    Simulator::Run();
    r.runS = WallNow() - wallSetupEnd;

    for (const auto& c : clients)
    {
        r.txPkts += c->GetTxPackets();
        r.txBytes += c->GetTxBytes();
    }
    CollectSinks(sinks, r.rxPkts, r.rxBytes, r.meanOwdMs);
    r.geomEvals = gcp.GetEvalCount();
    r.geomTicks = gcp.GetTickCount();
    r.servingReassigns = gcp.GetServingReassignments();

    Simulator::Destroy();

    r.wallS = WallNow() - wall0;
    r.peakRssMb = PeakRssMb();
    r.valid = true;
    return r;
}

// ---------------------------------------------------------------------------
// PHASE B — the identical scenario with the measured radio attached.
// ---------------------------------------------------------------------------
PhaseResult
RunRadioPhase(const Cfg& cfg)
{
    const double wall0 = WallNow();
    PhaseResult r;
    r.radioAttached = true;

    Scenario sc = BuildScenario(cfg);

    // The serving cell rides satellite 0 of the very same shell — the gNB is a
    // real orbiting node, not a stand-in.
    NodeContainer gnbNodes;
    gnbNodes.Add(sc.satNodes.Get(0));

    const bool useNr = (cfg.radio == "nr");
    double eirp = cfg.satEirpDbm;
    if (eirp < 0.0)
    {
        eirp = useNr ? 70.0 : 55.0; // backend-appropriate Friis budget
    }

    NtnRealStackHelper rs;
    rs.SetRadioBackend(useNr ? NtnRealStackHelper::RadioBackend::Nr
                             : NtnRealStackHelper::RadioBackend::Mmwave);
    if (useNr)
    {
        rs.SetNumerology(1); // FR1 30 kHz SCS
    }
    rs.SetSimTime(Seconds(cfg.duration));
    rs.SetOutputDir(cfg.outputDir);
    rs.SetRunTag("scal-" + cfg.regime + "-s" + std::to_string(cfg.sats) + "-u" +
                 std::to_string(cfg.ues));
    rs.SetCarrierFrequencyHz(cfg.freqGhz * 1e9);
    // NT-02: declared as CONDUCTED power at the array input. This carrier has
    // no TR 38.821 Set-1 reference in the toolkit, so the EIRP health gate
    // reports "not asserted" rather than certifying an uncalibrated budget.
    rs.SetSatConductedPowerDbm(eirp);
    rs.Build(gnbNodes, sc.ueNodes);

    const Time start = Seconds(1.0);
    const Time stop = Seconds(std::max(1.5, cfg.duration - 0.5));
    r.activeWindowS = (stop - start).GetSeconds();

    std::vector<Ptr<NtnOranApplication>> clients;
    std::vector<Ptr<NtnOranSink>> sinks;
    for (uint32_t u = 0; u < cfg.ues; ++u)
    {
        const RegimeSpec spec = LookupRegime(cfg.regime, u);
        ApplicationContainer flow =
            rs.InstallOranFlow(u, spec.fiveQi, 1, 0x000001, spec.profile, start, stop);
        Ptr<NtnOranApplication> client = DynamicCast<NtnOranApplication>(flow.Get(0));
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(flow.Get(1));
        NS_ABORT_MSG_IF(client == nullptr || sink == nullptr,
                        "InstallOranFlow must return {client, sink}");
        ApplyRegimeAttributes(client, spec, cfg);
        clients.push_back(client);
        sinks.push_back(sink);
    }

    // The SAME geometry/control workload as phase A, so the phase difference is
    // the radio and nothing else.
    GeometryControlPlane gcp;
    gcp.Init(sc.satMobs,
             sc.ueMobs,
             Seconds(1.0 / std::max(1e-6, cfg.controlHz)),
             Seconds(cfg.duration));
    Simulator::ScheduleNow(&GeometryControlPlane::Start, &gcp);

    const double wallSetupEnd = WallNow();
    r.setupS = wallSetupEnd - wall0;

    Simulator::Stop(Seconds(cfg.duration));
    Simulator::Run();
    r.runS = WallNow() - wallSetupEnd;

    rs.Collect();
    rs.WriteHealthReport();
    r.meanSinrDb = rs.GetMeanDlSinrDb();
    r.phyRxTb = rs.GetPhyRxTb();

    for (const auto& c : clients)
    {
        r.txPkts += c->GetTxPackets();
        r.txBytes += c->GetTxBytes();
    }
    CollectSinks(sinks, r.rxPkts, r.rxBytes, r.meanOwdMs);
    r.geomEvals = gcp.GetEvalCount();
    r.geomTicks = gcp.GetTickCount();
    r.servingReassigns = gcp.GetServingReassignments();

    Simulator::Destroy();

    r.wallS = WallNow() - wall0;
    r.peakRssMb = PeakRssMb();
    r.valid = true;
    return r;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

const char* kCsvHeader =
    "regime,sats,ues,duration_s,radio_attached,pkt_size_B,per_ue_pkt_per_s,"
    "per_ue_offered_bps,aggregate_offered_bps,total_tx_pkts,total_rx_pkts,"
    "delivery_ratio,mean_owd_ms,wall_clock_s,wall_per_sim_s,peak_rss_mb,"
    "serving_reassignments,geom_evals,bound_class";

void
WriteCsvRow(std::ostream& os,
            const Cfg& cfg,
            const PhaseResult& r,
            const std::string& boundClass)
{
    os << cfg.regime << ',' << cfg.sats << ',' << cfg.ues << ',' << cfg.duration << ','
       << (r.radioAttached ? 1 : 0) << ',' << std::fixed << std::setprecision(2)
       << r.MeanPktSizeB() << ',' << std::setprecision(4) << r.PerUePktPerS(cfg.ues) << ','
       << std::setprecision(1) << r.PerUeOfferedBps(cfg.ues) << ','
       << r.AggregateOfferedBps() << ',' << r.txPkts << ',' << r.rxPkts << ','
       << std::setprecision(6) << r.DeliveryRatio() << ',' << std::setprecision(3)
       << r.meanOwdMs << ',' << r.wallS << ',' << std::setprecision(4)
       << r.WallPerSimS(cfg.duration) << ',' << std::setprecision(2) << r.peakRssMb << ','
       << r.servingReassigns << ',' << r.geomEvals << ',' << boundClass << '\n';
    os << std::defaultfloat;
}

void
PrintRow(const Cfg& cfg, const PhaseResult& r, const std::string& boundClass)
{
    std::cout << std::left << std::setw(11) << cfg.regime << std::right << std::setw(7)
              << cfg.sats << std::setw(6) << cfg.ues << std::setw(8) << cfg.duration
              << std::setw(7) << (r.radioAttached ? 1 : 0) << std::setw(9)
              << std::fixed << std::setprecision(0) << r.MeanPktSizeB() << std::setw(11)
              << std::setprecision(2) << r.PerUePktPerS(cfg.ues) << std::setw(14)
              << std::setprecision(0) << r.PerUeOfferedBps(cfg.ues) << std::setw(16)
              << r.AggregateOfferedBps() << std::setw(12) << r.txPkts << std::setw(12)
              << r.rxPkts << std::setw(9) << std::setprecision(4) << r.DeliveryRatio()
              << std::setw(10) << std::setprecision(2) << r.meanOwdMs << std::setw(10)
              << r.wallS << std::setw(11) << std::setprecision(4)
              << r.WallPerSimS(cfg.duration) << std::setw(10) << std::setprecision(1)
              << r.peakRssMb << "  " << boundClass << std::defaultfloat << '\n';
}

} // namespace

int
main(int argc, char* argv[])
{
    Cfg cfg;

    CommandLine cmd(__FILE__);
    cmd.AddValue("regime",
                 "Offered-load regime: telemetry | cbr | embb | urllc | voice | mixed",
                 cfg.regime);
    cmd.AddValue("sats", "Walker-Delta shell size (real orbits propagated)", cfg.sats);
    cmd.AddValue("ues", "Number of TR 38.811 ground terminals", cfg.ues);
    cmd.AddValue("duration", "Simulated time [s]", cfg.duration);
    cmd.AddValue("radio",
                 "Radio backend for phase B: mmwave | nr | none (skip phase B)",
                 cfg.radio);
    cmd.AddValue("phase",
                 "both (paired, default) | control (phase A only) | radio (phase B only)",
                 cfg.phase);
    cmd.AddValue("planes", "Walker planes (0 = auto: largest divisor <= 72)", cfg.planes);
    cmd.AddValue("altKm", "Shell altitude [km]", cfg.altKm);
    // R1.2 — full Walker-Delta (T/P/F) control so an identical shell can be
    // pinned across simulators for the cross-platform comparison.
    cmd.AddValue("inclinationDeg", "Walker inclination [deg]", cfg.inclinationDeg);
    cmd.AddValue("phasingF", "Walker phasing factor F (T/P/F)", cfg.phasingF);
    cmd.AddValue("eccentricity", "Orbit eccentricity (0 = circular)", cfg.eccentricity);
    cmd.AddValue("argPerigeeDeg", "Argument of perigee [deg]", cfg.argPerigeeDeg);
    cmd.AddValue("epochUnix",
                 "Constellation reference epoch [Unix s] "
                 "(match the comparator; Hypatia pins 946684800 = 2000-01-01)",
                 cfg.epochUnix);
    cmd.AddValue("controlHz", "Geometry/control re-decision rate [Hz]", cfg.controlHz);
    cmd.AddValue("freqGhz", "Carrier frequency [GHz]", cfg.freqGhz);
    cmd.AddValue("satEirpDbm", "gNB Tx power [dBm]; -1 = backend default", cfg.satEirpDbm);
    cmd.AddValue("rngRun", "RNG run", cfg.rngRun);
    cmd.AddValue("pktSize", "Override packet size [B] (0 = regime preset)", cfg.pktSizeOverride);
    cmd.AddValue("periodMs",
                 "Override send period [ms] for periodic regimes (<0 = regime preset)",
                 cfg.periodMsOverride);
    cmd.AddValue("dataRate",
                 "Override DataRate for rate-driven regimes (empty = regime preset)",
                 cfg.dataRateOverride);
    cmd.AddValue("outputDir", "Output directory", cfg.outputDir);
    cmd.AddValue("csv", "CSV file name inside outputDir", cfg.csvName);
    cmd.AddValue("append", "Append to the CSV instead of overwriting", cfg.append);
    cmd.AddValue("boundThreshold",
                 "radio_share = (wallB - wallA)/wallB at or above which the run is "
                 "classified radio-bound",
                 cfg.boundThreshold);
    cmd.AddValue("controlWallRef",
                 "Externally measured phase-A wall clock [s] to classify a --phase=radio "
                 "run (<0 = unavailable)",
                 cfg.controlWallRef);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(cfg.ues == 0, "--ues must be >= 1");
    NS_ABORT_MSG_IF(cfg.sats == 0, "--sats must be >= 1");
    NS_ABORT_MSG_IF(cfg.duration <= 1.5, "--duration must exceed the 1.5 s ramp");
    NS_ABORT_MSG_IF(cfg.radio != "mmwave" && cfg.radio != "nr" && cfg.radio != "none",
                    "--radio must be mmwave | nr | none");
    NS_ABORT_MSG_IF(cfg.phase != "both" && cfg.phase != "control" && cfg.phase != "radio",
                    "--phase must be both | control | radio");

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(cfg.rngRun);

    std::error_code ec;
    std::filesystem::create_directories(cfg.outputDir, ec);

    const bool wantControl = (cfg.phase == "both" || cfg.phase == "control");
    const bool wantRadio =
        (cfg.radio != "none") && (cfg.phase == "both" || cfg.phase == "radio");

    std::cout << "\n=====================================================================\n"
              << "  ntn-scalability-load-regimes (R2.6 operating envelope)\n"
              << "=====================================================================\n"
              << "  regime      : " << cfg.regime << "\n"
              << "  shell       : " << cfg.sats << " satellites @ " << cfg.altKm
              << " km, " << (cfg.planes ? cfg.planes : AutoPlanes(cfg.sats)) << " planes\n"
              << "  terminals   : " << cfg.ues << "\n"
              << "  duration    : " << cfg.duration << " s, control tick "
              << cfg.controlHz << " Hz\n"
              << "  radio       : " << cfg.radio << "   phases: " << cfg.phase << "\n"
              << "=====================================================================\n\n";

    PhaseResult a;
    PhaseResult b;

    // Phase A must run FIRST: it touches no Config::SetDefault, so it cannot be
    // perturbed by the radio helper's global defaults.
    if (wantControl)
    {
        std::cout << "[phase A] geometry/control plane + offered load, NO radio ...\n";
        a = RunControlPhase(cfg);
        std::cout << "[phase A] wall=" << a.wallS << " s (setup " << a.setupS << " s, run "
                  << a.runS << " s), geometry evaluations=" << a.geomEvals << " over "
                  << a.geomTicks << " ticks, peak RSS=" << a.peakRssMb << " MB\n\n";
    }
    if (wantRadio)
    {
        std::cout << "[phase B] identical scenario WITH the measured " << cfg.radio
                  << " cell ...\n";
        b = RunRadioPhase(cfg);
        std::cout << "[phase B] wall=" << b.wallS << " s (setup " << b.setupS << " s, run "
                  << b.runS << " s), geometry evaluations=" << b.geomEvals
                  << ", PHY TBs=" << b.phyRxTb << ", mean DL SINR=" << b.meanSinrDb
                  << " dB, peak RSS=" << b.peakRssMb << " MB\n\n";
    }

    // ---- Measured bound classification ------------------------------------
    // The class is decided ONLY by the measured wall-clock split between the two
    // phases. When only one phase ran and no external reference was supplied,
    // the class is reported as "undetermined" rather than guessed.
    std::string boundClass = "undetermined";
    double radioShare = std::numeric_limits<double>::quiet_NaN();
    const double wallA = a.valid ? a.wallS : cfg.controlWallRef;
    if (b.valid && wallA >= 0.0 && b.wallS > 0.0)
    {
        radioShare = (b.wallS - wallA) / b.wallS;
        boundClass =
            (radioShare >= cfg.boundThreshold) ? "radio-bound" : "geometry-control-bound";
    }

    // ---- CSV ---------------------------------------------------------------
    const std::string csvPath = cfg.outputDir + "/" + cfg.csvName;
    const bool exists = std::filesystem::exists(csvPath);
    std::ofstream csv;
    if (cfg.append && exists)
    {
        csv.open(csvPath, std::ios::app);
    }
    else
    {
        csv.open(csvPath, std::ios::trunc);
        csv << kCsvHeader << '\n';
    }
    if (a.valid)
    {
        WriteCsvRow(csv, cfg, a, boundClass);
    }
    if (b.valid)
    {
        WriteCsvRow(csv, cfg, b, boundClass);
    }
    csv.close();

    // ---- stdout summary ----------------------------------------------------
    std::cout << "---------------------------------------------------------------------"
                 "------------------------------------------------------------------\n"
              << std::left << std::setw(11) << "regime" << std::right << std::setw(7)
              << "sats" << std::setw(6) << "ues" << std::setw(8) << "sim_s"
              << std::setw(7) << "radio" << std::setw(9) << "pkt_B" << std::setw(11)
              << "pkt/s/ue" << std::setw(14) << "bps/ue" << std::setw(16) << "bps_agg"
              << std::setw(12) << "tx_pkts" << std::setw(12) << "rx_pkts" << std::setw(9)
              << "deliv" << std::setw(10) << "owd_ms" << std::setw(10) << "wall_s"
              << std::setw(11) << "wall/sim" << std::setw(10) << "rss_MB"
              << "  bound_class\n"
              << "---------------------------------------------------------------------"
                 "------------------------------------------------------------------\n";
    if (a.valid)
    {
        PrintRow(cfg, a, boundClass);
    }
    if (b.valid)
    {
        PrintRow(cfg, b, boundClass);
    }
    std::cout << "---------------------------------------------------------------------"
                 "------------------------------------------------------------------\n";

    if (!std::isnan(radioShare))
    {
        std::cout << "\n[regime split] wall(control-only)=" << a.wallS
                  << " s   wall(with radio)=" << b.wallS
                  << " s   radio_share=" << radioShare << " (threshold "
                  << cfg.boundThreshold << ")  ->  " << boundClass << "\n";
    }
    else
    {
        std::cout << "\n[regime split] only one phase ran and no --controlWallRef was "
                     "supplied; bound_class reported as 'undetermined' rather than "
                     "assumed.\n";
    }
    std::cout << "[csv] " << csvPath << "\n\n";

    return 0;
}
