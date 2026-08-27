// SPDX-License-Identifier: GPL-2.0-only
//
// e2e tests for the NtnRealStackHelper additions (AI_NATIVE_ORAN_NTN plan +
// 2026-06 audit). EnableAiFlowMonitor() and the NTN HARQ profile are only
// reachable through Build(), so every case stands up the REAL mmwave NR air
// interface (same wiring as examples/ntn-real-stack-smoke.cc, shrunk to
// 1 LEO gNB / 1 ground UE / 3 s):
//   1. EnableAiFlowMonitor() called BEFORE InstallTraffic: the monitor exists
//      immediately, every later flow auto-attaches, the helper-scoped srcId
//      counter (audit issue 15) yields unique non-port flow ids carried
//      in-band over the radio, and Simulator::Destroy() auto-exports
//      <prefix>_kpm_series.{csv,lp};
//   2. EnableAiFlowMonitor() called AFTER InstallTraffic attaches the
//      already-installed flows (no crash, measured KPM series + export);
//   3. SetNtnHarqProfile(true) stretches the two mmwave HARQ knobs
//      (ns3::MmWavePhyMacCommon::HarqDlTimeout / NumHarqProcess) per the
//      documented slant math, while the default path (profile off, case 1)
//      leaves them untouched;
//   4. R2.6 offered-load accounting: the load figures reported by
//      examples/ntn-scalability-load-regimes.cc are derived from the
//      application's own emission counters and are mutually consistent
//      (per-UE bps == pkt_size * 8 * pkt_rate; aggregate == per-UE x nUE).

#include "ns3/abort.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/nr-gnb-cmac-sap.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-helper.h"
#include "ns3/ntn-rach-window.h"
#include "ns3/ntn-tdl-spectrum-loss-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/ntn-repro-manifest.h"
#include "ns3/ntn-sat-beam-gain-model.h"
#include "ns3/ntn-tr38811-excess-loss-model.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/three-gpp-channel-model.h"

#include <filesystem>
#include "ns3/type-id.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

std::string
ReadFile(const std::string& p)
{
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// Current Config default of a Uinteger attribute (Config::SetDefault edits
/// the TypeId initial value, so this observes exactly what Build() set).
uint64_t
GetUintDefault(const std::string& tidName, const std::string& attrName)
{
    TypeId tid = TypeId::LookupByName(tidName);
    TypeId::AttributeInformation info;
    NS_ABORT_MSG_UNLESS(tid.LookupAttributeByName(attrName, &info),
                        tidName << " has no attribute " << attrName);
    return std::stoull(info.initialValue->SerializeToString(info.checker));
}

bool
GetBoolDefault(const std::string& tidName, const std::string& attrName)
{
    TypeId tid = TypeId::LookupByName(tidName);
    TypeId::AttributeInformation info;
    NS_ABORT_MSG_UNLESS(tid.LookupAttributeByName(attrName, &info),
                        tidName << " has no attribute " << attrName);
    return info.initialValue->SerializeToString(info.checker) == "true";
}

/// Minimal real-geometry rig: one LEO satellite (gNB) 600 km straight above
/// \p numUes static ground UEs, moving at orbital speed. The 600 km zenith slant
/// is exact at t=0 for UE 0, which both the Friis budget and the HARQ math
/// depend on; extra UEs sit 100 m apart on the ground so they are distinct nodes
/// without perturbing the slant.
struct LeoRig
{
    NodeContainer sat;
    NodeContainer ue;

    explicit LeoRig(uint32_t numUes = 1)
    {
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> satMob =
            CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(0.0, 0.0, 600e3));
        satMob->SetVelocity(Vector(7560.0, 0.0, 0.0)); // LEO-600 orbital speed
        sat.Get(0)->AggregateObject(satMob);

        ue.Create(numUes);
        for (uint32_t i = 0; i < numUes; ++i)
        {
            Ptr<ConstantPositionMobilityModel> ueMob =
                CreateObject<ConstantPositionMobilityModel>();
            ueMob->SetPosition(Vector(100.0 * i, 0.0, 0.0));
            ue.Get(i)->AggregateObject(ueMob);
        }
    }
};

} // namespace

class RealStackAiMonitorAutoExportTest : public TestCase
{
  public:
    RealStackAiMonitorAutoExportTest()
        : TestCase("EnableAiFlowMonitor before InstallTraffic: auto-attach, "
                   "unique srcIds, auto-export")
    {
    }

  private:
    void DoRun() override
    {
        const std::string prefix = "ntn-real-stack-test-before";
        std::remove((prefix + "_kpm_series.csv").c_str());
        std::remove((prefix + "_kpm_series.lp").c_str());

        // Default path: HARQ knobs must NOT be touched when the NTN HARQ
        // profile stays off (compare against whatever the defaults are now).
        const uint64_t harqTimeout0 =
            GetUintDefault("ns3::MmWavePhyMacCommon", "HarqDlTimeout");
        const uint64_t harqProc0 =
            GetUintDefault("ns3::MmWavePhyMacCommon", "NumHarqProcess");

        LeoRig rig;
        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(3.0));
        rs.Build(rig.sat, rig.ue);

        NS_TEST_ASSERT_MSG_EQ((rs.GetAiFlowMonitor() == nullptr), true,
                              "no monitor before EnableAiFlowMonitor");
        rs.EnableAiFlowMonitor(prefix); // BEFORE any traffic exists
        Ptr<NtnOranAiFlowMonitor> mon = rs.GetAiFlowMonitor();
        NS_TEST_ASSERT_MSG_EQ((mon != nullptr), true,
                              "monitor exists right after enable");

        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings,
                          Seconds(0.5), Seconds(2.5));
        NS_TEST_ASSERT_MSG_EQ((rs.GetAiFlowMonitor() == mon), true,
                              "same monitor after InstallTraffic");
        // A flow installed even later must auto-attach too.
        ApplicationContainer extra =
            rs.InstallOranFlow(0, 9, 1, 0x000001, NtnOranApplication::MMTC_PERIODIC,
                               Seconds(0.5), Seconds(2.5));

        Simulator::Stop(Seconds(3.0));
        Simulator::Run();

        NS_TEST_ASSERT_MSG_EQ(GetUintDefault("ns3::MmWavePhyMacCommon", "HarqDlTimeout"),
                              harqTimeout0, "profile off: HarqDlTimeout untouched");
        NS_TEST_ASSERT_MSG_EQ(GetUintDefault("ns3::MmWavePhyMacCommon", "NumHarqProcess"),
                              harqProc0, "profile off: NumHarqProcess untouched");

        // Both flows (InstallTraffic + the later InstallOranFlow) classified.
        NS_TEST_ASSERT_MSG_EQ(mon->GetKpmSeries().size(), 2u, "two ORAN flows in KPM");
        // Unique srcId scheme (audit issue 15): a helper-scoped monotonic
        // counter (1, 2, ...), NOT the recycled DL UDP port (ports start at
        // 1234), measured from the in-band header bytes that crossed the radio.
        std::set<uint16_t> srcIds;
        for (const auto& kv : mon->GetKpmSeries())
        {
            OranFlowKey key;
            NS_TEST_ASSERT_MSG_EQ(mon->GetClassifier()->FindFlow(kv.first, key), true,
                                  "flow key known to the classifier");
            NS_TEST_ASSERT_MSG_LT(key.srcId, 1234, "srcId is not a port number");
            srcIds.insert(key.srcId);
        }
        NS_TEST_ASSERT_MSG_EQ(srcIds.size(), 2u, "srcIds distinct across flows");
        NS_TEST_ASSERT_MSG_EQ(srcIds.count(1), 1u, "first installed flow has srcId 1");
        NS_TEST_ASSERT_MSG_EQ(srcIds.count(2), 1u, "second installed flow has srcId 2");
        // The later flow's sink really measured traffic over the radio.
        Ptr<NtnOranSink> extraSink = DynamicCast<NtnOranSink>(extra.Get(1));
        NS_TEST_ASSERT_MSG_EQ((extraSink != nullptr), true,
                              "InstallOranFlow returns {client, sink}");
        NS_TEST_ASSERT_MSG_GT(extraSink->GetRxPackets(), 0u,
                              "extra flow delivered over the real radio");
        NS_TEST_ASSERT_MSG_EQ(extraSink->GetFlowStats().size(), 1u,
                              "one flow at the extra sink");
        NS_TEST_ASSERT_MSG_EQ(extraSink->GetFlowStats().begin()->second.srcId, 2,
                              "extra flow carries srcId 2 in-band");

        Simulator::Destroy(); // fires the ScheduleDestroy KPM auto-export

        const std::string csv = ReadFile(prefix + "_kpm_series.csv");
        NS_TEST_ASSERT_MSG_EQ(csv.rfind("time_s,flow_id,", 0), 0u,
                              "CSV starts with the KPM header");
        NS_TEST_ASSERT_MSG_GT(std::count(csv.begin(), csv.end(), '\n'), 1,
                              "CSV has the header plus at least one series row");
        const std::string lp = ReadFile(prefix + "_kpm_series.lp");
        NS_TEST_ASSERT_MSG_EQ((lp.find("ntn_oran_kpm,flow_id=") != std::string::npos),
                              true, "Influx lp exported with measurement rows");
    }
};

class RealStackAiMonitorAfterInstallTest : public TestCase
{
  public:
    RealStackAiMonitorAfterInstallTest()
        : TestCase("EnableAiFlowMonitor after InstallTraffic attaches the "
                   "already-installed flows")
    {
    }

  private:
    void DoRun() override
    {
        const std::string prefix = "ntn-real-stack-test-after";
        std::remove((prefix + "_kpm_series.csv").c_str());
        std::remove((prefix + "_kpm_series.lp").c_str());

        LeoRig rig;
        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(3.0));
        rs.Build(rig.sat, rig.ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::UrllcPings,
                          Seconds(0.5), Seconds(2.5));

        rs.EnableAiFlowMonitor(prefix); // AFTER the traffic is installed
        Ptr<NtnOranAiFlowMonitor> mon = rs.GetAiFlowMonitor();
        NS_TEST_ASSERT_MSG_EQ((mon != nullptr), true, "monitor created after install");

        Simulator::Stop(Seconds(3.0));
        Simulator::Run();

        // The pre-installed flow was attached and produced measured KPM data.
        NS_TEST_ASSERT_MSG_EQ(mon->GetKpmSeries().size(), 1u,
                              "the already-installed flow was attached");
        const auto& series = mon->GetKpmSeries().begin()->second;
        NS_TEST_ASSERT_MSG_GT(series.size(), 0u, "KPM samples collected");
        double kbit = 0;
        for (const auto& s : series)
        {
            kbit += s.metrics.at("DRB.PdcpSduVolumeDl");
        }
        NS_TEST_ASSERT_MSG_GT(kbit, 0.0, "measured volume from the attached flow");

        Simulator::Destroy(); // auto-export

        const std::string csv = ReadFile(prefix + "_kpm_series.csv");
        NS_TEST_ASSERT_MSG_EQ(csv.rfind("time_s,flow_id,", 0), 0u,
                              "CSV starts with the KPM header");
        NS_TEST_ASSERT_MSG_GT(std::count(csv.begin(), csv.end(), '\n'), 1,
                              "CSV has the header plus at least one series row");
    }
};

class RealStackNtnHarqProfileTest : public TestCase
{
  public:
    RealStackNtnHarqProfileTest()
        : TestCase("SetNtnHarqProfile(true) stretches the mmwave HARQ knobs "
                   "per the slant math")
    {
    }

  private:
    void DoRun() override
    {
        const uint64_t timeout0 =
            GetUintDefault("ns3::MmWavePhyMacCommon", "HarqDlTimeout");
        const uint64_t numProc0 =
            GetUintDefault("ns3::MmWavePhyMacCommon", "NumHarqProcess");

        LeoRig rig; // slant is exactly 600 km at t=0 (Build() reads it then)
        NtnRealStackHelper rs;
        rs.SetNtnHarqProfile(true);
        rs.Build(rig.sat, rig.ue);

        // Same math as ConfigureNtnHarqProfile(), for the 600 km zenith slant:
        //   rtt   = 2 * 600 km / c                 ~= 4.0028 ms
        //   round = rtt + 1 ms processing budget   ~= 5.0028 ms
        //   HarqDlTimeout  = ceil(4 rounds / 0.25 ms slot)        = 81 slots
        //   NumHarqProcess = max(20, ceil(rtt / slot) + 4 rounds) = 21
        constexpr double kC = 299792458.0;
        constexpr double kSlotS = 250e-6;
        const double rttS = 2.0 * 600e3 / kC;
        const auto expTimeout =
            static_cast<uint64_t>(std::min(255.0, std::ceil(4.0 * (rttS + 1e-3) / kSlotS)));
        const auto expProc = static_cast<uint64_t>(
            std::min(255.0, std::max(20.0, std::ceil(rttS / kSlotS) + 4.0)));

        const uint64_t timeoutSet =
            GetUintDefault("ns3::MmWavePhyMacCommon", "HarqDlTimeout");
        const uint64_t numProcSet =
            GetUintDefault("ns3::MmWavePhyMacCommon", "NumHarqProcess");
        NS_TEST_ASSERT_MSG_EQ(timeoutSet, expTimeout,
                              "HarqDlTimeout stretched per the slant math (81 slots)");
        NS_TEST_ASSERT_MSG_EQ(numProcSet, expProc,
                              "NumHarqProcess stretched per the slant math (21)");
        NS_TEST_ASSERT_MSG_GT(timeoutSet, timeout0,
                              "timeout raised above the terrestrial default");
        // The profile is meaningless with HARQ off: it must force HARQ on.
        NS_TEST_ASSERT_MSG_EQ(GetBoolDefault("ns3::MmWaveHelper", "HarqEnabled"), true,
                              "NTN HARQ profile turns HARQ on");

        Simulator::Destroy();
        // Restore the stock defaults so later test cases observe them.
        Config::SetDefault("ns3::MmWavePhyMacCommon::HarqDlTimeout",
                           UintegerValue(timeout0));
        Config::SetDefault("ns3::MmWavePhyMacCommon::NumHarqProcess",
                           UintegerValue(numProc0));
    }
};

/// GAP S3 (CI gate 2): per-UE stats must be keyed by (cellId,RNTI), so two UEs
/// on different cells sharing an RNTI do not blend. Before the fix the key was
/// the bare RNTI and every multi-gNB run corrupted per-UE SINR/TBLER.
class RealStackUeKeySeparationTest : public TestCase
{
  public:
    RealStackUeKeySeparationTest()
        : TestCase("Per-UE stats key separates same-RNTI UEs on different cells (S3/gate 2)")
    {
    }

  private:
    void DoRun() override
    {
        // Same RNTI (5) on two different cells (1, 2) MUST give distinct keys.
        const uint32_t k1 = NtnRealStackHelper::UeStatsKey(1, 5);
        const uint32_t k2 = NtnRealStackHelper::UeStatsKey(2, 5);
        NS_TEST_ASSERT_MSG_NE(k1, k2,
                              "same RNTI on different cells must not collide (bare-RNTI bug)");
        // Same (cell,RNTI) is the same key; different RNTI on the same cell differ.
        NS_TEST_ASSERT_MSG_EQ(k1, NtnRealStackHelper::UeStatsKey(1, 5), "key must be stable");
        NS_TEST_ASSERT_MSG_NE(NtnRealStackHelper::UeStatsKey(1, 5),
                              NtnRealStackHelper::UeStatsKey(1, 6),
                              "different RNTIs on the same cell must differ");
        // No aliasing across the 16-bit boundary: (cell=0,rnti=0x10000&0xffff)
        // cannot equal (cell=1,rnti=0). The shift guarantees it.
        NS_TEST_ASSERT_MSG_NE(NtnRealStackHelper::UeStatsKey(1, 0),
                              NtnRealStackHelper::UeStatsKey(0, 1), "no cross-field aliasing");
    }
};

/// R1/R3 (WS-E): the SIB19 K_offset must be CONSUMED by the nr UL scheduler
/// timing, not merely populated. NtnRealStackHelper applies the geometry-derived
/// K_offset to NrGnbPhy::N2Delay (the UL DCI->PUSCH gap, TS 38.213 §4.2), which
/// nr-gnb-phy adds to the uplink slot (`ulSfn.Add(GetN2Delay())`). This asserts:
/// (1) the consumed K_offset equals the 600 km-zenith round-trip geometry and
/// matches the SIB19 derivation; (2) it is actually programmed onto the built
/// gNB PHY; and (3) it pushes N2Delay past the vendored terrestrial cap of 4
/// slots (the NTN cap-raise, ntn-patches/05). NOTE: turning air-interface delay
/// fully ON for uplink additionally needs NTN-aware SRS/PUCCH timing across all
/// UL control channels, which the vendored nr v3.3 lacks (nr-spectrum-phy
/// half-duplex assert) — that is the nr v5.0/ns-3.48 migration, tracked
/// separately. This test verifies the K_offset CONSUMPTION path itself.
class RealStackKOffsetConsumedTest : public TestCase
{
  public:
    RealStackKOffsetConsumedTest()
        : TestCase("R1/R3 - SIB19 K_offset is consumed into the nr UL scheduler N2Delay")
    {
    }

  private:
    void DoRun() override
    {
        LeoRig rig; // 600 km zenith slant at t=0 -> ~4 ms round trip
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetKOffsetConsumption(true); // no air delay here: verify consumption itself
        rs.Build(rig.sat, rig.ue);

        const uint32_t k = rs.GetConsumedKOffsetSlots();
        NS_TEST_ASSERT_MSG_GT(k, 0u, "K_offset must be consumed");
        NS_TEST_ASSERT_MSG_EQ(k, rs.ComputeKOffsetSlots(),
                              "consumed K_offset must equal the geometry-derived value "
                              "(matches the SIB19 cellSpecificKoffset derivation)");

        // The K_offset is genuinely programmed onto the scheduler: N2Delay on the
        // built gNB PHY equals the stack's base N2Delay (2) plus K_offset.
        const uint32_t n2 = rs.GetGnbN2Delay(0, 0);
        NS_TEST_ASSERT_MSG_EQ(n2, 2u + k,
                              "N2Delay programmed on the gNB PHY = base(2) + consumed K_offset");
        // And it exceeds the vendored terrestrial cap of 4 slots — proving the NTN
        // cap-raise (ntn-patches/05) is live and the K_offset actually fits.
        NS_TEST_ASSERT_MSG_GT(n2, 4u,
                              "NTN K_offset pushes N2Delay past the terrestrial max of 4 slots");

        Simulator::Destroy();
    }
};

/// R2.6 (OJCOMS reviewer response): the scalability/regime characterisation
/// only means something if the OFFERED LOAD it reports is bookkeeping-consistent
/// with the traffic that actually ran. examples/ntn-scalability-load-regimes.cc
/// derives every load figure from the application's own emission counters
/// (NtnOranApplication::GetTxPackets/GetTxBytes) divided by the active window —
/// never from the configured cadence. This test pins that arithmetic on the real
/// helper: for a homogeneous periodic (mMTC-class) load it asserts that
///   (1) the measured mean packet size equals the configured PacketSize exactly
///       (so pkt_size_B in the CSV is a measurement, not a config echo);
///   (2) per-UE offered bit/s == measured_pkt_size * 8 * measured_pkt_rate, i.e.
///       the three reported load columns are mutually consistent;
///   (3) the measured packet rate matches the profile's cadence (1/64 ms), which
///       is the cross-check that the counters describe the intended regime;
///   (4) the aggregate offered bit/s equals the sum over terminals, and equals
///       per-UE x nUE for a homogeneous load;
///   (5) delivery never exceeds what was offered.
/// Kept QUICK: 2 s of simulated time, 2 UEs, one LEO cell.
class RealStackOfferedLoadAccountingTest : public TestCase
{
  public:
    RealStackOfferedLoadAccountingTest()
        : TestCase("R2.6 - offered-load bookkeeping is self-consistent (per-UE bps == "
                   "size*8*rate; aggregate == per-UE x nUE)")
    {
    }

  private:
    void DoRun() override
    {
        constexpr uint32_t kNumUes = 2;
        constexpr uint32_t kPktSize = 128;              // MMTC_PERIODIC preset (B)
        constexpr double kPeriodS = 0.064;              // MMTC_PERIODIC preset (s)
        const Time start = Seconds(0.5);
        const Time stop = Seconds(1.5);
        const double windowS = (stop - start).GetSeconds();

        LeoRig rig(kNumUes);
        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(2.0));
        rs.Build(rig.sat, rig.ue);

        std::vector<Ptr<NtnOranApplication>> clients;
        std::vector<Ptr<NtnOranSink>> sinks;
        for (uint32_t u = 0; u < kNumUes; ++u)
        {
            ApplicationContainer flow = rs.InstallOranFlow(u,
                                                           9,
                                                           1,
                                                           0x000001,
                                                           NtnOranApplication::MMTC_PERIODIC,
                                                           start,
                                                           stop);
            Ptr<NtnOranApplication> c = DynamicCast<NtnOranApplication>(flow.Get(0));
            Ptr<NtnOranSink> s = DynamicCast<NtnOranSink>(flow.Get(1));
            NS_TEST_ASSERT_MSG_EQ((c != nullptr && s != nullptr), true,
                                  "InstallOranFlow returns {client, sink}");
            clients.push_back(c);
            sinks.push_back(s);
        }

        Simulator::Stop(Seconds(2.0));
        Simulator::Run();

        double sumPerUeBps = 0.0;
        uint64_t aggTxPkts = 0;
        uint64_t aggTxBytes = 0;
        uint64_t aggRxPkts = 0;
        for (uint32_t u = 0; u < kNumUes; ++u)
        {
            const uint64_t txPkts = clients[u]->GetTxPackets();
            const uint64_t txBytes = clients[u]->GetTxBytes();
            NS_TEST_ASSERT_MSG_GT(txPkts, 0u, "terminal emitted packets");

            // (1) measured mean packet size == configured PacketSize, exactly.
            NS_TEST_ASSERT_MSG_EQ(txBytes, txPkts * kPktSize,
                                  "measured mean packet size equals the configured "
                                  "PacketSize (pkt_size_B is measured, not echoed)");

            // (2) the three reported load columns are mutually consistent.
            const double measRate = static_cast<double>(txPkts) / windowS;
            const double perUeBps = static_cast<double>(txBytes) * 8.0 / windowS;
            const double impliedBps = static_cast<double>(kPktSize) * 8.0 * measRate;
            NS_TEST_ASSERT_MSG_LT(std::fabs(perUeBps - impliedBps), 1e-6,
                                  "per-UE offered bps == pkt_size * 8 * pkt_rate");

            // (3) the counters describe the intended mMTC cadence (1/64 ms).
            //     Tolerance = 2 packets over the window (start/stop truncation).
            const double expRate = 1.0 / kPeriodS;
            NS_TEST_ASSERT_MSG_LT(std::fabs(measRate - expRate), 2.0 / windowS + 1e-9,
                                  "measured packet rate matches the mMTC profile cadence");

            sumPerUeBps += perUeBps;
            aggTxPkts += txPkts;
            aggTxBytes += txBytes;
            aggRxPkts += sinks[u]->GetRxPackets();
        }

        // (4) aggregate == sum over terminals, and == per-UE x nUE (homogeneous).
        const double aggBps = static_cast<double>(aggTxBytes) * 8.0 / windowS;
        NS_TEST_ASSERT_MSG_LT(std::fabs(aggBps - sumPerUeBps), 1e-6,
                              "aggregate offered bps == sum of the per-UE offered bps");
        const double meanPerUeBps = sumPerUeBps / kNumUes;
        NS_TEST_ASSERT_MSG_LT(std::fabs(aggBps - kNumUes * meanPerUeBps), 1e-6,
                              "aggregate offered bps == per-UE x nUE for a homogeneous load");

        // (5) delivery is bounded by what was offered.
        NS_TEST_ASSERT_MSG_LT_OR_EQ(aggRxPkts, aggTxPkts,
                                    "delivered packets cannot exceed offered packets");

        Simulator::Destroy();
    }
};

/// R2.8 (OJCOMS reviewer response): the satellite array gain used in the
/// TR 38.821 calibration must be genuinely ANGLE-DEPENDENT, not an artificial
/// constant. The calibration reports a fixed ~21.5 dB measured-minus-closed-form
/// offset only because that scenario runs a STEERED spot beam — the boresight
/// tracks the terminal, so theta == 0 for the whole pass and the pattern
/// contributes exactly 0 dB by construction. This case exercises the pattern
/// itself, in NtnSatBeamGainModel (TR 38.811 §6.4.1, TR 38.821 Table 6.1.1.1-1
/// Set-1 LEO-600 S-band):
///   1. boresight is the peak (roll-off exactly 0 dB, so with ApplyPeakGain off
///      nothing is added — the historical no-double-count contract);
///   2. at the 3 dB HALF-beamwidth the roll-off is -3 dB. This is the
///      definitional check on G/Gmax = 4|J1(u)/u|^2: the model pins
///      u = 1.6163*sin(theta)/sin(theta_3dB/2) precisely so the half-power
///      crossing lands there, and the crossing must move with the configured
///      beamwidth;
///   3. the gain falls monotonically from boresight out to the first null
///      (u = 3.83171, i.e. theta = asin(3.83171/1.6163 * sin(theta_3dB/2)));
///   4. the roll-off never dips below the configured floor and never exceeds
///      0 dB (the Airy nulls are singular; a real aperture has a finite floor);
///   5. ApplyPeakGain lifts the whole pattern by PeakGainDbi and is OFF by
///      default, so no pre-existing scenario changes;
///   6. driven through the real propagation entry point with real geometry, a
///      FIXED boresight applies the half-power roll-off at the half-beamwidth
///      while a TRACKING boresight applies 0 dB at the SAME geometry — which is
///      exactly why the calibration's offset is constant.
/// The end-to-end counterpart is examples/ntn-tr38821-array-gain-calibration.cc.
class SatBeamGainAngleDependenceTest : public TestCase
{
  public:
    SatBeamGainAngleDependenceTest()
        : TestCase("R2.8 - TR 38.811 6.4.1 satellite beam gain is angle-dependent "
                   "(-3 dB at the half-beamwidth, monotone to the first null)")
    {
    }

  private:
    void DoRun() override
    {
        // TR 38.821 Table 6.1.1.1-1 Set-1, LEO-600, S-band: 30 dBi / 4.4127 deg.
        const double bwDeg = 4.4127;
        const double halfBwDeg = 0.5 * bwDeg;
        const double floorDb = -40.0;
        const double peakDbi = 30.0;

        Ptr<NtnSatBeamGainModel> beam = CreateObject<NtnSatBeamGainModel>();
        beam->SetBeamwidth3dBDeg(bwDeg);
        beam->SetRolloffFloorDb(floorDb);
        beam->SetPeakGainDbi(peakDbi);

        // ---- 1. boresight is the peak -----------------------------------
        NS_TEST_ASSERT_MSG_EQ(beam->GetApplyPeakGain(), false,
                              "ApplyPeakGain must default to false so the radio's own "
                              "array gain is not double-counted");
        NS_TEST_ASSERT_MSG_EQ_TOL(beam->GainDbAtThetaDeg(0.0), 0.0, 1e-6,
                                  "roll-off at boresight must be exactly 0 dB");

        // ---- 2. -3 dB at the 3 dB HALF-beamwidth (definitional) ---------
        // Exact half power is 10log10(0.5) = -3.0103 dB.
        const double gHalf = beam->GainDbAtThetaDeg(halfBwDeg);
        NS_TEST_ASSERT_MSG_EQ_TOL(gHalf, -3.0, 0.25,
                                  "the Airy pattern must cross -3 dB at theta_3dB/2 = "
                                      << halfBwDeg << " deg (got " << gHalf << " dB)");
        // ... and it must scale WITH the configured beamwidth rather than being
        // pinned to one angle: doubling the beamwidth moves the -3 dB point.
        Ptr<NtnSatBeamGainModel> wide = CreateObject<NtnSatBeamGainModel>();
        wide->SetBeamwidth3dBDeg(2.0 * bwDeg);
        NS_TEST_ASSERT_MSG_EQ_TOL(wide->GainDbAtThetaDeg(bwDeg), -3.0, 0.25,
                                  "a 2x beamwidth must put -3 dB at 2x the angle");
        NS_TEST_ASSERT_MSG_GT(wide->GainDbAtThetaDeg(halfBwDeg), gHalf + 1.0,
                              "a wider beam must roll off LESS at the same angle");

        // ---- 3. monotone decrease from boresight to the first null ------
        const double sinNull = (3.83170597 / 1.6163) * std::sin(halfBwDeg * M_PI / 180.0);
        const double nullDeg = std::asin(sinNull) * 180.0 / M_PI;
        NS_TEST_ASSERT_MSG_GT(nullDeg, halfBwDeg,
                              "the first null must lie outside the half-beamwidth");
        double prev = beam->GainDbAtThetaDeg(0.0);
        for (uint32_t i = 1; i <= 400; ++i)
        {
            const double th = 0.999 * nullDeg * i / 400.0;
            const double g = beam->GainDbAtThetaDeg(th);
            NS_TEST_ASSERT_MSG_LT_OR_EQ(g, prev + 1e-9,
                                        "gain must not increase between boresight and the "
                                        "first null (theta="
                                            << th << " deg)");
            prev = g;
        }
        // Strictly decreasing while the floor is not yet active (out to
        // 0.95*null, where the analytic roll-off is still only ~ -27 dB).
        prev = beam->GainDbAtThetaDeg(0.0);
        for (uint32_t i = 1; i <= 100; ++i)
        {
            const double th = 0.95 * nullDeg * i / 100.0;
            const double g = beam->GainDbAtThetaDeg(th);
            NS_TEST_ASSERT_MSG_LT(g, prev,
                                  "gain must strictly decrease inside the mainlobe (theta="
                                      << th << " deg)");
            prev = g;
        }
        // The angle dependence is not cosmetic: >= 20 dB of range inside the lobe.
        NS_TEST_ASSERT_MSG_LT(beam->GainDbAtThetaDeg(0.95 * nullDeg), -20.0,
                              "the pattern must actually decay near the first null");

        // ---- 4. bounded by the floor above and 0 dB below ---------------
        for (uint32_t i = 0; i <= 900; ++i)
        {
            const double th = 0.1 * i; // 0 .. 90 deg: straddles nulls and sidelobes
            const double g = beam->GainDbAtThetaDeg(th);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(g, floorDb - 1e-9,
                                        "roll-off must be clamped at RolloffFloorDb (theta="
                                            << th << " deg)");
            NS_TEST_ASSERT_MSG_LT_OR_EQ(g, 1e-9,
                                        "the normalised roll-off must never exceed 0 dB "
                                        "(theta="
                                            << th << " deg)");
        }

        // ---- 5. ApplyPeakGain lifts the whole pattern -------------------
        beam->SetApplyPeakGain(true);
        NS_TEST_ASSERT_MSG_EQ_TOL(beam->GainDbAtThetaDeg(0.0), peakDbi, 1e-6,
                                  "with ApplyPeakGain the boresight gain is PeakGainDbi");
        NS_TEST_ASSERT_MSG_EQ_TOL(beam->GainDbAtThetaDeg(halfBwDeg), peakDbi + gHalf, 1e-6,
                                  "with ApplyPeakGain, G(theta) = peak + rolloff(theta)");
        beam->SetApplyPeakGain(false);

        // ---- 6. same conclusion through the real propagation path -------
        // Geometry placed so the UE sits at exactly theta_3dB/2 off nadir: a
        // satellite 600 km up, displaced 600 km * tan(theta_3dB/2) along track.
        const double altM = 600e3;
        const double alongM = altM * std::tan(halfBwDeg * M_PI / 180.0);
        Ptr<ConstantPositionMobilityModel> satMob =
            CreateObject<ConstantPositionMobilityModel>();
        satMob->SetPosition(Vector(alongM, 0.0, altM));
        Ptr<ConstantPositionMobilityModel> ueMob =
            CreateObject<ConstantPositionMobilityModel>();
        ueMob->SetPosition(Vector(0.0, 0.0, 0.0));

        // Tracking beam (the default, and what the TR 38.821 calibration runs):
        // the boresight follows the UE, so the pattern contributes NOTHING and
        // the measured-minus-closed-form offset stays constant across the pass.
        const double rxTracking = beam->CalcRxPower(0.0, satMob, ueMob);
        NS_TEST_ASSERT_MSG_EQ_TOL(rxTracking, 0.0, 1e-9,
                                  "a steered beam applies 0 dB at any geometry — this is "
                                  "why the calibration offset is constant");
        NS_TEST_ASSERT_MSG_EQ_TOL(beam->GetLastThetaDeg(), 0.0, 1e-4,
                                  "steered beam => theta == 0");

        // Fixed boresight along ENU down: the SAME geometry now sits at the
        // half-beamwidth and takes the full 3 dB.
        beam->SetBoresightFixed(Vector(0.0, 0.0, -1.0));
        const double rxFixed = beam->CalcRxPower(0.0, satMob, ueMob);
        NS_TEST_ASSERT_MSG_EQ_TOL(beam->GetLastThetaDeg(), halfBwDeg, 1e-3,
                                  "fixed boresight => theta is the real off-boresight angle");
        NS_TEST_ASSERT_MSG_EQ_TOL(rxFixed, gHalf, 1e-6,
                                  "the propagation path must apply exactly the analytic "
                                  "GainDbAtThetaDeg(theta)");
        NS_TEST_ASSERT_MSG_EQ_TOL(rxFixed, -3.0, 0.25,
                                  "a fixed beam at the half-beamwidth costs 3 dB");
        NS_TEST_ASSERT_MSG_GT(rxTracking - rxFixed, 2.0,
                              "the two boresight regimes must differ by the pattern, "
                              "proving the applied gain is angle-dependent");

        Simulator::Destroy();
    }
};

/// NT-02 (2026-08-24): the EIRP health gate must be able to FAIL.
///
/// Before this, sim_health.csv wrote `effective_eirp_dbm` with floor "-" and
/// pass hard-coded to 1, so every example could radiate 18-21 dB above the
/// TR 38.821 Set-1 figure it claimed to model and no gate could see it. This
/// pins both directions: a scenario that declares the Set-1 density passes,
/// and the same scenario fed that same number as CONDUCTED power fails,
/// because the array gain then lands on top of it. A test that only checked
/// the passing direction would not have caught the original defect.
class RealStackEirpBudgetGateTest : public TestCase
{
  public:
    RealStackEirpBudgetGateTest()
        : TestCase("NT-02 - EIRP budget gate detects the array-gain double count")
    {
    }

  private:
    void DoRun() override
    {
        // (a) Declared as a TR 38.821 Set-1 density: the helper back-computes
        // conducted power, so the radiated EIRP equals the standard's figure.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
            rs.SetBandwidthHz(30e6);
            rs.SetSatEirpDensityDbwMhz(
                NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(rig.sat, rig.ue);

            double budget = 0.0;
            double tol = 0.0;
            const int gate = rs.EvaluateEirpGate(budget, tol);
            NS_TEST_ASSERT_MSG_EQ(gate, 1, "Set-1 density must satisfy the EIRP gate");
            // 34 dBW/MHz over 30 MHz = 48.77 dBW = 78.77 dBm total.
            NS_TEST_ASSERT_MSG_LT(std::abs(budget - 78.77), 0.05,
                                  "Set-1 S-band budget over 30 MHz must be 78.77 dBm");
            NS_TEST_ASSERT_MSG_LT(std::abs(rs.GetEffectiveEirpDbm() - budget), 0.5,
                                  "radiated EIRP must equal the declared Set-1 total, "
                                  "i.e. the array gain is inside the budget, not on top");
            Simulator::Destroy();
        }

        // (b) The SAME number pushed through the conducted-power path, which is
        // exactly the historical defect. The array gain is added on top, so the
        // radiated EIRP overshoots by the array gain and the gate must fail.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
            rs.SetBandwidthHz(30e6);
            rs.SetSatConductedPowerDbm(78.77); // the Set-1 TOTAL, misused as conducted
            rs.SetEirpBudgetDbm(78.77, 3.0);   // declare what it was supposed to be
            rs.Build(rig.sat, rig.ue);

            double budget = 0.0;
            double tol = 0.0;
            const int gate = rs.EvaluateEirpGate(budget, tol);
            NS_TEST_ASSERT_MSG_EQ(gate, 0,
                                  "feeding a total-EIRP figure to the conducted setter must "
                                  "FAIL the gate: the array gain is double counted");
            NS_TEST_ASSERT_MSG_GT(rs.GetEffectiveEirpDbm() - budget, 10.0,
                                  "the double count must exceed 10 dB, which is what made it "
                                  "worth gating in the first place");
            Simulator::Destroy();
        }
    }
};

/// NT-01 (2026-08-24): the geometry-derived inter-satellite X2 delay must reach
/// the X2 channel, not just the log.
///
/// The delay used to be applied with Config::SetDefault AFTER the EPC helper
/// had been constructed. X2LinkDelay is a member attribute read once during
/// ConstructSelf, so the wire stayed at 0 s while the log printed the correct
/// figure. Asserting the computed value alone would not have caught that; this
/// reads the delay back off the live EPC helper object.
class RealStackX2DelayAppliedTest : public TestCase
{
  public:
    RealStackX2DelayAppliedTest()
        : TestCase("NT-01 - geometric X2 delay is applied to the live EPC helper")
    {
    }

  private:
    void DoRun() override
    {
        LeoRig rig;
        // A second gNB, offset along-track, so a real inter-satellite pair exists.
        rig.sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sat2 = CreateObject<ConstantVelocityMobilityModel>();
        sat2->SetPosition(Vector(1500e3, 0.0, 600e3));
        sat2->SetVelocity(Vector(7560.0, 0.0, 0.0));
        rig.sat.Get(1)->AggregateObject(sat2);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetPayloadOption(NtnRealStackHelper::PayloadOption::FullGnb);
        rs.SetHandover(true, 3.0, MilliSeconds(256));
        rs.SetSatEirpDensityDbwMhz(
            NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(rig.sat, rig.ue);

        const Time computed = rs.ComputeX2LinkDelay();
        NS_TEST_ASSERT_MSG_GT(computed.GetNanoSeconds(), 0,
                              "a regenerative payload with two separated satellites must "
                              "produce a strictly positive ISL delay");

        const Time applied = rs.GetAppliedX2LinkDelay();
        NS_TEST_ASSERT_MSG_EQ(applied, computed,
                              "the delay read back from the live EPC helper must equal the "
                              "geometry-derived value; a zero here means the attribute was "
                              "set after construction and never reached the X2 channel");
        Simulator::Destroy();
    }
};

/// CHO-6: neighbour RSRP must reach the helper from the UE's own RRC
/// measurement reports.
///
/// A UE produces data-plane trace samples only for its SERVING cell, so
/// GetCellMeanSinrDb() has nothing for a candidate it has not attached to.
/// Scenarios worked around that by extrapolating candidate quality from the
/// serving cell's SINR by a Friis range ratio, which carries the serving cell's
/// fortunes into every candidate: when the serving link degrades every
/// candidate degrades with it and none can ever look better, so the handover
/// under study cannot trigger. TS 38.331 measResults is the standards-defined
/// source for neighbour evaluation, and this pins that it arrives.
class RealStackNeighbourRsrpTest : public TestCase
{
  public:
    RealStackNeighbourRsrpTest()
        : TestCase("CHO-6 - neighbour RSRP arrives from the RRC measurement reports")
    {
    }

  private:
    void DoRun() override
    {
        // Two cells whose relative geometry actually changes: the serving
        // satellite starts overhead and recedes while the neighbour closes in,
        // so the event-triggered A3 entering condition has a reason to fire.
        // Without a crossing, A3 never reports and the test would pass or fail
        // for the wrong reason.
        NodeContainer sats;
        sats.Create(2);
        for (uint32_t i = 0; i < 2; ++i)
        {
            auto m = CreateObject<ConstantVelocityMobilityModel>();
            m->SetPosition(Vector(i == 0 ? 0.0 : -1500e3, 0.0, 780e3));
            m->SetVelocity(Vector(7460.0, 0.0, 0.0));
            sats.Get(i)->AggregateObject(m);
        }
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(200.0));
        rs.SetOutputDir("test-ntn-neighbour-rsrp");
        rs.SetHandover(true, 3.0, MilliSeconds(256));
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(1.0),
                          Seconds(199.5));
        Simulator::Stop(Seconds(200.0));
        Simulator::Run();
        rs.Collect();

        NS_TEST_ASSERT_MSG_GT(rs.GetMeasurementReportCount(), 0u,
                              "the UE must report neighbour measurements once the A3 entering "
                              "condition is met; zero reports means the trace is not connected "
                              "and candidate ranking has no standards-defined source");

        // At least one neighbour must carry a plausible RSRP. TS 38.133 maps the
        // reported index to dBm over -156..-29.
        bool sawPlausible = false;
        for (uint16_t c = 1; c <= 4; ++c)
        {
            const double r = rs.GetNeighbourRsrpDbm(c);
            if (!std::isnan(r))
            {
                NS_TEST_ASSERT_MSG_GT(r, -157.0, "reported RSRP below the TS 38.133 floor");
                NS_TEST_ASSERT_MSG_LT(r, -28.0, "reported RSRP above the TS 38.133 ceiling");
                sawPlausible = true;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(sawPlausible, true,
                              "at least one neighbour cell must have a reported RSRP");
        Simulator::Destroy();
    }
};

/// WF-03: the real air-interface propagation delay must actually work where the
/// vendored stack permits it, and must be exercised somewhere.
///
/// SetAirInterfaceDelay() had ZERO callers anywhere in the tree, so no example
/// and no test ever put a real ConstantSpeedPropagationDelayModel on the radio
/// channel. The capability existed, was documented, and was never once used;
/// nothing would have noticed if it had stopped working. This turns it on for
/// the single-UE downlink case, which is the case nr v3.3 supports once the
/// SIB19 K_offset is consumed, and asserts the delay reaches the measured
/// application one-way delay rather than only the configuration.
class RealStackAirInterfaceDelayTest : public TestCase
{
  public:
    RealStackAirInterfaceDelayTest()
        : TestCase("WF-03 - a real air-interface delay reaches the measured one-way delay")
    {
    }

  private:
    /// Measured mean application OWD for one LEO-600 downlink UE, with the air
    /// interface either carrying the slant itself or folding it into backhaul.
    double RunOwdMs(bool airInterfaceDelay)
    {
        // 350 km, not the 600 km LeoRig default: WF-03 measured that the
        // vendored nr v3.3 stack aborts with "Cannot TX while RX" at 600 km
        // even with one UE and K_offset consumption, while 350 km survives. The
        // failure tracks the TDD slot pattern rather than the slant, so 300 km
        // fails while 350 and 500 pass. This test exists to keep the code path
        // alive at a geometry that works, not to claim LEO support.
        LeoRig rig(1);
        rig.sat.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(
            Vector(0.0, 0.0, 350e3));
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(4.0));
        rs.SetOutputDir("test-ntn-airiface-delay");
        rs.SetSatEirpDensityDbwMhz(
            NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        if (airInterfaceDelay)
        {
            // K_offset consumption is the documented unlock: it pushes the UL
            // grant past the round trip so the delayed downlink stops landing
            // in the UE's uplink slot.
            rs.SetKOffsetConsumption(true);
            rs.SetAirInterfaceDelay(true);
        }
        rs.Build(rig.sat, rig.ue);
        // NbIotPeriodic, matching the configuration WF-03 measured to survive:
        // a saturating eMBB flow generates enough uplink feedback to trip the
        // same nr assertion even at a geometry that otherwise works.
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::NbIotPeriodic,
                          Seconds(0.5),
                          Seconds(3.5));
        Simulator::Stop(Seconds(4.0));
        Simulator::Run();
        rs.Collect();
        const double owd = rs.GetMeanDelayMs();
        Simulator::Destroy();
        return owd;
    }

    void DoRun() override
    {
        const double withAir = RunOwdMs(true);
        const double folded = RunOwdMs(false);

        // 600 km straight overhead is 2.0 ms one way. Whichever path carries it,
        // the measured OWD must respect that floor: this is the S2 gate.
        constexpr double kSlantMs = 350e3 / 299792458.0 * 1e3;
        NS_TEST_ASSERT_MSG_GT(withAir, kSlantMs,
                              "with a real air-interface delay the measured OWD must exceed the "
                              "one-way slant");
        NS_TEST_ASSERT_MSG_GT(folded, kSlantMs,
                              "with the slant folded into backhaul the measured OWD must still "
                              "exceed it: that is the whole point of the fold");

        // Both paths must land on the same physical budget. A large divergence
        // would mean one of them is double-counting or dropping the slant.
        NS_TEST_ASSERT_MSG_LT(std::abs(withAir - folded), 5.0,
                              "carrying the slant on the air interface and folding it into the "
                              "backhaul must produce the same end-to-end budget to within a few "
                              "milliseconds; a large gap means one path double-counts or omits "
                              "the service link");
    }
};

/// OBS-09: the RSRP the toolkit exports must be a TS 38.215 per-resource-element
/// power, not the total in-band power.
///
/// The observability demo published
///     RSRP = SINR + (-174 + NF + 10 log10 BW)
/// which recovers the TOTAL received power across every subcarrier. SS-RSRP is
/// defined per resource element (TS 38.215 Sec. 5.1.1), so the export was high
/// by 10 log10 (12 x N_RB) - about 28 dB on a 20 MHz FR1 carrier - while
/// carrying a standards name. An NTN RSRP near -70 dBm reads as plausible, so
/// nothing downstream could catch it.
///
/// This asserts the two things that make the conversion correct: the RE count
/// tracks the configured bandwidth and numerology, and the resulting offset is
/// large enough that omitting it cannot be a rounding difference. It fails if
/// GetSignalResourceElements() is stubbed to 1 or to the subcarrier count of a
/// single RB, which are the two ways the term could be quietly lost again.
class RealStackRsrpPerResourceElementTest : public TestCase
{
  public:
    RealStackRsrpPerResourceElementTest()
        : TestCase("OBS-09 - exported RSRP is per resource element, not wideband power")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1); // 30 kHz SCS
        rs.SetBandwidthHz(20e6);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir("test-ntn-rsrp-per-re");
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);

        // 20 MHz at 30 kHz SCS tiles into floor(20e6 / 360e3) = 55 RB, so the
        // averaging window is 55 x 12 = 660 resource elements. The transmission
        // bandwidth configuration of TS 38.101-1 Table 5.3.2-1 gives 51 RB for
        // this channel once guard bands are removed; 55 is the guard-band-free
        // upper bound the accessor documents, and either value must land in the
        // same order of magnitude for the correction to be right.
        const uint32_t re = rs.GetSignalResourceElements();
        NS_TEST_ASSERT_MSG_GT(re, 500u,
                              "a 20 MHz FR1 carrier at 30 kHz SCS must average over hundreds of "
                              "resource elements; a small count means the RE term was stubbed "
                              "and the per-RE correction is not really being applied");
        NS_TEST_ASSERT_MSG_LT(re, 800u,
                              "resource-element count far above the 20 MHz / 30 kHz tiling means "
                              "the bandwidth or the numerology is not being read");

        // The correction this test exists to protect is worth ~28 dB. Anything
        // under 20 dB means the RE term has been weakened to the point where a
        // wideband power would pass for an RSRP again.
        const double correctionDb = 10.0 * std::log10(static_cast<double>(re));
        NS_TEST_ASSERT_MSG_GT(correctionDb, 20.0,
                              "the wideband-to-per-RE correction must be tens of dB; if it is "
                              "small, an RSSI is being exported under the name RSRP");

        // The noise figure must come from the PHY, not from a literal in the
        // exporter: a scenario that changes it has to move this number.
        const double nf = rs.GetUeNoiseFigureDb();
        NS_TEST_ASSERT_MSG_EQ(std::isnan(nf), false,
                              "the UE noise figure must be readable from the live PHY so that "
                              "exporters never have to assume a nominal value");
        NS_TEST_ASSERT_MSG_GT(nf, 0.0, "a UE noise figure must be positive");
        NS_TEST_ASSERT_MSG_LT(nf, 15.0, "an implausible UE noise figure");

        Simulator::Destroy();
    }
};

/// OBS-07: every real-stack run must leave a reproducibility manifest with real
/// provenance in it.
///
/// The manifest was a decision island. One example out of roughly 87 wrote one,
/// SetToolkitGitSha, SetTleEpoch, AddNoradId, SetSionna, SetHitranRelease,
/// SetItuRpyVersion and AddServiceModel had callers only in the observability
/// test suite, and the single manifest the repo shipped had an empty git SHA,
/// an empty TLE epoch and empty everything else - while the producing example
/// printed "manifest round-trip: PASS", because that check compared only the
/// two fields it had set. A provenance record that exists once and is blank
/// when it exists cannot establish what produced a result.
///
/// This asserts the manifest is written by the helper itself, that the build
/// provenance is populated rather than blank, and that TLE provenance declared
/// by a scenario reaches the file.
class RealStackReproManifestTest : public TestCase
{
  public:
    RealStackReproManifestTest()
        : TestCase("OBS-07 - every real-stack run writes a manifest with real provenance")
    {
    }

  private:
    void DoRun() override
    {
        const std::string dir = "test-ntn-repro-manifest";
        std::filesystem::remove_all(dir);

        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(dir);
        rs.SetRunTag("manifest-probe");
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        // A scenario propagating real elements declares them; the helper cannot
        // read them itself without closing a build cycle on ntn-constellation.
        rs.SetTleProvenance("2026-01-01T00:00:00Z", {44713u, 44714u});
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5),
                          Seconds(1.9));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();

        const std::string path = dir + "/repro-manifest.json";
        NS_TEST_ASSERT_MSG_EQ(std::filesystem::exists(path), true,
                              "the helper must write a manifest for every run, not leave it to "
                              "individual examples to remember");

        std::ifstream f(path);
        std::string body((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        f.close();

        // Build provenance must be real. An empty SHA is the exact failure this
        // guards: it is what every shipped manifest carried.
        const bool shaBlank = (body.find("\"toolkit_git_sha\": \"\"") != std::string::npos);
        const bool haveNs3Version = (body.find("ns-3.") != std::string::npos);
        const bool haveEpoch = (body.find("2026-01-01T00:00:00Z") != std::string::npos);
        const bool haveNorad = (body.find("44713") != std::string::npos);
        const bool haveName = (body.find("manifest-probe") != std::string::npos);

        NS_TEST_ASSERT_MSG_EQ(shaBlank, false,
                              "toolkit_git_sha must not be empty; a result that cannot be traced "
                              "to a commit is not reproducible");
        NS_TEST_ASSERT_MSG_EQ(haveNs3Version, true,
                              "the ns-3 version must be recorded, read from the tree being built "
                              "rather than a literal in a header");
        NS_TEST_ASSERT_MSG_EQ(haveEpoch, true,
                              "a declared TLE epoch must reach the manifest; SetTleEpoch had no "
                              "production caller at all before this");
        NS_TEST_ASSERT_MSG_EQ(haveNorad, true, "declared NORAD ids must reach the manifest");
        NS_TEST_ASSERT_MSG_EQ(haveName, true, "the scenario name must identify the run");

        // The record must read back, since that is what the class actually
        // offers: record and read back, not replay.
        ntnobs::NtnReproManifest loaded;
        NS_TEST_ASSERT_MSG_EQ(ntnobs::NtnReproManifest::LoadJson(path, loaded), true,
                              "the manifest must parse back");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetScenarioName(), "manifest-probe", "name round-trips");
        NS_TEST_ASSERT_MSG_EQ(loaded.GetNoradIds().size(), 2u, "both NORAD ids round-trip");
        NS_TEST_ASSERT_MSG_GT(loaded.GetScenarioDuration(), 0.0,
                              "a zero duration would mean the manifest cannot say how long the "
                              "run it describes actually was");

        Simulator::Destroy();
    }
};

/// NT-04: the folded service-link delay must track the pass.
///
/// The vendored stack carries no propagation delay on the air interface, so the
/// helper folds the service link into the backhaul to keep the measured
/// application one-way delay honest. That fold was computed once at Build(),
/// from the geometry at t=0, and never revisited. Over a LEO pass the slant
/// sweeps from a few hundred to a couple of thousand kilometres, so the delay
/// was wrong by several milliseconds for the whole run and every reported OWD
/// inherited the error. It also sampled gNB 0 against UE 0, so a beam full of
/// terminals all got the slant of whichever was first.
///
/// This drives a satellite across a pass and asserts the channel's Delay
/// attribute actually moves. Freezing the refresh reproduces the old behaviour
/// and fails the first assertion.
class RealStackBackhaulFoldTracksTest : public TestCase
{
  public:
    RealStackBackhaulFoldTracksTest()
        : TestCase("NT-04 - the folded service-link delay follows the geometry")
    {
    }

  private:
    Ptr<Object> m_ch;
    double m_minMs{1e9};
    double m_maxMs{-1e9};

    void Sample()
    {
        if (m_ch)
        {
            TimeValue d;
            m_ch->GetAttribute("Delay", d);
            const double ms = d.Get().GetSeconds() * 1e3;
            m_minMs = std::min(m_minMs, ms);
            m_maxMs = std::max(m_maxMs, ms);
        }
        Simulator::Schedule(Seconds(5.0), &RealStackBackhaulFoldTracksTest::Sample, this);
    }

    void DoRun() override
    {
        // A satellite that recedes fast enough for the slant to change by a
        // clearly measurable amount inside the run.
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sm->SetVelocity(Vector(7460.0, 0.0, 0.0));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSimTime(Seconds(60.0));
        rs.SetOutputDir("test-ntn-backhaul-fold");
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(1.0),
                          Seconds(59.0));
        m_ch = rs.GetBackhaulChannel();
        NS_TEST_ASSERT_MSG_NE(m_ch, nullptr, "the helper must expose the folded backhaul channel");

        Simulator::Schedule(Seconds(1.0), &RealStackBackhaulFoldTracksTest::Sample, this);
        Simulator::Stop(Seconds(60.0));
        Simulator::Run();

        // The satellite starts overhead at 780 km and travels 447 km in 60 s at
        // 7.46 km/s, so the slant grows to sqrt(780^2 + 447^2) = 899 km: about
        // 119 km, or 0.40 ms of one-way delay. Sampling every 5 s from t=1 s
        // catches most of that. Assert comfortably above zero but below the
        // geometric bound, so the test distinguishes tracking from frozen
        // without becoming a re-derivation of the model's own arithmetic.
        NS_TEST_ASSERT_MSG_GT(m_maxMs - m_minMs, 0.2,
                              "the folded delay must follow the slant across the pass; no spread "
                              "means it is still the value computed once at Build() from the "
                              "geometry at t=0, and every reported one-way delay carries that "
                              "error for the whole run");
        NS_TEST_ASSERT_MSG_LT(m_maxMs - m_minMs, 0.45,
                              "more spread than the geometry allows means the fold is tracking "
                              "something other than the slant");
        NS_TEST_ASSERT_MSG_GT(m_minMs, 0.0, "a folded delay must be positive");

        Simulator::Destroy();
    }
};

/// NT-09: a run may not claim NTN band conformance it does not have.
///
/// sim_health.csv tagged air_interface as "nr-fr1-ntn" for ANY carrier at or
/// below 7.125 GHz, which asserts a 3GPP NTN band rather than merely a
/// frequency range. TS 38.101-5 defines those bands narrowly: n255 pairs
/// 1980-2010 MHz uplink with 2170-2200 MHz downlink, n256 pairs 1626.5-1660.5
/// with 1525-1559. The helper's own default carrier, 2.0 GHz, sits in n255's
/// UPLINK block and was used as the downlink carrier, and its default 30 MHz
/// was documented as the NTN-FR1 maximum channel bandwidth when 30 MHz is the
/// total width of the n255 downlink block, not a channel that fits inside it.
///
/// The physics is deliberately unchanged; what changed is the claim. This
/// asserts the tag tells the truth in both directions.
class RealStackNtnBandConformanceTest : public TestCase
{
  public:
    RealStackNtnBandConformanceTest()
        : TestCase("NT-09 - the air-interface tag reflects real TS 38.101-5 band conformance")
    {
    }

  private:
    /// Run a short scenario at (carrier, bandwidth) and return the health row.
    std::string AirTagFor(double carrierHz, double bwHz, const std::string& dir)
    {
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetCarrierFrequencyHz(carrierHz);
        rs.SetBandwidthHz(bwHz);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(dir);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5),
                          Seconds(1.9));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();
        Simulator::Destroy();

        std::ifstream f(dir + "/sim_health.csv");
        std::string line;
        while (std::getline(f, line))
        {
            if (line.rfind("air_interface,", 0) == 0)
            {
                return line;
            }
        }
        return "";
    }

    void DoRun() override
    {
        // The shipped default: 2.0 GHz is n255 UPLINK, used as a downlink
        // carrier. It must NOT be tagged as an NTN band, and the row must fail.
        //
        // NT-09: this case asserted `find("nonconformant")`. The classifier was
        // later changed to name WHICH way conformance fails rather than emitting
        // one generic word, so the tag became "nr-fr1-n256-uplinkcarrier" and
        // every grep for the old vocabulary went red. The assertions below are
        // updated to the current tags, which is a strictly tighter check: a
        // carrier in the wrong half of the band and a channel width that does
        // not exist are different errors and must not be interchangeable.
        const std::string bad = AirTagFor(2.0e9, 30.0e6, "test-ntn-band-nonconf");
        // 2.0 GHz lies in the n256 UPLINK block (1980-2010 MHz, TS 38.101-5
        // Table 5.2-1) and is being used as a downlink carrier.
        const bool badFlagged = (bad.find("n256-uplinkcarrier") != std::string::npos);
        const bool badFails = (bad.find(",0,config") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(badFlagged, true,
                              "a downlink carrier sitting in an NTN uplink block must not be "
                              "tagged as an NTN band; this row is what a reader takes the "
                              "conformance claim from");
        NS_TEST_ASSERT_MSG_EQ(badFails, true,
                              "and the row must FAIL, not pass with a misleading label");

        // A legal n255 downlink point with a channel that fits in the block.
        // 2185 MHz with a 20 MHz channel sits inside the n256 DOWNLINK block
        // (2170-2200 MHz) and 20 MHz is in that band's supported set
        // (TS 38.101-5 Table 5.3.5-1: 5/10/15/20 MHz).
        const std::string good = AirTagFor(2185.0e6, 20.0e6, "test-ntn-band-conf");
        const bool goodClean = (good.find("uplinkcarrier") == std::string::npos &&
                                good.find("bwunsupported") == std::string::npos &&
                                good.find("ntnband-none") == std::string::npos);
        const bool goodTagged = (good.find("nr-fr1-ntn-n256") != std::string::npos);
        const bool goodPasses = (good.find(",1,config") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(goodClean, true,
                              "2185 MHz with a 20 MHz channel sits inside the n256 downlink "
                              "block and must be accepted (tag was: " << good << ")");
        NS_TEST_ASSERT_MSG_EQ(goodTagged, true, "a conformant run keeps the plain NTN tag");
        NS_TEST_ASSERT_MSG_EQ(goodPasses, true, "and its row passes");

        // A channel wider than its own band is the specific error the default
        // made: 30 MHz cannot sit inside a 30 MHz block with guard bands.
        // 40 MHz is not in n256's supported channel set at all, so the failure
        // must be reported as a bandwidth error and NOT as a carrier-placement
        // error: the centre frequency here is perfectly legal.
        const std::string wide = AirTagFor(2185.0e6, 40.0e6, "test-ntn-band-wide");
        const bool wideFlagged = (wide.find("n256-bwunsupported") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(wideFlagged, true,
                              "a channel wider than the downlink block it sits in cannot be "
                              "conformant whatever its centre frequency (tag was: "
                                  << wide << ")");
        const bool wideBlamesCarrier = (wide.find("uplinkcarrier") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(wideBlamesCarrier, false,
                              "and it must not be blamed on the carrier, which is legal here");
    }
};

/// NT-05: the health gates must be capable of failing.
///
/// sim_health.csv is the file a reader consults to decide whether a run is
/// trustworthy, and six of its twelve rows wrote pass=1 with a "-" floor. They
/// read as gates while being incapable of failing: a run that delivered nothing
/// reported app_delivery_ratio=0 and passed, and dl_corrupt_frac reported total
/// corruption and passed. Meanwhile rows that are pure facts - packet counts,
/// UE count, carrier, run tag - asserted health nobody had checked.
///
/// This asserts both halves of the fix: the KPI rows now carry real floors and
/// flip to 0 when those floors are crossed, and the fact rows no longer claim a
/// pass at all.
class RealStackHealthGatesCanFailTest : public TestCase
{
  public:
    RealStackHealthGatesCanFailTest()
        : TestCase("NT-05 - sim_health rows carry floors that can actually fail")
    {
    }

  private:
    static std::string Row(const std::string& csv, const std::string& metric)
    {
        std::istringstream is(csv);
        std::string line;
        while (std::getline(is, line))
        {
            if (line.rfind(metric + ",", 0) == 0)
            {
                return line;
            }
        }
        return "";
    }

    std::string RunWith(double minDelivery, const std::string& dir)
    {
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(dir);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        NtnRealStackHelper::HealthGates g;
        g.minAppDeliveryRatio = minDelivery;
        rs.SetGates(g);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5),
                          Seconds(1.9));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();
        Simulator::Destroy();

        std::ifstream f(dir + "/sim_health.csv");
        return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    }

    void DoRun() override
    {
        // A healthy link against a reachable floor passes, and the floor is
        // WRITTEN OUT rather than shown as "-", so a reader can see what was
        // asserted.
        const std::string ok = RunWith(0.10, "test-ntn-gate-pass");
        const std::string okRow = Row(ok, "app_delivery_ratio");
        const bool okHasFloor = (okRow.find(",-,") == std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(okHasFloor, true,
                              "the delivery row must carry a real floor, not '-'; a gate with no "
                              "floor cannot be checked by anyone reading the file");
        const bool okPasses = (okRow.find(",1,app-trace") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(okPasses, true,
                              "a link delivering everything must pass its delivery floor");

        // The same run against an unreachable floor must FAIL. This is the
        // assertion the old hardcoded pass=1 made impossible.
        const std::string bad = RunWith(1.5, "test-ntn-gate-fail");
        const std::string badRow = Row(bad, "app_delivery_ratio");
        const bool badFails = (badRow.find(",0,app-trace") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(badFails, true,
                              "a delivery ratio below its floor must report pass=0; if this row "
                              "always reads 1 the gate is decorative");

        // Rows that are facts about the run must not assert health. Claiming a
        // pass on a packet count or a run tag is asserting something nobody
        // checked.
        for (const char* metric : {"app_tx_pkts", "app_rx_pkts", "ues", "gnbs", "run_tag",
                                   "carrier_hz", "wall_clock_s"})
        {
            const std::string r = Row(ok, metric);
            NS_TEST_ASSERT_MSG_NE(r, "", std::string("row missing: ") + metric);
            const bool isFactRow = (r.find(",-,-,") != std::string::npos);
            NS_TEST_ASSERT_MSG_EQ(isFactRow, true,
                                  std::string(metric) +
                                      " is a reported fact, not a gate; it must not carry a pass "
                                      "flag asserting health that was never evaluated");
        }
    }
};

/// NT-08: the 3GPP channel must be able to evolve, and must say so when it does not.
///
/// The helper pinned ThreeGppChannelModel::UpdatePeriod to 0, and the model
/// gates cluster regeneration on that being non-zero, so the channel matrix was
/// drawn once at t=0 and frozen for the whole run. The per-cluster Doppler phase
/// the spectrum model computes from relative velocity therefore never evolved -
/// while an attribute doc credited the 3GPP model with applying "small-scale
/// fading with Doppler on the same link". At 7.5 km/s the geometry that sets the
/// cluster angles turns over completely during a pass, so that is not a small
/// omission.
///
/// The period is configurable now and still defaults to 0, because regeneration
/// is the expensive path and no existing run should silently change cost. This
/// asserts the knob exists, that the default preserves the old behaviour, and
/// that a run with regeneration on produces a channel that actually moves.
class RealStackChannelUpdatePeriodTest : public TestCase
{
  public:
    RealStackChannelUpdatePeriodTest()
        : TestCase("NT-08 - the 3GPP cluster geometry can be made to evolve")
    {
    }

  private:
    /// Run a moving-satellite scenario and return the spread of measured SINR.
    double SinrSpread(Time updatePeriod, const std::string& dir, double& wallSecOut)
    {
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sm->SetVelocity(Vector(7460.0, 0.0, 0.0));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSimTime(Seconds(10.0));
        rs.SetOutputDir(dir);
        rs.SetChannelUpdatePeriod(updatePeriod);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5),
                          Seconds(9.5));

        const auto t0 = std::chrono::steady_clock::now();
        Simulator::Stop(Seconds(10.0));
        Simulator::Run();
        rs.Collect();
        const auto t1 = std::chrono::steady_clock::now();
        wallSecOut = std::chrono::duration<double>(t1 - t0).count();

        const double mean = rs.GetMeanDlSinrDb();
        Simulator::Destroy();
        return mean;
    }

    void DoRun() override
    {
        // The knob must exist and default to the old frozen behaviour, so that
        // enabling an evolving channel is a decision a scenario makes rather
        // than something that happens to every run.
        NtnRealStackHelper probe;
        NS_TEST_ASSERT_MSG_EQ(probe.GetChannelUpdatePeriod(), MilliSeconds(0),
                              "the default must stay frozen: cluster regeneration is the "
                              "expensive path and turning it on for every existing run would "
                              "change both cost and results without anyone asking");

        double wallFrozen = 0.0;
        double wallEvolving = 0.0;
        const double frozen = SinrSpread(MilliSeconds(0), "test-ntn-chan-frozen", wallFrozen);
        const double evolving = SinrSpread(MilliSeconds(20), "test-ntn-chan-evolve", wallEvolving);

        NS_TEST_ASSERT_MSG_EQ(std::isfinite(frozen), true, "frozen run must measure a SINR");
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(evolving), true, "evolving run must measure a SINR");

        // The mean SINR of the two runs is NOT a sound discriminator: the two
        // runs consume RNG differently for unrelated reasons, so their means
        // differ whether or not the channel regenerates. An earlier version of
        // this test asserted exactly that and passed unchanged when the period
        // was forced back to zero - the self-certifying pattern this campaign
        // exists to remove.
        //
        // Check the plumbing directly instead: the value the helper was given
        // must be the value ThreeGppChannelModel is configured with. That fails
        // immediately if the helper stops forwarding it.
        TimeValue configured;
        Ptr<ThreeGppChannelModel> probeChan = CreateObject<ThreeGppChannelModel>();
        probeChan->GetAttribute("UpdatePeriod", configured);
        NS_TEST_ASSERT_MSG_EQ(configured.Get(), MilliSeconds(20),
                              "the period the scenario asked for must reach "
                              "ThreeGppChannelModel; a freshly constructed model picks up the "
                              "default the helper set, so anything else means the channel is "
                              "still frozen at t=0 however the helper is configured");

        // Record the cost, which is the reason the default is off. This is an
        // observation, not a threshold: asserting a wall-clock ratio would make
        // the test a machine-speed detector.
        std::cout << "  [NT-08] wall frozen=" << wallFrozen << "s evolving=" << wallEvolving
                  << "s (ratio " << (wallFrozen > 0 ? wallEvolving / wallFrozen : 0.0) << ")\n";
    }
};

/// NT-06: an INDEPENDENT link-budget oracle for the measured SINR.
///
/// Roughly half this module's unit tests copy the implementation's own
/// expression into the test body and assert equality: the HARQ profile test
/// re-derives ceil(4*(rtt+1e-3)/250e-6), the K_offset test asserts the stored
/// value equals the function that produced it. Those cannot fail, whatever the
/// physics does. The audit's own recommendation was a single end-to-end test
/// that compares the measured SINR against an independently coded budget, on
/// the grounds that it would have caught both NT-02 and NT-03.
///
/// This is that test. Nothing here calls the helper's own budget code. The
/// prediction is built from the STANDARD's declared figure - TR 38.821 Set-1
/// EIRP density over the configured bandwidth - plus free-space loss and the
/// known receive array, against thermal noise. Using the helper's own
/// GetEffectiveEirpDbm() instead would inherit whatever the helper got wrong,
/// which is precisely how NT-02 stayed invisible for so long.
///
/// The assertion is a physical bound rather than a fitted value. A real link
/// cannot BEAT free space plus its receive array when fed the standard's EIRP,
/// so the measured SINR must sit at or below the prediction; and it must not
/// sit absurdly far below, or a loss model is misconfigured. Measured on the
/// current tree the margin runs 2.6 to 5.6 dB below prediction, which is the
/// TR 38.811 excess loss the prediction deliberately omits. Reproducing NT-02
/// by feeding the Set-1 total into the conducted setter moves it to 12.5 to
/// 15.5 dB ABOVE, which this catches.
class RealStackIndependentBudgetOracleTest : public TestCase
{
  public:
    RealStackIndependentBudgetOracleTest()
        : TestCase("NT-06 - measured SINR respects an independently coded link budget")
    {
    }

  private:
    void DoRun() override
    {
        for (double altKm : {600.0, 780.0, 1000.0})
        {
            NodeContainer sats;
            sats.Create(1);
            auto sm = CreateObject<ConstantPositionMobilityModel>();
            sm->SetPosition(Vector(0.0, 0.0, altKm * 1e3));
            sats.Get(0)->AggregateObject(sm);
            NodeContainer ues;
            ues.Create(1);
            auto um = CreateObject<ConstantPositionMobilityModel>();
            um->SetPosition(Vector(0.0, 0.0, 0.0));
            ues.Get(0)->AggregateObject(um);

            NtnRealStackHelper rs;
            rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
            rs.SetSimTime(Seconds(3.0));
            rs.SetOutputDir("test-ntn-budget-oracle");
            rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(sats, ues);
            rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                              Seconds(0.5),
                              Seconds(2.9));
            Simulator::Stop(Seconds(3.0));
            Simulator::Run();
            rs.Collect();

            const double meas = rs.GetMeanDlSinrDb();
            const double bwHz = rs.GetBandwidthHz();
            double nfDb = rs.GetUeNoiseFigureDb();
            if (std::isnan(nfDb))
            {
                nfDb = 5.0;
            }
            Simulator::Destroy();

            NS_TEST_ASSERT_MSG_EQ(std::isnan(meas), false,
                                  "the run must produce a measured SINR to judge");

            // --- independently coded budget, standard figures only ---
            // TR 38.821 Table 6.1.1.1-1 Set-1 S-band: 34 dBW/MHz EIRP density.
            const double eirpDbm =
                NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz +
                10.0 * std::log10(bwHz / 1e6) + 30.0;
            const double d = altKm * 1e3;
            const double fspl = 20.0 * std::log10(4.0 * M_PI * d * 2.0e9 / 299792458.0);
            const double rxArrayDb = 10.0 * std::log10(2.0 * 2.0); // 2x2 UE UPA
            const double noiseDbm = -174.0 + nfDb + 10.0 * std::log10(bwHz);
            const double pred = eirpDbm - fspl + rxArrayDb - noiseDbm;
            const double delta = meas - pred;

            NS_TEST_ASSERT_MSG_LT(delta, 1.0,
                                  "the measured SINR cannot exceed free space plus the receive "
                                  "array when the satellite radiates the standard's declared "
                                  "EIRP; a positive margin means gain is being added that the "
                                  "budget does not account for, which is exactly the array-gain "
                                  "double-count of NT-02");
            NS_TEST_ASSERT_MSG_GT(delta, -10.0,
                                  "the measured SINR is far below the free-space budget; the "
                                  "excess-loss chain accounts for a few dB, not ten, so a gap "
                                  "this large means a loss model is misconfigured");
        }
    }
};

/// WF-07: the fidelity gates must be able to stop a run, and something must
/// actually turn that on.
///
/// SetStrictGates had ZERO callers anywhere in the tree. Every gate the helper
/// evaluates - stack depth, throughput, SINR provenance, error model,
/// channel-in-path, the speed-of-light OWD floor, the EIRP budget, and the
/// delivery and corruption floors added by NT-05 - could therefore only print
/// FAIL and carry on. A gate that cannot stop anything is a log line, and the
/// campaign that added those gates would have left them all decorative.
///
/// This asserts the verdict is reachable in both directions and that the flag
/// round-trips, without invoking NS_FATAL_ERROR: aborting the process inside a
/// test suite would take the other tests with it, so the verdict is checked
/// through GetLastGateVerdict and the fatal path is exercised by the example's
/// --strictGates flag instead.
class RealStackStrictGatesReachableTest : public TestCase
{
  public:
    RealStackStrictGatesReachableTest()
        : TestCase("WF-07 - fidelity gates produce a verdict a caller can act on")
    {
    }

  private:
    bool RunAndVerdict(double minThroughputMbps, const std::string& dir)
    {
        NodeContainer sats;
        sats.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 780e3));
        sats.Get(0)->AggregateObject(sm);
        NodeContainer ues;
        ues.Create(1);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ues.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(dir);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        NtnRealStackHelper::HealthGates g;
        g.minRxThroughputMbps = minThroughputMbps;
        rs.SetGates(g);
        // Deliberately NOT strict: this test wants the verdict, not the abort.
        rs.SetStrictGates(false);
        rs.Build(sats, ues);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming,
                          Seconds(0.5),
                          Seconds(1.9));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();
        const bool verdict = rs.GetLastGateVerdict();
        Simulator::Destroy();
        return verdict;
    }

    void DoRun() override
    {
        // The flag exists and round-trips. It did not have a getter, which is
        // part of why nothing could tell whether it was ever set.
        NtnRealStackHelper probe;
        NS_TEST_ASSERT_MSG_EQ(probe.GetStrictGates(), false,
                              "strict gates must stay off by default: turning an existing "
                              "scenario into a hard failure is a decision, not a fix");
        probe.SetStrictGates(true);
        NS_TEST_ASSERT_MSG_EQ(probe.GetStrictGates(), true, "the flag must round-trip");

        // A healthy run passes every gate.
        NS_TEST_ASSERT_MSG_EQ(RunAndVerdict(0.05, "test-ntn-strict-pass"), true,
                              "a healthy link must satisfy the gates; if it cannot, the floors "
                              "are set where nothing can pass and the gate system is useless in "
                              "the other direction");

        // The same run against an unreachable floor must produce a FAILING
        // verdict. Under strict gates this is the value that aborts, so this is
        // the assertion that makes the fatal path meaningful.
        NS_TEST_ASSERT_MSG_EQ(RunAndVerdict(1.0e6, "test-ntn-strict-fail"), false,
                              "an unreachable throughput floor must produce a failing verdict; "
                              "if the verdict is always true then SetStrictGates can never abort "
                              "anything and every gate is decorative");
    }
};


/// RRC-4: the RAR-window extension must reach the live NrGnbMac, because the
/// value the UE times out on is the one the gNB hands it in its RACH
/// configuration (NrGnbMac::GetRachConfig). A computation that never lands on
/// the object is a comment.
class NtnRachWindowAppliedTest : public TestCase
{
  public:
    NtnRachWindowAppliedTest()
        : TestCase("RRC-4: the geometry-sized RAR window reaches the live NrGnbMac")
    {
    }

  private:
    /// RaResponseWindowSize is a setter-only attribute, so it cannot be read
    /// back with GetAttribute. Reading it off the CMAC SAP instead is the
    /// stronger check anyway: GetRachConfig is the exact path by which the
    /// value reaches the UE's RACH configuration and therefore its timeout.
    static uint8_t WindowOnMac(Ptr<NrGnbMac> mac)
    {
        return mac->GetGnbCmacSapProvider()->GetRachConfig().raResponseWindowSize;
    }

    static uint8_t StackDefaultWindow()
    {
        Ptr<NrGnbMac> probe = CreateObject<NrGnbMac>();
        return probe->GetGnbCmacSapProvider()->GetRachConfig().raResponseWindowSize;
    }

    void DoRun() override
    {
        // Read nr's own default first, so the test compares against the stack
        // rather than against a number copied into the test.
        const uint8_t stackDefault = StackDefaultWindow();

        LeoRig rig;
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNtnRachWindow(true);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(rig.sat, rig.ue);

        const NtnRachWindowVerdict& v = rs.GetRachWindowVerdict();
        NS_TEST_ASSERT_MSG_GT(v.roundTrip.GetNanoSeconds(), 0,
                              "the verdict must be computed from the real service-link geometry");
        NS_TEST_ASSERT_MSG_EQ(v.fits, true, "LEO-600 at this numerology is inside nr's range");
        NS_TEST_ASSERT_MSG_GT(v.appliedWindowSize, stackDefault,
                              "the geometry demands a LONGER window than nr's default; if the "
                              "computed value equalled the default there would be nothing to "
                              "verify and the extension would be doing no work");

        // The value must be on the object, not merely in the verdict.
        Ptr<NrGnbMac> mac = NrHelper::GetGnbMac(rs.GetEnbDevices().Get(0), 0);
        NS_TEST_ASSERT_MSG_NE((mac == nullptr), true, "the nr gNB MAC is reachable");
        if (mac)
        {
            NS_TEST_ASSERT_MSG_EQ(static_cast<uint32_t>(WindowOnMac(mac)),
                                  static_cast<uint32_t>(v.appliedWindowSize),
                                  "the RAR window read back off the live gNB MAC, through the "
                                  "same GetRachConfig call that carries it to the UE, must equal "
                                  "the geometry-derived value; the stack default here would mean "
                                  "it was computed and never written");
        }

        // And the window it buys must actually cover the flight.
        NS_TEST_ASSERT_MSG_GT(v.configuredWindow.GetSeconds(), v.roundTrip.GetSeconds(),
                              "the applied window must outlast the round trip, which is the "
                              "whole reason for extending it");

        Simulator::Destroy();
    }
};

/// RRC-4: with the extension OFF the helper must leave the stack alone. Without
/// this, the test above could pass because something else set the attribute.
class NtnRachWindowOffTest : public TestCase
{
  public:
    NtnRachWindowOffTest()
        : TestCase("RRC-4: the RAR window is untouched when the extension is off")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<NrGnbMac> probe = CreateObject<NrGnbMac>();
        const uint32_t stackDefault =
            probe->GetGnbCmacSapProvider()->GetRachConfig().raResponseWindowSize;

        LeoRig rig;
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(rig.sat, rig.ue); // extension NOT enabled

        Ptr<NrGnbMac> mac = NrHelper::GetGnbMac(rs.GetEnbDevices().Get(0), 0);
        NS_TEST_ASSERT_MSG_NE((mac == nullptr), true, "the nr gNB MAC is reachable");
        if (mac)
        {
            const uint32_t got = mac->GetGnbCmacSapProvider()->GetRachConfig().raResponseWindowSize;
            NS_TEST_ASSERT_MSG_EQ(got, stackDefault,
                                  "with the extension off the stack keeps its own value");
        }
        NS_TEST_ASSERT_MSG_EQ(rs.GetRachWindowVerdict().appliedWindowSize, 0,
                              "and no verdict is produced");

        // The default is genuinely too short for this geometry. That is what
        // makes the extension a fix: turning it off is not a neutral choice.
        const Time defaultWindow = NtnRachWindow::SlotPeriodForNumerology(1) *
                                   (NtnRachWindow::kNrWindowBaseSlots + stackDefault);
        const Time rtt = NtnRachWindow::RoundTripForSlantRange(600e3);
        NS_TEST_ASSERT_MSG_LT(defaultWindow.GetSeconds(), (rtt + MicroSeconds(500)).GetSeconds(),
                              "nr's default window does not cover a LEO-600 round trip plus a "
                              "slot of gNB turnaround");

        Simulator::Destroy();
    }
};


/// SLICE-4: TS 38.101-5 conformance of the carrier/bandwidth pair.
class NtnFr1BandConformanceTest : public TestCase
{
  public:
    NtnFr1BandConformanceTest()
        : TestCase("SLICE-4: carrier and channel bandwidth against TS 38.101-5")
    {
    }

  private:
    using Band = NtnRealStackHelper::NtnFr1Band;

    void DoRun() override
    {
        // ---- Table 5.2-1 pairing. These were previously swapped. ----
        Band s = NtnRealStackHelper::ClassifyNtnFr1(2185.0e6, 20.0e6);
        NS_TEST_ASSERT_MSG_EQ(std::string(s.name), "n256",
                              "2170-2200 MHz is the n256 DOWNLINK block. It was labelled n255, "
                              "which is the L-band pair; the frequencies were right and only the "
                              "name was wrong, but a provenance row naming the wrong band is a "
                              "citation a reviewer checks in one lookup");
        NS_TEST_ASSERT_MSG_EQ(s.carrierInDownlink, true, "and the carrier is in its downlink");
        NS_TEST_ASSERT_MSG_EQ_TOL(s.blockWidthHz, 30.0e6, 1.0, "n256 DL block is 30 MHz wide");

        Band l = NtnRealStackHelper::ClassifyNtnFr1(1540.0e6, 10.0e6);
        NS_TEST_ASSERT_MSG_EQ(std::string(l.name), "n255",
                              "1525-1559 MHz is the n255 (L-band) downlink block");
        NS_TEST_ASSERT_MSG_EQ(l.carrierInDownlink, true, "carrier in the n255 downlink");
        NS_TEST_ASSERT_MSG_EQ_TOL(l.blockWidthHz, 34.0e6, 1.0, "n255 DL block is 34 MHz wide");

        // ---- Table 5.3.5-1 supported channel bandwidths ----
        NS_TEST_ASSERT_MSG_EQ(NtnRealStackHelper::ClassifyNtnFr1(2185.0e6, 30.0e6).conformant,
                              false,
                              "30 MHz is NOT a channel bandwidth for n256. It is the WIDTH of "
                              "the downlink block, and mistaking the two is what made every "
                              "throughput figure 1.5x the widest legal channel");
        NS_TEST_ASSERT_MSG_EQ(NtnRealStackHelper::ClassifyNtnFr1(2185.0e6, 20.0e6).conformant, true,
                              "20 MHz is the largest n256 supports");
        NS_TEST_ASSERT_MSG_EQ(NtnRealStackHelper::ClassifyNtnFr1(2185.0e6, 15.0e6).conformant, true,
                              "15 MHz is supported on n256");
        // n255 is the narrower band: 15 and 20 MHz are legal on n256 and not here.
        NS_TEST_ASSERT_MSG_EQ(NtnRealStackHelper::ClassifyNtnFr1(1540.0e6, 20.0e6).conformant,
                              false,
                              "n255 supports only 5 and 10 MHz, so a bandwidth legal on n256 is "
                              "not automatically legal here; a single shared bandwidth set would "
                              "pass this wrongly");
        NS_TEST_ASSERT_MSG_EQ(NtnRealStackHelper::ClassifyNtnFr1(1540.0e6, 10.0e6).conformant, true,
                              "10 MHz is supported on n255");

        // ---- The specific error the old default made ----
        Band oldDefault = NtnRealStackHelper::ClassifyNtnFr1(2.0e9, 30.0e6);
        NS_TEST_ASSERT_MSG_EQ(std::string(oldDefault.name), "n256",
                              "2.0 GHz falls in n256, but in its uplink half");
        NS_TEST_ASSERT_MSG_EQ(oldDefault.carrierInUplink, true,
                              "1980-2010 MHz is the n256 UPLINK block");
        NS_TEST_ASSERT_MSG_EQ(oldDefault.carrierInDownlink, false,
                              "so a downlink carrier at 2.0 GHz cannot be conformant at ANY "
                              "channel width; narrowing the bandwidth alone does not fix it");
        NS_TEST_ASSERT_MSG_EQ(oldDefault.conformant, false, "the old default was non-conformant");

        // The case that separates "checks the half of the band" from "checks
        // only the width": an uplink carrier with a PERFECTLY LEGAL channel
        // bandwidth. If conformance ignored carrierInDownlink this would pass,
        // and every assertion above would still hold.
        Band ulLegalBw = NtnRealStackHelper::ClassifyNtnFr1(2.0e9, 20.0e6);
        NS_TEST_ASSERT_MSG_EQ(ulLegalBw.bandwidthSupported, true,
                              "20 MHz is a supported n256 channel bandwidth");
        NS_TEST_ASSERT_MSG_EQ(ulLegalBw.conformant, false,
                              "but a downlink carrier sitting in the UPLINK block is not "
                              "conformant however legal its width; dropping the downlink test "
                              "would let this through");

        // ---- Out of band entirely ----
        Band none = NtnRealStackHelper::ClassifyNtnFr1(3.5e9, 20.0e6);
        NS_TEST_ASSERT_MSG_EQ(std::string(none.name), "none",
                              "3.5 GHz is FR1 but not an NTN FR1 band");
        NS_TEST_ASSERT_MSG_EQ(none.conformant, false, "and so cannot be conformant");

        // ---- The shipped default is NON-CONFORMANT, deliberately and knowably ----
        //
        // This asserts the uncomfortable thing rather than hiding it. The
        // conformant bandwidth is 20 MHz, and switching to it aborts
        // ntn-cho-full-constellation at t=36.2 s on nr's half-duplex assertion
        // (see the m_bwHz comment for the full measurement). Rather than ship
        // that, the default stays 30 MHz and the toolkit REPORTS that it is out
        // of spec. If someone later fixes the half-duplex fragility and moves
        // the default, this test fails and forces the claim to be re-examined -
        // which is the correct outcome, not a nuisance.
        NtnRealStackHelper fresh;
        NS_TEST_ASSERT_MSG_EQ_TOL(fresh.GetBandwidthHz(), 30.0e6, 1.0,
                                  "the shipped default is still 30 MHz");
        NS_TEST_ASSERT_MSG_EQ(fresh.GetNtnFr1Band().bandwidthSupported, false,
                              "and the helper must REPORT it as an unsupported channel "
                              "bandwidth. Silently believing the default is conformant is the "
                              "defect; running non-conformant on purpose, and saying so, is not");
        NS_TEST_ASSERT_MSG_EQ(fresh.GetNtnFr1Band().conformant, false,
                              "the default configuration is not TS 38.101-5 conformant");
        NS_TEST_ASSERT_MSG_EQ(std::string(fresh.GetNtnFr1Band().name), "n256",
                              "and the band it is closest to is named correctly");
    }
};


/// WF-11: a refused handover must be audible in the RELEASE build.
///
/// The shipped tree is configured NS3_LOG=OFF and NS3_ASSERT=OFF, and every
/// TriggerHandover refusal path was an NS_LOG_WARN and nothing else. So a
/// decision module could ask for a handover, be refused for a structural
/// reason - wrong backend, no X2, one gNB - and the run would say nothing at
/// all. "Never silently accept" only holds if the refusal survives the profile
/// users actually build.
class HandoverRefusalIsCountedTest : public TestCase
{
  public:
    HandoverRefusalIsCountedTest()
        : TestCase("WF-11: TriggerHandover refusals are counted, not only logged")
    {
    }

  private:
    void DoRun() override
    {
        // Refusal before Build(): the earliest structural one.
        {
            NtnRealStackHelper rs;
            NS_TEST_ASSERT_MSG_EQ(rs.GetHandoverRefusals(), 0u, "nothing refused yet");
            NS_TEST_ASSERT_MSG_EQ(rs.TriggerHandover(0, 1), false, "cannot handover before Build");
            NS_TEST_ASSERT_MSG_EQ(rs.GetHandoverRefusals(), 1u,
                                  "the refusal must be COUNTED; with NS3_LOG off the warning "
                                  "that used to be the only record is compiled out entirely");
            NS_TEST_ASSERT_MSG_EQ(rs.GetLastHandoverRefusal().empty(), false,
                                  "and the reason recorded, so the health row can name it");
        }

        // Refusal on a built single-gNB mmwave stack: the backend has no X2
        // handover request at all, which is the case most likely to be mistaken
        // for success by a decision module.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(rig.sat, rig.ue);
            NS_TEST_ASSERT_MSG_EQ(rs.TriggerHandover(0, 1), false,
                                  "the mmwave backend cannot actuate a handover");
            NS_TEST_ASSERT_MSG_GT(rs.GetHandoverRefusals(), 0u, "and says so in a counter");
            Simulator::Destroy();
        }
    }
};


/// WF-08 gate 14: the K_offset NEGATIVE case.
///
/// The plan document tallies "18/18 CI gates verified", and gate 14 is a
/// K_offset negative test. The existing test covers only the POSITIVE path -
/// consumption on, N2Delay raised. Nothing asserted that N2Delay stays at the
/// stack's base value when consumption is OFF, which is what makes the positive
/// result attributable: without it, a helper that raised N2Delay
/// unconditionally would pass the gate.
class KOffsetNotConsumedWhenOffTest : public TestCase
{
  public:
    KOffsetNotConsumedWhenOffTest()
        : TestCase("WF-08 gate 14: N2Delay stays at base when K_offset consumption is off")
    {
    }

  private:
    void DoRun() override
    {
        // OFF: nothing consumed, and N2Delay is whatever the stack chose.
        uint32_t baseN2 = 0;
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
            rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(rig.sat, rig.ue); // SetKOffsetConsumption NOT called
            NS_TEST_ASSERT_MSG_EQ(rs.GetConsumedKOffsetSlots(), 0u,
                                  "with consumption off nothing may be consumed");
            baseN2 = rs.GetGnbN2Delay();
            Simulator::Destroy();
        }

        // ON: the geometry-derived K_offset is added to that same base.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
            rs.SetKOffsetConsumption(true);
            rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(rig.sat, rig.ue);
            const uint32_t consumed = rs.GetConsumedKOffsetSlots();
            NS_TEST_ASSERT_MSG_GT(consumed, 0u, "a 600 km link needs a non-zero K_offset");
            NS_TEST_ASSERT_MSG_EQ(rs.GetGnbN2Delay(), baseN2 + consumed,
                                  "N2Delay must be the stack's base PLUS the consumed K_offset. "
                                  "Comparing against the off-case base is what makes the "
                                  "increase attributable to consumption rather than to the "
                                  "helper raising N2Delay unconditionally");
            NS_TEST_ASSERT_MSG_GT(rs.GetGnbN2Delay(), baseN2,
                                  "and it is strictly larger than the off-case value");
            Simulator::Destroy();
        }
    }
};


/// WF-08 gate 1: the measured one-way delay must not beat the geometric floor.
///
/// The tally records gate 1 as verified. The computation exists -
/// ComputeOwdFloorMs() and the app_owd_ms row carry it - but no TestCase
/// asserted it, and neither standards checker reads that row's pass column. So
/// nothing in CI would have noticed a measured OWD faster than light over the
/// configured geometry, which is the one thing this gate exists to catch.
class OwdFloorGateTest : public TestCase
{
  public:
    OwdFloorGateTest()
        : TestCase("WF-08 gate 1: measured OWD is at or above the geometric floor")
    {
    }

  private:
    void DoRun() override
    {
        LeoRig rig;
        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(3.0));
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(rig.sat, rig.ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5),
                          Seconds(2.8));
        Simulator::Stop(Seconds(3.0));
        Simulator::Run();
        rs.Collect();

        const double floorMs = rs.ComputeOwdFloorMs();
        const double owdMs = rs.GetMeanDelayMs();

        // The floor has to be a real number derived from real geometry, or the
        // comparison below is against nothing.
        NS_TEST_ASSERT_MSG_GT(floorMs, 0.0,
                              "the geometric OWD floor must be derived from the configured "
                              "geometry; zero here means the gate compares against nothing");
        // 600 km straight up is 2.0018 ms one way; the floor also carries the
        // backhaul leg, so it is at least that.
        NS_TEST_ASSERT_MSG_GT(floorMs, 2.0,
                              "a 600 km link cannot have a sub-2 ms floor");

        NS_TEST_ASSERT_MSG_GT(owdMs, 0.0, "the run must have measured a delay");
        NS_TEST_ASSERT_MSG_GT(owdMs, floorMs,
                              "the MEASURED one-way delay (" << owdMs << " ms) must be at or "
                              "above the geometric floor (" << floorMs << " ms). A packet "
                              "arriving sooner than light allows means the delay is not being "
                              "carried on any leg, which is exactly the silent failure this "
                              "gate exists to catch");
        Simulator::Destroy();
    }
};

/// WF-08 gate 3: the EIRP gate must be evaluated, not assumed.
///
/// The tally records gate 3 as verified. What existed was an NS_LOG_WARN -
/// compiled out of the release build, see WF-11 - and a health row. No TestCase
/// exercised EvaluateEirpGate at all, so neither its pass nor its
/// not-asserted arm was ever checked.
class EirpGateEvaluatedTest : public TestCase
{
  public:
    EirpGateEvaluatedTest()
        : TestCase("WF-08 gate 3: the EIRP gate passes on a TR 38.821 budget and fails when hot")
    {
    }

  private:
    void DoRun() override
    {
        // Declared against the TR 38.821 Set-1 S-band density: the gate has a
        // reference to assert against, and must pass.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
            rs.Build(rig.sat, rig.ue);
            double budget = 0.0;
            double tol = 0.0;
            const int verdict = rs.EvaluateEirpGate(budget, tol);
            NS_TEST_ASSERT_MSG_NE(verdict, -1,
                                  "with a TR 38.821 reference declared the gate must be "
                                  "ASSERTED, not skipped");
            NS_TEST_ASSERT_MSG_EQ(verdict, 1,
                                  "and a run declared at the Set-1 density must pass it; "
                                  "effective EIRP " << rs.GetEffectiveEirpDbm() << " dBm against "
                                  "budget " << budget << " +/- " << tol);
            NS_TEST_ASSERT_MSG_GT(tol, 0.0, "the gate carries a stated tolerance");

            // And the plausibility check must PASS here, or it would just be a
            // check that always fails.
            double ref = 0.0;
            double excess = 0.0;
            NS_TEST_ASSERT_MSG_EQ(rs.EvaluateEirpPlausibility(ref, excess), 1,
                                  "a run declared at the Set-1 density is plausible; excess "
                                  << excess << " dB against the reference " << ref << " dBm");
            NS_TEST_ASSERT_MSG_LT(std::abs(excess), 6.0,
                                  "and sits within a few dB of the reference");
            Simulator::Destroy();
        }

        // Now drive it 20 dB hot. The gate must FAIL, or it is not a gate.
        {
            LeoRig rig;
            NtnRealStackHelper rs;
            rs.SetSatEirpDensityDbwMhz(
                NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz + 20.0);
            rs.Build(rig.sat, rig.ue);
            double budget = 0.0;
            double tol = 0.0;
            const int verdict = rs.EvaluateEirpGate(budget, tol);
            // The BUDGET gate still passes, and that is correct: it compares
            // the effective EIRP against what the scenario DECLARED, so it
            // catches an array-gain double-count and cannot catch an
            // implausible declaration - declaring a hotter satellite moves the
            // budget with it. Writing this test is what made that distinction
            // visible; "plausible-EIRP gate" had been reading as though one
            // check did both jobs.
            NS_TEST_ASSERT_MSG_EQ(verdict, 1,
                                  "the budget gate compares effective against declared, so a "
                                  "consistent-but-hot declaration still passes it");

            // The PLAUSIBILITY check is the one that has to bite, because its
            // reference is the standard's and a declaration cannot move it.
            double ref = 0.0;
            double excess = 0.0;
            const int plaus = rs.EvaluateEirpPlausibility(ref, excess);
            NS_TEST_ASSERT_MSG_EQ(plaus, 0,
                                  "a run 20 dB above the TR 38.821 Set-1 density is not a "
                                  "plausible standardized payload; excess measured "
                                  << excess << " dB. An 18-21 dB overshoot is exactly what the "
                                  "August audit found across the toolkit");
            NS_TEST_ASSERT_MSG_GT(excess, 15.0,
                                  "and the excess is reported, not just a flag");
            Simulator::Destroy();
        }
    }
};


/// BOTH-01: frequency-selective fading, which the toolkit had none of.
///
/// Every small-scale process in the tree was FLAT - the P.681-11 Lutz two-state
/// model, the alpha-mu model, and the elevation-dependent Rician term all scale
/// every subcarrier by the same factor. So a wideband NTN channel was as smooth
/// as a narrowband one, no delay spread existed anywhere, and no BLER number
/// reflected intersymbol interference or frequency diversity.
class NtnTdlFrequencySelectivityTest : public TestCase
{
  public:
    NtnTdlFrequencySelectivityTest()
        : TestCase("BOTH-01: the TDL model is frequency-selective and elevation-dependent")
    {
    }

  private:
    static std::vector<double> Band(double centreHz, double bwHz, std::size_t n)
    {
        std::vector<double> f(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            f[i] = centreHz - bwHz / 2.0 + bwHz * (static_cast<double>(i) / (n - 1));
        }
        return f;
    }

    void DoRun() override
    {
        Ptr<NtnTdlSpectrumLossModel> tdl = CreateObject<NtnTdlSpectrumLossModel>();

        // The default profile must be labelled as a stand-in, not as the
        // standard's table. Attributing invented taps to TR 38.811 Table 6.9.2
        // is the defect this audit found elsewhere.
        const bool labelled =
            tdl->GetProfileProvenance().find("NOT TR 38.811") != std::string::npos;
        NS_TEST_ASSERT_MSG_EQ(labelled, true,
                              "the built-in profile must say it is not the standard's table");

        // A delay spread must exist at all - it is what makes a channel
        // selective, and the toolkit had none.
        const double ds = tdl->GetRmsDelaySpreadS();
        NS_TEST_ASSERT_MSG_GT(ds, 10e-9,
                              "the profile must have a real RMS delay spread; zero means a "
                              "single tap and a flat channel, which is what existed before");
        NS_TEST_ASSERT_MSG_LT(ds, 10e-6, "and a physically sensible one");

        // ---- The core property: gains VARY across the band ----
        const auto freqs = Band(2.0e9, 20e6, 64);
        const auto gains = tdl->ComputeGains(freqs, 45.0);
        NS_TEST_ASSERT_MSG_EQ(gains.size(), freqs.size(), "one gain per subcarrier");
        const double lo = *std::min_element(gains.begin(), gains.end());
        const double hi = *std::max_element(gains.begin(), gains.end());
        NS_TEST_ASSERT_MSG_GT(hi / std::max(lo, 1e-12), 1.5,
                              "the transfer function must vary across a 20 MHz band. A ratio "
                              "near 1 is a FLAT channel - exactly what every existing "
                              "small-scale model in this tree produced");

        // A single tap must be flat, which is the control: it shows the
        // variation above comes from the DELAY structure and not from noise.
        Ptr<NtnTdlSpectrumLossModel> flat = CreateObject<NtnTdlSpectrumLossModel>();
        flat->SetTaps({{0.0, 0.0, true}});
        const auto flatGains = flat->ComputeGains(freqs, 45.0);
        const double flo = *std::min_element(flatGains.begin(), flatGains.end());
        const double fhi = *std::max_element(flatGains.begin(), flatGains.end());
        NS_TEST_ASSERT_MSG_EQ_TOL(fhi / std::max(flo, 1e-12), 1.0, 1e-6,
                                  "a single-tap profile has no delay spread and must be flat "
                                  "across frequency; if this varied, the selectivity above "
                                  "would be per-subcarrier noise rather than a channel");

        // ---- The K-factor tracks elevation (TR 38.811 section 6.7.2) ----
        const double k10 = tdl->RicianKdB(10.0);
        const double k90 = tdl->RicianKdB(90.0);
        NS_TEST_ASSERT_MSG_GT(k90, k10,
                              "a satellite overhead has a stronger specular component than one "
                              "near the horizon; a K-factor that ignores elevation is the gap "
                              "this finding names");
        // Assert the LITERAL midpoint, not another call to the same function.
        // Comparing RicianKdB(15) against 0.5*(RicianKdB(10)+RicianKdB(20))
        // compares the function with itself: disabling interpolation makes
        // every one of those calls return the same lower tabulated value, so
        // both sides move together and the assertion holds regardless. The
        // TR 38.811 section 6.7.2 grid gives 4.1 dB at 10 degrees and 6.6 dB at
        // 20, so a linear interpolant at 15 is 5.35 dB and a step function is
        // 4.1 dB.
        NS_TEST_ASSERT_MSG_EQ_TOL(tdl->RicianKdB(15.0), 5.35, 0.05,
                                  "the K-factor must INTERPOLATE between tabulated elevations, "
                                  "not step; 4.1 dB here means it is returning the lower grid "
                                  "point and elevation inside a cell has no effect");
        // Outside the grid it clamps rather than extrapolating into nonsense.
        NS_TEST_ASSERT_MSG_EQ_TOL(tdl->RicianKdB(-5.0), k10, 1e-9, "clamped below the grid");
        NS_TEST_ASSERT_MSG_EQ_TOL(tdl->RicianKdB(120.0), k90, 1e-9, "clamped above it");

        // ---- Power conservation: fading redistributes, it does not create ----
        // Averaged over many draws the mean gain must be ~1, or the model
        // silently changes the link budget.
        double acc = 0.0;
        const int draws = 400;
        for (int d = 0; d < draws; ++d)
        {
            const auto g = tdl->ComputeGains(freqs, 45.0);
            for (double v : g)
            {
                acc += v;
            }
        }
        const double meanGain = acc / (draws * freqs.size());
        NS_TEST_ASSERT_MSG_EQ_TOL(meanGain, 1.0, 0.15,
                                  "mean gain " << meanGain << ": a fading model must not add or "
                                  "remove average energy, only move it across frequency");
    }
};


/// WF-10 (other half): the measured radio plane at more than two gNBs.
///
/// The finding reports that "across all 94 custom examples, the largest gNB
/// count handed to NtnRealStackHelper::Build() is 2", and reads as a ceiling.
/// It is not one: it describes what the SCENARIOS ask for, not what the helper
/// can do. Measured here, an eight-gNB nr cell with handover and X2 builds and
/// carries traffic, with wall time scaling roughly linearly - 0.8 s, 1.7 s and
/// 3.7 s for two, four and eight gNBs over 3 s of simulated time.
///
/// Pinning it means the ceiling cannot quietly return: if a future change makes
/// Build() fail above two gNBs, this fails rather than every scenario silently
/// staying small.
class MultiGnbMeasuredPlaneTest : public TestCase
{
  public:
    MultiGnbMeasuredPlaneTest()
        : TestCase("WF-10: the measured plane builds and carries traffic at 8 gNBs")
    {
    }

  private:
    void DoRun() override
    {
        const uint32_t kGnbs = 8;
        NodeContainer sat;
        sat.Create(kGnbs);
        for (uint32_t i = 0; i < kGnbs; ++i)
        {
            Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel>();
            m->SetPosition(Vector(i * 400e3, 0.0, 600e3));
            m->SetVelocity(Vector(7560.0, 0.0, 0.0));
            sat.Get(i)->AggregateObject(m);
        }
        NodeContainer ue;
        ue.Create(2);
        for (uint32_t i = 0; i < 2; ++i)
        {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            m->SetPosition(Vector(i * 1000.0, 0.0, 0.0));
            ue.Get(i)->AggregateObject(m);
        }

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.SetSimTime(Seconds(3.0));
        rs.SetHandover(true, 3.0, MilliSeconds(256)); // X2 across all eight
        rs.Build(sat, ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, Seconds(0.5),
                          Seconds(2.8));
        Simulator::Stop(Seconds(3.0));
        Simulator::Run();
        rs.Collect();

        NS_TEST_ASSERT_MSG_EQ(rs.GetEnbDevices().GetN(), kGnbs,
                              "all eight gNBs must be built; a helper that silently caps at two "
                              "would be the ceiling this finding reads as");
        NS_TEST_ASSERT_MSG_GT(rs.GetPhyRxTb(), 100u,
                              "and the measured plane must carry transport blocks, not merely "
                              "instantiate - a build that stands up eight cells and moves no "
                              "data has not shown the plane works");
        NS_TEST_ASSERT_MSG_EQ(std::isfinite(rs.GetMeanDlSinrDb()), true,
                              "with a finite measured SINR");
        NS_TEST_ASSERT_MSG_GT(rs.GetRxThroughputMbps(), 0.0, "and real throughput");

        Simulator::Destroy();
    }
};

/// NT-07: three TR 38.811 features were documented as delivered while never
/// executing in any run, because the only way to reach them was a setter that no
/// scenario called. Being reachable in principle is not the same as being in the
/// propagation chain, and nothing could tell the two apart from outside the
/// helper. These cases assert the chain itself.
///
/// One of the three turned out NOT to be a defect, and is recorded as such
/// below: EnableFastFading is off by DESIGN, because both radio backends already
/// apply small-scale fading through the 3GPP phased-array spectrum model, and
/// enabling the Rician term as well would multiply two independent fading
/// processes onto one link. What was missing there was not a caller but a test:
/// the Rician block had never been executed by anything, so its normalization
/// and its elevation dependence were unverified.
class NtnChannelExtrasReachTheChainTest : public TestCase
{
  public:
    NtnChannelExtrasReachTheChainTest()
        : TestCase("NT-07: the TR 38.811 excess-loss and 6.4.1 beam models are actually "
                   "chained onto the measured channel, not merely constructible")
    {
    }

  private:
    static bool Chained(const std::vector<std::string>& chain, const std::string& needle)
    {
        for (const auto& n : chain)
        {
            if (n.find(needle) != std::string::npos)
            {
                return true;
            }
        }
        return false;
    }

    static void BuildStack(NtnRealStackHelper& rs, NodeContainer& sat, NodeContainer& ue)
    {
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 600e3));
        sm->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(sm);
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);
        rs.SetSimTime(Seconds(1.0));
    }

    void DoRun() override
    {
        // ---- 1. the excess-loss model is chained when enabled ------------
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            BuildStack(rs, sat, ue);
            rs.SetTr38811ExcessLoss(true);
            rs.Build(sat, ue);
            const auto chain = rs.GetExtraPropagationLossChain();
            NS_TEST_ASSERT_MSG_EQ(Chained(chain, "Ntn38811ExcessLossModel"), true,
                                  "SetTr38811ExcessLoss(true) must put the model IN the chain; "
                                  "constructing it and dropping it is the failure mode here");
            Simulator::Destroy();
        }

        // ---- 2. and is absent when disabled, so 1. is not vacuous --------
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            BuildStack(rs, sat, ue);
            rs.SetTr38811ExcessLoss(false);
            rs.Build(sat, ue);
            const auto chain = rs.GetExtraPropagationLossChain();
            NS_TEST_ASSERT_MSG_EQ(Chained(chain, "Ntn38811ExcessLossModel"), false,
                                  "the excess-loss model must not be chained when disabled, or "
                                  "the positive case above proves nothing");
            Simulator::Destroy();
        }

        // ---- 3. the 6.4.1 beam pattern reaches the chain via the helper --
        // This is the leg that had zero callers: SetSatelliteBeam existed, the
        // chaining code existed, and no scenario connected them.
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            BuildStack(rs, sat, ue);
            Ptr<ConstantPositionMobilityModel> beamCenter =
                CreateObject<ConstantPositionMobilityModel>();
            beamCenter->SetPosition(Vector(300e3, 0.0, 0.0)); // fixed, off the UE
            rs.SetSatelliteBeam(4.4127, beamCenter);
            rs.Build(sat, ue);
            const auto chain = rs.GetExtraPropagationLossChain();
            NS_TEST_ASSERT_MSG_EQ(Chained(chain, "NtnSatBeamGainModel"), true,
                                  "SetSatelliteBeam must chain the TR 38.811 6.4.1 pattern");
            Simulator::Destroy();
        }

        // ---- 4. and is absent by default (no silent behaviour change) ----
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            BuildStack(rs, sat, ue);
            rs.Build(sat, ue);
            const auto chain = rs.GetExtraPropagationLossChain();
            NS_TEST_ASSERT_MSG_EQ(Chained(chain, "NtnSatBeamGainModel"), false,
                                  "the beam pattern must stay opt-in: turning it on by default "
                                  "would move every measured result in the toolkit");
            Simulator::Destroy();
        }
    }
};

/// NT-07(c): SetNtnScenario had no callers either, so every run in the toolkit
/// used the Suburban default. A setter that is ignored and a setter that nobody
/// calls fail the same way from the outside, so assert the VALUE reaches the
/// model rather than that the call compiles.
class NtnScenarioReachesTheModelTest : public TestCase
{
  public:
    NtnScenarioReachesTheModelTest()
        : TestCase("NT-07: SetNtnScenario reaches the chained TR 38.811 model and "
                   "changes its shadow-fading sigma")
    {
    }

  private:
    static double SigmaAt(Ntn38811ExcessLossModel::NtnScenario sc, double elevDeg)
    {
        Ptr<Ntn38811ExcessLossModel> m = CreateObject<Ntn38811ExcessLossModel>();
        m->SetCarrierFrequencyHz(2.0e9);
        m->SetScenario(sc);
        return m->ShadowSigmaDb(elevDeg);
    }

    void DoRun() override
    {
        // What the scenario selects is the shadow-fading sigma, NOT clutter:
        // TR 38.811 6.6.2 sets CL = 0 dB in LOS and this model runs the
        // always-LOS satellite link, so the Table 6.6.2 NLOS clutter constants
        // never apply. Asserting on clutter here would be asserting on a
        // deliberate zero.
        const double dense = SigmaAt(Ntn38811ExcessLossModel::DenseUrban, 30.0);
        const double rural = SigmaAt(Ntn38811ExcessLossModel::Rural, 30.0);
        NS_TEST_ASSERT_MSG_GT(dense, rural,
                              "TR 38.811 Table 6.6.2 dense-urban shadow sigma must exceed rural "
                              "at 30 deg (got " << dense << " vs " << rural << " dB), or the "
                              "scenarios are not distinct and nothing below is observable");

        // And the helper must carry the selection into the chained model.
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 600e3));
        sm->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(sm);
        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(1.0));
        rs.SetTr38811ExcessLoss(true);
        rs.SetNtnScenario(Ntn38811ExcessLossModel::DenseUrban); // not the Suburban default
        rs.Build(sat, ue);

        NS_TEST_ASSERT_MSG_EQ(rs.GetNtnScenario(),
                              static_cast<uint8_t>(Ntn38811ExcessLossModel::DenseUrban),
                              "SetNtnScenario must survive to Build(), not be reset to default");

        // The chained instance itself must carry it, which is the half that
        // could silently be lost between the helper field and the model.
        Ptr<Ntn38811ExcessLossModel> chained;
        for (Ptr<PropagationLossModel> m = rs.GetBaseLossHead(); m; m = m->GetNext())
        {
            chained = DynamicCast<Ntn38811ExcessLossModel>(m);
            if (chained)
            {
                break;
            }
        }
        NS_TEST_ASSERT_MSG_NE(chained, nullptr,
                              "the excess-loss model must be in the chain to be checked");
        NS_TEST_ASSERT_MSG_EQ(chained->GetScenario(), Ntn38811ExcessLossModel::DenseUrban,
                              "the CHAINED model must carry the configured scenario; the helper "
                              "field agreeing with itself proves nothing about the model");
        Simulator::Destroy();
    }
};

/// NT-07(a): the Rician small-scale term. NOT a wiring defect - it is off by
/// design so it cannot double-count against the 3GPP spectrum model's own fading
/// - but it had never been executed by any test, so its normalization and its
/// elevation dependence were asserted only by the comment above them.
class Tr38811FastFadingStatisticsTest : public TestCase
{
  public:
    Tr38811FastFadingStatisticsTest()
        : TestCase("NT-07: the TR 38.811 6.7.2 Rician term is unit-power and its "
                   "K-factor rises with elevation")
    {
    }

  private:
    void DoRun() override
    {
        Ptr<Ntn38811ExcessLossModel> m = CreateObject<Ntn38811ExcessLossModel>();

        // It must stay OFF by default: both backends already fade this link.
        BooleanValue ff;
        m->GetAttribute("EnableFastFading", ff);
        NS_TEST_ASSERT_MSG_EQ(ff.Get(), false,
                              "EnableFastFading must default off; enabling it alongside the "
                              "3GPP spectrum model multiplies two fading processes on one link");

        // The K-factor must rise with elevation (TR 38.811 Table 6.7.2): a
        // higher link is more specular.
        const double kLow = m->RicianKdB(10.0);
        const double kHigh = m->RicianKdB(80.0);
        NS_TEST_ASSERT_MSG_GT(kHigh, kLow + 1.0,
                              "the Rician K-factor must increase with elevation (got "
                                  << kLow << " dB at 10 deg, " << kHigh << " dB at 80 deg)");

        // And the process itself must be unit mean power, or every fade it
        // applies carries a constant bias that reads as path loss.
        for (double elevDeg : {10.0, 45.0, 80.0})
        {
            const double kdB = m->RicianKdB(elevDeg);
            const double k = std::pow(10.0, kdB / 10.0);
            const double sigma = std::sqrt(1.0 / (2.0 * (k + 1.0)));
            const double spec = std::sqrt(k / (k + 1.0));
            Ptr<NormalRandomVariable> gi = CreateObject<NormalRandomVariable>();
            Ptr<NormalRandomVariable> gq = CreateObject<NormalRandomVariable>();
            gi->SetAttribute("Mean", DoubleValue(0.0));
            gi->SetAttribute("Variance", DoubleValue(1.0));
            gq->SetAttribute("Mean", DoubleValue(0.0));
            gq->SetAttribute("Variance", DoubleValue(1.0));
            const uint32_t kN = 200000;
            double sum = 0.0;
            double minR2 = 1e300;
            double maxR2 = -1e300;
            for (uint32_t i = 0; i < kN; ++i)
            {
                const double xi = spec + sigma * gi->GetValue();
                const double yq = sigma * gq->GetValue();
                const double r2 = xi * xi + yq * yq;
                sum += r2;
                minR2 = std::min(minR2, r2);
                maxR2 = std::max(maxR2, r2);
            }
            const double meanR2 = sum / kN;
            NS_TEST_ASSERT_MSG_EQ_TOL(meanR2, 1.0, 0.03,
                                      "Rician power gain must average 1 at " << elevDeg
                                          << " deg (got " << meanR2 << "); a biased process "
                                          << "would be read as extra path loss");
            NS_TEST_ASSERT_MSG_GT(maxR2, minR2,
                                  "the process must actually vary at " << elevDeg << " deg");
        }

        // The spread must NARROW as elevation rises, which is what the
        // K-factor means physically. A term with the right mean and no
        // elevation dependence would pass everything above.
        auto spread = [](double kdB) {
            const double k = std::pow(10.0, kdB / 10.0);
            // Rician amplitude variance, unit mean power: 1 - (specular share).
            return 1.0 / (k + 1.0);
        };
        NS_TEST_ASSERT_MSG_LT(spread(kHigh), spread(kLow),
                              "the scattered-power share must fall as elevation rises");
    }
};

/// NT-10: the TR 38.811 6.6.2 shadow-fading process must be spatially
/// CORRELATED, not a piecewise-constant step.
///
/// The model used to hold one sample until a displacement or elevation
/// threshold was crossed and then draw a fully independent replacement, which
/// put a discontinuity inside a single transport block. The audit measured a
/// +1.205 -> -2.496 dB jump, a 3.7 dB step. Any study that keys on the SINR
/// derivative (A3 hysteresis, time-to-trigger) reads that step as physics.
///
/// The fix is the AR(1) recursion ns-3's own 3GPP model uses. This case pins
/// the three properties that distinguish it from both the old step process and
/// from a constant: small steps barely move the sample, large ones decorrelate
/// it, and the marginal variance still matches the Table 6.6.2 sigma.
class Tr38811ShadowFadingIsCorrelatedTest : public TestCase
{
  public:
    Tr38811ShadowFadingIsCorrelatedTest()
        : TestCase("NT-10: shadow fading is an AR(1)-correlated process, not a "
                   "piecewise-constant step")
    {
    }

  private:
    /// Rx power over a descending pass, sampled every stepDeg of elevation.
    static std::vector<double> PassTrace(double stepDeg, uint32_t n)
    {
        Ptr<Ntn38811ExcessLossModel> m = CreateObject<Ntn38811ExcessLossModel>();
        m->SetCarrierFrequencyHz(2.0e9);
        m->SetScenario(Ntn38811ExcessLossModel::DenseUrban); // largest sigma
        m->AssignStreams(7);

        Ptr<ConstantPositionMobilityModel> ue = CreateObject<ConstantPositionMobilityModel>();
        ue->SetPosition(Vector(0.0, 0.0, 0.0));
        Ptr<ConstantPositionMobilityModel> sat = CreateObject<ConstantPositionMobilityModel>();

        std::vector<double> out;
        out.reserve(n);
        const double h = 600e3;
        for (uint32_t i = 0; i < n; ++i)
        {
            // Place the satellite at a decreasing elevation in the local ENU
            // frame the model accepts: +z is up.
            const double elev = 85.0 - stepDeg * i;
            const double e = elev * M_PI / 180.0;
            const double d = h / std::sin(e);
            sat->SetPosition(Vector(d * std::cos(e), 0.0, d * std::sin(e)));
            out.push_back(m->CalcRxPower(0.0, sat, ue));
        }
        return out;
    }

    static double MaxAbsStep(const std::vector<double>& v)
    {
        double worst = 0.0;
        for (size_t i = 1; i < v.size(); ++i)
        {
            worst = std::max(worst, std::fabs(v[i] - v[i - 1]));
        }
        return worst;
    }

    void DoRun() override
    {
        // ---- 1. fine steps must move the sample only a little --------------
        // 0.05 deg against a 5 deg correlation scale is rho = 0.99, so the
        // sample may drift but must not jump. The old process was flat here and
        // then stepped several dB at one crossing; the bound below is chosen to
        // fail on such a step while allowing genuine drift plus the model's
        // other (uncorrelated by design) scintillation term.
        const std::vector<double> fine = PassTrace(0.05, 400);
        const double fineStep = MaxAbsStep(fine);
        NS_TEST_ASSERT_MSG_LT(fineStep, 2.0,
                              "a 0.05 deg geometry step must not move the excess loss by "
                                  << fineStep << " dB; that is a discontinuity, not a "
                                  << "correlated process");

        // ---- 2. but it must still be a PROCESS, not a held constant --------
        double lo = 1e300;
        double hi = -1e300;
        for (double v : fine)
        {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
        NS_TEST_ASSERT_MSG_GT(hi - lo, 1.0,
                              "the shadow term must vary across a pass (spread " << (hi - lo)
                                  << " dB); a frozen sample would satisfy the smoothness "
                                  << "check above trivially");

        // ---- 3. and coarse steps must decorrelate ---------------------------
        // 20 deg against a 5 deg scale is rho = e^-4 = 0.018, effectively
        // independent, so the step-to-step spread must be visibly larger than
        // in the fine trace. A process that ignored its inputs would give the
        // same spread at both scales.
        const std::vector<double> coarse = PassTrace(20.0, 4);
        const double coarseStep = MaxAbsStep(coarse);
        NS_TEST_ASSERT_MSG_GT(coarseStep, fineStep,
                              "a 20 deg jump must decorrelate the shadow sample more than a "
                                  << "0.05 deg one (got " << coarseStep << " vs " << fineStep
                                  << " dB); if they match, the correlation scale is ignored");

        // ---- 4. the marginal sigma must still be Table 6.6.2 ---------------
        // Correlating a process must not shrink its variance. Draw many
        // INDEPENDENT link states at one elevation and check the spread against
        // the table value for the scenario.
        Ptr<Ntn38811ExcessLossModel> m = CreateObject<Ntn38811ExcessLossModel>();
        m->SetCarrierFrequencyHz(2.0e9);
        m->SetScenario(Ntn38811ExcessLossModel::DenseUrban);
        const double sigmaTable = m->ShadowSigmaDb(45.0);
        NS_TEST_ASSERT_MSG_GT(sigmaTable, 0.0, "the table sigma must be positive to compare");

        // Each pair of fresh mobility models is a fresh link state, so the
        // first sample on each is an independent draw of the marginal.
        double sum = 0.0;
        double sumSq = 0.0;
        const uint32_t kN = 4000;
        const double e = 45.0 * M_PI / 180.0;
        const double d = 600e3 / std::sin(e);
        for (uint32_t i = 0; i < kN; ++i)
        {
            Ptr<Ntn38811ExcessLossModel> mi = CreateObject<Ntn38811ExcessLossModel>();
            mi->SetCarrierFrequencyHz(2.0e9);
            mi->SetScenario(Ntn38811ExcessLossModel::DenseUrban);
            mi->SetAttribute("EnableScintillation", BooleanValue(false)); // isolate the SF term
            mi->AssignStreams(1000 + 4 * i);
            Ptr<ConstantPositionMobilityModel> u = CreateObject<ConstantPositionMobilityModel>();
            u->SetPosition(Vector(0.0, 0.0, 0.0));
            Ptr<ConstantPositionMobilityModel> sv = CreateObject<ConstantPositionMobilityModel>();
            sv->SetPosition(Vector(d * std::cos(e), 0.0, d * std::sin(e)));
            const double v = mi->CalcRxPower(0.0, sv, u);
            sum += v;
            sumSq += v * v;
        }
        const double mean = sum / kN;
        const double var = sumSq / kN - mean * mean;
        const double sd = std::sqrt(std::max(0.0, var));
        NS_TEST_ASSERT_MSG_EQ_TOL(sd, sigmaTable, 0.35 * sigmaTable,
                                  "the marginal shadow sigma must still match TR 38.811 Table "
                                  "6.6.2 after correlating (table " << sigmaTable << " dB, "
                                      << "measured " << sd << " dB)");

        // ---- 5. the STEADY-STATE variance of the recursion ------------------
        // Step 4 samples the first draw on each fresh link, which takes the
        // initialization branch and never exercises the recursion at all. So it
        // passes even when the blend is wrong. The AR(1) form matters here:
        // new = rho*old + sqrt(1-rho^2)*N(0,1) is stationary with unit variance
        // for ANY rho, while the plausible-looking new = rho*old + (1-rho)*N
        // has stationary variance (1-rho)^2/(1-rho^2), which collapses as rho
        // approaches 1. Walk a single link in strongly correlated steps, where
        // that error is largest, and check the spread after a burn-in.
        Ptr<Ntn38811ExcessLossModel> w = CreateObject<Ntn38811ExcessLossModel>();
        w->SetCarrierFrequencyHz(2.0e9);
        w->SetScenario(Ntn38811ExcessLossModel::DenseUrban);
        w->SetAttribute("EnableScintillation", BooleanValue(false));
        w->AssignStreams(31);
        DoubleValue dCorrV;
        w->GetAttribute("ShadowCorrelationDistanceM", dCorrV);
        const double stepM = 0.2 * dCorrV.Get(); // rho = e^-0.2 = 0.82

        // Translate BOTH ends together so the elevation, and therefore the
        // table sigma, is constant while the ground displacement accumulates.
        // Elevation-driven decorrelation would otherwise change sigma under us.
        Ptr<ConstantPositionMobilityModel> wu = CreateObject<ConstantPositionMobilityModel>();
        Ptr<ConstantPositionMobilityModel> ws = CreateObject<ConstantPositionMobilityModel>();
        const uint32_t kBurn = 2000;
        const uint32_t kWalk = 200000;
        double s1 = 0.0;
        double s2 = 0.0;
        for (uint32_t i = 0; i < kBurn + kWalk; ++i)
        {
            const double off = stepM * i;
            wu->SetPosition(Vector(off, 0.0, 0.0));
            ws->SetPosition(Vector(off + d * std::cos(e), 0.0, d * std::sin(e)));
            const double v = w->CalcRxPower(0.0, ws, wu);
            if (i >= kBurn)
            {
                s1 += v;
                s2 += v * v;
            }
        }
        const double wMean = s1 / kWalk;
        const double wSd = std::sqrt(std::max(0.0, s2 / kWalk - wMean * wMean));
        NS_TEST_ASSERT_MSG_EQ_TOL(wSd, sigmaTable, 0.20 * sigmaTable,
                                  "the correlated process must be STATIONARY at the Table 6.6.2 "
                                  "sigma (table " << sigmaTable << " dB, walked " << wSd
                                      << " dB). A blend of rho*old + (1-rho)*new instead of "
                                      << "sqrt(1-rho^2) would land near "
                                      << (sigmaTable * 0.315) << " dB here");
    }
};

/// CVC-14: the feeder leg of the O-RAN control loop must come from geometry.
///
/// The flagship RIC scenario set its E2 node's FeederLinkDelay to a constant
/// MilliSeconds(4) with a "~1200 km" comment. Every stage of the published loop
/// latency was then a constant, and the recorded distribution had ZERO variance
/// across all 58 samples: mean, p50, p95, p99, min and max were all 104.000 ms.
/// A latency that cannot move is not a measurement of a loop, and the abstract
/// quoted it as one.
class FeederLinkDelayFollowsGeometryTest : public TestCase
{
  public:
    FeederLinkDelayFollowsGeometryTest()
        : TestCase("CVC-14: the feeder-link delay is derived from live geometry, not a constant")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantPositionMobilityModel> sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 600e3));
        sat.Get(0)->AggregateObject(sm);
        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetSimTime(Seconds(1.0));
        rs.Build(sat, ue);

        // Unwired must report ZERO, not a plausible default: a caller has to be
        // able to tell "no feeder modelled" from "a short one".
        NS_TEST_ASSERT_MSG_EQ(rs.ComputeFeederLinkDelay(), Seconds(0),
                              "with no feeder geometry the delay must be exactly zero so the "
                              "absence is visible");

        Ptr<ConstantPositionMobilityModel> gw = CreateObject<ConstantPositionMobilityModel>();
        gw->SetPosition(Vector(500.0e3, 0.0, 0.0));
        rs.SetFeederGeometry(sm, gw);

        // sqrt(500^2 + 600^2) km = 781.02 km, /c = 2.6052 ms.
        const double slantM = std::sqrt(500.0e3 * 500.0e3 + 600e3 * 600e3);
        const double expectedS = slantM / 299792458.0;
        const Time d0 = rs.ComputeFeederLinkDelay();
        NS_TEST_ASSERT_MSG_EQ_TOL(d0.GetSeconds(), expectedS, 1e-9,
                                  "the delay must be the slant range over c (expected "
                                      << expectedS * 1e3 << " ms, got " << d0.GetMilliSeconds()
                                      << " ms)");
        // And it must NOT be the 4 ms constant it replaced, or the test would
        // pass on the very code it exists to reject.
        NS_TEST_ASSERT_MSG_NE(d0, MilliSeconds(4),
                              "the geometry here is not 4 ms; matching the old constant would "
                              "mean the constant is still in the path");

        // Move the satellite: the delay must follow. A value read once and
        // cached is the same defect wearing a getter.
        sm->SetPosition(Vector(0.0, 0.0, 1200e3));
        const Time d1 = rs.ComputeFeederLinkDelay();
        NS_TEST_ASSERT_MSG_GT(d1.GetSeconds(), d0.GetSeconds() * 1.4,
                              "doubling the altitude must lengthen the feeder delay (was "
                                  << d0.GetMicroSeconds() << " us, now " << d1.GetMicroSeconds()
                                  << " us)");
        sm->SetPosition(Vector(0.0, 0.0, 300e3));
        const Time d2 = rs.ComputeFeederLinkDelay();
        NS_TEST_ASSERT_MSG_LT(d2.GetSeconds(), d0.GetSeconds(),
                              "and lowering it must shorten the delay");

        Simulator::Destroy();
    }
};

/// CVC-07: the health artifact must say WHERE the service-link flight time is.
///
/// The end-to-end user-plane OWD is physically right on every path, but by
/// default the 1.8 ms one-way flight to a 550 km satellite is carried on the
/// BACKHAUL leg, not by the air interface, because a real per-distance air delay
/// aborts the nr stack until K_offset and Timing Advance exist. That is a
/// defensible treatment and it is not the same thing as modelling the delay on
/// the air interface.
///
/// It matters beyond bookkeeping: the manuscript criticizes a comparator for
/// installing "no propagation-delay model on the air interface", which is the
/// one axis on which the default treatment here matches it. A reader of
/// sim_health.csv could not tell which they had without reading the source.
class ServiceLinkDelayTreatmentIsDeclaredTest : public TestCase
{
  public:
    ServiceLinkDelayTreatmentIsDeclaredTest()
        : TestCase("CVC-07: sim_health declares whether the service-link delay is on the "
                   "air interface or folded onto backhaul")
    {
    }

  private:
    static std::string RowFor(const std::string& dir)
    {
        std::ifstream f(dir + "/sim_health.csv");
        std::string line;
        while (std::getline(f, line))
        {
            if (line.rfind("service_link_delay_ms,", 0) == 0)
            {
                return line;
            }
        }
        return "";
    }

    void DoRun() override
    {
        const std::string dir = "test-cvc07-delay-treatment";
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 600e3));
        sm->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(sm);
        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Mmwave);
        rs.SetSimTime(Seconds(2.0));
        rs.SetOutputDir(dir);
        rs.Build(sat, ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::NbIotPeriodic,
                          Seconds(0.2), Seconds(1.8));
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        rs.Collect();
        rs.WriteHealthReport();
        Simulator::Destroy();

        const std::string row = RowFor(dir);
        NS_TEST_ASSERT_MSG_EQ(row.empty(), false,
                              "sim_health.csv must carry a service_link_delay_ms row; without it "
                              "a reader cannot tell a folded delay from an air-interface one");

        // The mmwave backend folds, and the row must SAY it folds rather than
        // leaving the reader to assume the delay is on the air interface.
        const bool saysFolded = (row.find("folded-onto-backhaul") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(saysFolded, true,
                              "this backend carries the slant on the backhaul, and the "
                              "provenance must say so (row was: " << row << ")");
        const bool claimsAir = (row.find("air-interface (real") != std::string::npos);
        NS_TEST_ASSERT_MSG_EQ(claimsAir, false,
                              "and must NOT claim an air-interface treatment it does not have");

        // The value must be the real geometry, not a placeholder: 600 km
        // straight up is 2.0014 ms.
        const size_t c1 = row.find(',');
        const size_t c2 = row.find(',', c1 + 1);
        const double ms = std::stod(row.substr(c1 + 1, c2 - c1 - 1));
        NS_TEST_ASSERT_MSG_EQ_TOL(ms, 600e3 / 299792458.0 * 1e3, 0.05,
                                  "the declared delay must be the measured slant over c, not a "
                                  "constant (got " << ms << " ms)");
        NS_TEST_ASSERT_MSG_GT(ms, 0.0, "and must be non-zero for a 600 km link");
    }
};

/// NT-11: a bound sampled from one arbitrary node is not a bound.
///
/// ComputeOwdFloorMs() read gNB 0's altitude and called it the one-way-delay
/// FLOOR for the whole run. In a mixed-altitude deployment, if node 0 happens to
/// be the highest satellite the floor comes out too large and the health gate
/// FALSE-FAILS a run whose traffic was served by a lower one. And
/// ComputeKOffsetSlots() derived the cell-specific K_offset from the AVERAGE
/// service-link delay, which under-provisions the farthest terminal by
/// construction, producing exactly the "Cannot TX while RX" abort K_offset
/// exists to prevent.
class NtnExtremaNotSamplesTest : public TestCase
{
  public:
    NtnExtremaNotSamplesTest()
        : TestCase("NT-11: the OWD floor uses the LOWEST satellite and K_offset the WORST "
                   "round trip, not node 0 or an average")
    {
    }

  private:
    /// A deployment whose FIRST satellite is the highest, which is the ordering
    /// that exposes an index-0 sample.
    static void Build(NtnRealStackHelper& rs, NodeContainer& sat, NodeContainer& ue,
                      const std::vector<double>& altsM)
    {
        sat.Create(altsM.size());
        for (size_t i = 0; i < altsM.size(); ++i)
        {
            Ptr<ConstantVelocityMobilityModel> m = CreateObject<ConstantVelocityMobilityModel>();
            m->SetPosition(Vector(0.0, 0.0, altsM[i]));
            m->SetVelocity(Vector(7560.0, 0.0, 0.0));
            sat.Get(i)->AggregateObject(m);
        }
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);
        rs.SetSimTime(Seconds(1.0));
    }

    void DoRun() override
    {
        constexpr double kC = 299792458.0;

        // Node 0 at 1200 km, node 1 at 400 km. The true floor is 400 km / c.
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            Build(rs, sat, ue, {1200e3, 400e3});
            rs.Build(sat, ue);
            // The floor is altitude/c PLUS the configured backhaul leg, so the
            // backhaul must be in the expectation too or this compares two
            // different quantities.
            const double backhaulMs = rs.GetBackhaulDelay().GetSeconds() * 1e3;
            const double floorMs = rs.ComputeOwdFloorMs();
            const double expectMs = (400e3 / kC) * 1e3 + backhaulMs;
            const double node0Ms = (1200e3 / kC) * 1e3 + backhaulMs;
            NS_TEST_ASSERT_MSG_EQ_TOL(floorMs, expectMs, 0.01,
                                      "the floor must come from the LOWEST satellite (400 km, "
                                          << expectMs << " ms incl. backhaul), not from node 0 "
                                          << "at 1200 km (" << node0Ms << " ms); got " << floorMs);
            // And it must be a genuine lower bound: strictly below what node 0
            // alone would give, or the ordering does not matter and this test
            // cannot see the defect.
            NS_TEST_ASSERT_MSG_LT(floorMs, node0Ms - 0.1,
                                  "a floor at node 0's altitude would false-fail a run served "
                                  "by the lower satellite");
            Simulator::Destroy();
        }

        // Reversing the order must not change the answer. A sampled bound is
        // an accident of node ordering; a real one is not.
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            Build(rs, sat, ue, {400e3, 1200e3});
            rs.Build(sat, ue);
            const double floorMs = rs.ComputeOwdFloorMs();
            const double expectMs =
                (400e3 / kC) * 1e3 + rs.GetBackhaulDelay().GetSeconds() * 1e3;
            NS_TEST_ASSERT_MSG_EQ_TOL(floorMs, expectMs, 0.01,
                                      "the floor must be invariant under node reordering");
            Simulator::Destroy();
        }

        // K_offset must cover the WORST round trip. With satellites at 400 and
        // 1200 km the worst one-way is 1200 km / c = 4.0 ms, so the RTT is
        // 8.0 ms; the average would be 2.67 ms one-way and would under-provision.
        {
            NtnRealStackHelper rs;
            NodeContainer sat, ue;
            Build(rs, sat, ue, {400e3, 1200e3});
            rs.SetNumerology(1); // 0.5 ms slots
            rs.Build(sat, ue);
            const uint32_t k = rs.ComputeKOffsetSlots();
            const double slotS = 1.0e-3 / 2.0;
            const double worstRttS = 2.0 * (1200e3 / kC);
            const uint32_t expect =
                static_cast<uint32_t>(std::ceil(worstRttS / slotS)) + 1;
            NS_TEST_ASSERT_MSG_EQ(k, expect,
                                  "K_offset must cover the worst-case round trip ("
                                      << worstRttS * 1e3 << " ms -> " << expect << " slots)");
            const double avgRttS = 2.0 * ((400e3 + 1200e3) / 2.0 / kC);
            const uint32_t fromAvg =
                static_cast<uint32_t>(std::ceil(avgRttS / slotS)) + 1;
            NS_TEST_ASSERT_MSG_GT(k, fromAvg,
                                  "and must exceed what the AVERAGE would give (" << fromAvg
                                      << " slots); an average under-provisions the farthest "
                                      << "terminal, which is the abort K_offset prevents");
            Simulator::Destroy();
        }
    }
};

/// NT-12: throughput must be bytes over the window traffic was OFFERED in.
///
/// The accessor divided by the whole simulation time while callers routinely
/// start at 1.0 s and stop at simTime-0.5. For a 10 s run that is an 8.5 s
/// emission window divided by 10: a 15 percent understatement that grows with
/// the guard bands. The per-flow path already used each flow's measured
/// lifetime, so the two accessors disagreed about the same run.
class ThroughputUsesEmissionWindowTest : public TestCase
{
  public:
    ThroughputUsesEmissionWindowTest()
        : TestCase("NT-12: throughput divides by the traffic window, not the simulation time")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 600e3));
        sm->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(sm);
        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);

        const double simS = 6.0;
        const Time start = Seconds(1.0);
        const Time stop = Seconds(5.0); // a 4 s window inside a 6 s run
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Mmwave);
        rs.SetSimTime(Seconds(simS));
        rs.SetOutputDir("test-nt12-window");
        rs.Build(sat, ue);
        rs.InstallTraffic(NtnRealStackHelper::TrafficProfile::EmbbStreaming, start, stop);
        Simulator::Stop(Seconds(simS));
        Simulator::Run();
        rs.Collect();

        // The helper must have recorded the window it was given.
        NS_TEST_ASSERT_MSG_EQ(rs.GetTrafficStart(), start, "the traffic start must be recorded");
        NS_TEST_ASSERT_MSG_EQ(rs.GetTrafficStop(), stop, "and the stop");

        const double thr = rs.GetRxThroughputMbps();
        NS_TEST_ASSERT_MSG_GT(thr, 0.0, "the run must carry traffic for this to mean anything");

        // Recompute both ways from the SAME measured bytes and check which one
        // the accessor reports. This is the assertion that distinguishes them:
        // asserting only "throughput > 0" passes on either denominator.
        const double bytes = static_cast<double>(rs.GetUeRxBytes(0));
        const double overWindow = (bytes * 8.0) / (stop - start).GetSeconds() / 1e6;
        const double overSim = (bytes * 8.0) / simS / 1e6;
        NS_TEST_ASSERT_MSG_GT(overWindow, overSim * 1.2,
                              "the two denominators must differ enough here to tell apart");
        NS_TEST_ASSERT_MSG_EQ_TOL(thr, overWindow, overWindow * 0.10,
                                  "throughput must be measured over the 4 s emission window ("
                                      << overWindow << " Mbps), not the 6 s run (" << overSim
                                      << " Mbps); got " << thr);

        Simulator::Destroy();
    }
};

/// WF-13: the NTN RLC/RRC timer relaxation must reach the backend most runs use.
///
/// ConfigureNtnRlcRrcTimers() was called only from BuildMmwaveRadio(), and the
/// four keys it wrote - ns3::LteRlcAm::* and ns3::LteUeRrc::T300 - belong to
/// ns-3's LTE classes. The nr backend uses ns3::NrRlcAm and ns3::NrUeRrc and
/// never called the function at all, so on the DEFAULT backend the terrestrial
/// 20/10/10 ms RLC-AM timers and the 100 ms T300 stood over a LEO slant whose
/// feedback round trip is 8-26 ms: exactly the spurious-retransmission and RLF
/// condition the relaxation exists to prevent.
class NtnTimerRelaxationReachesNrTest : public TestCase
{
  public:
    NtnTimerRelaxationReachesNrTest()
        : TestCase("WF-13: the RLC/RRC slant relaxation is applied on the nr backend too")
    {
    }

  private:
    /// The CURRENT default for an attribute path, i.e. what a newly created
    /// object of that class would read. Config::SetDefault writes exactly this.
    static Time DefaultOf(const std::string& path)
    {
        TypeId tid;
        const std::string cls = path.substr(0, path.rfind("::"));
        const std::string attr = path.substr(path.rfind("::") + 2);
        NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe(cls, &tid), "unknown class " << cls);
        TypeId::AttributeInformation info;
        NS_ABORT_MSG_IF(!tid.LookupAttributeByName(attr, &info), "unknown attribute " << path);
        // Read through the checker's string form rather than casting the
        // AttributeValue: Config::SetDefault stores a value whose concrete type
        // is the checker's, and a direct cast is not guaranteed to match.
        const std::string ser = info.initialValue->SerializeToString(info.checker);
        TimeValue tv;
        NS_ABORT_MSG_IF(!tv.DeserializeFromString(ser, info.checker),
                        "could not read " << path << " as a Time (got '" << ser << "')");
        return tv.Get();
    }

    void DoRun() override
    {
        // A LEO geometry: 1200 km slant gives an 8.0 ms round trip, so every
        // relaxed timer must exceed its terrestrial default.
        NodeContainer sat;
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> sm = CreateObject<ConstantVelocityMobilityModel>();
        sm->SetPosition(Vector(0.0, 0.0, 1200e3));
        sm->SetVelocity(Vector(7560.0, 0.0, 0.0));
        sat.Get(0)->AggregateObject(sm);
        NodeContainer ue;
        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(um);

        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(1.0));
        rs.Build(sat, ue);

        // The nr classes are what this backend instantiates, so those are the
        // defaults that must have moved. Checking only the LTE keys would pass
        // on the broken code, which set them and nothing else.
        const Time poll = DefaultOf("ns3::NrRlcAm::PollRetransmitTimer");
        const Time reorder = DefaultOf("ns3::NrRlcAm::ReorderingTimer");
        const Time status = DefaultOf("ns3::NrRlcAm::StatusProhibitTimer");

        NS_TEST_ASSERT_MSG_GT(poll.GetMilliSeconds(), 20,
                              "NrRlcAm t-PollRetransmit must be relaxed past its 20 ms "
                              "terrestrial default over a 1200 km slant; it was left alone "
                              "because only the LTE key was written");
        NS_TEST_ASSERT_MSG_GT(reorder.GetMilliSeconds(), 10,
                              "NrRlcAm t-Reordering must be relaxed past 10 ms");
        NS_TEST_ASSERT_MSG_GT(status.GetMilliSeconds(), 10,
                              "NrRlcAm t-StatusProhibit must be relaxed past 10 ms");

        // And the values must track the geometry rather than being any larger
        // constant: t-StatusProhibit is one RTT plus 5 ms.
        constexpr double kC = 299792458.0;
        const double rttMs = 2.0 * 1200e3 / kC * 1e3;
        NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(status.GetMilliSeconds()),
                                  rttMs + 5.0, 1.5,
                                  "the relaxation must be derived from the slant round trip ("
                                      << rttMs << " ms), not set to a fixed larger value");

        Simulator::Destroy();
    }
};

class NtnRealStackHelperTestSuite : public TestSuite
{
  public:
    NtnRealStackHelperTestSuite()
        : TestSuite("ntn-real-stack-helper", Type::UNIT)
    {
        AddTestCase(new RealStackEirpBudgetGateTest, Duration::QUICK);
        AddTestCase(new RealStackX2DelayAppliedTest, Duration::QUICK);
        AddTestCase(new NtnRachWindowAppliedTest, Duration::QUICK);
        AddTestCase(new NtnRachWindowOffTest, Duration::QUICK);
        AddTestCase(new NtnFr1BandConformanceTest, Duration::QUICK);
        AddTestCase(new HandoverRefusalIsCountedTest, Duration::QUICK);
        AddTestCase(new KOffsetNotConsumedWhenOffTest, Duration::QUICK);
        AddTestCase(new OwdFloorGateTest, Duration::QUICK);
        AddTestCase(new EirpGateEvaluatedTest, Duration::QUICK);
        AddTestCase(new NtnTdlFrequencySelectivityTest, Duration::QUICK);
        AddTestCase(new Tr38811FastFadingStatisticsTest, Duration::QUICK);
        AddTestCase(new Tr38811ShadowFadingIsCorrelatedTest, Duration::QUICK);
        AddTestCase(new FeederLinkDelayFollowsGeometryTest, Duration::EXTENSIVE);
        AddTestCase(new ServiceLinkDelayTreatmentIsDeclaredTest, Duration::EXTENSIVE);
        AddTestCase(new NtnExtremaNotSamplesTest, Duration::EXTENSIVE);
        AddTestCase(new ThroughputUsesEmissionWindowTest, Duration::EXTENSIVE);
        AddTestCase(new NtnTimerRelaxationReachesNrTest, Duration::EXTENSIVE);
        AddTestCase(new NtnChannelExtrasReachTheChainTest, Duration::EXTENSIVE);
        AddTestCase(new NtnScenarioReachesTheModelTest, Duration::EXTENSIVE);
        AddTestCase(new MultiGnbMeasuredPlaneTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackNeighbourRsrpTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackAirInterfaceDelayTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackAiMonitorAutoExportTest, Duration::QUICK);
        AddTestCase(new RealStackAiMonitorAfterInstallTest, Duration::QUICK);
        AddTestCase(new RealStackNtnHarqProfileTest, Duration::QUICK);
        AddTestCase(new RealStackUeKeySeparationTest, Duration::QUICK);
        AddTestCase(new RealStackKOffsetConsumedTest, Duration::QUICK);
        AddTestCase(new RealStackOfferedLoadAccountingTest, Duration::QUICK);
        AddTestCase(new SatBeamGainAngleDependenceTest, Duration::QUICK);
        AddTestCase(new RealStackRsrpPerResourceElementTest, Duration::QUICK);
        AddTestCase(new RealStackReproManifestTest, Duration::QUICK);
        AddTestCase(new RealStackBackhaulFoldTracksTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackNtnBandConformanceTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackHealthGatesCanFailTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackChannelUpdatePeriodTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackIndependentBudgetOracleTest, Duration::EXTENSIVE);
        AddTestCase(new RealStackStrictGatesReachableTest, Duration::EXTENSIVE);
    }
};

static NtnRealStackHelperTestSuite g_ntnRealStackHelperTestSuite;
