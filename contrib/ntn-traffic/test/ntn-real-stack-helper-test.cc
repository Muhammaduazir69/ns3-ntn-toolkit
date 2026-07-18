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
//      leaves them untouched.

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
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/type-id.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

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
/// a static ground UE, moving at orbital speed. The 600 km zenith slant is
/// exact at t=0, which both the Friis budget and the HARQ math depend on.
struct LeoRig
{
    NodeContainer sat;
    NodeContainer ue;

    LeoRig()
    {
        sat.Create(1);
        Ptr<ConstantVelocityMobilityModel> satMob =
            CreateObject<ConstantVelocityMobilityModel>();
        satMob->SetPosition(Vector(0.0, 0.0, 600e3));
        satMob->SetVelocity(Vector(7560.0, 0.0, 0.0)); // LEO-600 orbital speed
        sat.Get(0)->AggregateObject(satMob);

        ue.Create(1);
        Ptr<ConstantPositionMobilityModel> ueMob =
            CreateObject<ConstantPositionMobilityModel>();
        ueMob->SetPosition(Vector(0.0, 0.0, 0.0));
        ue.Get(0)->AggregateObject(ueMob);
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

class NtnRealStackHelperTestSuite : public TestSuite
{
  public:
    NtnRealStackHelperTestSuite()
        : TestSuite("ntn-real-stack-helper", Type::UNIT)
    {
        AddTestCase(new RealStackAiMonitorAutoExportTest, Duration::QUICK);
        AddTestCase(new RealStackAiMonitorAfterInstallTest, Duration::QUICK);
        AddTestCase(new RealStackNtnHarqProfileTest, Duration::QUICK);
        AddTestCase(new RealStackUeKeySeparationTest, Duration::QUICK);
        AddTestCase(new RealStackKOffsetConsumedTest, Duration::QUICK);
    }
};

static NtnRealStackHelperTestSuite g_ntnRealStackHelperTestSuite;
