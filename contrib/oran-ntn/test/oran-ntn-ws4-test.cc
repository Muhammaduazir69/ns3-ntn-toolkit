/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// WS4 tests (AI_NATIVE_ORAN_NTN plan): payload-option delay ladder, FH split
// feasibility at real NTN latencies, Table-3 endurance enforcement, and the
// role switch driven by a REAL battery model with failure injection.

#include "ns3/constant-position-mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ntn-command-and-control-app.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/ntn-platform-spec.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/oran-ntn-fh-split.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"

using namespace ns3;

class PayloadDelayLadderTest : public TestCase
{
  public:
    PayloadDelayLadderTest()
        : TestCase("payload options order exactly as the paper's delay ladder")
    {
    }

  private:
    void DoRun() override
    {
        using PO = NtnRealStackHelper::PayloadOption;
        NtnRealStackHelper rs;
        const double slantM = 600e3;
        auto delayOf = [&rs, slantM](PO p) {
            rs.SetPayloadOption(p);
            return rs.ComputePayloadExtraDelay(slantM);
        };
        const Time tTrans = delayOf(PO::Transparent);
        const Time tRu = delayOf(PO::RegenerativeRu);
        const Time tRuDu = delayOf(PO::RegenerativeRuDu);
        const Time tFull = delayOf(PO::FullGnb);
        NS_TEST_ASSERT_MSG_GT(tTrans.GetSeconds(), tRu.GetSeconds(),
                              "transparent > regenerative RU");
        NS_TEST_ASSERT_MSG_GT(tRu.GetSeconds(), tRuDu.GetSeconds(), "RU > RU+DU");
        NS_TEST_ASSERT_MSG_GT(tRuDu.GetSeconds(), tFull.GetSeconds(), "RU+DU > full gNB");
        // Transparent = exactly two propagation legs.
        NS_TEST_ASSERT_MSG_EQ_TOL(tTrans.GetSeconds(), 2.0 * 600e3 / 299792458.0, 1e-9,
                                  "bent-pipe is 2 x slant/c");
    }
};

class FhSplitFeasibilityTest : public TestCase
{
  public:
    FhSplitFeasibilityTest()
        : TestCase("FH split feasibility at real NTN latencies")
    {
    }

  private:
    void DoRun() override
    {
        using S = NtnFhSplitModel::Split;
        // LEO feeder: ~4 ms one-way (paper band 2.1-18 ms). Every lower-PHY
        // split (<=0.25 ms) is impossible — the regenerative argument.
        const Time leoFh = MilliSeconds(4);
        NS_TEST_ASSERT_MSG_EQ(NtnFhSplitModel::IsFeasible(S::Opt8, leoFh, 10, 1000),
                              false, "Opt8 impossible at LEO");
        NS_TEST_ASSERT_MSG_EQ(NtnFhSplitModel::IsFeasible(S::Opt7_2a, leoFh, 10, 1000),
                              false, "Opt7.2a impossible at LEO");
        NS_TEST_ASSERT_MSG_EQ(NtnFhSplitModel::IsFeasible(S::Opt2, leoFh, 10, 1000),
                              true, "Opt2 (F1) feasible at LEO");
        bool ok = false;
        NS_TEST_ASSERT_MSG_EQ((NtnFhSplitModel::ChooseBestSplit(leoFh, 10, 1000, ok) ==
                               S::Opt2),
                              true, "best split at LEO is Opt2");
        NS_TEST_ASSERT_MSG_EQ(ok, true, "Opt2 fits");

        // Terrestrial-grade FH (0.1 ms): capacity decides the depth.
        const Time fastFh = MicroSeconds(100);
        NS_TEST_ASSERT_MSG_EQ((NtnFhSplitModel::ChooseBestSplit(fastFh, 10, 1000, ok) ==
                               S::Opt8),
                              true, "Opt8 when 16x rate fits");
        NS_TEST_ASSERT_MSG_EQ((NtnFhSplitModel::ChooseBestSplit(fastFh, 10, 80, ok) ==
                               S::Opt7_2b),
                              true, "Opt7.2b when only 7x fits");
        NS_TEST_ASSERT_MSG_EQ((NtnFhSplitModel::ChooseBestSplit(fastFh, 10, 5, ok) ==
                               S::Opt2),
                              true, "fallback to Opt2 under starved FH");
        NS_TEST_ASSERT_MSG_EQ(ok, false, "even Opt2 flagged infeasible at 5 Mbps");
    }
};

class EnduranceEnforcementTest : public TestCase
{
  public:
    EnduranceEnforcementTest()
        : TestCase("Table-3 endurance ends an untethered UAV's service")
    {
    }

  private:
    void DoRun() override
    {
        bool uavEnded = false;
        bool hapEnded = false;
        NtnPlatformSpec::ScheduleEnduranceEnd(NtnPlatformSpec::Class::UavUntethered,
                                              [&uavEnded] { uavEnded = true; });
        // LEO is mission-life: no endurance event may fire.
        NtnPlatformSpec::ScheduleEnduranceEnd(NtnPlatformSpec::Class::Leo,
                                              [&hapEnded] { hapEnded = true; });
        Simulator::Stop(Minutes(31));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ(uavEnded, true, "untethered UAV ended at 30 min");
        NS_TEST_ASSERT_MSG_EQ(hapEnded, false, "LEO unconstrained");
        NS_TEST_ASSERT_MSG_EQ(
            NtnPlatformSpec::Get(NtnPlatformSpec::Class::Geo).userPlaneLatency,
            MilliSeconds(600), "GEO latency class from Table 3");
        Simulator::Destroy();
    }
};

class RoleSwitchE2eTest : public TestCase
{
  public:
    RoleSwitchE2eTest()
        : TestCase("role switch on REAL battery drain + FH breach + failure")
    {
    }

  private:
    void DoRun() override
    {
        // Real energy model: the C&C app's battery drains 1%/s (100 Wh at
        // 3600 W) — the role switch consumes the MEASURED fraction.
        NodeContainer nodes;
        nodes.Create(2);
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        p2p.SetChannelAttribute("Delay", StringValue("5ms"));
        NetDeviceContainer devs = p2p.Install(nodes);
        InternetStackHelper internet;
        internet.Install(nodes);
        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.3.1.0", "255.255.255.0");
        Ipv4InterfaceContainer ifaces = ipv4.Assign(devs);
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        nodes.Get(0)->AggregateObject(mob);

        Ptr<NtnOranSink> smoSink = CreateObject<NtnOranSink>();
        smoSink->SetAttribute("Local",
                              AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 9000)));
        nodes.Get(1)->AddApplication(smoSink);
        smoSink->SetStartTime(Seconds(0.0));

        Ptr<NtnCommandAndControlApp> cnc = CreateObject<NtnCommandAndControlApp>();
        cnc->SetRemote(InetSocketAddress(ifaces.GetAddress(1), 9000));
        cnc->SetAttribute("SrcId", UintegerValue(7));
        cnc->SetAttribute("BatteryCapacityWh", DoubleValue(100.0));
        cnc->SetAttribute("PowerDrawW", DoubleValue(3600.0));
        nodes.Get(0)->AddApplication(cnc);
        cnc->SetStartTime(Seconds(0.0));

        Ptr<OranNtnRoleSwitch> rsw = CreateObject<OranNtnRoleSwitch>();
        using Role = OranNtnRoleSwitch::Role;
        Role hapApplied = Role::Ru;
        rsw->RegisterPlatform("hap", Role::RuDu,
                              [&hapApplied](Role r) { hapApplied = r; });
        // Battery source = telemetry RECEIVED at the SMO (crossed the network).
        rsw->SetBatterySource("hap", [smoSink] {
            NtnCncTelemetry t;
            return smoSink->GetLatestTelemetry(7, t) ? t.batteryFraction : 1.0;
        });
        rsw->SetBatteryThreshold(0.2);
        rsw->SetInterruptionWindow(MilliSeconds(500));
        rsw->SetCheckPeriod(Seconds(1.0));

        Role leoApplied = Role::FullGnb;
        rsw->RegisterPlatform("leo", Role::Ru, [&leoApplied](Role r) { leoApplied = r; });
        // FH latency breach after t=30 s: elevation Ru -> RuDu.
        rsw->SetFhLatencySource("leo", [] {
            return Simulator::Now() > Seconds(30.0) ? MilliSeconds(15) : MilliSeconds(4);
        });
        rsw->SetFhLatencyBound(MilliSeconds(10));
        rsw->Start();

        Simulator::Stop(Seconds(100.0));
        Simulator::Run();

        // Battery hits 20% at t=80 s -> hap offloads RuDu -> Ru.
        NS_TEST_ASSERT_MSG_EQ((rsw->GetRole("hap") == Role::Ru), true,
                              "hap offloaded on measured battery");
        NS_TEST_ASSERT_MSG_EQ((hapApplied == Role::Ru), true, "applier ran");
        // FH breach at t=30 s -> leo elevated Ru -> RuDu.
        NS_TEST_ASSERT_MSG_EQ((rsw->GetRole("leo") == Role::RuDu), true,
                              "leo elevated on measured FH latency");
        NS_TEST_ASSERT_MSG_EQ((leoApplied == Role::RuDu), true, "applier ran");

        bool sawBattery = false;
        bool sawFh = false;
        for (const auto& ev : rsw->GetEvents())
        {
            if (ev.platform == "hap" && ev.trigger == "battery" &&
                ev.time > Seconds(75.0) && ev.time < Seconds(85.0))
            {
                sawBattery = true;
                NS_TEST_ASSERT_MSG_EQ(ev.interruption, MilliSeconds(500),
                                      "interruption window recorded");
            }
            if (ev.platform == "leo" && ev.trigger == "fh-latency" &&
                ev.time > Seconds(30.0) && ev.time < Seconds(33.0))
            {
                sawFh = true;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(sawBattery, true, "battery switch at the drain time");
        NS_TEST_ASSERT_MSG_EQ(sawFh, true, "fh switch right after the breach");
        Simulator::Destroy();

        // Failure injection demotes immediately (fresh scheduler).
        Ptr<OranNtnRoleSwitch> rsw2 = CreateObject<OranNtnRoleSwitch>();
        rsw2->RegisterPlatform("sat", Role::FullGnb, nullptr);
        rsw2->InjectFailure("sat", "o-du-crash");
        Simulator::Stop(Seconds(2.0));
        Simulator::Run();
        NS_TEST_ASSERT_MSG_EQ((rsw2->GetRole("sat") == Role::Ru), true,
                              "failure demotes to bare RU");
        Simulator::Destroy();
    }
};

class OranNtnWs4TestSuite : public TestSuite
{
  public:
    OranNtnWs4TestSuite()
        : TestSuite("oran-ntn-ws4", Type::UNIT)
    {
        AddTestCase(new PayloadDelayLadderTest, Duration::QUICK);
        AddTestCase(new FhSplitFeasibilityTest, Duration::QUICK);
        AddTestCase(new EnduranceEnforcementTest, Duration::QUICK);
        AddTestCase(new RoleSwitchE2eTest, Duration::QUICK);
    }
};

static OranNtnWs4TestSuite g_oranNtnWs4TestSuite;
