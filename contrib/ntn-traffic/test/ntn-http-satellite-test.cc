/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * End-to-end test for the module's OWN satellite HTTP traffic model.
 *
 * Why this file exists. `ThreeGppHttpSatelliteClient`, its helper and the
 * module's `ThreeGppHttpVariables` shipped registered and documented, and
 * nothing ran them: no registered example, and no test. What the module's
 * README pointed at instead, "the three-gpp-http-client-server-test system
 * suite", was a stale fork of the upstream ns-3 test sitting unregistered in
 * test/. It could not be registered either, because it does not compile against
 * ns-3.43 and its NS_LOG_COMPONENT collides with the upstream copy. It carried
 * no NTN content, so it is gone and this replaces it.
 *
 * This exercises the satellite client over a real link with a real one-way
 * delay, across Simulator::Run(), and asserts on traffic that actually moved.
 */
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/node-container.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/packet.h"
#include "ns3/three-gpp-http-satellite-client.h"
#include "ns3/three-gpp-http-satellite-helper.h"
#include "ns3/uinteger.h"

#include <cstdint>

using namespace ns3;

/**
 * \brief A browsing session over a satellite-like one-way delay must actually
 *        deliver main objects, and the client's measured page-load time must be
 *        at least the round trip the link imposes.
 *
 * The second half is what makes this more than a smoke test. Page-load time is
 * the KPI this model exists to produce; if it came back smaller than 2x the
 * configured one-way delay it would not be measuring transport at all.
 */
class NtnHttpSatelliteBrowsingTestCase : public TestCase
{
  public:
    NtnHttpSatelliteBrowsingTestCase()
        : TestCase("Satellite HTTP client completes a browsing session over a slant delay")
    {
    }

  private:
    void RxMainObject(Ptr<const ThreeGppHttpSatelliteClient>, Ptr<const Packet>)
    {
        ++m_mainObjects;
    }

    void RxPlt(const Time& pageLoadTime, const Address&)
    {
        ++m_pltSamples;
        if (pageLoadTime > m_maxPlt)
        {
            m_maxPlt = pageLoadTime;
        }
    }

    void DoRun() override
    {
        // A LEO service link is a few milliseconds one way; use 20 ms so the
        // floor this test asserts on is unambiguous against scheduler jitter.
        const Time owd = MilliSeconds(20);

        NodeContainer nodes;
        nodes.Create(2);

        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", StringValue("20Mbps"));
        p2p.SetChannelAttribute("Delay", TimeValue(owd));
        NetDeviceContainer devices = p2p.Install(nodes);

        InternetStackHelper internet;
        internet.Install(nodes);
        Ipv4AddressHelper addr;
        addr.SetBase("10.1.1.0", "255.255.255.0");
        Ipv4InterfaceContainer ifaces = addr.Assign(devices);

        ThreeGppHttpHelper http;
        ApplicationContainer apps = http.InstallUsingIpv4(nodes.Get(0), nodes.Get(1));
        apps.Start(Seconds(0.0));
        apps.Stop(Seconds(60.0));

        Ptr<ThreeGppHttpSatelliteClient> client =
            http.GetClients().Get(0)->GetObject<ThreeGppHttpSatelliteClient>();
        NS_TEST_ASSERT_MSG_NE(client, nullptr, "the helper must install the module's own client");

        client->TraceConnectWithoutContext(
            "RxMainObject",
            MakeCallback(&NtnHttpSatelliteBrowsingTestCase::RxMainObject, this));
        client->TraceConnectWithoutContext(
            "RxPlt",
            MakeCallback(&NtnHttpSatelliteBrowsingTestCase::RxPlt, this));

        Simulator::Stop(Seconds(61.0));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_GT(m_mainObjects, 0u,
                              "the browsing session delivered no main object at all, so the "
                              "satellite HTTP model is not moving traffic");

        // Page-load time has to clear one round trip: request out, object back.
        //
        // Measured while writing this, varying only the link delay: 5 ms one way
        // gives 178.19 ms, 20 ms gives 247.26 ms, 200 ms gives 2047.26 ms. The
        // 20 to 200 ms step adds 1800 ms for 180 ms of extra one-way delay,
        // exactly ten times it, so a page costs about five round trips and the
        // KPI is genuinely transport-bound rather than incidentally non-zero.
        const Time floor = owd * 2;
        NS_TEST_ASSERT_MSG_GT(m_pltSamples, 0u,
                              "no page-load-time sample was produced, so the KPI this model "
                              "exists for was never measured");
        NS_TEST_ASSERT_MSG_GT(m_maxPlt, floor,
                              "measured page-load time " << m_maxPlt.As(Time::MS)
                                  << " is below the " << floor.As(Time::MS)
                                  << " round trip the link imposes, so it cannot be a "
                                     "transport measurement");
    }

    uint32_t m_mainObjects{0};
    uint32_t m_pltSamples{0};
    Time m_maxPlt{Seconds(0)};
};

class NtnHttpSatelliteTestSuite : public TestSuite
{
  public:
    NtnHttpSatelliteTestSuite()
        : TestSuite("ntn-http-satellite", Type::SYSTEM)
    {
        AddTestCase(new NtnHttpSatelliteBrowsingTestCase, Duration::QUICK);
    }
};

static NtnHttpSatelliteTestSuite g_ntnHttpSatelliteTestSuite;
