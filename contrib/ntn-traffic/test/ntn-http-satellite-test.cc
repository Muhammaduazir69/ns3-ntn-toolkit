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
#include "ns3/constant-position-mobility-model.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/error-model.h"
#include "ns3/ntn-real-stack-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/three-gpp-http-header.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

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

// ---------------------------------------------------------------------------
// In-depth cases (Nov 2026 test campaign).
// ---------------------------------------------------------------------------

namespace
{

/// TR 38.821 Table 5.3.2.1-1, transparent payload: maximum round-trip
/// propagation delay 25.77 ms for LEO at 600 km and 541.46 ms for GEO.
constexpr double kLeoOwdMs = 25.77 / 2.0;
constexpr double kGeoOwdMs = 541.46 / 2.0;

Time
Ms(double ms)
{
    return MicroSeconds(static_cast<int64_t>(std::llround(ms * 1000.0)));
}

/// Server node 0 and client node 1 on one point-to-point link.
NodeContainer
HttpLink(const std::string& rate, Time owd, NetDeviceContainer* devsOut = nullptr)
{
    NodeContainer nodes;
    nodes.Create(2);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue(rate));
    p2p.SetChannelAttribute("Delay", TimeValue(owd));
    NetDeviceContainer devices = p2p.Install(nodes);
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper addr;
    addr.SetBase("10.1.1.0", "255.255.255.0");
    addr.Assign(devices);
    if (devsOut)
    {
        *devsOut = devices;
    }
    return nodes;
}

/// Per-client browsing record built from the client's own traces.
struct Browse
{
    std::vector<double> mainSizes;    //!< content length of each main object
    std::vector<uint32_t> embedded;   //!< embedded objects of each finished page
    std::vector<double> parsingS;     //!< main object received to first embedded request
    std::vector<double> readingS;     //!< page complete to next main request
    std::vector<double> pltS;         //!< page load times
    std::vector<Time> mainRxTimes;    //!< arrival time of each main object
    uint32_t curEmbedded{0};
    bool inPage{false};
    bool awaitingFirstEmbedded{false};
    Time mainRx;
    Time pageDone{Time(-1)};
    uint32_t failures{0};

    void RxMain(Ptr<const ThreeGppHttpSatelliteClient>, Ptr<const Packet> p)
    {
        ThreeGppHttpHeader h;
        p->PeekHeader(h);
        mainSizes.push_back(h.GetContentLength());
        mainRx = Simulator::Now();
        mainRxTimes.push_back(mainRx);
        curEmbedded = 0;
        inPage = true;
        awaitingFirstEmbedded = true;
    }

    void TxEmbReq(Ptr<const Packet>)
    {
        if (awaitingFirstEmbedded)
        {
            parsingS.push_back((Simulator::Now() - mainRx).GetSeconds());
            awaitingFirstEmbedded = false;
        }
    }

    void RxEmb(Ptr<const ThreeGppHttpSatelliteClient>, Ptr<const Packet>)
    {
        ++curEmbedded;
    }

    void Plt(const Time& plt, const Address&)
    {
        pltS.push_back(plt.GetSeconds());
        if (inPage)
        {
            embedded.push_back(curEmbedded);
            inPage = false;
        }
        pageDone = Simulator::Now();
    }

    void TxMainReq(Ptr<const Packet>)
    {
        if (!pageDone.IsNegative())
        {
            readingS.push_back((Simulator::Now() - pageDone).GetSeconds());
            pageDone = Time(-1);
        }
    }

    void Failed(Ptr<const ThreeGppHttpSatelliteClient>)
    {
        ++failures;
    }

    void Connect(Ptr<Application> app)
    {
        app->TraceConnectWithoutContext("RxMainObject", MakeCallback(&Browse::RxMain, this));
        app->TraceConnectWithoutContext("TxEmbeddedObjectRequest", MakeCallback(&Browse::TxEmbReq, this));
        app->TraceConnectWithoutContext("RxEmbeddedObject", MakeCallback(&Browse::RxEmb, this));
        app->TraceConnectWithoutContext("RxPlt", MakeCallback(&Browse::Plt, this));
        app->TraceConnectWithoutContext("TxMainObjectRequest", MakeCallback(&Browse::TxMainReq, this));
        app->TraceConnectWithoutContext("ConnectionFailed", MakeCallback(&Browse::Failed, this));
    }
};

double
Mean(const std::vector<double>& v)
{
    double s = 0.0;
    for (double x : v)
    {
        s += x;
    }
    return v.empty() ? 0.0 : s / v.size();
}

double
StdDev(const std::vector<double>& v)
{
    const double m = Mean(v);
    double s = 0.0;
    for (double x : v)
    {
        s += (x - m) * (x - m);
    }
    return v.size() < 2 ? 0.0 : std::sqrt(s / (v.size() - 1));
}

} // namespace

/// 3GPP HTTP traffic model conformance on the measured plane, TR 36.814
/// Annex A.2.1.3 (Table A.2.1.3-1, the model ns-3's ThreeGppHttpVariables
/// implements): main object size truncated lognormal with sigma 1.37 and mu
/// 8.37 (mean 10710 B, min 100 B, max 2 MB); number of embedded objects a
/// truncated Pareto with mean 5.64 and max 53; parsing time exponential with
/// mean 0.13 s; reading time exponential (mean set to 0.5 s here through
/// ThreeGppHttpHelper::SetVariablesAttribute, which had no effect before this
/// campaign). Four clients browse a LEO-600 link for 300 s; every quantity is
/// read from the client's own traces, so it is what the transport delivered.
class NtnHttpTr36814ConformanceTest : public TestCase
{
  public:
    NtnHttpTr36814ConformanceTest()
        : TestCase("TR 36.814 A.2.1.3 HTTP model on the wire: object sizes, embedded count, times")
    {
    }

  private:
    void DoRun() override
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer n = HttpLink("100Mbps", Ms(kLeoOwdMs));
        NodeContainer clients;
        for (int c = 0; c < 4; ++c)
        {
            clients.Add(n.Get(1));
        }
        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(0.5)));
        ApplicationContainer apps = http.InstallUsingIpv4(n.Get(0), clients);
        apps.Start(Seconds(0.0));
        apps.Stop(Seconds(300.0));
        std::vector<Browse> rec(4);
        for (uint32_t c = 0; c < 4; ++c)
        {
            rec[c].Connect(http.GetClients().Get(c));
        }
        Simulator::Stop(Seconds(301.0));
        Simulator::Run();
        Simulator::Destroy();

        std::vector<double> lnMain;
        std::vector<double> emb;
        std::vector<double> parsing;
        std::vector<double> reading;
        double minMain = 1e30;
        double maxMain = 0.0;
        uint32_t maxEmb = 0;
        for (const auto& r : rec)
        {
            NS_TEST_ASSERT_MSG_GT(r.mainSizes.size(), 100u, "each client browsed many pages");
            for (double s : r.mainSizes)
            {
                lnMain.push_back(std::log(s));
                minMain = std::min(minMain, s);
                maxMain = std::max(maxMain, s);
            }
            for (uint32_t e : r.embedded)
            {
                emb.push_back(e);
                maxEmb = std::max(maxEmb, e);
            }
            parsing.insert(parsing.end(), r.parsingS.begin(), r.parsingS.end());
            reading.insert(reading.end(), r.readingS.begin(), r.readingS.end());
            NS_TEST_ASSERT_MSG_EQ(r.failures, 0u, "no failed connection on a clean link");
        }
        std::cout << "[http-tr36814] pages " << lnMain.size() << " ln(main) mean " << Mean(lnMain)
                  << " sd " << StdDev(lnMain) << " min " << minMain << " max " << maxMain
                  << "; embedded mean " << Mean(emb) << " max " << maxEmb << " n " << emb.size()
                  << "; parsing mean " << Mean(parsing) << " n " << parsing.size()
                  << "; reading mean " << Mean(reading) << " n " << reading.size() << std::endl;

        // Tolerances are three standard errors of the sample mean (plus the
        // small shift of ns-3's mu = 8.35 derived from the table's mean and
        // standard deviation).
        NS_TEST_ASSERT_MSG_EQ_TOL(Mean(lnMain), 8.37, 0.15, "main object size: lognormal mu 8.37");
        NS_TEST_ASSERT_MSG_EQ_TOL(StdDev(lnMain), 1.37, 0.1, "main object size: lognormal sigma 1.37");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(minMain, 100.0, "main object min 100 B");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxMain, 2e6, "main object max 2 MB");
        const double embTol = 3.0 * 10.6 / std::sqrt(std::max<double>(emb.size(), 1.0));
        NS_TEST_ASSERT_MSG_EQ_TOL(Mean(emb), 5.64, embTol, "embedded objects per page: mean 5.64");
        NS_TEST_ASSERT_MSG_LT_OR_EQ(maxEmb, 53u, "embedded objects per page: max 53");
        const double parTol = 3.0 * 0.13 / std::sqrt(std::max<double>(parsing.size(), 1.0));
        NS_TEST_ASSERT_MSG_EQ_TOL(Mean(parsing), 0.13, parTol, "parsing time: exponential mean 0.13 s");
        const double readTol = 3.0 * 0.5 / std::sqrt(std::max<double>(reading.size(), 1.0));
        NS_TEST_ASSERT_MSG_EQ_TOL(Mean(reading), 0.5, readTol,
                                  "reading time follows the helper's Variables setting");
    }
};

/// Page-load time is transport-bound. The same pages (same RngRun and stream
/// numbering) are fetched over the TR 38.821 LEO-600 and GEO transparent
/// one-way delays. Every GEO page takes at least one GEO round trip
/// (541.46 ms), and the PLT difference of each page is a whole number of
/// round-trip differences: the extra time is round trips, nothing else.
class NtnHttpRttScalingTest : public TestCase
{
  public:
    NtnHttpRttScalingTest()
        : TestCase("PLT grows by whole round trips from the LEO-600 to the GEO slant (TR 38.821)")
    {
    }

  private:
    std::vector<double> Run(double owdMs)
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer n = HttpLink("100Mbps", Ms(owdMs));
        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(2.0)));
        ApplicationContainer apps = http.InstallUsingIpv4(n.Get(0), n.Get(1));
        apps.Start(Seconds(0.0));
        apps.Stop(Seconds(60.0));
        Browse b;
        b.Connect(http.GetClients().Get(0));
        Simulator::Stop(Seconds(61.0));
        Simulator::Run();
        Simulator::Destroy();
        return b.pltS;
    }

    void DoRun() override
    {
        const auto leo = Run(kLeoOwdMs);
        const auto geo = Run(kGeoOwdMs);
        NS_TEST_ASSERT_MSG_GT(geo.size(), 5u, "several GEO pages");
        const double dRtt = 2.0 * (kGeoOwdMs - kLeoOwdMs) / 1e3;
        const std::size_t n = std::min(leo.size(), geo.size());
        uint32_t whole = 0;
        for (std::size_t k = 0; k < n; ++k)
        {
            NS_TEST_ASSERT_MSG_GT(geo[k], 541.46e-3, "a GEO page costs at least one GEO RTT");
            const double rounds = (geo[k] - leo[k]) / dRtt;
            std::cout << "[http-rtt] page " << k << " LEO " << leo[k] << " s GEO " << geo[k]
                      << " s extra round trips " << rounds << std::endl;
            NS_TEST_ASSERT_MSG_GT(rounds, 0.95, "at least one extra round trip per page");
            if (std::abs(rounds - std::round(rounds)) < 0.05)
            {
                ++whole;
            }
        }
        NS_TEST_ASSERT_MSG_GT_OR_EQ(whole * 10, n * 9, "nine pages in ten differ by whole round trips");
    }
};

/// Link outage in the middle of a page (a feeder-link switch or blockage):
/// both directions are cut for 5 s right after a main-object request. The
/// session survives (no fatal error), that page completes after the outage
/// with a PLT above 5 s and within the TCP retransmission backoff (RFC 6298
/// section 5.5: the timer doubles from RTO_min, so the first retransmission
/// after the outage comes within 2 D + RTO_min of its start, then the page
/// loads like any other), and browsing continues afterwards.
class NtnHttpOutageRecoveryTest : public TestCase
{
  public:
    NtnHttpOutageRecoveryTest()
        : TestCase("A 5 s outage mid-page: the page completes after it and browsing resumes")
    {
    }

  private:
    void TxMainReq(Ptr<const Packet>)
    {
        if (!m_outageArmed && Simulator::Now() > Seconds(10))
        {
            m_outageArmed = true;
            m_outageStart = Simulator::Now();
            m_em0->Enable();
            m_em1->Enable();
            Simulator::Schedule(Seconds(5), &NtnHttpOutageRecoveryTest::EndOutage, this);
        }
    }

    void EndOutage()
    {
        m_em0->Disable();
        m_em1->Disable();
    }

    void DoRun() override
    {
        RngSeedManager::ResetNextStreamIndex();
        NetDeviceContainer devs;
        NodeContainer n = HttpLink("20Mbps", Ms(kLeoOwdMs), &devs);
        m_em0 = CreateObject<RateErrorModel>();
        m_em1 = CreateObject<RateErrorModel>();
        for (auto em : {m_em0, m_em1})
        {
            em->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
            em->SetAttribute("ErrorRate", DoubleValue(1.0));
            em->Disable();
        }
        devs.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(m_em0));
        devs.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(m_em1));
        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(1.0)));
        ApplicationContainer apps = http.InstallUsingIpv4(n.Get(0), n.Get(1));
        apps.Start(Seconds(0.0));
        apps.Stop(Seconds(60.0));
        Browse b;
        Ptr<Application> client = http.GetClients().Get(0);
        b.Connect(client);
        client->TraceConnectWithoutContext("TxMainObjectRequest",
                                           MakeCallback(&NtnHttpOutageRecoveryTest::TxMainReq, this));
        Simulator::Stop(Seconds(61.0));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_ASSERT_MSG_EQ(m_outageArmed, true, "the outage hit a page request");
        uint32_t during = 0;
        uint32_t after = 0;
        for (Time t : b.mainRxTimes)
        {
            if (t > m_outageStart && t < m_outageStart + Seconds(5))
            {
                ++during;
            }
            if (t >= m_outageStart + Seconds(5))
            {
                ++after;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(during, 0u, "nothing arrives while the link is cut");
        NS_TEST_ASSERT_MSG_GT(after, 3u, "browsing resumes after the outage");
        std::vector<double> plts = b.pltS;
        std::sort(plts.begin(), plts.end());
        const double outagePlt = plts.empty() ? 0.0 : plts.back();
        const double normalMax = plts.size() < 2 ? 0.0 : plts[plts.size() - 2];
        // RFC 6298 5.5 doubles the timer from RTO_min = 1 s (ns-3 MinRto), so
        // the first retransmission after an outage of D seconds comes at most
        // 2 D + 1 s after it began; the rest of the page then loads as any page.
        const double bound = 2.0 * 5.0 + 1.0 + normalMax;
        std::cout << "[http-outage] outage-page PLT " << outagePlt << " s, longest other "
                  << normalMax << " s, bound " << bound << " s, pages after " << after << std::endl;
        NS_TEST_ASSERT_MSG_GT(outagePlt, 5.0, "the page hit by the outage waits it out");
        NS_TEST_ASSERT_MSG_LT(outagePlt, bound, "and recovers within the RTO backoff");
        NS_TEST_ASSERT_MSG_EQ(b.failures, 0u, "the established connection is never re-opened");
    }

    Ptr<RateErrorModel> m_em0;
    Ptr<RateErrorModel> m_em1;
    bool m_outageArmed{false};
    Time m_outageStart;
};

/// Missing peer, with TcpSocket::ConnCount = 2. ns-3 TCP ignores the RST a
/// closed port answers in SYN-SENT (reported as an open item, RFC 9293
/// 3.10.7.3) and retransmits the SYN at 3 s and 9 s (ConnTimeout 3 s,
/// doubled), then reports ConnectionFailed. (a) The web server comes up 4.5 s
/// late: the attempt opened at 0 s fails at 9 s, the client counts it, retries
/// after ConnectionRetryInterval (1 s) and browses from 10 s on. (b) Nobody ever
/// listens: a failure every 10 s (9 s of SYN timeouts plus the 1 s retry
/// wait); with the retry disabled it fails exactly once. (c) A client with no server address fails every second. None of these
/// stops the simulation (a failed connection used to be NS_FATAL_ERROR, and an
/// invalid address left the client silently in CONNECTING).
class NtnHttpMissingPeerTest : public TestCase
{
  public:
    NtnHttpMissingPeerTest()
        : TestCase("Missing peer: late server, refused port with and without retry, invalid address")
    {
    }

  private:
    void DoRun() override
    {
        RngSeedManager::ResetNextStreamIndex();
        Config::SetDefault("ns3::TcpSocket::ConnCount", UintegerValue(2));
        NodeContainer n = HttpLink("20Mbps", Ms(kLeoOwdMs));
        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(1.0)));
        ApplicationContainer apps = http.InstallUsingIpv4(n.Get(0), n.Get(1));
        http.GetServer().Start(Seconds(4.5));
        http.GetClients().Start(Seconds(0.0));
        apps.Stop(Seconds(40.0));
        Browse late;
        late.Connect(http.GetClients().Get(0));

        ThreeGppHttpSatelliteClientHelper noAddr{Address()};
        ApplicationContainer bad = noAddr.Install(n.Get(1));
        bad.Start(Seconds(0.0));
        bad.Stop(Seconds(10.0));
        Browse invalid;
        invalid.Connect(bad.Get(0));

        // Port 8080: nobody ever listens.
        ThreeGppHttpSatelliteClientHelper refusedH{Ipv4Address("10.1.1.1")};
        refusedH.SetAttribute("RemoteServerPort", UintegerValue(8080));
        ApplicationContainer refused = refusedH.Install(n.Get(1));
        refused.Start(Seconds(0.0));
        refused.Stop(Seconds(40.0));
        Browse retrying;
        retrying.Connect(refused.Get(0));
        std::vector<Time> failTimes;
        refused.Get(0)->TraceConnectWithoutContext(
            "ConnectionFailed",
            MakeBoundCallback(+[](std::vector<Time>* v, Ptr<const ThreeGppHttpSatelliteClient>) {
                                  v->push_back(Simulator::Now());
                              },
                              &failTimes));

        refusedH.SetAttribute("ConnectionRetryInterval", TimeValue(Seconds(0)));
        ApplicationContainer one = refusedH.Install(n.Get(1));
        one.Start(Seconds(0.0));
        one.Stop(Seconds(40.0));
        Browse single;
        single.Connect(one.Get(0));

        Simulator::Stop(Seconds(40.5));
        Simulator::Run();
        const auto rc = DynamicCast<ThreeGppHttpSatelliteClient>(refused.Get(0));
        const uint32_t counter = rc->GetConnectionFailures();
        Simulator::Destroy();
        Config::Reset();

        std::cout << "[http-peer] late first main " << (late.mainRxTimes.empty() ? -1.0 : late.mainRxTimes.front().GetSeconds())
                  << " s, refused failures " << retrying.failures << " at";
        for (Time t : failTimes)
        {
            std::cout << " " << t.GetSeconds();
        }
        std::cout << ", single " << single.failures << ", invalid " << invalid.failures << std::endl;

        NS_TEST_ASSERT_MSG_EQ(late.failures, 1u, "the attempt made before the server existed fails once");
        NS_TEST_ASSERT_MSG_GT(late.mainSizes.size(), 5u, "and the retry browses once it is up");
        if (!late.mainRxTimes.empty())
        {
            NS_TEST_ASSERT_MSG_GT(late.mainRxTimes.front(), Seconds(10.0), "first page after the 10 s retry");
            NS_TEST_ASSERT_MSG_LT(late.mainRxTimes.front(), Seconds(10.0 + 1.0), "and right after it");
        }
        NS_TEST_ASSERT_MSG_EQ(retrying.failures, 4u, "refused port: failures at 9, 19, 29 and 39 s, not fatal");
        NS_TEST_ASSERT_MSG_EQ(counter, retrying.failures, "counter and trace agree");
        NS_TEST_ASSERT_MSG_EQ(retrying.mainSizes.size(), 0u, "and loads nothing");
        for (std::size_t k = 0; k < failTimes.size(); ++k)
        {
            NS_TEST_ASSERT_MSG_EQ_TOL(failTimes[k].GetSeconds(), 9.0 + 10.0 * k, 1e-6,
                                      "SYN at +0, +3, +9 s, fail, wait 1 s, again");
        }
        NS_TEST_ASSERT_MSG_EQ(single.failures, 1u, "retry disabled: one failure");
        NS_TEST_ASSERT_MSG_EQ(single.mainSizes.size(), 0u, "and no page");
        NS_TEST_ASSERT_MSG_EQ(invalid.failures, 10u, "an invalid address fails once per second, no abort");
        NS_TEST_ASSERT_MSG_EQ(invalid.mainSizes.size(), 0u, "and loads nothing");
    }
};

/// Determinism. The same RngRun gives the identical PLT sequence; another run
/// gives a different one, and every PLT of both stays above one LEO round trip.
class NtnHttpDeterminismTest : public TestCase
{
  public:
    NtnHttpDeterminismTest()
        : TestCase("HTTP browsing: same RngRun identical PLTs, another run differs within bounds")
    {
    }

  private:
    Browse Run(uint32_t run)
    {
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(run);
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer n = HttpLink("20Mbps", Ms(kLeoOwdMs));
        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(1.0)));
        ApplicationContainer apps = http.InstallUsingIpv4(n.Get(0), n.Get(1));
        apps.Start(Seconds(0.0));
        apps.Stop(Seconds(60.0));
        Browse b;
        b.Connect(http.GetClients().Get(0));
        Simulator::Stop(Seconds(61.0));
        Simulator::Run();
        Simulator::Destroy();
        return b;
    }

    void DoRun() override
    {
        const Browse a = Run(1);
        const Browse b = Run(1);
        const Browse c = Run(5);
        RngSeedManager::SetRun(1);
        NS_TEST_ASSERT_MSG_GT(a.pltS.size(), 10u, "enough pages to compare");
        NS_TEST_ASSERT_MSG_EQ((a.pltS == b.pltS), true, "same RngRun: identical PLT sequence");
        NS_TEST_ASSERT_MSG_EQ((a.mainSizes == b.mainSizes), true, "same RngRun: identical objects");
        NS_TEST_ASSERT_MSG_EQ((a.pltS == c.pltS), false, "another RngRun browses other pages");
        const double rtt = 2.0 * kLeoOwdMs / 1e3;
        for (const auto* r : {&a, &c})
        {
            for (double p : r->pltS)
            {
                NS_TEST_ASSERT_MSG_GT(p, rtt, "every PLT above one LEO round trip");
                NS_TEST_ASSERT_MSG_LT(p, 30.0, "and finite on a clean 20 Mb/s link");
            }
        }
    }
};

/// Cross-module: the satellite HTTP client on a UE of the NR-NTN stack
/// (NtnRealStackHelper, nr backend, Set-1 EIRP, 600 km zenith) browsing a
/// server on the remote host: TCP over PDCP/RLC/MAC/PHY both ways and the
/// EPC. Pages complete, no connection fails, and every PLT is above the
/// 2 x 600 km / c propagation floor of the round trip.
class NtnHttpOverNrNtnTest : public TestCase
{
  public:
    NtnHttpOverNrNtnTest()
        : TestCase("Cross-module: satellite HTTP client browsing over the NR-NTN stack")
    {
    }

  private:
    void DoRun() override
    {
        RngSeedManager::ResetNextStreamIndex();
        NodeContainer sats;
        NodeContainer ues;
        sats.Create(1);
        ues.Create(1);
        auto sm = CreateObject<ConstantPositionMobilityModel>();
        sm->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 600e3));
        sats.Get(0)->AggregateObject(sm);
        auto um = CreateObject<ConstantPositionMobilityModel>();
        um->SetPosition(NtnRealStackHelper::GeodeticToEcef(40.0, -3.0, 0.0));
        ues.Get(0)->AggregateObject(um);
        NtnRealStackHelper rs;
        rs.SetRadioBackend(NtnRealStackHelper::RadioBackend::Nr);
        rs.SetNumerology(1);
        rs.SetSimTime(Seconds(8.0));
        rs.SetOutputDir(CreateTempDirFilename("http-nr"));
        rs.SetSatEirpDensityDbwMhz(NtnRealStackHelper::kTr38821Set1SBandEirpDensityDbwMhz);
        rs.Build(sats, ues);

        ThreeGppHttpHelper http;
        http.SetVariablesAttribute("ReadingTimeMean", TimeValue(Seconds(0.5)));
        ApplicationContainer apps = http.InstallUsingIpv4(rs.GetRemoteHost(), ues.Get(0));
        apps.Start(Seconds(0.5));
        apps.Stop(Seconds(8.0));
        Browse b;
        b.Connect(http.GetClients().Get(0));
        Simulator::Stop(Seconds(8.0));
        Simulator::Run();
        Simulator::Destroy();

        std::cout << "[http-nr] pages " << b.pltS.size() << " main " << b.mainSizes.size()
                  << " mean PLT " << Mean(b.pltS) << " s" << std::endl;
        NS_TEST_ASSERT_MSG_EQ(b.failures, 0u, "the TCP connection opens across the NR-NTN stack");
        NS_TEST_ASSERT_MSG_GT(b.pltS.size(), 2u, "several pages complete");
        const double floorS = 2.0 * 600e3 / 299792458.0;
        for (double p : b.pltS)
        {
            NS_TEST_ASSERT_MSG_GT(p, floorS, "PLT above the 600 km round-trip propagation floor");
        }
    }
};

class NtnHttpSatelliteTestSuite : public TestSuite
{
  public:
    NtnHttpSatelliteTestSuite()
        : TestSuite("ntn-http-satellite", Type::SYSTEM)
    {
        AddTestCase(new NtnHttpSatelliteBrowsingTestCase, Duration::QUICK);
        AddTestCase(new NtnHttpTr36814ConformanceTest, Duration::QUICK);
        AddTestCase(new NtnHttpRttScalingTest, Duration::QUICK);
        AddTestCase(new NtnHttpOutageRecoveryTest, Duration::QUICK);
        AddTestCase(new NtnHttpMissingPeerTest, Duration::QUICK);
        AddTestCase(new NtnHttpDeterminismTest, Duration::QUICK);
        AddTestCase(new NtnHttpOverNrNtnTest, Duration::QUICK);
    }
};

static NtnHttpSatelliteTestSuite g_ntnHttpSatelliteTestSuite;
