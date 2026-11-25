// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <algorithm>
#include <set>
#include <utility>
#include <vector>

/**
 * @file nr-test-tdd-ul-dl-regression.cc
 * @ingroup test
 *
 * @brief Regression tests of the UL and DL service of a cell with an all-flexible TDD
 * pattern, for issue #298.
 */
namespace ns3
{

namespace
{

/// Scenario of a run
struct Scenario
{
    double bandwidth{20e6};                              ///< Bandwidth, in Hz
    uint16_t numerology{1};                              ///< Numerology
    uint32_t numUes{1};                                  ///< Number of UEs
    std::string scheduler{"ns3::NrMacSchedulerOfdmaRR"}; ///< Scheduler
    uint32_t ulPacketSize{0};         ///< UL packet size, in bytes; 0 for no UL flow
    Time ulInterval{MilliSeconds(1)}; ///< UL packet interval
    uint32_t dlPacketSize{0};         ///< DL packet size, in bytes; 0 for no DL flow
    Time dlInterval{MilliSeconds(1)}; ///< DL packet interval
    Time bsrTimer{MilliSeconds(10)};  ///< RLC UM BufferStatusReportTimer
    Time duration{Seconds(1)};        ///< Duration of the traffic
    int fixedDlMcs{-1};               ///< Fixed DL MCS, if not negative
};

/// Observations of a run
struct Result
{
    std::vector<double> ulDelaysMs; ///< Delay of the received UL packets
    std::vector<double> dlDelaysMs; ///< Delay of the received DL packets
    uint64_t dlSent{0};             ///< DL packets expected from the flow
    uint64_t dlTbs{0};              ///< DL TBs scheduled, including retransmissions
    uint64_t dlAcks{0};             ///< DL HARQ ACKs
    uint64_t dlNacks{0};            ///< DL HARQ NACKs
    uint64_t dlLostRetx{0};         ///< DL NACKs whose process moved on without a retransmission
};

/// Traffic start, leaving time for the UEs to connect
const Time TRAFFIC_START = MilliSeconds(400);

/**
 * @param values Samples, which get sorted.
 * @param quantile Quantile, between 0 and 1.
 * @return the quantile of the samples, or 0 without samples
 */
double
Quantile(std::vector<double> values, double quantile)
{
    if (values.empty())
    {
        return 0;
    }
    std::sort(values.begin(), values.end());
    return values[static_cast<std::size_t>(quantile * (values.size() - 1))];
}

/**
 * Create the devices of a cell with UEs close to the gNB, so that the link has no errors
 * of its own.
 *
 * @param scenario Scenario of the run.
 * @param nrHelper NR helper.
 * @param gnbNodes Output gNB node.
 * @param ueNodes Output UE nodes.
 * @param gnbDevs Output gNB device.
 * @param ueDevs Output UE devices.
 */
void
InstallCell(const Scenario& scenario,
            Ptr<NrHelper> nrHelper,
            NodeContainer& gnbNodes,
            NodeContainer& ueNodes,
            NetDeviceContainer& gnbDevs,
            NetDeviceContainer& ueDevs)
{
    // A single-beam, deterministic link
    auto beamformingHelper = CreateObject<IdealBeamformingHelper>();
    beamformingHelper->SetAttribute("BeamformingMethod",
                                    TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetBeamformingHelper(beamformingHelper);

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, scenario.bandwidth, 1);
    auto band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    // All UEs at the same distance of the gNB
    gnbNodes.Create(1);
    ueNodes.Create(scenario.numUes);
    MobilityHelper mobility;
    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    for (uint32_t i = 0; i < scenario.numUes; ++i)
    {
        const double angle = 2 * M_PI * i / scenario.numUes;
        positions->Add(Vector(15 * std::cos(angle), 15 * std::sin(angle), 1.5));
    }
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(scenario.numerology));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scenario.scheduler));
    if (scenario.fixedDlMcs >= 0)
    {
        nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(scenario.fixedDlMcs));
    }

    gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
}

/**
 * Run UDP flows between a remote host and the UEs of a cell.
 *
 * @param scenario Scenario of the run.
 * @return the observations of the run
 */
Result
RunUdpScenario(const Scenario& scenario)
{
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    Config::SetDefault("ns3::NrRlcUm::BufferStatusReportTimer", TimeValue(scenario.bsrTimer));

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    NetDeviceContainer gnbDevs;
    NetDeviceContainer ueDevs;
    InstallCell(scenario, nrHelper, gnbNodes, ueNodes, gnbDevs, ueDevs);

    // Core network with a remote host behind an ideal link
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    auto remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(ueNodes);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0)));
    auto internetDevices = p2ph.Install(epcHelper->GetPgwNode(), remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    auto internetIfaces = ipv4h.Assign(internetDevices);
    Ipv4StaticRoutingHelper routingHelper;
    routingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>())
        ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    auto ueIfaces = epcHelper->AssignUeIpv4Address(ueDevs);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    Result result;
    const auto recordDelay = [](std::vector<double>& delays) {
        return Callback<void, Ptr<const Packet>, const Address&, const Address&>(
            [&delays](Ptr<const Packet> packet, const Address&, const Address&) {
                SeqTsHeader header;
                packet->Copy()->RemoveHeader(header);
                delays.push_back((Simulator::Now() - header.GetTs()).GetSeconds() * 1e3);
            });
    };

    // Each flow stops before the end of the simulation, so that its last packets arrive
    const Time stop = TRAFFIC_START + scenario.duration;
    uint16_t port = 1000;
    if (scenario.ulPacketSize > 0)
    {
        UdpServerHelper server(port);
        auto serverApps = server.Install(remoteHost);
        serverApps.Get(0)->TraceConnectWithoutContext("RxWithAddresses",
                                                      recordDelay(result.ulDelaysMs));
        UdpClientHelper client(internetIfaces.GetAddress(1), port);
        client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        client.SetAttribute("PacketSize", UintegerValue(scenario.ulPacketSize));
        client.SetAttribute("Interval", TimeValue(scenario.ulInterval));
        auto clientApps = client.Install(ueNodes);
        clientApps.Start(TRAFFIC_START);
        clientApps.Stop(stop);
    }
    if (scenario.dlPacketSize > 0)
    {
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            ++port;
            UdpServerHelper server(port);
            auto serverApps = server.Install(ueNodes.Get(i));
            serverApps.Get(0)->TraceConnectWithoutContext("RxWithAddresses",
                                                          recordDelay(result.dlDelaysMs));
            UdpClientHelper client(ueIfaces.GetAddress(i), port);
            client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            client.SetAttribute("PacketSize", UintegerValue(scenario.dlPacketSize));
            client.SetAttribute("Interval", TimeValue(scenario.dlInterval));
            auto clientApps = client.Install(remoteHost);
            clientApps.Start(TRAFFIC_START);
            clientApps.Stop(stop);
        }
        result.dlSent = ueNodes.GetN() * (scenario.duration / scenario.dlInterval).GetHigh();
    }

    NrHelper::GetGnbMac(gnbDevs.Get(0), 0)
        ->TraceConnectWithoutContext("DlScheduling",
                                     Callback<void, NrSchedulingCallbackInfo>(
                                         [&result](NrSchedulingCallbackInfo) { result.dlTbs++; }));
    std::set<std::pair<uint16_t, uint8_t>> pendingNacks;
    NrHelper::GetGnbMac(gnbDevs.Get(0), 0)
        ->TraceConnectWithoutContext(
            "DlHarqFeedback",
            Callback<void, const DlHarqInfo&>([&result, &pendingNacks](const DlHarqInfo& info) {
                info.IsReceivedOk() ? result.dlAcks++ : result.dlNacks++;
                // A process with a NACK must retransmit before reporting a new TB
                const auto key = std::make_pair(info.m_rnti, info.m_harqProcessId);
                const auto nack = pendingNacks.find(key);
                if (nack != pendingNacks.end())
                {
                    result.dlLostRetx += info.m_numRetx == 0 ? 1 : 0;
                    pendingNacks.erase(nack);
                }
                if (!info.IsReceivedOk())
                {
                    pendingNacks.insert(key);
                }
            }));

    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDevs});

    Simulator::Stop(stop + MilliSeconds(200));
    Simulator::Run();
    Simulator::Destroy();
    return result;
}

} // namespace

/**
 * @ingroup test
 * @brief Issue #298: the RLC bytes of a segmented UL SDU must not wait for the BSR timer.
 *
 * Each isolated SDU starts from an empty buffer and spans several grants. If the MAC drains
 * the RLC, the BSR timer never matters, so the UL delay must not depend on it.
 */
class NrUlStrandedRlcBytesTestCase : public TestCase
{
  public:
    NrUlStrandedRlcBytesTestCase()
        : TestCase("#298: UL delay of segmented SDUs does not follow the RLC BSR timer")
    {
    }

  private:
    void DoRun() override
    {
        // Isolated packets, on a timer short and another one long enough to tell them apart
        Scenario scenario;
        scenario.ulPacketSize = 1700;
        scenario.ulInterval = MilliSeconds(100);
        scenario.duration = Seconds(2);
        scenario.bsrTimer = MilliSeconds(10);
        const auto shortTimer = RunUdpScenario(scenario);
        scenario.bsrTimer = MilliSeconds(40);
        const auto longTimer = RunUdpScenario(scenario);

        // All packets must arrive, with a median delay independent of the timer
        NS_TEST_ASSERT_MSG_EQ(shortTimer.ulDelaysMs.size(), 20, "UL packets lost");
        NS_TEST_ASSERT_MSG_EQ(longTimer.ulDelaysMs.size(), 20, "UL packets lost");
        const double shortDelay = Quantile(shortTimer.ulDelaysMs, 0.5);
        const double longDelay = Quantile(longTimer.ulDelaysMs, 0.5);
        NS_TEST_EXPECT_MSG_LT(std::abs(longDelay - shortDelay),
                              3.0,
                              "The UL delay follows the RLC BSR timer: stranded bytes");
    }
};

/**
 * @ingroup test
 * @brief A DL HARQ retransmission must survive a slot without DL symbols.
 *
 * A heavy UL takes every symbol of many slots, so the NACKs of a high MCS, which fails now and
 * then, reach the scheduler in such slots, and can wait for symbols for several slots. The
 * retransmission must wait for a later slot instead of being forgotten or expiring.
 */
class NrDlHarqRetxNoSymbolsTestCase : public TestCase
{
  public:
    NrDlHarqRetxNoSymbolsTestCase()
        : TestCase("DL HARQ retransmissions survive slots without DL symbols")
    {
    }

  private:
    void DoRun() override
    {
        Scenario scenario;
        scenario.bandwidth = 100e6;
        scenario.numerology = 2;
        scenario.numUes = 4;
        scenario.ulPacketSize = 1250;
        scenario.ulInterval = MicroSeconds(350);
        scenario.dlPacketSize = 1250;
        scenario.dlInterval = MilliSeconds(1);
        scenario.fixedDlMcs = 28;
        const auto result = RunUdpScenario(scenario);

        NS_TEST_ASSERT_MSG_GT(result.dlNacks, 0, "The scenario must have DL NACKs");
        NS_TEST_EXPECT_MSG_EQ(result.dlLostRetx, 0, "DL HARQ retransmissions were forgotten");
    }
};

/**
 * @ingroup test
 * @brief Test suite for an issue of the UL and DL service on an all-flexible TDD pattern.
 *
 * There is a suite per issue, since a suite stops at its first failing test case.
 */
class NrTddUlDlRegressionTestSuite : public TestSuite
{
  public:
    /**
     * Constructor.
     *
     * @param name Name of the suite.
     * @param testCase Test case of the suite.
     */
    NrTddUlDlRegressionTestSuite(const std::string& name, TestCase* testCase)
        : TestSuite(name, Type::SYSTEM)
    {
        AddTestCase(testCase, Duration::QUICK);
    }
};

/// Test suite of the DL HARQ retransmissions in slots without DL symbols
static NrTddUlDlRegressionTestSuite g_nrDlHarqRetxNoSymbolsTestSuite(
    "nr-dl-harq-retx-no-symbols",
    new NrDlHarqRetxNoSymbolsTestCase());
/// Test suite of issue #298
static NrTddUlDlRegressionTestSuite g_nrUlStrandedRlcBytesTestSuite(
    "nr-ul-stranded-rlc-bytes",
    new NrUlStrandedRlcBytesTestCase());

} // namespace ns3
