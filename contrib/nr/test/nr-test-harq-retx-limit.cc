// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/**
 * @file nr-test-harq-retx-limit.cc
 * @ingroup test
 *
 * @brief End-to-end tests for MaxHarqReTx.
 *
 * Uses fixed one-gNB/one-UE DL operating points to check HARQ-chain length,
 * application delivery, and the default retransmission limit.
 */

using namespace ns3;

namespace
{

constexpr double CENTRAL_FREQ = 3.5e9; //!< Hz
constexpr uint32_t BANDWIDTH = 20000000;
constexpr auto ERROR_MODEL = "ns3::NrEesmIrT1";

const Time APP_START = MilliSeconds(200);
const Time APP_STOP = MilliSeconds(500);
const Time SIM_STOP = MilliSeconds(500);
constexpr uint16_t DL_PORT = 1234;

constexpr double BYTES_RATIO = 10.0;
constexpr double CORRUPT_FRACTION = 0.90;
constexpr std::size_t MIN_CHAINS = 50;

/**
 * @brief Operating point used by the HARQ retransmission tests.
 */
struct OpPoint
{
    uint8_t mcs;
    double txPowerDbm;
    double distanceM;
    uint32_t intervalUs;
    uint32_t pktSize;
};

// Calibration with MaxHarqReTx=6: 0% decode at attempt 1, 2% at attempt 2,
// and 98% at attempt 3.
const OpPoint P2_POINT{/*mcs*/ 9,
                       /*txPowerDbm*/ 4.0,
                       /*distanceM*/ 145.0,
                       /*intervalUs*/ 2000,
                       /*pktSize*/ 100};

// Calibration with MaxHarqReTx=6: 0% decode at attempt 1, 0.5% at attempt 2,
// 0.3% at attempt 3, 0% at attempts 4-5, 73.2% at attempt 6, 15.5% at
// attempt 7, and 10.6% remain corrupt. P5 is therefore the harder operating
// point despite its higher transmit power: it carries 20x the load of P2
// (100 us instead of 2000 us packet interval), so its difficulty comes from
// the traffic rather than from the radio conditions.
const OpPoint P5_POINT{/*mcs*/ 9,
                       /*txPowerDbm*/ 4.5,
                       /*distanceM*/ 145.0,
                       /*intervalUs*/ 100,
                       /*pktSize*/ 100};

/**
 * @brief HARQ reception information collected from the PHY trace.
 */
struct TraceEntry
{
    uint16_t rnti;
    uint8_t harqProcessId;
    uint8_t rv;
    bool corrupt;
};

/**
 * @brief Receptions belonging to one HARQ transport-block transmission chain.
 */
struct Chain
{
    std::vector<uint8_t> rvs;
    bool decoded{false};
};

/**
 * @brief Collect PHY reception trace entries for later HARQ-chain analysis.
 */
struct TraceCollector
{
    std::vector<TraceEntry> entries;

    void Rx(RxPacketTraceParams p)
    {
        entries.push_back({p.m_rnti, p.m_harqProcessId, p.m_rv, p.m_corrupt});
    }
};

/**
 * @brief Group receptions into HARQ chains by RNTI and HARQ process.
 */
std::vector<Chain>
GroupChains(const std::vector<TraceEntry>& entries)
{
    std::map<std::pair<uint16_t, uint8_t>, std::vector<const TraceEntry*>> byProcess;
    for (const auto& e : entries)
    {
        byProcess[{e.rnti, e.harqProcessId}].push_back(&e);
    }

    std::vector<Chain> chains;
    for (const auto& item : byProcess)
    {
        const auto& procEntries = item.second;
        Chain current;
        bool open = false;

        for (const auto* e : procEntries)
        {
            if (e->rv == 0)
            {
                if (open)
                {
                    chains.push_back(current);
                }
                current = Chain{};
                open = true;
            }

            if (!open)
            {
                continue;
            }

            current.rvs.push_back(e->rv);

            if (!e->corrupt)
            {
                current.decoded = true;
                chains.push_back(current);
                current = Chain{};
                open = false;
            }
        }

        if (open)
        {
            chains.push_back(current);
        }
    }

    return chains;
}

/**
 * @brief Results collected from one end-to-end HARQ scenario run.
 */
struct RunResult
{
    uint64_t rxBytes{0};
    std::vector<Chain> chains;

    size_t maxChainLen() const
    {
        size_t m = 0;
        for (const auto& c : chains)
        {
            m = std::max(m, c.rvs.size());
        }
        return m;
    }

    double corruptFraction() const
    {
        if (chains.empty())
        {
            return 1.0;
        }
        size_t corrupt = 0;
        for (const auto& c : chains)
        {
            if (!c.decoded)
            {
                ++corrupt;
            }
        }
        return static_cast<double>(corrupt) / static_cast<double>(chains.size());
    }
};

/**
 * @brief Run the one-gNB/one-UE DL scenario.
 */
RunResult
RunScenario(const OpPoint& point, std::optional<uint8_t> maxHarqReTx)
{
    Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0.0, 0.0, 10.0));
    positions->Add(Vector(point.distanceM, 0.0, 1.5));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bfHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(bfHelper);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(CENTRAL_FREQ, BANDWIDTH, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    bfHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetDlErrorModel(ERROR_MODEL);
    nrHelper->SetUlErrorModel(ERROR_MODEL);
    nrHelper->SetGnbDlAmcAttribute("ErrorModelType", StringValue(ERROR_MODEL));
    nrHelper->SetGnbUlAmcAttribute("ErrorModelType", StringValue(ERROR_MODEL));

    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(0)));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaRR"));
    if (maxHarqReTx.has_value())
    {
        nrHelper->SetSchedulerAttribute("MaxHarqReTx", UintegerValue(*maxHarqReTx));
    }
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(point.mcs));
    nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(point.mcs));

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(3));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("TxPower", DoubleValue(point.txPowerDbm));

    nrHelper->AssignStreams({.gnbDevs = gnbDevs, .ueDevs = ueDevs});

    // internet / remote host
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // DL UDP flow
    UdpServerHelper dlPacketSinkHelper(DL_PORT);
    ApplicationContainer serverAppsDl = dlPacketSinkHelper.Install(ueNodes);

    UdpClientHelper dlClient(ueIfaces.GetAddress(0), DL_PORT);
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(point.pktSize));
    dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(point.intervalUs)));
    ApplicationContainer clientAppsDl = dlClient.Install(remoteHost);

    serverAppsDl.Start(APP_START);
    clientAppsDl.Start(APP_START);
    serverAppsDl.Stop(APP_STOP);
    clientAppsDl.Stop(APP_STOP);

    TraceCollector collector;
    Ptr<NrSpectrumPhy> ueSpectrumPhy =
        DynamicCast<NrUeNetDevice>(ueDevs.Get(0))->GetPhy(0)->GetSpectrumPhy();
    ueSpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                              MakeCallback(&TraceCollector::Rx, &collector));

    Simulator::Stop(SIM_STOP);
    Simulator::Run();

    RunResult result;
    result.chains = GroupChains(collector.entries);
    Ptr<UdpServer> server = DynamicCast<UdpServer>(serverAppsDl.Get(0));
    result.rxBytes = server->GetReceived() * static_cast<uint64_t>(point.pktSize);

    Simulator::Destroy();
    return result;
}

} // namespace

/**
 * @ingroup test
 * @brief Test the P2 operating point.
 */
class NrHarqP2Case : public TestCase
{
  public:
    NrHarqP2Case()
        : TestCase("P2: chain limit for N=1..6 and MaxHarqReTx=3 delivers >=10x bytes vs =1")
    {
    }

  private:
    void DoRun() override
    {
        std::vector<RunResult> byN(7); // index 1..6
        for (uint8_t n = 1; n <= 6; ++n)
        {
            byN[n] = RunScenario(P2_POINT, n);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(byN[n].chains.size(),
                                        MIN_CHAINS,
                                        "P2 N=" + std::to_string(n) +
                                            ": fewer than 50 HARQ chains observed");

            NS_TEST_ASSERT_MSG_LT_OR_EQ(byN[n].maxChainLen(),
                                        static_cast<size_t>(n) + 1,
                                        "P2 N=" + std::to_string(n) +
                                            ": a chain exceeded N + 1 transmissions");
        }

        const auto base = static_cast<double>(byN[1].rxBytes);
        NS_TEST_ASSERT_MSG_GT(byN[3].rxBytes, 0, "P2 N=3: delivered no bytes");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(
            static_cast<double>(byN[3].rxBytes),
            BYTES_RATIO * base,
            "P2 N=3: did not deliver at least 10x the bytes of MaxHarqReTx=1");
    }
};

/**
 * @ingroup test
 * @brief Test the P5 operating point.
 */
class NrHarqP5Case : public TestCase
{
  public:
    NrHarqP5Case()
        : TestCase("P5: chain limit; 3,4 >=90% corrupt; late decodes at 5,6")
    {
    }

  private:
    void DoRun() override
    {
        std::vector<RunResult> byN(7); // index 1..6
        for (uint8_t n = 3; n <= 6; ++n)
        {
            byN[n] = RunScenario(P5_POINT, n);
            NS_TEST_ASSERT_MSG_GT_OR_EQ(byN[n].chains.size(),
                                        MIN_CHAINS,
                                        "P5 N=" + std::to_string(n) +
                                            ": fewer than 50 HARQ chains observed");

            NS_TEST_ASSERT_MSG_LT_OR_EQ(byN[n].maxChainLen(),
                                        static_cast<size_t>(n) + 1,
                                        "P5 N=" + std::to_string(n) +
                                            ": a chain exceeded N + 1 transmissions");
        }

        NS_TEST_ASSERT_MSG_GT_OR_EQ(byN[3].corruptFraction(),
                                    CORRUPT_FRACTION,
                                    "P5 N=3: fewer than 90% of chains are corrupt");
        NS_TEST_ASSERT_MSG_GT_OR_EQ(byN[4].corruptFraction(),
                                    CORRUPT_FRACTION,
                                    "P5 N=4: fewer than 90% of chains are corrupt");

        const auto baseBytes = std::max(byN[3].rxBytes, byN[4].rxBytes);

        NS_TEST_ASSERT_MSG_GT(baseBytes,
                              0,
                              "P5: N=3/4 application-byte baseline is zero; "
                              "the 10x comparison would be vacuous");

        NS_TEST_ASSERT_MSG_GT_OR_EQ(
            static_cast<double>(byN[5].rxBytes),
            BYTES_RATIO * static_cast<double>(baseBytes),
            "P5 N=5: did not deliver at least 10x the N=3/4 application-byte baseline");

        NS_TEST_ASSERT_MSG_GT_OR_EQ(
            static_cast<double>(byN[6].rxBytes),
            BYTES_RATIO * static_cast<double>(baseBytes),
            "P5 N=6: did not deliver at least 10x the N=3/4 application-byte baseline");

        NS_TEST_ASSERT_MSG_GT_OR_EQ(byN[6].rxBytes,
                                    byN[5].rxBytes,
                                    "P5 N=6 delivered fewer bytes than N=5");

        bool sawLengthFour = false;
        for (const auto& c : byN[3].chains)
        {
            NS_TEST_ASSERT_MSG_LT_OR_EQ(c.rvs.size(),
                                        static_cast<size_t>(4),
                                        "P5 N=3: a chain has more than 4 entries");

            if (c.rvs.size() == 4)
            {
                sawLengthFour = true;
            }

            for (size_t i = 0; i < c.rvs.size(); ++i)
            {
                NS_TEST_ASSERT_MSG_EQ(static_cast<size_t>(c.rvs[i]),
                                      i,
                                      "P5 N=3: rv values are not in transmission order");
            }
        }

        NS_TEST_ASSERT_MSG_EQ(sawLengthFour, true, "P5 N=3: no chain reached four transmissions");
    }
};

/**
 * @ingroup test
 * @brief Verify the default value of MaxHarqReTx.
 */
class NrHarqDefaultValueCase : public TestCase
{
  public:
    NrHarqDefaultValueCase()
        : TestCase("default MaxHarqReTx is 3")
    {
    }

  private:
    void DoRun() override
    {
        const TypeId schedTid = TypeId::LookupByName("ns3::NrMacSchedulerNs3");
        std::string defaultRetxStr;
        bool found = false;
        for (std::size_t i = 0; i < schedTid.GetAttributeN(); ++i)
        {
            const TypeId::AttributeInformation& info = schedTid.GetAttribute(i);
            if (info.name == "MaxHarqReTx")
            {
                defaultRetxStr = info.originalInitialValue->SerializeToString(info.checker);
                found = true;
                break;
            }
        }
        NS_TEST_ASSERT_MSG_EQ(found, true, "MaxHarqReTx attribute not found on NrMacSchedulerNs3");
        NS_TEST_ASSERT_MSG_EQ(defaultRetxStr,
                              "3",
                              "MaxHarqReTx default changed from 3; update this test and the docs");
    }
};

/**
 * @ingroup test
 * @brief End-to-end test suite for the MaxHarqReTx attribute.
 */
class NrHarqRetxLimitSuite : public TestSuite
{
  public:
    NrHarqRetxLimitSuite()
        : TestSuite("nr-test-harq-retx-limit", Type::SYSTEM)
    {
        AddTestCase(new NrHarqP2Case(), Duration::QUICK);
        AddTestCase(new NrHarqP5Case(), Duration::QUICK);
        AddTestCase(new NrHarqDefaultValueCase(), Duration::QUICK);
    }
};

static NrHarqRetxLimitSuite g_nrHarqRetxLimitSuite; //!< Static test suite instance
