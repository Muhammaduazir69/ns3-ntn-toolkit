// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-attach-window.cc
 *
 * @brief Test suite `nr-attach-window`: exercises the NrHelper AttachWindow attachment-window
 * policy added for TR 36.839-style handover deployments.
 *
 * The case stands up a minimal NR + EPC scenario with a single gNB and several UEs and checks
 * that spreading the UE attachment over the NrHelper AttachWindow spreads the RRC
 * ConnectionEstablished events in time relative to attaching all UEs at once.
 */

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrAttachWindowTest");

/**
 * @ingroup nr-test
 *
 * @brief Verifies that the NrHelper AttachWindow policy spreads UE attachment in time.
 *
 * A single gNB serves several UEs, all attached through the ordinary NrHelper::AttachToGnb() call.
 * With a zero window every UE attaches at once; with a non-zero window the helper defers the k-th
 * attachment by AttachWindow times the radical inverse of k in base 2. The test runs both
 * configurations, records the RRC ConnectionEstablished time of each UE, and asserts that the
 * non-zero window produces a strictly larger spread (max - min) of establishment times.
 */
class NrAttachWindowTestCase : public TestCase
{
  public:
    NrAttachWindowTestCase();
    ~NrAttachWindowTestCase() override = default;

  private:
    void DoRun() override;

    /**
     * Run one scenario and return the ConnectionEstablished times.
     * @param attachWindow attachment window (zero disables staggering)
     * @return one establishment time per UE that connected
     */
    std::vector<Time> RunScenario(Time attachWindow);

    /**
     * ConnectionEstablished trace sink.
     * @param imsi UE IMSI
     * @param cellId serving cell id
     * @param rnti UE RNTI
     */
    void ConnectionEstablishedCallback(uint64_t imsi, uint16_t cellId, uint16_t rnti);

    static constexpr uint32_t m_nUes{4}; ///< number of UEs in the scenario
    std::vector<Time> m_establishTimes;  ///< establishment times collected for the current run
};

NrAttachWindowTestCase::NrAttachWindowTestCase()
    : TestCase("NrHelper AttachWindow spreads UE ConnectionEstablished times")
{
}

void
NrAttachWindowTestCase::ConnectionEstablishedCallback(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    m_establishTimes.push_back(Simulator::Now());
}

std::vector<Time>
NrAttachWindowTestCase::RunScenario(Time attachWindow)
{
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    m_establishTimes.clear();

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(m_nUes);

    // gNB at the origin; UEs spread on a short line, all well within coverage.
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0, 0, 10));
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPos);
    gnbMobility.Install(gnbNodes);

    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < m_nUes; i++)
    {
        uePos->Add(Vector(10.0 + 5.0 * i, 0, 1.5));
    }
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.SetPositionAllocator(uePos);
    ueMobility.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetAttribute("AttachWindow", TimeValue(attachWindow));
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    // Connect the ConnectionEstablished trace on each UE RRC before attaching.
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        Ptr<NrUeNetDevice> ueNetDev = DynamicCast<NrUeNetDevice>(ueDevices.Get(i));
        ueNetDev->GetRrc()->TraceConnectWithoutContext(
            "ConnectionEstablished",
            MakeCallback(&NrAttachWindowTestCase::ConnectionEstablishedCallback, this));
    }

    // Attach every UE the ordinary way. Any staggering is the helper's job: that is exactly
    // what the AttachWindow attribute set above is being tested for.
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueDevices.Get(i), gnbDevices.Get(0));
    }

    Simulator::Stop(Seconds(1));
    Simulator::Run();

    std::vector<Time> times = m_establishTimes;
    Simulator::Destroy();
    return times;
}

void
NrAttachWindowTestCase::DoRun()
{
    std::vector<Time> noWindow = RunScenario(Seconds(0));
    std::vector<Time> withWindow = RunScenario(MilliSeconds(200));

    NS_TEST_ASSERT_MSG_EQ(noWindow.size(), m_nUes, "not all UEs connected with AttachWindow = 0");
    NS_TEST_ASSERT_MSG_EQ(withWindow.size(),
                          m_nUes,
                          "not all UEs connected with AttachWindow = 200ms");

    const auto spread = [](const std::vector<Time>& v) {
        Time lo = v.front();
        Time hi = v.front();
        for (const Time& t : v)
        {
            lo = std::min(lo, t);
            hi = std::max(hi, t);
        }
        return hi - lo;
    };

    Time spreadNoWindow = spread(noWindow);
    Time spreadWithWindow = spread(withWindow);

    // A non-zero window must not shrink the spread, and must produce a clearly larger spread
    // than the (near-synchronous) zero-window baseline.
    NS_TEST_ASSERT_MSG_GT_OR_EQ(spreadWithWindow.GetNanoSeconds(),
                                spreadNoWindow.GetNanoSeconds(),
                                "AttachWindow reduced the ConnectionEstablished spread");
    NS_TEST_ASSERT_MSG_GT(spreadWithWindow.GetNanoSeconds(),
                          MilliSeconds(50).GetNanoSeconds(),
                          "AttachWindow did not spread ConnectionEstablished times");
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite `nr-attach-window`.
 */
class NrAttachWindowTestSuite : public TestSuite
{
  public:
    NrAttachWindowTestSuite()
        : TestSuite("nr-attach-window", Type::SYSTEM)
    {
        AddTestCase(new NrAttachWindowTestCase(), Duration::QUICK);
    }
};

/// Static instance of the test suite.
static NrAttachWindowTestSuite g_nrAttachWindowTestSuite;
