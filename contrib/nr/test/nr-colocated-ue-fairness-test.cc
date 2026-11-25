// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/test.h"

/**
 * @file nr-colocated-ue-fairness-test.cc
 * @ingroup tests
 * @brief Co-located UEs under a round-robin scheduler
 */

namespace ns3
{

/**
 * @ingroup tests
 * @brief Two co-located UEs perceive the same channel and obtain the same throughput.
 *
 * Two UEs placed at the exact same position under one gNB receive identical downlink
 * CBR traffic starting at the same time, served by the TDMA round-robin scheduler.
 * With InterUeSpatialConsistency enabled on the channel condition model, which the pathloss
 * and fading models consult, the UEs sample the same LOS-state and shadow-fading random fields and
 * the same spatially consistent fading parameters, so their reported DL data SINR must be
 * identical, and the round-robin scheduler must then serve them the same number of
 * packets, up to the last scheduling round. Without spatial consistency the two UEs draw
 * independent channels, so their SINR must differ.
 *
 * Every check holds whatever the random draws are: the data error model is disabled so that
 * the throughput is a deterministic function of the channel, the co-located UEs are compared
 * only with each other, and the no-sc case asserts a difference that independent continuous
 * draws produce with certainty.
 */
class NrColocatedUeFairnessTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name Name of the test case.
     * @param interUeSpatialConsistency Enable drop-based spatial consistency.
     * @param ueDistance The gNB-UE 2D distance in meters.
     */
    NrColocatedUeFairnessTestCase(const std::string& name,
                                  bool interUeSpatialConsistency,
                                  double ueDistance)
        : TestCase(name),
          m_sc(interUeSpatialConsistency),
          m_ueDistance(ueDistance)
    {
    }

  private:
    void DoRun() override;

    /**
     * Accumulate a DL data SINR report.
     *
     * @param cellId Serving cell.
     * @param rnti RNTI of the reporting UE.
     * @param sinr Effective SINR, linear.
     * @param bwpId Bandwidth part of the report.
     */
    void SinrCb(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId);

    bool m_sc;                                             ///< Spatial consistency enabled
    double m_ueDistance;                                   ///< gNB-UE 2D distance in meters
    std::map<uint16_t, std::vector<double>> m_sinrsByRnti; ///< SINR samples per RNTI
};

void
NrColocatedUeFairnessTestCase::SinrCb(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    m_sinrsByRnti[rnti].push_back(sinr);
}

void
NrColocatedUeFairnessTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    const Time simTime = MilliSeconds(800);
    const Time appStart = MilliSeconds(400);
    const uint32_t packetSize = 1252;
    const uint32_t lambda = 4000; // packets/s per UE

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    // Each UE decodes its transport blocks with its own random stream, so even an identical
    // SINR yields different HARQ outcomes. Decoding deterministically keeps the throughput a
    // function of the channel alone.
    Config::SetDefault("ns3::NrSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(2);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0, 0, 25.0));
    pos->Add(Vector(m_ueDistance, 0, 1.5));
    pos->Add(Vector(m_ueDistance, 0, 1.5)); // co-located with the first UE
    mob.SetPositionAllocator(pos);
    mob.Install(gnbNodes);
    mob.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetChannelConditionModelAttribute("InterUeSpatialConsistency",
                                                     BooleanValue(m_sc));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    beamHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    NrHelper::GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(1));
    NrHelper::GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(30.0));

    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    const uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    UdpServerHelper dlSink(dlPort);
    serverApps.Add(dlSink.Install(ueNodes));

    UdpClientHelper dlClient;
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(packetSize));
    dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda)));
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        dlClient.SetAttribute(
            "Remote",
            AddressValue(addressUtils::ConvertToSocketAddress(ueIpIface.GetAddress(i), dlPort)));
        clientApps.Add(dlClient.Install(remoteHost));
    }
    serverApps.Start(appStart);
    clientApps.Start(appStart);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    for (uint32_t i = 0; i < ueNetDev.GetN(); i++)
    {
        NrHelper::GetUePhy(ueNetDev.Get(i), 0)
            ->TraceConnectWithoutContext(
                "DlDataSinr",
                MakeCallback(&NrColocatedUeFairnessTestCase::SinrCb, this));
    }

    Simulator::Stop(simTime);
    Simulator::Run();

    uint64_t rx0 = DynamicCast<UdpServer>(serverApps.Get(0))->GetReceived();
    uint64_t rx1 = DynamicCast<UdpServer>(serverApps.Get(1))->GetReceived();
    Simulator::Destroy();

    NS_TEST_ASSERT_MSG_EQ(m_sinrsByRnti.size(), 2, "Expected SINR reports from both UEs");
    const auto& s1 = m_sinrsByRnti.begin()->second;
    const auto& s2 = std::next(m_sinrsByRnti.begin())->second;
    NS_TEST_ASSERT_MSG_GT(s1.size(), 0, "Expected SINR reports from the first UE");
    NS_TEST_ASSERT_MSG_GT(s2.size(), 0, "Expected SINR reports from the second UE");
    NS_TEST_ASSERT_MSG_GT(rx0, 0, "Expected the first UE to receive traffic");
    NS_TEST_ASSERT_MSG_GT(rx1, 0, "Expected the second UE to receive traffic");

    if (!m_sc)
    {
        // Independent LOS states, shadowing and fading are continuous draws, so the two
        // SINR sequences cannot coincide: this is the control that makes the equality
        // asserted below meaningful.
        bool identical = s1.size() == s2.size();
        for (size_t k = 0; identical && k < s1.size(); k++)
        {
            identical = std::abs(10 * std::log10(s1[k]) - 10 * std::log10(s2[k])) < 1e-6;
        }
        NS_TEST_ASSERT_MSG_EQ(identical,
                              false,
                              "Co-located UEs must report different DL data SINR "
                              "without InterUeSpatialConsistency");
        return;
    }

    NS_TEST_ASSERT_MSG_EQ(s1.size(), s2.size(), "Expected the same number of SINR reports");
    for (size_t k = 0; k < s1.size(); k++)
    {
        NS_TEST_ASSERT_MSG_EQ_TOL(10 * std::log10(s1[k]),
                                  10 * std::log10(s2[k]),
                                  1e-6,
                                  "Co-located UEs must report identical DL data SINR "
                                  "with InterUeSpatialConsistency");
    }

    // An identical SINR gives an identical MCS, and with deterministic decoding the
    // round-robin scheduler serves both UEs the same number of identical packets;
    // allow a couple of packets of slack for the last scheduling round.
    NS_TEST_ASSERT_MSG_EQ_TOL(static_cast<double>(rx0),
                              static_cast<double>(rx1),
                              4.0,
                              "Co-located UEs under RR must obtain the same throughput");
}

/**
 * @ingroup tests
 * @brief Test suite for co-located UE fairness under the round-robin scheduler.
 */
class NrColocatedUeFairnessTestSuite : public TestSuite
{
  public:
    NrColocatedUeFairnessTestSuite()
        : TestSuite("nr-colocated-ue-fairness", Type::SYSTEM)
    {
        AddTestCase(new NrColocatedUeFairnessTestCase("colocated-ue-fairness-sc", true, 100),
                    Duration::QUICK);
        // Control: without spatial consistency the co-located UEs draw independent
        // channels, so their SINR differs whatever the draws are.
        AddTestCase(new NrColocatedUeFairnessTestCase("colocated-ue-fairness-no-sc", false, 100),
                    Duration::QUICK);
        // Towards the cell edge the MCS follows the SINR, so equal throughput holds only
        // because spatially consistent co-located UEs perceive an identical channel. The
        // distance keeps the UEs decodable under the deepest UMa NLOS shadowing draws.
        AddTestCase(new NrColocatedUeFairnessTestCase("colocated-ue-fairness-sc-edge", true, 300),
                    Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrColocatedUeFairnessTestSuite g_nrColocatedUeFairnessTestSuite;

} // namespace ns3
