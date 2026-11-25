/*
 * Copyright (c) 2013 Budiarto Herman
 * Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 * Modified by:
 *     Gabriel Ferreira <gabriel.carvalho@cttc.es> (port code from upstream 4G LENA module to NR
 * 5G-LENA)
 */

/**
 * @ingroup test
 * @file nr-test-cell-selection.cc
 *
 * @brief Test suite (nr-cell-selection) for the initial cell selection procedure executed by
 * NrUeRrc in IDLE state at the beginning of the simulation. The scenario deploys four cells (two
 * of them CSG) and six UEs whose positions, CSG membership, checkpoint times and expected
 * serving cells are given as test vectors. The simulation runs with the EPC and ideal RRC; UE
 * state transitions and the InitialCellSelectionEndOk/EndError traces are recorded, and at each
 * UE checkpoint the test asserts that the UE is attached to one of its expected cells and has
 * reached the CONNECTED_NORMALLY state.
 */

#include "nr-test-cell-selection.h"

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-epc-ue-nas.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrCellSelectionTest");

/*
 * This test suite sets up four cells (four gNBs) and six UEs.  Two of the
 * cells are CSG and two are non-CSG.  The six UEs associate with cells
 * based on their positions and random access procedures.  The test checks
 * that the UEs, at specific simulation times, are associated to expected
 * cells.  See the header Doxygen for more specific descriptions.
 *
 * The test conditions that are checked rely on the RA preamble values
 * (randomly selected) being unique for the six UEs.  For most values of
 * random seed and run number, this will be the case, but certain values
 * will cause a collision (same RA preamble drawn for two UEs).  Therefore,
 * this test fixes the RngSeed and RngRun values, and uses AssignStreams,
 * to ensure that an RA preamble collision does not occur.
 */

NrCellSelectionTestSuite::NrCellSelectionTestSuite()
    : TestSuite("nr-cell-selection", Type::SYSTEM)
{
    std::vector<NrCellSelectionTestCase::UeSetup_t> w;

    // REAL RRC PROTOCOL

    w = {
        // x, y, csgMember, checkPoint, cell1, cell2
        NrCellSelectionTestCase::UeSetup_t(0.0, 0.55, false, MilliSeconds(283), 1, 0),
        NrCellSelectionTestCase::UeSetup_t(0.0, 0.45, false, MilliSeconds(283), 2, 0),
        NrCellSelectionTestCase::UeSetup_t(0.5, 0.45, false, MilliSeconds(363), 2, 0),
        NrCellSelectionTestCase::UeSetup_t(0.5, 0.0, true, MilliSeconds(283), 2, 4),
        NrCellSelectionTestCase::UeSetup_t(1.0, 0.55, true, MilliSeconds(283), 3, 0),
        NrCellSelectionTestCase::UeSetup_t(1.0, 0.45, true, MilliSeconds(283), 4, 0),
    };

    // todo: re-enable when RRC real is fully working
    // AddTestCase(new NrCellSelectionTestCase("EPC, real RRC", true, false, 60.0 /* isd */, w),
    //            TestCase::Duration::QUICK);

    // IDEAL RRC PROTOCOL

    w = {
        // x, y, csgMember, checkPoint, cell1, cell2
        NrCellSelectionTestCase::UeSetup_t(0.0, 0.55, false, MilliSeconds(266), 1, 0),
        NrCellSelectionTestCase::UeSetup_t(0.0, 0.45, false, MilliSeconds(266), 2, 0),
        NrCellSelectionTestCase::UeSetup_t(0.5, 0.45, false, MilliSeconds(346), 2, 0),
        NrCellSelectionTestCase::UeSetup_t(0.5, 0.0, true, MilliSeconds(266), 2, 4),
        NrCellSelectionTestCase::UeSetup_t(1.0, 0.55, true, MilliSeconds(266), 3, 0),
        NrCellSelectionTestCase::UeSetup_t(1.0, 0.45, true, MilliSeconds(266), 4, 0),
    };

    AddTestCase(new NrCellSelectionTestCase("EPC, ideal RRC", true, true, 60.0 /* isd */, w),
                TestCase::Duration::QUICK);

} // end of NrCellSelectionTestSuite::NrCellSelectionTestSuite ()

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrCellSelectionTestSuite g_lteCellSelectionTestSuite;

/*
 * Test Case
 */

NrCellSelectionTestCase::UeSetup_t::UeSetup_t(double relPosX,
                                              double relPosY,
                                              bool isCsgMember,
                                              Time checkPoint,
                                              uint16_t expectedCellId1,
                                              uint16_t expectedCellId2)
    : position(Vector(relPosX, relPosY, 0.0)),
      isCsgMember(isCsgMember),
      checkPoint(checkPoint),
      expectedCellId1(expectedCellId1),
      expectedCellId2(expectedCellId2)
{
}

NrCellSelectionTestCase::NrCellSelectionTestCase(std::string name,
                                                 bool isEpcMode,
                                                 bool isIdealRrc,
                                                 double interSiteDistance,
                                                 std::vector<UeSetup_t> ueSetupList)
    : TestCase(name),
      m_isEpcMode(isEpcMode),
      m_isIdealRrc(isIdealRrc),
      m_interSiteDistance(interSiteDistance),
      m_ueSetupList(ueSetupList)
{
    NS_LOG_FUNCTION(this << GetName());
    m_lastState.resize(20, NrUeRrc::NUM_STATES);
}

NrCellSelectionTestCase::~NrCellSelectionTestCase()
{
    NS_LOG_FUNCTION(this << GetName());
}

void
NrCellSelectionTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());

    // In ns-3 test suite operation, static variables persist across all
    // tests (all test suites execute within a single ns-3 process).
    // Therefore, to fix a seed and run number for a specific test, the
    // current values of seed and run number should be saved and restored
    // after the test is run.
    uint32_t previousSeed = RngSeedManager::GetSeed();
    uint64_t previousRun = RngSeedManager::GetRun();
    // Values of 1 and 2 here will prevent RA preamble collisions
    Config::SetGlobal("RngSeed", UintegerValue(1));
    Config::SetGlobal("RngRun", UintegerValue(2));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_isIdealRrc));

    Ptr<NrPointToPointEpcHelper> epcHelper;

    if (m_isEpcMode)
    {
        epcHelper = CreateObject<NrPointToPointEpcHelper>();
        nrHelper->SetEpcHelper(epcHelper);
    }

    /*
     * The topology is the following (the number on the node indicate the cell ID)
     *
     *      [1]        [3]
     *    non-CSG -- non-CSG
     *       |          |
     *       |          | 60 m
     *       |          |
     *      [2]        [4]
     *      CSG ------ CSG
     *           60 m
     */

    // Create Nodes
    NodeContainer gnbNodes;
    gnbNodes.Create(4);
    NodeContainer ueNodes;
    auto nUe = static_cast<uint16_t>(m_ueSetupList.size());
    ueNodes.Create(nUe);

    // Assign nodes to position
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // gNB
    positionAlloc->Add(Vector(0.0, m_interSiteDistance, 0.0));
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(m_interSiteDistance, m_interSiteDistance, 0.0));
    positionAlloc->Add(Vector(m_interSiteDistance, 0.0, 0.0));
    // UE
    std::vector<UeSetup_t>::const_iterator itSetup;
    for (itSetup = m_ueSetupList.begin(); itSetup != m_ueSetupList.end(); itSetup++)
    {
        Vector uePos(m_interSiteDistance * itSetup->position.x,
                     m_interSiteDistance * itSetup->position.y,
                     m_interSiteDistance * itSetup->position.z);
        NS_LOG_INFO("UE position " << uePos);
        positionAlloc->Add(uePos);
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (eNB and UE)
    NetDeviceContainer enbDevs;

    // cell ID 1 is a non-CSG cell
    // nrHelper->SetGnbDeviceAttribute("CsgId", UintegerValue(0));
    // nrHelper->SetGnbDeviceAttribute("CsgIndication", BooleanValue(false));
    enbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(0), allBwps));

    // cell ID 2 is a CSG cell
    // nrHelper->SetGnbDeviceAttribute("CsgId", UintegerValue(1));
    // nrHelper->SetGnbDeviceAttribute("CsgIndication", BooleanValue(true));
    enbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(1), allBwps));

    // cell ID 3 is a non-CSG cell
    // nrHelper->SetGnbDeviceAttribute("CsgId", UintegerValue(0));
    // nrHelper->SetGnbDeviceAttribute("CsgIndication", BooleanValue(false));
    enbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(2), allBwps));

    // cell ID 4 is a CSG cell
    // nrHelper->SetGnbDeviceAttribute("CsgId", UintegerValue(1));
    // nrHelper->SetGnbDeviceAttribute("CsgIndication", BooleanValue(true));
    enbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(3), allBwps));

    for (auto gnbIt = enbDevs.Begin(); gnbIt != enbDevs.End(); gnbIt++)
    {
        DynamicCast<NrGnbNetDevice>(*gnbIt)->ConfigureCell();
    }

    NetDeviceContainer ueDevs;
    Time lastCheckPoint;
    NS_ASSERT(m_ueSetupList.size() == ueNodes.GetN());
    NodeContainer::Iterator itNode;
    for (itSetup = m_ueSetupList.begin(), itNode = ueNodes.Begin();
         itSetup != m_ueSetupList.end() || itNode != ueNodes.End();
         itSetup++, itNode++)
    {
        // if (itSetup->isCsgMember)
        //{
        //     nrHelper->SetUeDeviceAttribute("CsgId", UintegerValue(1));
        // }
        // else
        //{
        //     nrHelper->SetUeDeviceAttribute("CsgId", UintegerValue(0));
        // }

        NetDeviceContainer devs = nrHelper->InstallUeDevice(*itNode, allBwps);
        Ptr<NrUeNetDevice> ueDev = devs.Get(0)->GetObject<NrUeNetDevice>();
        NS_ASSERT(ueDev);

        Ptr<NrEpcUeNas> ueNas = ueDev->GetNas();
        NS_ASSERT(ueNas);

        // Enable idle mode cell selection
        Simulator::Schedule(MilliSeconds(20),
                            &NrEpcUeNas::StartCellSelection,
                            ueNas,
                            NrPhy::FrequencyHzToArfcn(2.8e9));

        ueDevs.Add(devs);
        Simulator::Schedule(itSetup->checkPoint,
                            &NrCellSelectionTestCase::CheckPoint,
                            this,
                            ueDev,
                            itSetup->expectedCellId1,
                            itSetup->expectedCellId2);

        if (lastCheckPoint < itSetup->checkPoint)
        {
            lastCheckPoint = itSetup->checkPoint;
        }
    }

    if (m_isEpcMode)
    {
        epcHelper->AssignStreams(0);
    }
    nrHelper->AssignStreams({.gnbDevs = enbDevs, .ueDevs = ueDevs});

    // Tests
    NS_ASSERT(m_ueSetupList.size() == ueDevs.GetN());
    NetDeviceContainer::Iterator itDev;
    for (itSetup = m_ueSetupList.begin(), itDev = ueDevs.Begin();
         itSetup != m_ueSetupList.end() || itDev != ueDevs.End();
         itSetup++, itDev++)
    {
        Ptr<NrUeNetDevice> ueDev = (*itDev)->GetObject<NrUeNetDevice>();
    }

    if (m_isEpcMode)
    {
        // Create P-GW node
        Ptr<Node> pgw = epcHelper->GetPgwNode();

        // Create a single RemoteHost
        NodeContainer remoteHostContainer;
        remoteHostContainer.Create(1);
        Ptr<Node> remoteHost = remoteHostContainer.Get(0);
        InternetStackHelper internet;
        internet.Install(remoteHostContainer);
        nrHelper->AssignStreams({.remoteHostNodes = remoteHostContainer});

        // Create the Internet
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
        NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
        Ipv4AddressHelper ipv4h;
        ipv4h.SetBase("1.0.0.0", "255.0.0.0");
        Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

        // Routing of the Internet Host (towards the NR network)
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                   Ipv4Mask("255.0.0.0"),
                                                   1);

        // Install the IP stack on the UEs
        internet.Install(ueNodes);
        nrHelper->AssignStreams({.ueNodes = ueNodes});
        Ipv4InterfaceContainer ueIpIfaces;
        ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

        // Assign IP address to UEs
        for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
        {
            Ptr<Node> ueNode = ueNodes.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
    }
    else
    {
        NS_FATAL_ERROR("No support yet for LTE-only simulations");
    }

    // Connect to trace sources in UEs
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/StateTransition",
                    MakeCallback(&NrCellSelectionTestCase::StateTransitionCallback, this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrUeRrc/InitialCellSelectionEndOk",
        MakeCallback(&NrCellSelectionTestCase::InitialCellSelectionEndOkCallback, this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrUeRrc/InitialCellSelectionEndError",
        MakeCallback(&NrCellSelectionTestCase::InitialCellSelectionEndErrorCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
                    MakeCallback(&NrCellSelectionTestCase::ConnectionEstablishedCallback, this));

    // Enable Idle mode cell selection
    // nrHelper->Attach(ueDevs);

    // Run simulation
    Simulator::Stop(lastCheckPoint);
    Simulator::Run();

    NS_LOG_INFO("Simulation ends");
    Simulator::Destroy();

    // Restore the seed and run number that were in effect before this test
    Config::SetGlobal("RngSeed", UintegerValue(previousSeed));
    Config::SetGlobal("RngRun", UintegerValue(previousRun));
}

void
NrCellSelectionTestCase::CheckPoint(Ptr<NrUeNetDevice> ueDev,
                                    uint16_t expectedCellId1,
                                    uint16_t expectedCellId2)
{
    uint16_t actualCellId = ueDev->GetRrc()->GetCellId();

    if (expectedCellId2 == 0)
    {
        NS_TEST_ASSERT_MSG_EQ(actualCellId,
                              expectedCellId1,
                              "IMSI "
                                  << ueDev->GetImsi() << " has attached to an unexpected cell "
                                  << ueDev->GetNode()->GetObject<MobilityModel>()->GetPosition());
    }
    else
    {
        bool pass = (actualCellId == expectedCellId1) || (actualCellId == expectedCellId2);
        NS_TEST_ASSERT_MSG_EQ(pass,
                              true,
                              "IMSI " << ueDev->GetImsi() << " has attached to an unexpected cell"
                                      << " (actual: " << actualCellId << ","
                                      << " expected: " << expectedCellId1 << " or "
                                      << expectedCellId2 << ")");
    }

    if (expectedCellId1 > 0)
    {
        NS_TEST_ASSERT_MSG_EQ(m_lastState.at(static_cast<unsigned int>(ueDev->GetImsi() - 1)),
                              NrUeRrc::CONNECTED_NORMALLY,
                              "UE " << ueDev->GetImsi() << " is not at CONNECTED_NORMALLY state");
    }
}

void
NrCellSelectionTestCase::StateTransitionCallback(std::string context,
                                                 uint64_t imsi,
                                                 uint16_t cellId,
                                                 uint16_t rnti,
                                                 NrUeRrc::State oldState,
                                                 NrUeRrc::State newState)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti << oldState << newState);
    m_lastState.at(static_cast<unsigned int>(imsi - 1)) = newState;
}

void
NrCellSelectionTestCase::InitialCellSelectionEndOkCallback(std::string context,
                                                           uint64_t imsi,
                                                           uint16_t cellId)
{
    NS_LOG_FUNCTION(this << imsi << cellId);
}

void
NrCellSelectionTestCase::InitialCellSelectionEndErrorCallback(std::string context,
                                                              uint64_t imsi,
                                                              uint16_t cellId)
{
    NS_LOG_FUNCTION(this << imsi << cellId);
}

void
NrCellSelectionTestCase::ConnectionEstablishedCallback(std::string context,
                                                       uint64_t imsi,
                                                       uint16_t cellId,
                                                       uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}
