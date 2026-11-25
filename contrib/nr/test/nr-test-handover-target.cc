/*
 * Copyright (c) 2013 Budiarto Herman
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *
 */

/**
 * @ingroup test
 * @file nr-test-handover-target.cc
 *
 * @brief Test suite `nr-handover-target`: verifies that handover algorithms select the correct
 * target cell when several candidates are available. Each case builds a rectangular grid of
 * micro-cell gNBs (2x2 or 3x2, 130 m apart) over an NR EPC with a single stationary UE attached
 * to a given source cell, and at 0.5 s "shuts down" that cell by lowering its TX power to 1 dBm.
 * The configured algorithm (NrA2A4RsrpHandoverAlgorithm or NrA3RsrpHandoverAlgorithm) must then
 * react: the HandoverStart trace is checked to fire after the shutdown with the expected source
 * and target cell IDs, and teardown asserts that a handover actually occurred.
 */

#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uinteger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverTargetTest");

/**
 * @ingroup nr-test
 *
 * @brief Testing a handover algorithm, verifying that it selects the right
 *        target cell when more than one options available.
 *
 * Part of the `nr-handover-target` test suite.
 *
 * The test case will run a 1-second NR-EPC simulation using the parameters
 * provided to the constructor function.
 *
 * \sa ns3::NrHandoverTargetTestCase
 */
class NrHandoverTargetTestCase : public TestCase
{
  public:
    /**
     * @brief Construct a new test case and providing input parameters for the
     *        simulation.
     * @param name the name of the test case, to be displayed in the test result
     * @param uePosition the point in (x, y, z) coordinate where the UE will be
     *                   placed in the simulation
     * @param gridSizeX number of gNBs in a row
     * @param gridSizeY number of gNBs in a column
     * @param sourceCellId the cell ID of the gNB which the UE will be
     *                     initially attached to in the beginning of simulation,
     *                     and also the gNB which will "shutdown" in the
     *                     middle of simulation
     * @param targetCellId the cell ID of the expected gNB where the UE will
     *                     perform handover to after the "shutdown" of the source
     *                     cell
     * @param handoverAlgorithmType the type of handover algorithm to be used in
     *                              all gNBs
     */
    NrHandoverTargetTestCase(std::string name,
                             Vector uePosition,
                             uint8_t gridSizeX,
                             uint8_t gridSizeY,
                             uint16_t sourceCellId,
                             uint16_t targetCellId,
                             std::string handoverAlgorithmType);

    ~NrHandoverTargetTestCase() override;

    /**
     * @brief Triggers when an gNB starts a handover and then verifies that
     *        the handover has the right source and target cells.
     *
     * The trigger is set up beforehand by connecting to the
     * `NrgNBRrc::HandoverStart` trace source.
     *
     * @param context the context string
     * @param imsi the IMSI
     * @param sourceCellId the source cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverStartCallback(std::string context,
                               uint64_t imsi,
                               uint16_t sourceCellId,
                               uint16_t rnti,
                               uint16_t targetCellId);

    /**
     * @brief A trigger that can be scheduled to "shutdown" the cell pointed by
     *        `m_sourceCellId` by reducing its power to 1 dB.
     */
    void CellShutdownCallback();

  private:
    /**
     * @brief Run a simulation of a micro-cell network using the parameters
     *        provided to the constructor function.
     */
    void DoRun() override;

    /**
     * @brief Called at the end of simulation and verifies that a handover has
     *        occurred in the simulation.
     */
    void DoTeardown() override;

    // simulation parameters
    Vector m_uePosition;                 ///< UE positions
    uint8_t m_gridSizeX;                 ///< X grid size
    uint8_t m_gridSizeY;                 ///< Y grid size
    uint16_t m_sourceCellId;             ///< source cell ID
    uint16_t m_targetCellId;             ///< target cell ID
    std::string m_handoverAlgorithmType; ///< handover algorithm type

    Ptr<NrGnbNetDevice> m_sourceGnbDev; ///< source gNB device
    bool m_hasHandoverOccurred;         ///< has handover occurred?

    // end of class NrHandoverTargetTestCase
};

NrHandoverTargetTestCase::NrHandoverTargetTestCase(std::string name,
                                                   Vector uePosition,
                                                   uint8_t gridSizeX,
                                                   uint8_t gridSizeY,
                                                   uint16_t sourceCellId,
                                                   uint16_t targetCellId,
                                                   std::string handoverAlgorithmType)
    : TestCase(name),
      m_uePosition(uePosition),
      m_gridSizeX(gridSizeX),
      m_gridSizeY(gridSizeY),
      m_sourceCellId(sourceCellId),
      m_targetCellId(targetCellId),
      m_handoverAlgorithmType(handoverAlgorithmType),
      m_sourceGnbDev(nullptr),
      m_hasHandoverOccurred(false)
{
    NS_LOG_INFO(this << " name=" << name);

    // SANITY CHECK

    uint16_t nGnb = gridSizeX * gridSizeY;

    if (sourceCellId > nGnb)
    {
        NS_FATAL_ERROR("Invalid source cell ID " << sourceCellId);
    }

    if (targetCellId > nGnb)
    {
        NS_FATAL_ERROR("Invalid target cell ID " << targetCellId);
    }
}

NrHandoverTargetTestCase::~NrHandoverTargetTestCase()
{
    NS_LOG_FUNCTION(this);
}

void
NrHandoverTargetTestCase::HandoverStartCallback(std::string context,
                                                uint64_t imsi,
                                                uint16_t sourceCellId,
                                                uint16_t rnti,
                                                uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << sourceCellId << rnti << targetCellId);

    uint64_t timeNowMs = Simulator::Now().GetMilliSeconds();
    NS_TEST_ASSERT_MSG_GT(timeNowMs, 500, "Handover occurred but too early");
    NS_TEST_ASSERT_MSG_EQ(sourceCellId,
                          m_sourceCellId,
                          "Handover occurred but with wrong source cell");
    NS_TEST_ASSERT_MSG_EQ(targetCellId,
                          m_targetCellId,
                          "Handover occurred but with wrong target cell");
    m_hasHandoverOccurred = true;
}

void
NrHandoverTargetTestCase::CellShutdownCallback()
{
    NS_LOG_FUNCTION(this);

    if (m_sourceGnbDev)
    {
        // set the Tx power to 1 dBm
        NS_ASSERT(m_sourceGnbDev->GetCellId() == m_sourceCellId);
        NS_LOG_INFO("Shutting down cell " << m_sourceCellId);
        Ptr<NrGnbPhy> phy = m_sourceGnbDev->GetPhy(0);
        phy->SetTxPower(1);
    }
}

void
NrHandoverTargetTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(36));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    if (m_handoverAlgorithmType == "ns3::NrA2A4RsrpHandoverAlgorithm")
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrA2A4RsrpHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold", UintegerValue(70));
        nrHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset", UintegerValue(1));
    }
    else if (m_handoverAlgorithmType == "ns3::NrA3RsrpHandoverAlgorithm")
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.5));
        nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(128)));
    }
    else
    {
        NS_FATAL_ERROR("Unknown handover algorithm " << m_handoverAlgorithmType);
    }

    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);

    // Create Nodes: gNB and UE
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(m_gridSizeX * m_gridSizeY);
    ueNodes.Create(1);

    /*
     * The size of the grid is determined by m_gridSizeX and m_gridSizeY. The
     * following figure is the topology when m_gridSizeX = 4 and m_gridSizeY = 3.
     *
     *                  9 -- 10 -- 11 -- 12
     *                  |     |     |     |
     *                  |     |     |     |
     *                  5 --- 6 --- 7 --- 8
     *                  |     |     |     |
     *                  |     |     |     |
     *   (0, 0, 0) ---> 1 --- 2 --- 3 --- 4
     *
     * The grid starts at (0, 0, 0) point on the bottom left corner. The distance
     * between two adjacent gNBs is 130 m.
     */

    // Set up gNB position
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                     "MinX",
                                     DoubleValue(0.0),
                                     "MinY",
                                     DoubleValue(0.0),
                                     "DeltaX",
                                     DoubleValue(130.0),
                                     "DeltaY",
                                     DoubleValue(130.0),
                                     "GridWidth",
                                     UintegerValue(m_gridSizeX),
                                     "LayoutType",
                                     StringValue("RowFirst"));
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);

    // Setup UE position
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(m_uePosition);
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(positionAlloc);
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);

    // Create P-GW node
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Configure and create a channel band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (gNB and UE)
    NetDeviceContainer gnbDevs;
    NetDeviceContainer ueDevs;
    gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Assign IP address to UEs
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Add X2 interface
    nrHelper->AddX2Interface(gnbNodes);

    // Connect to trace sources in all gNB
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                    MakeCallback(&NrHandoverTargetTestCase::HandoverStartCallback, this));

    // Get the source gNB
    Ptr<NetDevice> sourceGnb = gnbDevs.Get(m_sourceCellId - 1);
    m_sourceGnbDev = sourceGnb->GetObject<NrGnbNetDevice>();
    NS_ASSERT(m_sourceGnbDev);
    NS_ASSERT(m_sourceGnbDev->GetCellId() == m_sourceCellId);

    // Attach UE to the source gNB
    nrHelper->AttachToGnb(ueDevs.Get(0), sourceGnb);

    // Schedule a "shutdown" of the source gNB
    Simulator::Schedule(Seconds(0.5), &NrHandoverTargetTestCase::CellShutdownCallback, this);

    nrHelper->AssignStreams({.assignEpc = true,
                             .ueNodes = ueNodes,
                             .gnbNodes = gnbNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDevs,
                             .gnbNodeStream = 1000,
                             .gnbDevStream = 3000,
                             .ueDevStream = 4000});

    // Run simulation
    Simulator::Stop(Seconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

void
NrHandoverTargetTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    NS_TEST_ASSERT_MSG_EQ(m_hasHandoverOccurred, true, "Handover did not occur");
}

/**
 * @brief Test suite ``nr-handover-target``, verifying that handover
 *        algorithms are able to select the right target cell.
 *
 * Handover algorithm tested in this test suite:
 * - A2-A4-RSRP handover algorithm (ns3::NrA2A4RsrpHandoverAlgorithm)
 * - Strongest cell handover algorithm (ns3::NrA3RsrpHandoverAlgorithm)
 */
class NrHandoverTargetTestSuite : public TestSuite
{
  public:
    NrHandoverTargetTestSuite();
};

NrHandoverTargetTestSuite::NrHandoverTargetTestSuite()
    : TestSuite("nr-handover-target", Type::SYSTEM)
{
    // LogComponentEnable ("NrHandoverTargetTest", LOG_PREFIX_ALL);
    // LogComponentEnable ("NrHandoverTargetTest", LOG_LEVEL_ALL);
    // LogComponentEnable ("NrA2A4RsrpHandoverAlgorithm", LOG_PREFIX_ALL);
    // LogComponentEnable ("NrA2A4RsrpHandoverAlgorithm", LOG_LEVEL_ALL);
    // LogComponentEnable ("NrA3RsrpHandoverAlgorithm", LOG_PREFIX_ALL);
    // LogComponentEnable ("NrA3RsrpHandoverAlgorithm", LOG_LEVEL_ALL);

    /*
     *    3 --- 4
     *    |     |
     *    |o    |
     *    1 --- 2   o = UE
     */
    AddTestCase(new NrHandoverTargetTestCase("4 cells and A2-A4-RSRP algorithm",
                                             Vector(20, 40, 0),
                                             2,
                                             2,
                                             1,
                                             3,
                                             "ns3::NrA2A4RsrpHandoverAlgorithm"),
                TestCase::Duration::QUICK);
    AddTestCase(new NrHandoverTargetTestCase("4 cells and strongest cell algorithm",
                                             Vector(20, 40, 0),
                                             2,
                                             2,
                                             1,
                                             3,
                                             "ns3::NrA3RsrpHandoverAlgorithm"),
                TestCase::Duration::QUICK);

    /*
     *    4 --- 5 --- 6
     *    |     |o    |
     *    |     |     |
     *    1 --- 2 --- 3   o = UE
     */
    AddTestCase(new NrHandoverTargetTestCase("6 cells and A2-A4-RSRP algorithm",
                                             Vector(150, 90, 0),
                                             3,
                                             2,
                                             5,
                                             2,
                                             "ns3::NrA2A4RsrpHandoverAlgorithm"),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrHandoverTargetTestCase("6 cells and strongest cell algorithm",
                                             Vector(150, 90, 0),
                                             3,
                                             2,
                                             5,
                                             2,
                                             "ns3::NrA3RsrpHandoverAlgorithm"),
                TestCase::Duration::EXTENSIVE);

} // end of NrHandoverTargetTestSuite ()

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrHandoverTargetTestSuite g_nrHandoverTargetTestSuiteInstance;
