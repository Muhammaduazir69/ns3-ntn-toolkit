// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-cell-individual-offset.cc
 *
 * @brief Test suite for the Cell Individual Offset (CIO, cell range expansion) feature of
 * NrUeRrc. The scenario places two gNBs 130 m apart and a single static UE much closer to the
 * serving gNB, so the neighbour RSRP alone can never satisfy the A3 handover event. The same
 * geometry is simulated twice: with CIO = 0 dB the test asserts that no handover happens and the
 * UE remains on the serving cell, while with a large positive CIO configured for the neighbour
 * the biased A3 measurement must trigger a handover to the weaker cell. Handover traces, the
 * offset value stored in the UE RRC, and the final serving cell ID are checked in both runs.
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
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrCellIndividualOffsetTest");

/**
 * @ingroup nr-test
 *
 * @brief Test the NR Cell Individual Offset (CIO / cell-range-expansion)
 *        feature implemented in NrUeRrc.
 *
 * Scenario: two gNBs 130 m apart and a single static UE placed at x=40 m,
 * i.e. clearly closer to gNB1 (the serving cell) than to gNB2. Both cells
 * transmit at the same power and the channel has no shadowing, so the RSRP
 * of gNB1 stays above that of gNB2 by a margin that, on its own, never
 * satisfies the Event A3 entering condition
 *   Mn + Ofn + Ocn - Hys > Mp + Ofp + Ocp + Off
 * (with a positive hysteresis and zero offsets). Hence, with no Cell
 * Individual Offset, no handover ever happens.
 *
 * When a large positive Cell Individual Offset (Ocn) is configured on the UE
 * for gNB2, the neighbour measurement is biased upward enough to clear the
 * hysteresis, so the A3 condition is met and the UE hands over to gNB2 even
 * though its real RSRP is weaker. This is exactly the cell-range-expansion
 * bias the feature is meant to provide.
 *
 * The same geometry is run twice:
 *  - m_cioDb == 0  : expect NO handover (UE stays camped on gNB1).
 *  - m_cioDb >  0  : expect a handover from gNB1 to gNB2.
 *
 * The two cases must therefore produce different handover outcomes, which is
 * what proves the offset is actually exercised.
 */
class NrCellIndividualOffsetTestCase : public TestCase
{
  public:
    /**
     * @brief Constructor.
     * @param name human-readable test case name
     * @param cioDb the Cell Individual Offset (in dB) configured on the UE for
     *              the neighbour cell (gNB2). 0 means no offset.
     * @param expectHandover whether a handover to gNB2 is expected to occur
     */
    NrCellIndividualOffsetTestCase(std::string name, double cioDb, bool expectHandover);

    ~NrCellIndividualOffsetTestCase() override;

    /**
     * @brief Callback connected to the NrGnbRrc::HandoverStart trace; records
     *        that a handover from gNB1 to gNB2 was triggered.
     * @param context the trace context string
     * @param imsi the UE IMSI
     * @param sourceCellId the source cell ID
     * @param rnti the UE RNTI
     * @param targetCellId the target cell ID
     */
    void HandoverStartCallback(std::string context,
                               uint64_t imsi,
                               uint16_t sourceCellId,
                               uint16_t rnti,
                               uint16_t targetCellId);

  private:
    void DoRun() override;
    void DoTeardown() override;

    /**
     * @brief Apply the configured Cell Individual Offset to the UE RRC for the
     *        neighbour cell (called via Simulator::Schedule once RRC is up).
     */
    void ApplyCio();

    double m_cioDb;             ///< Cell Individual Offset for gNB2 (dB)
    bool m_expectHandover;      ///< whether a handover is expected
    bool m_handoverOccurred;    ///< set true when handover gNB1 -> gNB2 fires
    uint16_t m_sourceCellId;    ///< serving cell ID (gNB1)
    uint16_t m_targetCellId;    ///< neighbour cell ID (gNB2)
    Ptr<NrUeNetDevice> m_ueDev; ///< the UE device (for CIO + final cell check)
};

NrCellIndividualOffsetTestCase::NrCellIndividualOffsetTestCase(std::string name,
                                                               double cioDb,
                                                               bool expectHandover)
    : TestCase(name),
      m_cioDb(cioDb),
      m_expectHandover(expectHandover),
      m_handoverOccurred(false),
      m_sourceCellId(1),
      m_targetCellId(2),
      m_ueDev(nullptr)
{
}

NrCellIndividualOffsetTestCase::~NrCellIndividualOffsetTestCase()
{
}

void
NrCellIndividualOffsetTestCase::HandoverStartCallback(std::string context,
                                                      uint64_t imsi,
                                                      uint16_t sourceCellId,
                                                      uint16_t rnti,
                                                      uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << sourceCellId << rnti << targetCellId);
    NS_TEST_ASSERT_MSG_EQ(sourceCellId,
                          m_sourceCellId,
                          "Handover started from an unexpected source cell");
    NS_TEST_ASSERT_MSG_EQ(targetCellId,
                          m_targetCellId,
                          "Handover started toward an unexpected target cell");
    m_handoverOccurred = true;
}

void
NrCellIndividualOffsetTestCase::ApplyCio()
{
    NS_LOG_FUNCTION(this << m_cioDb);
    NS_ASSERT(m_ueDev);
    Ptr<NrUeRrc> ueRrc = m_ueDev->GetRrc();
    // Bias the neighbour cell (gNB2) upward. With m_cioDb == 0 this is a no-op
    // (the map entry is erased), so the baseline case truly has no offset.
    ueRrc->SetCellIndividualOffset(m_targetCellId, m_cioDb);
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetCellIndividualOffset(m_targetCellId),
                          m_cioDb,
                          "Cell Individual Offset was not stored as configured");
}

void
NrCellIndividualOffsetTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

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

    // Strongest-cell (Event A3) handover algorithm. The hysteresis is large
    // enough that the natural RSRP gap between the (closer) serving cell and
    // the (farther) neighbour never triggers a handover on its own.
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(128)));

    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);

    // Nodes: two gNBs and one UE.
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(2);
    ueNodes.Create(1);

    // gNB positions: 130 m apart on the x-axis.
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0.0, 0.0, 10.0));   // gNB1 -> cell 1 (serving)
    gnbPositionAlloc->Add(Vector(130.0, 0.0, 10.0)); // gNB2 -> cell 2 (neighbour)
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);

    // UE at x=40 m: closer to gNB1, so gNB1 is the stronger cell and gNB2 is a
    // close second that does not, by itself, satisfy the A3 entering condition.
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(Vector(40.0, 0.0, 1.5));
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);

    // EPC / Internet plumbing.
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Channel: UMi LOS, no shadowing, so the geometry is deterministic.
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    internet.Install(ueNodes);
    nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    nrHelper->AddX2Interface(gnbNodes);

    // Connect to the HandoverStart trace on all gNBs.
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                    MakeCallback(&NrCellIndividualOffsetTestCase::HandoverStartCallback, this));

    m_ueDev = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    NS_ASSERT(m_ueDev);

    // Attach the UE to gNB1 (cell 1).
    Ptr<NrGnbNetDevice> sourceGnb = gnbDevs.Get(0)->GetObject<NrGnbNetDevice>();
    NS_ASSERT(sourceGnb->GetCellId() == m_sourceCellId);
    nrHelper->AttachToGnb(ueDevs.Get(0), gnbDevs.Get(0));

    // Apply the Cell Individual Offset after the RRC connection is established.
    Simulator::Schedule(Seconds(0.3), &NrCellIndividualOffsetTestCase::ApplyCio, this);

    // Deterministic, stream-isolated RNG assignment.
    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDevs,
                             .gnbDevStream = 3000,
                             .ueDevStream = 4000});

    Simulator::Stop(Seconds(1.5));
    Simulator::Run();

    // Final camped cell check (independent of the trace flag).
    uint16_t finalCellId = m_ueDev->GetRrc()->GetCellId();
    if (m_expectHandover)
    {
        NS_TEST_ASSERT_MSG_EQ(finalCellId,
                              m_targetCellId,
                              "With a positive CIO the UE should have handed over to gNB2");
    }
    else
    {
        NS_TEST_ASSERT_MSG_EQ(finalCellId,
                              m_sourceCellId,
                              "With no CIO the UE should have stayed camped on gNB1");
    }

    Simulator::Destroy();
    Config::Reset();
}

void
NrCellIndividualOffsetTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    NS_TEST_ASSERT_MSG_EQ(m_handoverOccurred,
                          m_expectHandover,
                          "Observed handover outcome differs from the expectation for this CIO");
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite ``nr-test-cell-individual-offset``.
 *
 * Verifies that the per-cell Cell Individual Offset (Ocn / Ocp) configured on
 * NrUeRrc biases the Event A3 handover decision: the identical geometry yields
 * NO handover with zero offset, but DOES hand over to the neighbour once a
 * large positive offset is applied to it.
 */
class NrCellIndividualOffsetTestSuite : public TestSuite
{
  public:
    NrCellIndividualOffsetTestSuite();
};

NrCellIndividualOffsetTestSuite::NrCellIndividualOffsetTestSuite()
    : TestSuite("nr-test-cell-individual-offset", Type::SYSTEM)
{
    // Baseline: no offset -> the closer serving cell keeps the UE.
    AddTestCase(new NrCellIndividualOffsetTestCase("CIO=0 dB -> no handover", 0.0, false),
                TestCase::Duration::QUICK);
    // CRE bias: large positive offset on the neighbour -> handover triggers.
    AddTestCase(
        new NrCellIndividualOffsetTestCase("CIO=+20 dB -> handover to neighbour", 20.0, true),
        TestCase::Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization.
 */
static NrCellIndividualOffsetTestSuite g_nrCellIndividualOffsetTestSuiteInstance;
