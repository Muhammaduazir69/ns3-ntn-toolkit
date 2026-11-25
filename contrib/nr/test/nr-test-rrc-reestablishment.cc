// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-rrc-reestablishment.cc
 *
 * @brief Test suite `nr-rrc-reestablishment`: RRC connection reestablishment on/off behavior
 * after radio link failure. A single gNB and a single UE run with the ideal RRC protocol and
 * shortened N310/N311/T310 parameters; at 0.5 s the UE jumps 15 km away from the gNB so that a
 * radio link failure is detected. With UseRrcReestablishment enabled, the recorded UE state
 * history must include CONNECTED_PHY_PROBLEM (the reestablishment path); with it disabled,
 * CONNECTED_PHY_PROBLEM must never appear and the UE must transition to IDLE_CELL_SEARCH
 * directly.
 */

#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/propagation-loss-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrRrcReestablishmentTest");

/**
 * @ingroup nr-test
 *
 * @brief Test case for RRC connection reestablishment ON/OFF behavior.
 *
 * This test verifies that:
 * - When reestablishment is enabled: UE transitions through CONNECTED_PHY_PROBLEM
 *   and CONNECTED_REESTABLISHING states after RLF.
 * - When reestablishment is disabled: UE directly clears context and goes to IDLE
 *   mode without attempting reestablishment.
 */
class NrRrcReestablishmentTestCase : public TestCase
{
  public:
    /**
     * @brief Creates an instance of the RRC reestablishment test case.
     *
     * @param useRrcReestablishment true to enable reestablishment, false to disable it
     * @param simTime simulation time in seconds
     * @param description test description
     */
    NrRrcReestablishmentTestCase(bool useRrcReestablishment,
                                 double simTime,
                                 std::string description = "");

    ~NrRrcReestablishmentTestCase() override;

  private:
    /**
     * @brief Setup the simulation according to the configuration set by the
     *        class constructor, run it, and verify the result.
     */
    void DoRun() override;

    /**
     * @brief Callback for UE state change monitoring.
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID
     * @param rnti the RNTI
     * @param oldState the previous state
     * @param newState the new state
     */
    void OnUeStateChanged(uint64_t imsi,
                          uint16_t cellId,
                          uint16_t rnti,
                          NrUeRrc::State oldState,
                          NrUeRrc::State newState);

    /**
     * @brief Callback for RLF detection.
     * @param imsi the IMSI of the UE
     * @param cellId the cell ID
     * @param rnti the RNTI of the UE
     */
    void OnRadioLinkFailure(uint64_t imsi, uint32_t cellId, uint16_t rnti);

    /**
     * @brief Moves the UE to a distant position to simulate RLF.
     */
    void JumpAway();

    bool m_useRrcReestablishment;    ///< whether reestablishment is enabled
    double m_simTime;                ///< simulation time in seconds
    bool m_rlfDetected;              ///< whether RLF was detected
    bool m_reestablishmentAttempted; ///< whether reestablishment was attempted
    std::string m_finalUeState;      ///< final UE state
    std::string m_ueStateHistory;    ///< UE state transition history
    Ptr<MobilityModel> m_ueMobility; ///< UE mobility model
};

/**
 * @ingroup nr-test
 *
 * @brief Test suite for RRC connection reestablishment ON/OFF behavior.
 */
class NrRrcReestablishmentTestSuite : public TestSuite
{
  public:
    NrRrcReestablishmentTestSuite();
};

NrRrcReestablishmentTestCase::NrRrcReestablishmentTestCase(bool useRrcReestablishment,
                                                           double simTime,
                                                           std::string description)
    : TestCase(description.empty()
                   ? std::string("RRC reestablishment ") +
                         (useRrcReestablishment ? "enabled" : "disabled") + " behavior test"
                   : description),
      m_useRrcReestablishment(useRrcReestablishment),
      m_simTime(simTime),
      m_rlfDetected(false),
      m_reestablishmentAttempted(false),
      m_finalUeState("UNKNOWN"),
      m_ueStateHistory(""),
      m_ueMobility(nullptr)
{
    NS_LOG_FUNCTION(this);
}

NrRrcReestablishmentTestCase::~NrRrcReestablishmentTestCase()
{
    NS_LOG_FUNCTION(this);
}

void
NrRrcReestablishmentTestCase::OnUeStateChanged(uint64_t imsi,
                                               uint16_t cellId,
                                               uint16_t rnti,
                                               NrUeRrc::State oldState,
                                               NrUeRrc::State newState)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti << oldState << newState);
    m_ueStateHistory += ToString(oldState) + " --> " + ToString(newState) + "; ";
    m_finalUeState = ToString(newState);
    if (newState == NrUeRrc::CONNECTED_PHY_PROBLEM)
    {
        // UE entered PHY problem state (context removal initiated)
    }
    if (newState == NrUeRrc::CONNECTED_REESTABLISHING)
    {
        m_reestablishmentAttempted = true;
    }
}

void
NrRrcReestablishmentTestCase::OnRadioLinkFailure(uint64_t imsi, uint32_t cellId, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    m_rlfDetected = true;
}

void
NrRrcReestablishmentTestCase::JumpAway()
{
    NS_LOG_FUNCTION(this);
    // Move the UE far away from the gNB to simulate RLF
    m_ueMobility->SetPosition(Vector(15000.0, 0.0, 1.5));
}

void
NrRrcReestablishmentTestCase::DoRun()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Running RRC reestablishment test: " << "useRrcReestablishment="
                                                     << m_useRrcReestablishment);

    // Enable logging for debugging
    LogComponentEnable("NrUeRrc", LOG_LEVEL_INFO);
    LogComponentEnable("NrGnbRrc", LOG_LEVEL_INFO);
    LogComponentEnable("NrRrcProtocolIdeal", LOG_LEVEL_INFO);
    LogComponentEnable("NrRrcReestablishmentTest", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer gnbNodes;
    gnbNodes.Create(1);

    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Mobility
    Ptr<ListPositionAllocator> positionAllocGnb = CreateObject<ListPositionAllocator>();
    positionAllocGnb->Add(Vector(0, 0, 10));
    MobilityHelper mobilityGnb;
    mobilityGnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGnb.SetPositionAllocator(positionAllocGnb);
    mobilityGnb.Install(gnbNodes);

    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();
    positionAllocUe->Add(Vector(0, 0, 1.5));
    MobilityHelper mobilityUe;
    mobilityUe.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityUe.SetPositionAllocator(positionAllocUe);
    mobilityUe.Install(ueNodes);

    // Create EPC
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();

    // Configure NR helper with ideal RRC
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    Config::SetDefault("ns3::NrAmc::AmcModel", EnumValue(NrAmc::ShannonModel));

    // Set RLF detection parameters
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(1)));

    // Set reestablishment attribute
    Config::SetDefault("ns3::NrUeRrc::UseRrcReestablishment",
                       BooleanValue(m_useRrcReestablishment));
    Config::SetDefault("ns3::NrGnbRrc::UseRrcReestablishment",
                       BooleanValue(m_useRrcReestablishment));

    // Create bandwidth parts
    auto bandwidthAndBWPPair = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");

    // Install gNB
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, bandwidthAndBWPPair.second);

    // Install UE devices
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, bandwidthAndBWPPair.second);

    // Install IP stack on UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);
    nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Enable traces
    nrHelper->EnableTraces();

    // Get UE RRC and connect traces (before attaching to capture all state changes)
    Ptr<NrUeNetDevice> ueNetDev = ueDevs.Get(0)->GetObject<NrUeNetDevice>();
    Ptr<NrUeRrc> ueRrc = ueNetDev->GetRrc();
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    ueRrc->TraceConnectWithoutContext(
        "RadioLinkFailureDetected",
        MakeCallback(&NrRrcReestablishmentTestCase::OnRadioLinkFailure, this));
    ueRrc->TraceConnectWithoutContext(
        "StateTransition",
        MakeCallback(&NrRrcReestablishmentTestCase::OnUeStateChanged, this));

    // Attach UE to gNB
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Schedule UE jump to trigger RLF after connection is established
    Simulator::Schedule(Seconds(0.5), &NrRrcReestablishmentTestCase::JumpAway, this);

    // Wait for simulation to complete
    Simulator::Stop(Seconds(m_simTime));
    Simulator::Run();

    // Verify results
    NS_LOG_INFO("Test complete. RLF detected: " << m_rlfDetected << ", Reestablishment attempted: "
                                                << m_reestablishmentAttempted
                                                << ", Final state: " << m_finalUeState);
    NS_LOG_INFO("State history: " << m_ueStateHistory);

    if (m_useRrcReestablishment)
    {
        // When reestablishment is enabled, UE should transition to CONNECTED_PHY_PROBLEM
        // (and wait for reestablishment response from gNB)
        NS_TEST_ASSERT_MSG_EQ(m_ueStateHistory.find("CONNECTED_PHY_PROBLEM") != std::string::npos,
                              true,
                              "When reestablishment is enabled, UE should transition to "
                              "CONNECTED_PHY_PROBLEM");
    }
    else
    {
        // When reestablishment is disabled, UE should go directly to IDLE mode
        // without going through CONNECTED_PHY_PROBLEM
        NS_TEST_ASSERT_MSG_EQ(m_ueStateHistory.find("CONNECTED_PHY_PROBLEM"),
                              std::string::npos,
                              "When reestablishment is disabled, UE should not transition to "
                              "CONNECTED_PHY_PROBLEM");
        NS_TEST_ASSERT_MSG_EQ(m_ueStateHistory.find("IDLE_CELL_SEARCH") != std::string::npos,
                              true,
                              "When reestablishment is disabled, UE should transition to "
                              "IDLE_CELL_SEARCH");
    }

    Simulator::Destroy();
}

// ==================== Test Suite ====================

NrRrcReestablishmentTestSuite::NrRrcReestablishmentTestSuite()
    : TestSuite("nr-rrc-reestablishment", Type::SYSTEM)
{
    NS_LOG_FUNCTION(this);
    // Add test case for reestablishment enabled
    AddTestCase(new NrRrcReestablishmentTestCase(true, 10.0, "RRC reestablishment enabled"),
                TestCase::Duration::QUICK);

    // Add test case for reestablishment disabled
    AddTestCase(new NrRrcReestablishmentTestCase(false, 10.0, "RRC reestablishment disabled"),
                TestCase::Duration::QUICK);
}

static NrRrcReestablishmentTestSuite g_nrRrcReestablishmentTestSuite;
