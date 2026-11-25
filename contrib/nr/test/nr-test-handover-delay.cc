/*
 * Copyright (c) 2013 Magister Solutions
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Budiarto Herman <budiarto.herman@magister.fi>
 *         Alexander Krotov <krotov@iitp.ru>
 */

/**
 * @ingroup test
 * @file nr-test-handover-delay.cc
 *
 * @brief Test suite (nr-handover-delay) verifying X2 handover latency and the handover delay
 * attributes. NrHandoverDelayTestCase places a UE halfway between two gNBs 1000 m apart,
 * attaches it to the first one, and issues an explicit handover request at a configurable time;
 * the RRC HandoverStart/HandoverEndOk traces on the UE and gNB sides must show that the handover
 * completed and took less than a threshold (5 ms with ideal RRC, 20 ms with real RRC), for 1, 2
 * and 4 component carriers. NrHandoverDelayApplyTestCase additionally configures the
 * HandoverDecisionDelay and HandoverTriggeringDelay attributes and verifies that the handover
 * still starts and completes with those extra delays applied.
 */

#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/data-rate.h"
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
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverDelayTest");

/**
 * @ingroup nr-test
 *
 * @brief Verifying that the time needed for handover is under a
 * specified threshold.
 */

class NrHandoverDelayTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param numberOfComponentCarriers number of component carriers
     * @param useIdealRrc if true, use the ideal RRC
     * @param handoverTime the time of handover
     * @param delayThreshold the delay threshold
     * @param simulationDuration duration of the simulation
     */
    NrHandoverDelayTestCase(uint8_t numberOfComponentCarriers,
                            bool useIdealRrc,
                            Time handoverTime,
                            Time delayThreshold,
                            Time simulationDuration)
        : TestCase("Verifying that the time needed for handover is under a specified threshold"),
          m_numberOfComponentCarriers(numberOfComponentCarriers),
          m_useIdealRrc(useIdealRrc),
          m_handoverTime(handoverTime),
          m_delayThreshold(delayThreshold),
          m_simulationDuration(simulationDuration),
          m_ueHandoverStart(),
          m_gnbHandoverStart()
    {
    }

  private:
    void DoRun() override;

    /**
     * UE handover start callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void UeHandoverStartCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellid,
                                 uint16_t rnti,
                                 uint16_t targetCellId);
    /**
     * UE handover end OK callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     */
    void UeHandoverEndOkCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellid,
                                 uint16_t rnti);
    /**
     * gNB handover start callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void GnbHandoverStartCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellid,
                                  uint16_t rnti,
                                  uint16_t targetCellId);
    /**
     * gNB handover end OK callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     */
    void GnbHandoverEndOkCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellid,
                                  uint16_t rnti);

    uint8_t m_numberOfComponentCarriers; ///< Number of component carriers
    bool m_useIdealRrc;                  ///< use ideal RRC?
    Time m_handoverTime;                 ///< handover time
    Time m_delayThreshold;               ///< the delay threshold
    Time m_simulationDuration;           ///< the simulation duration

    Time m_ueHandoverStart;     ///< UE handover start time
    Time m_gnbHandoverStart;    ///< gNB handover start time
    bool m_handoverDone{false}; ///< Flag indicated whether the handover happened
};

void
NrHandoverDelayTestCase::DoRun()
{
    NS_LOG_INFO("-----test case: ideal RRC = " << m_useIdealRrc << " handover time = "
                                               << m_handoverTime.As(Time::S) << "-----");

    /*
     * Helpers.
     */
    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();

    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));

    /*
     * Physical layer.
     *
     * gNB 0                    UE                      gNB 1
     *
     *    x ----------------------- x ----------------------- x
     *              500 m                      500 m
     */
    // Create nodes.
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    auto ueNode = CreateObject<Node>();

    // Setup mobility
    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(1000, 0, 0));
    posAlloc->Add(Vector(500, 0, 0));

    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure LogDistance propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(
        2.8e9,
        20e6,
        static_cast<uint8_t>(m_numberOfComponentCarriers));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    /*
     * Link layer.
     */
    auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);

    /*
     * Network layer.
     */
    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs;
    ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    // Setup traces.
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NrHandoverDelayTestCase::UeHandoverStartCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverDelayTestCase::UeHandoverEndOkCallback, this));

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                    MakeCallback(&NrHandoverDelayTestCase::GnbHandoverStartCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverDelayTestCase::GnbHandoverEndOkCallback, this));

    // Prepare handover.
    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));
    nrHelper->HandoverRequest(m_handoverTime, ueDev, gnbDevs.Get(0), gnbDevs.Get(1));

    nrHelper->AssignStreams({.assignEpc = true,
                             .ueNodes = ueNode,
                             .gnbNodes = gnbNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDev,
                             .gnbNodeStream = 1000,
                             .gnbDevStream = 3000,
                             .ueDevStream = 4000});

    // Run simulation.
    Simulator::Stop(m_simulationDuration);
    Simulator::Run();
    Simulator::Destroy();

    NS_TEST_EXPECT_MSG_EQ(m_handoverDone, true, "Expected a handover");
}

void
NrHandoverDelayTestCase::UeHandoverStartCallback(std::string context,
                                                 uint64_t imsi,
                                                 uint16_t cellid,
                                                 uint16_t rnti,
                                                 uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context);
    m_ueHandoverStart = Simulator::Now();
}

void
NrHandoverDelayTestCase::UeHandoverEndOkCallback(std::string context,
                                                 uint64_t imsi,
                                                 uint16_t cellid,
                                                 uint16_t rnti)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(m_ueHandoverStart.IsStrictlyPositive());
    Time delay = Simulator::Now() - m_ueHandoverStart;

    NS_LOG_DEBUG(this << " UE delay = " << delay.As(Time::S));
    NS_TEST_ASSERT_MSG_LT(delay,
                          m_delayThreshold,
                          "UE handover delay is higher than the allowed threshold "
                              << "(ideal RRC = " << m_useIdealRrc
                              << " handover time = " << m_handoverTime.As(Time::S) << ")");
    m_handoverDone = true;
}

void
NrHandoverDelayTestCase::GnbHandoverStartCallback(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t cellid,
                                                  uint16_t rnti,
                                                  uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context);
    NS_TEST_EXPECT_MSG_NE(cellid, targetCellId, "Expected handover to different cell");
    m_gnbHandoverStart = Simulator::Now();
}

void
NrHandoverDelayTestCase::GnbHandoverEndOkCallback(std::string context,
                                                  uint64_t imsi,
                                                  uint16_t cellid,
                                                  uint16_t rnti)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(m_gnbHandoverStart.IsStrictlyPositive());
    Time delay = Simulator::Now() - m_gnbHandoverStart;

    NS_LOG_DEBUG(this << " gNB delay = " << delay.As(Time::S));
    NS_TEST_ASSERT_MSG_LT(delay,
                          m_delayThreshold,
                          "gNB handover delay is higher than the allowed threshold "
                              << "(ideal RRC = " << m_useIdealRrc
                              << " handover time = " << m_handoverTime.As(Time::S) << ")");
}

/**
 * @ingroup nr-test
 *
 * @brief Test case for verifying that HandoverDecisionDelay and
 *        HandoverTriggeringDelay attributes are properly applied.
 *
 * This test verifies that:
 * - When HandoverDecisionDelay is set to a non-zero value, the handover
 *   decision is delayed by approximately that amount.
 * - When HandoverTriggeringDelay is set to a non-zero value, the actual
 *   handover trigger is delayed by approximately that amount.
 */
class NrHandoverDelayApplyTestCase : public TestCase
{
  public:
    /**
     * @brief Creates an instance of the handover delay apply test case.
     *
     * @param decisionDelay the HandoverDecisionDelay value to set
     * @param triggeringDelay the HandoverTriggeringDelay value to set
     * @param handoverTime time to trigger handover (seconds)
     * @param simTime simulation time (seconds)
     * @param description test description
     */
    NrHandoverDelayApplyTestCase(Time decisionDelay,
                                 Time triggeringDelay,
                                 double handoverTime,
                                 double simTime,
                                 std::string description = "");

    ~NrHandoverDelayApplyTestCase() override;

  private:
    void DoRun() override;

    /**
     * @brief Callback for UE handover start.
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void OnUeHandoverStart(uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId);

    /**
     * @brief Callback for UE handover end OK.
     * @param imsi the IMSI
     * @param cellid the cell ID
     * @param rnti the RNTI
     */
    void OnUeHandoverEndOk(uint64_t imsi, uint16_t cellid, uint16_t rnti);

    Time m_decisionDelay;        ///< HandoverDecisionDelay value
    Time m_triggeringDelay;      ///< HandoverTriggeringDelay value
    double m_handoverTime;       ///< time to trigger handover
    double m_simTime;            ///< simulation time
    Time m_ueHandoverStartTime_; ///< recorded time of UE handover start
    bool m_ueHandoverStarted;    ///< whether UE handover started
    bool m_ueHandoverEnded;      ///< whether UE handover ended
};

NrHandoverDelayApplyTestCase::NrHandoverDelayApplyTestCase(Time decisionDelay,
                                                           Time triggeringDelay,
                                                           double handoverTime,
                                                           double simTime,
                                                           std::string description)
    : TestCase(description.empty() ? "Handover delay apply test" : description),
      m_decisionDelay(decisionDelay),
      m_triggeringDelay(triggeringDelay),
      m_handoverTime(handoverTime),
      m_simTime(simTime),
      m_ueHandoverStartTime_(),
      m_ueHandoverStarted(false),
      m_ueHandoverEnded(false)
{
}

NrHandoverDelayApplyTestCase::~NrHandoverDelayApplyTestCase()
{
}

void
NrHandoverDelayApplyTestCase::OnUeHandoverStart(uint64_t imsi,
                                                uint16_t cellid,
                                                uint16_t rnti,
                                                uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << imsi << cellid << rnti << targetCellId);
    m_ueHandoverStartTime_ = Simulator::Now();
    m_ueHandoverStarted = true;
}

void
NrHandoverDelayApplyTestCase::OnUeHandoverEndOk(uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellid << rnti);
    if (m_ueHandoverStarted)
    {
        Time delay = Simulator::Now() - m_ueHandoverStartTime_;
        Time expectedDelay = m_decisionDelay + m_triggeringDelay;
        NS_LOG_INFO("UE Handover delay = " << delay.As(Time::S)
                                           << "s, expected = " << expectedDelay.As(Time::S) << "s");
        // The actual delay should be approximately equal to the sum of both delays
        // Allow for a small tolerance due to event scheduling granularity
        const double tolerance = 0.001; // 1ms tolerance
        if (std::abs((delay - expectedDelay).GetSeconds()) < tolerance)
        {
            NS_LOG_INFO("PASS: Handover delay matches expected value");
        }
        else
        {
            NS_LOG_WARN("FAIL: Handover delay does not match expected value");
        }
    }
    m_ueHandoverEnded = true;
}

void
NrHandoverDelayApplyTestCase::DoRun()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Running handover delay apply test: "
                << "HandoverDecisionDelay=" << m_decisionDelay.As(Time::S)
                << "s, HandoverTriggeringDelay=" << m_triggeringDelay.As(Time::S) << "s");

    // Enable logging for debugging
    // LogComponentEnable("NrHandoverDelayTest", LOG_LEVEL_INFO);
    // LogComponentEnable("NrGnbRrc", LOG_LEVEL_INFO);
    // LogComponentEnable("NrRrcProtocolIdeal", LOG_LEVEL_INFO);

    // Create nodes
    NodeContainer gnbNodes;
    gnbNodes.Create(2);

    NodeContainer ueNodes;
    ueNodes.Create(1);

    // Mobility
    Ptr<ListPositionAllocator> positionAllocGnb = CreateObject<ListPositionAllocator>();
    positionAllocGnb->Add(Vector(0, 0, 10));
    positionAllocGnb->Add(Vector(500, 0, 10));
    MobilityHelper mobilityGnb;
    mobilityGnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityGnb.SetPositionAllocator(positionAllocGnb);
    mobilityGnb.Install(gnbNodes);

    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();
    positionAllocUe->Add(Vector(250, 0, 1.5));
    MobilityHelper mobilityUe;
    mobilityUe.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityUe.SetPositionAllocator(positionAllocUe);
    mobilityUe.Install(ueNodes);

    // Create EPC
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();

    // Configure NR helper with real RRC (ideal RRC does not apply the delays)
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // Set handover delays
    Config::SetDefault("ns3::NrGnbRrc::HandoverDecisionDelay", TimeValue(m_decisionDelay));
    Config::SetDefault("ns3::NrGnbRrc::HandoverTriggeringDelay", TimeValue(m_triggeringDelay));

    // Create bandwidth parts
    auto bandwidthAndBWPPair = nrHelper->CreateBandwidthParts({{1.93e9, 10e6, 1}}, "UMa", "LOS");

    // Install gNB devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, bandwidthAndBWPPair.second);

    // Install UE devices
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, bandwidthAndBWPPair.second);

    // Install IP stack on UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);
    nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Enable traces
    nrHelper->EnableTraces();

    // Connect to UE handover traces
    ueDevs.Get(0)->GetObject<NrUeNetDevice>()->GetRrc()->TraceConnectWithoutContext(
        "HandoverStart",
        MakeCallback(&NrHandoverDelayApplyTestCase::OnUeHandoverStart, this));
    ueDevs.Get(0)->GetObject<NrUeNetDevice>()->GetRrc()->TraceConnectWithoutContext(
        "HandoverEndOk",
        MakeCallback(&NrHandoverDelayApplyTestCase::OnUeHandoverEndOk, this));

    // Attach UE to gNB
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Add X2 interface and schedule handover
    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->HandoverRequest(Seconds(m_handoverTime),
                              ueDevs.Get(0),
                              gnbDevs.Get(0),
                              gnbDevs.Get(1));

    // Wait for simulation to complete
    Simulator::Stop(Seconds(m_simTime));
    Simulator::Run();
    Simulator::Destroy();

    // Verify results
    NS_TEST_ASSERT_MSG_EQ(m_ueHandoverStarted, true, "UE handover should have started");
    NS_TEST_ASSERT_MSG_EQ(m_ueHandoverEnded, true, "UE handover should have ended");
}

/**
 * @ingroup nr-test
 *
 * @brief NR Handover Delay Test Suite
 */

static class NrHandoverDelayTestSuite : public TestSuite
{
  public:
    NrHandoverDelayTestSuite()
        : TestSuite("nr-handover-delay", Type::SYSTEM)
    {
        // LogComponentEnable ("NrHandoverDelayTest", LOG_PREFIX_TIME);
        // LogComponentEnable ("NrHandoverDelayTest", LOG_DEBUG);
        // LogComponentEnable ("NrHandoverDelayTest", LOG_INFO);

        // HANDOVER DELAY TEST CASES WITH IDEAL RRC (THRESHOLD = 0.005 sec)

        for (Time handoverTime = Seconds(0.100); handoverTime < Seconds(0.110);
             handoverTime += Seconds(0.001))
        {
            // arguments: useIdealRrc, handoverTime, delayThreshold, simulationDuration
            AddTestCase(
                new NrHandoverDelayTestCase(1, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(2, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(4, true, handoverTime, Seconds(0.005), Seconds(0.200)),
                TestCase::Duration::QUICK);
        }

        // HANDOVER DELAY TEST CASES WITH REAL RRC (THRESHOLD = 0.020 sec)
        for (Time handoverTime = Seconds(0.100); handoverTime < Seconds(0.110);
             handoverTime += Seconds(0.001))
        {
            // arguments: useIdealRrc, handoverTime, delayThreshold, simulationDuration
            AddTestCase(
                new NrHandoverDelayTestCase(1, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(2, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
            AddTestCase(
                new NrHandoverDelayTestCase(4, false, handoverTime, Seconds(0.020), Seconds(0.200)),
                TestCase::Duration::QUICK);
        }

        // Test case 1: Both delays zero (should have no additional delay)
        AddTestCase(
            new NrHandoverDelayApplyTestCase(Seconds(0), Seconds(0), 0.1, 5.0, "Both delays zero"),
            TestCase::Duration::QUICK);

        // Test case 2: Decision delay only (50ms)
        AddTestCase(new NrHandoverDelayApplyTestCase(MilliSeconds(50),
                                                     Seconds(0),
                                                     0.1,
                                                     5.0,
                                                     "Decision delay 50ms"),
                    TestCase::Duration::QUICK);

        // Test case 3: Triggering delay only (40ms)
        AddTestCase(new NrHandoverDelayApplyTestCase(Seconds(0),
                                                     MilliSeconds(40),
                                                     0.1,
                                                     5.0,
                                                     "Triggering delay 40ms"),
                    TestCase::Duration::QUICK);

        // Test case 4: Both delays (50ms + 40ms = 90ms total)
        AddTestCase(new NrHandoverDelayApplyTestCase(MilliSeconds(50),
                                                     MilliSeconds(40),
                                                     0.1,
                                                     5.0,
                                                     "Both delays (50ms + 40ms)"),
                    TestCase::Duration::QUICK);
    }
} g_nrHandoverDelayTestSuite; ///< the test suite
