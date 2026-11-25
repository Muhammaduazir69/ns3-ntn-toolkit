// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only and NIST-Software

/**
 * @ingroup tests
 * @file nr-test-harq-feedback-timing.cc
 * @brief Test suite `nr-harq-feedback-timing`: the UE sends DL HARQ feedback in the slot
 * given by K1, including slots in which it also transmits an SRS.
 *
 * A UL CTRL opportunity with nothing to send used to leave the UE channel status GRANTED
 * into the next slot, so that slot requested no channel access. When the next slot carried
 * an SRS, the SRS reset the channel status and the UL CTRL that followed found the channel
 * not granted; the DL HARQ feedback due in that slot was sent one slot late.
 *
 * Regression test for the issue reported in 5G LENA nr Work Item #292.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/test.h"

#include <map>
#include <set>
#include <vector>

namespace ns3
{

/**
 * @ingroup tests
 * @brief Checks that each DL HARQ feedback is sent in the slot of its DL DCI plus K1.
 *
 * One UE receives DL packets at an interval that is not a whole number of slots, so that
 * the slots in which feedback is due sweep across the UE SRS slots, and so that the slot
 * before such a slot often has an empty UL CTRL opportunity.
 */
class NrHarqFeedbackTimingTestCase : public TestCase
{
  public:
    NrHarqFeedbackTimingTestCase();

  private:
    void DoRun() override;

    /**
     * Record the slot in which the feedback for a DL DCI is due.
     *
     * @param sfn Slot in which the DCI was received.
     * @param cellId Cell ID.
     * @param rnti UE RNTI.
     * @param bwpId BWP ID.
     * @param harqId HARQ process ID.
     * @param k1 K1 delay in slots.
     */
    void DlDciTrace(SfnSf sfn,
                    uint16_t cellId,
                    uint16_t rnti,
                    uint8_t bwpId,
                    uint8_t harqId,
                    uint32_t k1);

    /**
     * Compare the slot in which a DL HARQ feedback is sent with the slot in which it is due.
     *
     * @param sfn Slot in which the feedback is sent.
     * @param cellId Cell ID.
     * @param rnti UE RNTI.
     * @param bwpId BWP ID.
     * @param harqId HARQ process ID.
     * @param k1 K1 delay in slots.
     */
    void HarqFeedbackTrace(SfnSf sfn,
                           uint16_t cellId,
                           uint16_t rnti,
                           uint8_t bwpId,
                           uint8_t harqId,
                           uint32_t k1);

    /**
     * Record the slots in which the UE transmits an SRS.
     *
     * @param sfn Slot in which the message is sent.
     * @param cellId Cell ID.
     * @param rnti UE RNTI.
     * @param bwpId BWP ID.
     * @param msg The control message.
     */
    void TxCtrlMsgTrace(SfnSf sfn,
                        uint16_t cellId,
                        uint16_t rnti,
                        uint8_t bwpId,
                        Ptr<const NrControlMessage> msg);

    std::map<uint8_t, SfnSf> m_dueSlot; ///< Slot in which feedback is due, per HARQ process
    std::vector<SfnSf> m_feedbackDue;   ///< Due slot of each feedback sent
    std::set<uint64_t> m_srsSlots;      ///< Encodings of the slots carrying an SRS
    uint32_t m_late{0};                 ///< Feedbacks sent after their due slot
};

NrHarqFeedbackTimingTestCase::NrHarqFeedbackTimingTestCase()
    : TestCase("DL HARQ feedback is sent in the slot given by K1, also in SRS slots")
{
}

void
NrHarqFeedbackTimingTestCase::DlDciTrace(SfnSf sfn,
                                         uint16_t cellId,
                                         uint16_t rnti,
                                         uint8_t bwpId,
                                         uint8_t harqId,
                                         uint32_t k1)
{
    sfn.Add(k1);
    m_dueSlot.insert_or_assign(harqId, sfn);
}

void
NrHarqFeedbackTimingTestCase::HarqFeedbackTrace(SfnSf sfn,
                                                uint16_t cellId,
                                                uint16_t rnti,
                                                uint8_t bwpId,
                                                uint8_t harqId,
                                                uint32_t k1)
{
    auto it = m_dueSlot.find(harqId);
    NS_TEST_ASSERT_MSG_EQ((it != m_dueSlot.end()),
                          true,
                          "DL HARQ feedback sent for a HARQ process with no DL DCI");
    m_feedbackDue.push_back(it->second);
    if (!(sfn == it->second))
    {
        m_late++;
    }
}

void
NrHarqFeedbackTimingTestCase::TxCtrlMsgTrace(SfnSf sfn,
                                             uint16_t cellId,
                                             uint16_t rnti,
                                             uint8_t bwpId,
                                             Ptr<const NrControlMessage> msg)
{
    if (msg->GetMessageType() == NrControlMessage::SRS)
    {
        m_srsSlots.insert(sfn.GetEncoding());
    }
}

void
NrHarqFeedbackTimingTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // SRS every 10 slots gives many SRS slots in a short simulation
    Config::SetDefault("ns3::NrMacSchedulerSrsDefault::StartingPeriodicity", UintegerValue(10));

    NodeContainer gnbNode;
    NodeContainer ueNode;
    gnbNode.Create(1);
    ueNode.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNode);
    mobility.Install(ueNode);
    gnbNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    ueNode.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(20, 0, 1.5));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(28e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});

    auto allBwps = CcBwpCreator::GetAllBwps({band});
    NetDeviceContainer gnbDevice = nrHelper->InstallGnbDevice(gnbNode, allBwps);
    NetDeviceContainer ueDevice = nrHelper->InstallUeDevice(ueNode, allBwps);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNode);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDevice);

    nrHelper->AttachToClosestGnb(ueDevice, gnbDevice);

    uint16_t dlPort = 1234;
    UdpServerHelper dlServer(dlPort);
    ApplicationContainer serverApps = dlServer.Install(ueNode);
    UdpClientHelper dlClient(ueIpIface.GetAddress(0), dlPort);
    dlClient.SetAttribute("MaxPackets", UintegerValue(0));
    dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(3300)));
    dlClient.SetAttribute("PacketSize", UintegerValue(100));
    ApplicationContainer clientApps = dlClient.Install(remoteHost);
    serverApps.Start(MilliSeconds(400));
    clientApps.Start(MilliSeconds(400));

    Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevice.Get(0), 0);
    uePhy->TraceConnectWithoutContext(
        "UePhyRxedDlDciTrace",
        MakeCallback(&NrHarqFeedbackTimingTestCase::DlDciTrace, this));
    uePhy->TraceConnectWithoutContext(
        "UePhyTxedHarqFeedbackTrace",
        MakeCallback(&NrHarqFeedbackTimingTestCase::HarqFeedbackTrace, this));
    uePhy->TraceConnectWithoutContext(
        "UePhyTxedCtrlMsgsTrace",
        MakeCallback(&NrHarqFeedbackTimingTestCase::TxCtrlMsgTrace, this));

    nrHelper->AssignStreams(gnbDevice, 1);
    nrHelper->AssignStreams(ueDevice, 1000);

    Simulator::Stop(MilliSeconds(1000));
    Simulator::Run();

    uint32_t dueInSrsSlot = 0;
    for (const auto& due : m_feedbackDue)
    {
        if (m_srsSlots.contains(due.GetEncoding()))
        {
            dueInSrsSlot++;
        }
    }

    NS_TEST_ASSERT_MSG_GT(dueInSrsSlot,
                          0,
                          "no DL HARQ feedback was due in an SRS slot; the scenario does not "
                          "exercise the SRS case");
    NS_TEST_ASSERT_MSG_EQ(m_late, 0, "DL HARQ feedback was sent after the slot given by K1");

    Simulator::Destroy();
}

/**
 * @ingroup tests
 * @brief Test suite for the timing of DL HARQ feedback.
 */
class NrHarqFeedbackTimingTestSuite : public TestSuite
{
  public:
    NrHarqFeedbackTimingTestSuite();
};

NrHarqFeedbackTimingTestSuite::NrHarqFeedbackTimingTestSuite()
    : TestSuite("nr-harq-feedback-timing", Type::SYSTEM)
{
    AddTestCase(new NrHarqFeedbackTimingTestCase(), Duration::QUICK);
}

static NrHarqFeedbackTimingTestSuite g_nrHarqFeedbackTimingTestSuite; //!< Test suite

} // namespace ns3
