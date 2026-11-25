// Copyright (c) 2018 Fraunhofer ESK
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Vignesh Babu <ns3-dev@esk.fraunhofer.de>
// Modified by:
//         Gabriel Ferreira <gabrielcarvfer@gmail.com> (included initial cell selection,
//         rlf and reselection, and state machine checks. Originally nr-test-radio-link-failure)
//

/**
 * @ingroup test
 * @file nr-test-cell-selection-with-rlf.cc
 *
 * @brief Test suite (nr-cell-selection-with-rlf) covering initial cell selection followed by
 * radio link failure (RLF) and reselection. One gNB and one UE are set up over the EPC with two
 * dedicated QoS flows carrying bidirectional UDP traffic. The UE first camps and connects
 * normally, is teleported 10 km away at 4 s so that out-of-sync indications accumulate and RLF
 * is declared, and is teleported back at 6 s so that it reselects the cell and reconnects. The
 * test asserts full attachment (RRC state CONNECTED_NORMALLY, matching UeManager state, cell ID,
 * bandwidths, ARFCNs and bearer configuration on both sides) at 2 s and again at 8 s after the
 * recovery, while tracking the RadioLinkFailure and PhySyncDetection traces. Currently only the
 * ideal RRC variant is exercised.
 */

#include "nr-test-radio-link-failure.h"

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/udp-client-server-helper.h"

#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrCellSelectionWithRadioLinkFailureTest");

/**
 * @ingroup nr
 *
 * @brief Testing the cell reselection procedure by UE at IDLE state, then after radio link failure
 */
class NrCellSelectionWithRlfTestCase : public TestCase
{
  public:
    /**
     * @brief Creates an instance of the radio link failure test case.
     *
     * @param isIdealRrc if true, simulation uses Ideal RRC protocol, otherwise
     *                   simulation uses Real RRC protocol
     */
    NrCellSelectionWithRlfTestCase(bool isIdealRrc);

    ~NrCellSelectionWithRlfTestCase() override;

  private:
    /**
     * @brief Setup the simulation according to the configuration set by the
     *        class constructor, run it, and verify the result.
     */
    void DoRun() override;

    /**
     * Check connected function
     * @param ueDevice the UE device
     * @param gnbDevices the gNB devices
     */
    void CheckConnected(Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices);

    /**
     * Check if the UE is in idle state
     * @param ueDevice the UE device
     * @param gnbDevices the gNB devices
     */
    void CheckIdle(Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices);

    /**
     * @brief Check if the UE exist at the gNB
     * @param rnti the RNTI of the UE
     * @param gnbDevice the gNB device
     * @return true if the UE exist at the eNB, otherwise false
     */
    bool CheckUeExistAtGnb(uint16_t rnti, Ptr<NetDevice> gnbDevice);

    /**
     * @brief State transition callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     * @param oldState the old state
     * @param newState the new state
     */
    void UeStateTransitionCallback(std::string context,
                                   uint64_t imsi,
                                   uint16_t cellId,
                                   uint16_t rnti,
                                   NrUeRrc::State oldState,
                                   NrUeRrc::State newState);

    /**
     * @brief Connection established at UE callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionEstablishedUeCallback(std::string context,
                                         uint64_t imsi,
                                         uint16_t cellId,
                                         uint16_t rnti);

    /**
     * @brief Connection established at eNodeB callback function
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionEstablishedGnbCallback(std::string context,
                                          uint64_t imsi,
                                          uint16_t cellId,
                                          uint16_t rnti);

    /**
     * @brief This callback function is executed when UE context is removed at eNodeB
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void ConnectionReleaseAtGnbCallback(std::string context,
                                        uint64_t imsi,
                                        uint16_t cellId,
                                        uint16_t rnti);

    /**
     * @brief This callback function is executed when UE RRC receives an in-sync or out-of-sync
     * indication
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param cellId the cell ID
     * @param type in-sync or out-of-sync indication
     * @param count the number of in-sync or out-of-sync indications
     */
    void PhySyncDetectionCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t rnti,
                                  uint16_t cellId,
                                  std::string type,
                                  uint8_t count);

    /**
     * @brief This callback function is executed when radio link failure is detected
     * @param context the context string
     * @param imsi the IMSI
     * @param rnti the RNTI
     * @param cellId the cell ID
     */
    void RadioLinkFailureCallback(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti);

    /**
     * @brief Jump away function
     *
     * @param UeJumpAwayPositionList A list of positions where UE would jump
     */
    void JumpAway(Vector UeJumpAwayPositionList);

    bool m_isIdealRrc;           ///< whether the NR is configured to use ideal RRC
    Vector m_ueJumpAwayPosition; ///< Position where the UE(s) would jump

    /// The current UE RRC state.
    NrUeRrc::State m_lastState;

    bool m_radioLinkFailureDetected;      ///< true if radio link fails
    uint32_t m_numOfInSyncIndications;    ///< number of in-sync indications detected
    uint32_t m_numOfOutOfSyncIndications; ///< number of out-of-sync indications detected
    Ptr<MobilityModel> m_ueMobility;      ///< UE mobility model

}; // end of class NrRadioLinkFailureTestCase

NrCellSelectionWithRlfTestCase::NrCellSelectionWithRlfTestCase(bool isIdealRrc)
    : TestCase("Initial cell selection with RLF and reselection"),
      m_isIdealRrc(isIdealRrc),
      m_lastState(NrUeRrc::NUM_STATES),
      m_radioLinkFailureDetected(false),
      m_numOfInSyncIndications(0),
      m_numOfOutOfSyncIndications(0)
{
    NS_LOG_FUNCTION(this << GetName());
}

NrCellSelectionWithRlfTestCase::~NrCellSelectionWithRlfTestCase()
{
    NS_LOG_FUNCTION(this << GetName());
}

void
NrCellSelectionWithRlfTestCase::DoRun()
{
    NS_LOG_FUNCTION(this << GetName());
    uint16_t numBearersPerUe = 2;
    double eNodeB_txPower = 40;

    Config::SetDefault("ns3::NrHelper::UseIdealRrc", BooleanValue(m_isIdealRrc));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);

    //----power related (equal for all base stations)----
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(eNodeB_txPower));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));

    //----frequency related----
    auto bandwidthAndBWPPair = nrHelper->CreateBandwidthParts({{1.93e9, 5e6, 1}}, "UMa");
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Enable PMI so we have precoding matrices too
    NrHelper::MimoPmiParams mimoPmiParams;
    mimoPmiParams.subbandSize = 4;
    nrHelper->SetupMimoPmi(mimoPmiParams);

    // Create the internet
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    // Create a single RemoteHost0x18ab460
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
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Create Nodes: eNodeB and UE
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(1);
    ueNodes.Create(1);

    // Mobility
    Ptr<ListPositionAllocator> positionAllocGnb = CreateObject<ListPositionAllocator>();
    positionAllocGnb->Add(Vector3D(0, 0, 0));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAllocGnb);
    mobility.Install(gnbNodes);

    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();
    positionAllocUe->Add(Vector3D(100, 0, 0));

    mobility.SetPositionAllocator(positionAllocUe);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    // Install NR Devices in gNB and UEs
    NetDeviceContainer gnbDevs;
    NetDeviceContainer ueDevs;

    gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, bandwidthAndBWPPair.second);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, bandwidthAndBWPPair.second);
    nrHelper->AssignStreams({.gnbDevs = gnbDevs, .ueDevs = ueDevs});

    auto ueNetDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(0));
    ueNetDev->GetNas()->Connect();

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;

    DataRateValue dataRateValue = DataRate("18.6Mbps");
    uint64_t bitRate = dataRateValue.Get().GetBitRate();
    uint32_t packetSize = 1024; // bytes
    NS_LOG_DEBUG("bit rate " << bitRate);
    double interPacketInterval = static_cast<double>(packetSize * 8) / bitRate;
    Time udpInterval = Seconds(interPacketInterval);

    NS_LOG_DEBUG("UDP will use application interval " << udpInterval.As(Time::S));

    for (uint32_t b = 0; b < numBearersPerUe; ++b)
    {
        ApplicationContainer ulClientApps;
        ApplicationContainer ulServerApps;
        ApplicationContainer dlClientApps;
        ApplicationContainer dlServerApps;

        ++dlPort;
        ++ulPort;

        NS_LOG_LOGIC("installing UDP DL app for UE 0");
        UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(0), dlPort);
        dlClientHelper.SetAttribute("Interval", TimeValue(udpInterval));
        dlClientHelper.SetAttribute("MaxPackets", UintegerValue(1000000));
        dlClientApps.Add(dlClientHelper.Install(remoteHost));

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        dlServerApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(0)));

        NS_LOG_LOGIC("installing UDP UL app for UE 0");
        UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
        ulClientHelper.SetAttribute("Interval", TimeValue(udpInterval));
        ulClientHelper.SetAttribute("MaxPackets", UintegerValue(1000000));
        ulClientApps.Add(ulClientHelper.Install(ueNodes.Get(0)));

        PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        ulServerApps.Add(ulPacketSinkHelper.Install(remoteHost));

        Ptr<NrQosRule> rule = Create<NrQosRule>();
        NrQosRule::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        rule->Add(dlpf);
        NrQosRule::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd = ulPort;
        rule->Add(ulpf);
        NrQosFlow flow(NrQosFlow::NGBR_IMS);
        nrHelper->ActivateDedicatedQosFlow(ueDevs.Get(0), flow, rule);

        dlServerApps.Start(Seconds(0.27));
        dlClientApps.Start(Seconds(0.27));
        ulServerApps.Start(Seconds(0.27));
        ulClientApps.Start(Seconds(0.27));
    }

    auto gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0));
    DynamicCast<NrUeNetDevice>(ueDevs.Get(0))
        ->GetNas()
        ->Connect(gnbDev->GetCellId(), gnbDev->GetBwpArfcn(0));

    // Check that by second 2, the UE is properly attached to the gNB
    Simulator::Schedule(Seconds(2),
                        &NrCellSelectionWithRlfTestCase::CheckConnected,
                        this,
                        ueDevs.Get(0),
                        gnbDevs);

    // Jump far away to lose the connection
    Simulator::Schedule(Seconds(4),
                        &NrCellSelectionWithRlfTestCase::JumpAway,
                        this,
                        Vector3D(10000, 0, 0));

    // Let RLF with default settings take its place, and gNB kick out UE
    // Then teleport back to near the gNB
    Simulator::Schedule(Seconds(6),
                        &NrCellSelectionWithRlfTestCase::JumpAway,
                        this,
                        Vector3D(100, 0, 0));
    Simulator::Schedule(Seconds(8),
                        &NrCellSelectionWithRlfTestCase::CheckConnected,
                        this,
                        ueDevs.Get(0),
                        gnbDevs);
    // connect custom trace sinks
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrGnbRrc/ConnectionEstablished",
        MakeCallback(&NrCellSelectionWithRlfTestCase::ConnectionEstablishedGnbCallback, this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
        MakeCallback(&NrCellSelectionWithRlfTestCase::ConnectionEstablishedUeCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/StateTransition",
                    MakeCallback(&NrCellSelectionWithRlfTestCase::UeStateTransitionCallback, this));
    Config::Connect(
        "/NodeList/*/DeviceList/*/NrGnbRrc/NotifyConnectionRelease",
        MakeCallback(&NrCellSelectionWithRlfTestCase::ConnectionReleaseAtGnbCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/PhySyncDetection",
                    MakeCallback(&NrCellSelectionWithRlfTestCase::PhySyncDetectionCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
                    MakeCallback(&NrCellSelectionWithRlfTestCase::RadioLinkFailureCallback, this));

    Simulator::Stop(Seconds(9));
    Simulator::Run();
    Simulator::Destroy();
} // end of void NrCellSelectionWithRlfTestCase::DoRun ()

void
NrCellSelectionWithRlfTestCase::JumpAway(Vector UeJumpAwayPosition)
{
    NS_LOG_FUNCTION(this);
    // move to a far away location so that transmission errors occur

    m_ueMobility->SetPosition(UeJumpAwayPosition);
}

void
NrCellSelectionWithRlfTestCase::CheckConnected(Ptr<NetDevice> ueDevice,
                                               NetDeviceContainer gnbDevices)
{
    NS_LOG_FUNCTION(ueDevice);

    Ptr<NrUeNetDevice> ueNrDevice = ueDevice->GetObject<NrUeNetDevice>();
    Ptr<NrUeRrc> ueRrc = ueNrDevice->GetRrc();
    NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(), NrUeRrc::CONNECTED_NORMALLY, "Wrong NrUeRrc state!");
    uint16_t cellId = ueRrc->GetCellId();

    Ptr<NrGnbNetDevice> nrGnbDevice;

    for (auto gnbDevIt = gnbDevices.Begin(); gnbDevIt != gnbDevices.End(); ++gnbDevIt)
    {
        if (((*gnbDevIt)->GetObject<NrGnbNetDevice>())->GetRrc()->HasCellId(cellId))
        {
            nrGnbDevice = (*gnbDevIt)->GetObject<NrGnbNetDevice>();
        }
    }

    NS_TEST_ASSERT_MSG_NE(nrGnbDevice, nullptr, "LTE gNB device not found");
    Ptr<NrGnbRrc> gnbRrc = nrGnbDevice->GetRrc();
    uint16_t rnti = ueRrc->GetRnti();
    Ptr<NrUeManager> ueManager = gnbRrc->GetUeManager(rnti);
    NS_TEST_ASSERT_MSG_NE(ueManager, nullptr, "RNTI " << rnti << " not found in eNB");

    NrUeManager::State ueManagerState = ueManager->GetState();
    NS_TEST_ASSERT_MSG_EQ(ueManagerState,
                          NrUeManager::CONNECTED_NORMALLY,
                          "Wrong NrUeManager state!");
    NS_ASSERT_MSG(ueManagerState == NrUeManager::CONNECTED_NORMALLY, "Wrong NrUeManager state!");

    uint16_t ueCellId = ueRrc->GetCellId();
    uint16_t gnbCellId = nrGnbDevice->GetCellId();
    NS_TEST_ASSERT_MSG_EQ(ueCellId, gnbCellId, "gNB does not contain UE cellId");

    // Verifying other attributes on both sides.
    uint16_t ueDlBwp = ueRrc->GetPrimaryDlIndex();
    uint16_t ueUlBwp = ueRrc->GetPrimaryUlIndex();
    uint32_t ueDlArfcn = ueNrDevice->GetBwpArfcn(ueDlBwp);
    uint32_t ueUlArfcn = ueNrDevice->GetBwpArfcn(ueUlBwp);
    uint8_t ueDlBandwidth = ueRrc->GetDlBandwidth();
    uint8_t ueUlBandwidth = ueRrc->GetUlBandwidth();

    uint16_t gnbDlBwp = nrGnbDevice->GetArfcnBwpId(ueDlArfcn);
    uint16_t gnbUlBwp = nrGnbDevice->GetArfcnBwpId(ueUlArfcn);
    uint8_t gnbDlBandwidth = nrGnbDevice->GetBwpDlBandwidth(gnbDlBwp);
    uint8_t gnbUlBandwidth = nrGnbDevice->GetBwpUlBandwidth(gnbUlBwp);
    uint32_t gnbDlArfcn = nrGnbDevice->GetBwpArfcn(gnbDlBwp);
    uint32_t gnbUlArfcn = nrGnbDevice->GetBwpArfcn(gnbUlBwp);

    NS_TEST_ASSERT_MSG_EQ(gnbRrc->HasCellId(ueCellId), true, "inconsistent CellId");
    NS_TEST_ASSERT_MSG_EQ(ueDlBandwidth, gnbDlBandwidth, "inconsistent DlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueUlBandwidth, gnbUlBandwidth, "inconsistent UlBandwidth");
    NS_TEST_ASSERT_MSG_EQ(ueDlArfcn, gnbDlArfcn, "inconsistent DlArfcn");
    NS_TEST_ASSERT_MSG_EQ(ueUlArfcn, gnbUlArfcn, "inconsistent UlArfcn");

    ObjectMapValue gnbDataRadioBearerMapValue;
    ueManager->GetAttribute("DataRadioBearerMap", gnbDataRadioBearerMapValue);
    NS_TEST_ASSERT_MSG_EQ(gnbDataRadioBearerMapValue.GetN(), 1 + 1, "wrong num bearers at eNB");

    ObjectMapValue ueDataRadioBearerMapValue;
    ueRrc->GetAttribute("DataRadioBearerMap", ueDataRadioBearerMapValue);
    NS_TEST_ASSERT_MSG_EQ(ueDataRadioBearerMapValue.GetN(), 1 + 1, "wrong num bearers at UE");

    auto gnbBearerIt = gnbDataRadioBearerMapValue.Begin();
    auto ueBearerIt = ueDataRadioBearerMapValue.Begin();
    while (gnbBearerIt != gnbDataRadioBearerMapValue.End() &&
           ueBearerIt != ueDataRadioBearerMapValue.End())
    {
        Ptr<NrDataRadioBearerInfo> gnbDrbInfo =
            gnbBearerIt->second->GetObject<NrDataRadioBearerInfo>();
        Ptr<NrDataRadioBearerInfo> ueDrbInfo =
            ueBearerIt->second->GetObject<NrDataRadioBearerInfo>();
        NS_TEST_ASSERT_MSG_EQ((uint32_t)gnbDrbInfo->m_qosFlowIdentity,
                              (uint32_t)ueDrbInfo->m_qosFlowIdentity,
                              "qosFlowIdentity differs");
        NS_TEST_ASSERT_MSG_EQ((uint32_t)gnbDrbInfo->m_drbIdentity,
                              (uint32_t)ueDrbInfo->m_drbIdentity,
                              "drbIdentity differs");
        NS_TEST_ASSERT_MSG_EQ((uint32_t)gnbDrbInfo->m_logicalChannelIdentity,
                              (uint32_t)ueDrbInfo->m_logicalChannelIdentity,
                              "logicalChannelIdentity differs");

        ++gnbBearerIt;
        ++ueBearerIt;
    }
    NS_ASSERT_MSG(gnbBearerIt == gnbDataRadioBearerMapValue.End(), "too many bearers at eNB");
    NS_ASSERT_MSG(ueBearerIt == ueDataRadioBearerMapValue.End(), "too many bearers at UE");
}

void
NrCellSelectionWithRlfTestCase::CheckIdle(Ptr<NetDevice> ueDevice, NetDeviceContainer gnbDevices)
{
    NS_LOG_FUNCTION(ueDevice);

    Ptr<NrUeNetDevice> ueNrDevice = ueDevice->GetObject<NrUeNetDevice>();
    Ptr<NrUeRrc> ueRrc = ueNrDevice->GetRrc();
    uint16_t rnti = ueRrc->GetRnti();
    uint32_t numGnbDevices = gnbDevices.GetN();
    bool ueManagerFound = false;

    switch (numGnbDevices)
    {
    // 1 eNB
    case 1:
        NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(), NrUeRrc::IDLE_CELL_SEARCH, "Wrong NrUeRrc state!");
        ueManagerFound = CheckUeExistAtGnb(rnti, gnbDevices.Get(0));
        NS_TEST_ASSERT_MSG_EQ(ueManagerFound,
                              false,
                              "Unexpected RNTI with value " << rnti << " found in eNB");
        break;
    // 2 eNBs
    case 2:
        NS_TEST_ASSERT_MSG_EQ(ueRrc->GetState(),
                              NrUeRrc::CONNECTED_NORMALLY,
                              "Wrong NrUeRrc state!");
        ueManagerFound = CheckUeExistAtGnb(rnti, gnbDevices.Get(1));
        NS_TEST_ASSERT_MSG_EQ(ueManagerFound,
                              true,
                              "RNTI " << rnti << " is not attached to the eNB");
        break;
    default:
        NS_FATAL_ERROR("The RRC state of the UE in more then 2 gNB scenario is not defined. "
                       "Consider creating more cases");
        break;
    }
}

bool
NrCellSelectionWithRlfTestCase::CheckUeExistAtGnb(uint16_t rnti, Ptr<NetDevice> gnbDevice)
{
    NS_LOG_FUNCTION(this << rnti);
    Ptr<NrGnbNetDevice> nrGnbDevice = DynamicCast<NrGnbNetDevice>(gnbDevice);
    NS_ABORT_MSG_IF(!nrGnbDevice, "LTE gNB device not found");
    Ptr<NrGnbRrc> gnbRrc = nrGnbDevice->GetRrc();
    bool ueManagerFound = gnbRrc->HasUeManager(rnti);
    return ueManagerFound;
}

void
NrCellSelectionWithRlfTestCase::UeStateTransitionCallback(std::string context,
                                                          uint64_t imsi,
                                                          uint16_t cellId,
                                                          uint16_t rnti,
                                                          NrUeRrc::State oldState,
                                                          NrUeRrc::State newState)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti << oldState << newState);
    m_lastState = newState;
}

void
NrCellSelectionWithRlfTestCase::ConnectionEstablishedGnbCallback(std::string context,
                                                                 uint64_t imsi,
                                                                 uint16_t cellId,
                                                                 uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
NrCellSelectionWithRlfTestCase::ConnectionEstablishedUeCallback(std::string context,
                                                                uint64_t imsi,
                                                                uint16_t cellId,
                                                                uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
NrCellSelectionWithRlfTestCase::ConnectionReleaseAtGnbCallback(std::string context,
                                                               uint64_t imsi,
                                                               uint16_t cellId,
                                                               uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
}

void
NrCellSelectionWithRlfTestCase::PhySyncDetectionCallback(std::string context,
                                                         uint64_t imsi,
                                                         uint16_t rnti,
                                                         uint16_t cellId,
                                                         std::string type,
                                                         uint8_t count)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    if (type == "Notify out of sync")
    {
        m_numOfOutOfSyncIndications = count;
    }
    else if (type == "Notify in sync")
    {
        m_numOfInSyncIndications = count;
    }
}

void
NrCellSelectionWithRlfTestCase::RadioLinkFailureCallback(std::string context,
                                                         uint64_t imsi,
                                                         uint16_t cellId,
                                                         uint16_t rnti)
{
    NS_LOG_FUNCTION(this << imsi << cellId << rnti);
    NS_LOG_DEBUG("RLF at " << Simulator::Now());
    m_radioLinkFailureDetected = true;
    m_numOfOutOfSyncIndications = 0;
}

/*
 * Test Suite
 */

/**
 * @brief Test suite for
 *
 * \sa ns3::NrRadioLinkFailureTestCase
 */
class NrCellSelectionWithRlfTestSuite : public TestSuite
{
  public:
    NrCellSelectionWithRlfTestSuite();
};

NrCellSelectionWithRlfTestSuite::NrCellSelectionWithRlfTestSuite()
    : TestSuite("nr-cell-selection-with-rlf", Type::SYSTEM)
{
    // One gNB: Ideal RRC PROTOCOL
    //
    AddTestCase(new NrCellSelectionWithRlfTestCase(true), TestCase::Duration::QUICK);

    // One eNB: Real RRC PROTOCOL todo: re-enable when RRC real is fully working
    // AddTestCase(new NrCellSelectionWithRlfTestCase(false),
    //            TestCase::Duration::QUICK);

} // end of NrCellSelectionWithRlfTestSuite::NrCellSelectionWithRlfTestSuite ()

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrCellSelectionWithRlfTestSuite g_nrCellSelectionWithRlfTestSuite;
