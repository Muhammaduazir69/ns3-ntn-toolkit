// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/test.h"

/**
 * @file nr-test-numerology-reception.cc
 * @ingroup tests
 * @brief Numerology (re)configuration of a UE and its ongoing data receptions
 */

namespace ns3
{

/**
 * @ingroup tests
 * @brief Where the numerology of the UE is (re)applied while it receives downlink data.
 */
enum class NumerologyEvent
{
    NONE,            //!< Only the periodic MIB and SIB1 of the serving cell
    MIB,             //!< A MIB announcing another numerology on the serving carrier
    SIB1_RECONFIG,   //!< NrUeRrc::ReconfigureFromSib1 with the serving numerology
    PHY_NUMEROLOGY,  //!< NrPhy::SetNumerology with the serving numerology
    RRC_RECONFIG,    //!< An RRC connection reconfiguration of the unchanged serving BWP
    RECONFIG_CHANGE, //!< NrUeRrc::ReconfigureFromSib1 with another numerology, after the run
    PHY_CHANGE,      //!< NrPhy::SetNumerology with another numerology, after the run
};

/**
 * @ingroup tests
 * @brief A connected UE keeps decoding its downlink data while its numerology is (re)applied.
 *
 * The numerology of the BWP serving a connected UE changes only with its configuration: RRC
 * (re)configuration, handover or BWP switch (TS 38.331, TS 38.321 clause 5.15). The subcarrier
 * spacing of a MIB is the one of SIB1, Msg2/4, paging and broadcast SI, used to acquire a cell.
 * Changing the numerology recomputes the number of RBs and the noise PSD of the PHY, which aborts
 * the reception in progress; the transport block of an aborted reception is decoded with the
 * SINR of a previous one.
 *
 * A UE attaches to one gNB, which starts it at numerology 0 and moves it to the numerology of
 * the cell through cell selection, MIB, SIB1 and the RRC configuration, and receives saturating
 * downlink traffic. While it receives data, the numerology is (re)applied through the given
 * event. Every decoded transport block must come from a reception whose SINR was evaluated, the
 * UE must stay on the numerology of the cell, and a numerology change requested by RRC or on the
 * PHY must still reconfigure the slot and the RBs.
 */
class NrNumerologyReceptionTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name Name of the test case.
     * @param numerology Numerology of the gNB.
     * @param idealRrc Whether the RRC messages are delivered ideally.
     * @param event The numerology (re)configuration exercised.
     */
    NrNumerologyReceptionTestCase(const std::string& name,
                                  uint16_t numerology,
                                  bool idealRrc,
                                  NumerologyEvent event)
        : TestCase(name),
          m_numerology(numerology),
          m_idealRrc(idealRrc),
          m_event(event)
    {
    }

  private:
    void DoRun() override;

    /**
     * Start of a data reception on the UE: inject the event in the middle of some of them.
     *
     * @param sfnSf Slot of the reception.
     * @param psd Received PSD.
     * @param duration Duration of the reception.
     * @param bwpId BWP of the reception.
     * @param cellId Cell of the transmitter.
     */
    void RxDataStart(const SfnSf& sfnSf,
                     Ptr<const SpectrumValue> psd,
                     const Time& duration,
                     uint16_t bwpId,
                     uint16_t cellId);

    /// Apply the event under test to the UE.
    void InjectEvent();

    /**
     * End of a reception whose SINR was evaluated.
     *
     * @param snr Average SNR of the reception.
     */
    void SnrEvaluated(double snr);

    /**
     * The UE applied an RRC connection reconfiguration.
     *
     * @param imsi IMSI of the UE.
     * @param cellId Serving cell.
     * @param rnti RNTI of the UE.
     */
    void Reconfigured(uint64_t imsi, uint16_t cellId, uint16_t rnti);

    /**
     * A downlink transport block was decoded.
     *
     * @param params Parameters of the transport block.
     */
    void TbDecoded(RxPacketTraceParams params);

    uint16_t m_numerology;          ///< Numerology of the gNB
    bool m_idealRrc;                ///< Ideal RRC message delivery
    NumerologyEvent m_event;        ///< Event under test
    Ptr<NrUeNetDevice> m_ueDev;     ///< The UE
    Ptr<NrGnbNetDevice> m_gnbDev;   ///< The gNB
    Time m_lastEvaluation{-1};      ///< Time of the last reception with an evaluated SINR
    uint32_t m_receptions{0};       ///< Data receptions started
    uint32_t m_injected{0};         ///< Events injected
    uint32_t m_decodedTbs{0};       ///< Downlink transport blocks decoded
    uint32_t m_unevaluatedTbs{0};   ///< Transport blocks decoded from an aborted reception
    uint32_t m_reconfigurations{0}; ///< RRC connection reconfigurations applied by the UE
};

void
NrNumerologyReceptionTestCase::RxDataStart(const SfnSf& sfnSf,
                                           Ptr<const SpectrumValue> psd,
                                           const Time& duration,
                                           uint16_t bwpId,
                                           uint16_t cellId)
{
    // Every 50th reception, so that the UE keeps receiving between events
    if (m_ueDev->GetRrc()->GetState() == NrUeRrc::CONNECTED_NORMALLY && ++m_receptions % 50 == 0)
    {
        Simulator::Schedule(duration / 2, &NrNumerologyReceptionTestCase::InjectEvent, this);
    }
}

void
NrNumerologyReceptionTestCase::InjectEvent()
{
    const auto bwp = 0;
    const auto& gnbPhy = m_gnbDev->GetPhy(bwp);
    switch (m_event)
    {
    case NumerologyEvent::MIB: {
        NrRrcSap::MasterInformationBlock mib;
        mib.numerology = m_numerology + 1;
        mib.dlBandwidth = m_ueDev->GetRrc()->GetDlBandwidth();
        mib.systemFrameNumber = 0;
        m_ueDev->GetRrc()->GetNrUeCphySapUser(bwp)->RecvMasterInformationBlock(
            m_gnbDev->GetCellId(),
            m_gnbDev->GetBwpArfcn(bwp),
            mib);
        break;
    }
    case NumerologyEvent::SIB1_RECONFIG:
        m_ueDev->GetRrc()->ReconfigureFromSib1(bwp,
                                               m_gnbDev->GetCellId(),
                                               m_gnbDev->GetScheduler(bwp)->GetDlCtrlSyms(),
                                               m_gnbDev->GetScheduler(bwp)->GetUlCtrlSyms(),
                                               gnbPhy->GetSymbolsPerSlot(),
                                               gnbPhy->GetNumerology(),
                                               gnbPhy->GetPattern(),
                                               gnbPhy->GetNumRbPerRbg());
        break;
    case NumerologyEvent::PHY_NUMEROLOGY:
        m_ueDev->GetPhy(bwp)->SetNumerology(m_numerology);
        break;
    case NumerologyEvent::RRC_RECONFIG:
        // Delivered to the UE later, at a time that depends on the RRC message transport
        m_gnbDev->GetRrc()
            ->GetUeManager(m_ueDev->GetRrc()->GetRnti())
            ->ScheduleRrcConnectionReconfiguration();
        break;
    default:
        return;
    }
    m_injected++;
}

void
NrNumerologyReceptionTestCase::Reconfigured(uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    m_reconfigurations++;
}

void
NrNumerologyReceptionTestCase::SnrEvaluated(double snr)
{
    m_lastEvaluation = Simulator::Now();
}

void
NrNumerologyReceptionTestCase::TbDecoded(RxPacketTraceParams params)
{
    // The SINR of a reception is evaluated at its end, just before its transport blocks are
    // decoded, so a transport block whose reception was aborted finds no evaluation at its time
    m_decodedTbs++;
    if (m_lastEvaluation != Simulator::Now())
    {
        m_unevaluatedTbs++;
    }
}

void
NrNumerologyReceptionTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    const Time simTime = MilliSeconds(600);
    const Time appStart = MilliSeconds(300);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // A neighbour gNB on the same carrier: its farther broadcast arrives while the UE already
    // receives the data of its serving cell
    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(0, 0, 10.0));
    pos->Add(Vector(400, 0, 10.0));
    pos->Add(Vector(50, 0, 1.5));
    mob.SetPositionAllocator(pos);
    mob.Install(gnbNodes);
    mob.Install(ueNodes);

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_idealRrc));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    beamHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbNetDev, .ueDevs = ueNetDev});

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        NrHelper::GetGnbPhy(gnbNetDev.Get(i), 0)
            ->SetAttribute("Numerology", UintegerValue(m_numerology));
    }

    m_gnbDev = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0));
    m_ueDev = DynamicCast<NrUeNetDevice>(ueNetDev.Get(0));

    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(ueNetDev);
    nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));

    // Saturating downlink traffic, so that the UE receives data in almost every DL slot
    const uint16_t dlPort = 1234;
    UdpServerHelper dlSink(dlPort);
    ApplicationContainer serverApps = dlSink.Install(ueNodes);
    UdpClientHelper dlClient(addressUtils::ConvertToSocketAddress(ueIpIface.GetAddress(0), dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(1252));
    dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(20)));
    ApplicationContainer clientApps = dlClient.Install(remoteHost);
    serverApps.Start(appStart);
    clientApps.Start(appStart);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    auto ueSpectrumPhy = m_ueDev->GetPhy(0)->GetSpectrumPhy();
    ueSpectrumPhy->TraceConnectWithoutContext(
        "RxDataTrace",
        MakeCallback(&NrNumerologyReceptionTestCase::RxDataStart, this));
    ueSpectrumPhy->TraceConnectWithoutContext(
        "RxPacketTraceUe",
        MakeCallback(&NrNumerologyReceptionTestCase::TbDecoded, this));
    ueSpectrumPhy->GetNrInterference()->TraceConnectWithoutContext(
        "SnrPerProcessedChunk",
        MakeCallback(&NrNumerologyReceptionTestCase::SnrEvaluated, this));

    m_ueDev->GetRrc()->TraceConnectWithoutContext(
        "ConnectionReconfiguration",
        MakeCallback(&NrNumerologyReceptionTestCase::Reconfigured, this));

    Simulator::Stop(simTime);
    Simulator::Run();

    const uint64_t rxPackets = DynamicCast<UdpServer>(serverApps.Get(0))->GetReceived();
    const auto& uePhy = m_ueDev->GetPhy(0);

    NS_TEST_EXPECT_MSG_EQ(m_ueDev->GetRrc()->GetState(),
                          NrUeRrc::CONNECTED_NORMALLY,
                          "The UE should be connected");
    NS_TEST_EXPECT_MSG_GT(rxPackets, 0, "The UE should receive downlink traffic");
    NS_TEST_EXPECT_MSG_GT(m_decodedTbs, 100, "The UE should decode downlink transport blocks");
    NS_TEST_EXPECT_MSG_EQ(uePhy->GetNumerology(),
                          m_numerology,
                          "The UE should be on the numerology of its cell");
    NS_TEST_EXPECT_MSG_EQ(uePhy->GetSlotPeriod(),
                          m_gnbDev->GetPhy(0)->GetSlotPeriod(),
                          "The UE should use the slot of its cell");
    NS_TEST_EXPECT_MSG_EQ(uePhy->GetRbNum(),
                          m_gnbDev->GetPhy(0)->GetRbNum(),
                          "The UE should use the RBs of its cell");
    NS_TEST_EXPECT_MSG_EQ(m_unevaluatedTbs,
                          0,
                          "A transport block was decoded from an aborted reception");
    if (m_event == NumerologyEvent::MIB || m_event == NumerologyEvent::SIB1_RECONFIG ||
        m_event == NumerologyEvent::PHY_NUMEROLOGY || m_event == NumerologyEvent::RRC_RECONFIG)
    {
        NS_TEST_EXPECT_MSG_GT(m_injected, 0, "The event should be applied during receptions");
    }
    if (m_event == NumerologyEvent::RRC_RECONFIG)
    {
        NS_TEST_EXPECT_MSG_GT(m_reconfigurations, 0, "The UE should be reconfigured");
    }

    // A numerology change must still reconfigure the slot and the RBs
    if (m_event == NumerologyEvent::RECONFIG_CHANGE || m_event == NumerologyEvent::PHY_CHANGE)
    {
        const uint16_t newNumerology = m_numerology + 1;
        const uint32_t rbNum = uePhy->GetRbNum();
        if (m_event == NumerologyEvent::RECONFIG_CHANGE)
        {
            const auto& gnbPhy = m_gnbDev->GetPhy(0);
            m_ueDev->GetRrc()->ReconfigureFromSib1(0,
                                                   m_gnbDev->GetCellId(),
                                                   m_gnbDev->GetScheduler(0)->GetDlCtrlSyms(),
                                                   m_gnbDev->GetScheduler(0)->GetUlCtrlSyms(),
                                                   gnbPhy->GetSymbolsPerSlot(),
                                                   newNumerology,
                                                   gnbPhy->GetPattern(),
                                                   gnbPhy->GetNumRbPerRbg());
        }
        else
        {
            uePhy->SetNumerology(newNumerology);
        }
        NS_TEST_EXPECT_MSG_EQ(uePhy->GetNumerology(), newNumerology, "Numerology not applied");
        NS_TEST_EXPECT_MSG_EQ(uePhy->GetSlotPeriod(),
                              MicroSeconds(1000 >> newNumerology),
                              "The slot should follow the new numerology");
        NS_TEST_EXPECT_MSG_LT(uePhy->GetRbNum(),
                              rbNum,
                              "A larger subcarrier spacing fits fewer RBs in the bandwidth");
    }

    Simulator::Destroy();
}

/**
 * @ingroup tests
 * @brief Test suite for the numerology (re)configuration of a UE while it receives data.
 */
class NrNumerologyReceptionTestSuite : public TestSuite
{
  public:
    NrNumerologyReceptionTestSuite()
        : TestSuite("nr-numerology-reception", Type::SYSTEM)
    {
        struct Case
        {
            std::string name;      ///< Case name
            NumerologyEvent event; ///< Event under test
        };

        const std::vector<Case> cases = {
            {"periodic-mib-sib1", NumerologyEvent::NONE},
            {"mib-other-numerology", NumerologyEvent::MIB},
            {"sib1-reconfig-same-numerology", NumerologyEvent::SIB1_RECONFIG},
            {"phy-same-numerology", NumerologyEvent::PHY_NUMEROLOGY},
            {"rrc-reconfiguration", NumerologyEvent::RRC_RECONFIG},
            {"sib1-reconfig-numerology-change", NumerologyEvent::RECONFIG_CHANGE},
            {"phy-numerology-change", NumerologyEvent::PHY_CHANGE},
        };
        for (const auto& c : cases)
        {
            for (uint16_t numerology : {0, 1})
            {
                for (bool idealRrc : {true, false})
                {
                    AddTestCase(new NrNumerologyReceptionTestCase(
                                    c.name + "-mu" + std::to_string(numerology) +
                                        (idealRrc ? "-ideal-rrc" : "-real-rrc"),
                                    numerology,
                                    idealRrc,
                                    c.event),
                                Duration::QUICK);
                }
            }
        }
    }
};

/// Static variable for test initialization
static NrNumerologyReceptionTestSuite g_nrNumerologyReceptionTestSuite;

} // namespace ns3
