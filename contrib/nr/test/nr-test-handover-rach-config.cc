/*
 * Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/**
 * @ingroup test
 * @file nr-test-handover-rach-config.cc
 *
 * @brief Test suite (nr-handover-rach-config) verifying that an X2 handover applies the
 * target cell's RACH configuration to the UE MAC. The handover command carries the target
 * cell's RachConfigCommon precisely because the UE may never have decoded the target's
 * system information; before this was applied, the UE MAC kept whatever RACH configuration
 * it had from the source cell's SIB2.
 */

#include "ns3/boolean.h"
#include "ns3/bwp-manager-gnb.h"
#include "ns3/bwp-manager-ue.h"
#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-gnb-mac.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-mac.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nstime.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverRachConfigTest");

/**
 * @ingroup nr-test
 *
 * @brief Verify that after an X2 handover the UE MAC uses the target gNB's RACH
 * configuration, carried in the handover command, rather than the source cell's.
 */
class NrHandoverRachConfigTestCase : public TestCase
{
  public:
    /**
     * Constructor
     *
     * @param name the name of the test case, to be displayed in the test result
     * @param useIdealRrc if true, use the ideal RRC
     * @param fdd if true, use FDD carriers with UL/DL roles inverted between the
     *            two gNBs; otherwise a single TDD carrier shared by both
     */
    NrHandoverRachConfigTestCase(std::string name, bool useIdealRrc, bool fdd)
        : TestCase(name),
          m_useIdealRrc(useIdealRrc),
          m_fdd(fdd)
    {
    }

  private:
    void DoRun() override;

    /**
     * UE handover start callback; schedules the RACH configuration check to run
     * right after the handover command has been fully processed
     * @param context the context string
     * @param imsi the IMSI
     * @param sourceCellId the source cell ID
     * @param rnti the RNTI
     * @param targetCellId the target cell ID
     */
    void UeHandoverStartCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t sourceCellId,
                                 uint16_t rnti,
                                 uint16_t targetCellId);

    /**
     * UE handover success callback
     * @param context the context string
     * @param imsi the IMSI
     * @param cellId the cell ID
     * @param rnti the RNTI
     */
    void UeHandoverEndOkCallback(std::string context,
                                 uint64_t imsi,
                                 uint16_t cellId,
                                 uint16_t rnti);

    /**
     * @brief Check that the UE MAC holds the target cell's RACH configuration
     */
    void CheckUeMacRachConfig();

    Ptr<NrUeNetDevice> m_ueDev; ///< the UE device under test

    static constexpr uint8_t targetNumberOfRaPreambles = 40; ///< target numberOfRaPreambles
    static constexpr uint8_t targetPreambleTransMax = 100;   ///< target preambleTransMax
    static constexpr uint8_t targetRaResponseWindowSize = 8; ///< target raResponseWindowSize
    static constexpr uint8_t targetConnEstFailCount = 2;     ///< target connEstFailCount

    bool m_useIdealRrc;                 ///< use ideal RRC?
    bool m_fdd;                         ///< FDD carriers with inverted UL/DL roles?
    bool m_hasHandoverOccurred{false};  ///< whether the handover command reached the UE
    bool m_hasHandoverCompleted{false}; ///< whether the handover completed successfully
};

void
NrHandoverRachConfigTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());
    uint32_t previousSeed = RngSeedManager::GetSeed();
    uint64_t previousRun = RngSeedManager::GetRun();
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(2);

    // Source cell RACH configuration, applied to the UE MAC from the source SIB2
    // on initial access. Every field differs from the target's below so a stale
    // configuration cannot pass the final check.
    Config::SetDefault("ns3::NrGnbMac::NumberOfRaPreambles", UintegerValue(52));
    Config::SetDefault("ns3::NrGnbMac::PreambleTransMax", UintegerValue(50));
    Config::SetDefault("ns3::NrGnbMac::RaResponseWindowSize", UintegerValue(3));
    Config::SetDefault("ns3::NrGnbMac::ConnEstFailCount", UintegerValue(1));

    auto nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(m_useIdealRrc));
    Config::SetDefault("ns3::NrGnbRrc::HandoverDecisionDelay", TimeValue(Seconds(0)));

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Disable RLF detection for real RRC to prevent premature RLF while the UE
    // is connecting far from the target cell (same approach as the X2 handover suite)
    if (!m_useIdealRrc)
    {
        Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(20));
        Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(10));
        Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(Seconds(2)));
    }
    Config::SetDefault("ns3::NrGnbRrc::HandoverTriggeringDelay", TimeValue(Seconds(0)));

    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(35));

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(LogDistancePropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    BandwidthPartInfoPtrVector allBwps;
    std::vector<BandwidthPartInfoPtrVector> allBwpsPerGnb;
    OperationBandInfo band;
    OperationBandInfo band2;
    if (m_fdd)
    {
        // Wide enough carriers that even numerology 3 (120 kHz SCS) has room
        // for the msg3 grant
        CcBwpCreator::SimpleOperationBandConf bandConf(2.4e9, 40e6, static_cast<uint8_t>(1));
        CcBwpCreator::SimpleOperationBandConf bandConf2(2.7e9, 40e6, static_cast<uint8_t>(1));
        band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
        band2 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf2);
        channelHelper->AssignChannelsToBands({band, band2});
        allBwps = CcBwpCreator::GetAllBwps({band, band2});
        // Only the primary component carrier (index 0) broadcasts MIB/SIB, so
        // each gNB must have its DL carrier first: 2.7 GHz for the source gNB,
        // 2.4 GHz for the target
        allBwpsPerGnb.push_back({allBwps[1], allBwps[0]});
        allBwpsPerGnb.push_back({allBwps[0], allBwps[1]});
    }
    else
    {
        CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, static_cast<uint8_t>(1));
        band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
        channelHelper->AssignChannelsToBands({band});
        allBwps = CcBwpCreator::GetAllBwps({band});
    }

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    auto ueNode = CreateObject<Node>();

    auto posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));
    posAlloc->Add(Vector(1500, 0, 0));
    posAlloc->Add(Vector(200, 0, 0));

    MobilityHelper mobilityHelper;
    mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityHelper.SetPositionAllocator(posAlloc);
    mobilityHelper.Install(gnbNodes);
    mobilityHelper.Install(ueNode);

    NetDeviceContainer gnbDevs;
    if (m_fdd)
    {
        gnbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(0), allBwpsPerGnb[0]));
        gnbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(1), allBwpsPerGnb[1]));
    }
    else
    {
        gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    }
    auto ueDev = nrHelper->InstallUeDevice(ueNode, allBwps).Get(0);

    if (m_fdd)
    {
        const std::string dlPattern = "DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|";
        const std::string ulPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
        // Each gNB: CC 0 is DL (2.7 GHz for the source, 2.4 GHz for the target),
        // CC 1 is UL, i.e. the two cells have inverted UL/DL frequency roles.
        // Four distinct numerologies make any misapplied configuration obvious.
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Pattern", StringValue(dlPattern));
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(1));
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)->SetAttribute("Pattern", StringValue(ulPattern));
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 1)->SetAttribute("Numerology", UintegerValue(0));

        NrHelper::GetGnbPhy(gnbDevs.Get(1), 0)->SetAttribute("Pattern", StringValue(dlPattern));
        NrHelper::GetGnbPhy(gnbDevs.Get(1), 0)->SetAttribute("Numerology", UintegerValue(2));
        NrHelper::GetGnbPhy(gnbDevs.Get(1), 1)->SetAttribute("Pattern", StringValue(ulPattern));
        NrHelper::GetGnbPhy(gnbDevs.Get(1), 1)->SetAttribute("Numerology", UintegerValue(3));
    }

    // Give the target gNB a RACH configuration entirely distinct from the source's
    for (uint32_t bwp = 0; bwp < (m_fdd ? 2U : 1U); bwp++)
    {
        auto targetMac = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(1))->GetMac(bwp);
        targetMac->SetAttribute("NumberOfRaPreambles", UintegerValue(targetNumberOfRaPreambles));
        targetMac->SetAttribute("PreambleTransMax", UintegerValue(targetPreambleTransMax));
        targetMac->SetAttribute("RaResponseWindowSize", UintegerValue(targetRaResponseWindowSize));
        targetMac->SetAttribute("ConnEstFailCount", UintegerValue(targetConnEstFailCount));
    }

    InternetStackHelper inetStackHelper;
    inetStackHelper.Install(ueNode);
    Ipv4InterfaceContainer ueIfs = nrEpcHelper->AssignUeIpv4Address(ueDev);

    m_ueDev = DynamicCast<NrUeNetDevice>(ueDev);
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverRachConfigTestCase::UeHandoverEndOkCallback, this));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NrHandoverRachConfigTestCase::UeHandoverStartCallback, this));

    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->AttachToGnb(ueDev, gnbDevs.Get(0));
    nrHelper->HandoverRequest(Seconds(1), ueDev, gnbDevs.Get(0), gnbDevs.Get(1));

    nrHelper->AssignStreams({.assignEpc = true,
                             .ueNodes = ueNode,
                             .gnbNodes = gnbNodes,
                             .gnbDevs = gnbDevs,
                             .ueDevs = ueDev,
                             .gnbNodeStream = 1000,
                             .gnbDevStream = 3000,
                             .ueDevStream = 4000});

    Simulator::Stop(Seconds(2));
    Simulator::Run();

    NS_TEST_ASSERT_MSG_EQ(m_hasHandoverOccurred, true, "handover did not start");
    NS_TEST_ASSERT_MSG_EQ(m_hasHandoverCompleted, true, "handover did not complete");

    Simulator::Destroy();
    m_ueDev = nullptr;
    RngSeedManager::SetSeed(previousSeed);
    RngSeedManager::SetRun(previousRun);
}

void
NrHandoverRachConfigTestCase::UeHandoverStartCallback(std::string context,
                                                      uint64_t imsi,
                                                      uint16_t sourceCellId,
                                                      uint16_t rnti,
                                                      uint16_t targetCellId)
{
    NS_LOG_FUNCTION(this << context << imsi << sourceCellId << rnti << targetCellId);
    m_hasHandoverOccurred = true;
    // The HandoverStart trace fires while the handover command is still being
    // processed; a zero-delay event runs after it completes, but before any
    // system information from the target cell can reconfigure the MAC again.
    Simulator::ScheduleNow(&NrHandoverRachConfigTestCase::CheckUeMacRachConfig, this);
}

void
NrHandoverRachConfigTestCase::UeHandoverEndOkCallback(std::string context,
                                                      uint64_t imsi,
                                                      uint16_t cellId,
                                                      uint16_t rnti)
{
    NS_LOG_FUNCTION(this << context << imsi << cellId << rnti);
    m_hasHandoverCompleted = true;
}

void
NrHandoverRachConfigTestCase::CheckUeMacRachConfig()
{
    // The primary UL BWP may have changed with the handover (it does in the
    // FDD-inverted scenario); check the MAC the RRC currently uses for RA
    NrUeCmacSapProvider::RachConfig rc =
        m_ueDev->GetMac(m_ueDev->GetRrc()->GetPrimaryUlIndex())->GetRachConfig();
    NS_TEST_ASSERT_MSG_EQ(+rc.numberOfRaPreambles,
                          +targetNumberOfRaPreambles,
                          "UE MAC did not apply the target cell's numberOfRaPreambles");
    NS_TEST_ASSERT_MSG_EQ(+rc.preambleTransMax,
                          +targetPreambleTransMax,
                          "UE MAC did not apply the target cell's preambleTransMax");
    NS_TEST_ASSERT_MSG_EQ(+rc.raResponseWindowSize,
                          +targetRaResponseWindowSize,
                          "UE MAC did not apply the target cell's raResponseWindowSize");
    NS_TEST_ASSERT_MSG_EQ(+rc.connEstFailCount,
                          +targetConnEstFailCount,
                          "UE MAC did not apply the target cell's connEstFailCount");
}

/**
 * @ingroup nr-test
 *
 * @brief NR handover RACH configuration test suite
 */
class NrHandoverRachConfigTestSuite : public TestSuite
{
  public:
    NrHandoverRachConfigTestSuite()
        : TestSuite("nr-handover-rach-config", Type::SYSTEM)
    {
        AddTestCase(new NrHandoverRachConfigTestCase("Target RACH config applied, TDD, ideal RRC",
                                                     true,
                                                     false),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverRachConfigTestCase("Target RACH config applied, TDD, real RRC",
                                                     false,
                                                     false),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverRachConfigTestCase(
                        "Target RACH config applied, FDD inverted UL-DL, ideal RRC",
                        true,
                        true),
                    TestCase::Duration::QUICK);
        AddTestCase(new NrHandoverRachConfigTestCase(
                        "Target RACH config applied, FDD inverted UL-DL, real RRC",
                        false,
                        true),
                    TestCase::Duration::QUICK);
    }
};

/// Static variable for test initialization
static NrHandoverRachConfigTestSuite g_nrHandoverRachConfigTestSuite;
