// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-initial-association.cc
 *
 * @brief Test suite `nr-initial-association`: checks the per-gNB RSRP and best beam computed by
 * NrInitialAssociation, and the UE panel it activates.
 */

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

using namespace ns3;

/**
 * @ingroup nr-test
 *
 * @brief Verifies that each gNB is credited with its own best beam, whatever the scan order.
 *
 * Two co-located sectors, one with its boresight towards the UE and one facing away, share the
 * same path loss, so only the antenna orientation separates them.
 */
class NrInitialAssociationOwnBeamTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param facingScannedFirst Whether the sector facing the UE is scanned first.
     */
    NrInitialAssociationOwnBeamTestCase(bool facingScannedFirst);

  private:
    void DoRun() override;

    bool m_facingScannedFirst; ///< Whether the sector facing the UE is scanned first
};

NrInitialAssociationOwnBeamTestCase::NrInitialAssociationOwnBeamTestCase(bool facingScannedFirst)
    : TestCase(std::string("Sector facing the UE wins when scanned ") +
               (facingScannedFirst ? "first" : "last")),
      m_facingScannedFirst(facingScannedFirst)
{
}

void
NrInitialAssociationOwnBeamTestCase::DoRun()
{
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 25));
    positions->Add(Vector(0, 0, 25));
    positions->Add(Vector(100, 0, 1.5));
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    auto band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    auto gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    const uint32_t facingIdx = m_facingScannedFirst ? 0 : 1;
    const uint32_t awayIdx = 1 - facingIdx;
    const auto setBearing = [&](uint32_t idx, double bearing) {
        DynamicCast<NrGnbNetDevice>(gnbDevices.Get(idx))
            ->GetPhy(0)
            ->GetSpectrumPhy()
            ->GetAntenna()
            ->SetAttribute("BearingAngle", DoubleValue(bearing));
    };
    setBearing(facingIdx, 0.0);
    setBearing(awayIdx, M_PI);

    // Neither grid holds the 90 degree default of NrAnglePair, so a best beam left unset is
    // detectable.
    const std::vector<double> beamAngles{60, 120};

    Simulator::Schedule(MilliSeconds(1), [&]() {
        auto association = CreateObject<NrInitialAssociation>();
        association->AssignStreams(1);
        association->SetUeDevice(ueDevices.Get(0));
        association->SetGnbDevices(gnbDevices);
        association->SetRowBeamAngles(beamAngles);
        association->SetColBeamAngles(beamAngles);
        const auto [associatedGnb, associatedRsrp] = association->FindAssociatedGnb();

        NS_TEST_EXPECT_MSG_EQ(associatedGnb,
                              gnbDevices.Get(facingIdx),
                              "The UE should associate with the sector facing it");
        NS_TEST_EXPECT_MSG_GT(association->GetMaxRsrp(facingIdx) - association->GetMaxRsrp(awayIdx),
                              10.0,
                              "The sector facing away should lose its antenna gain");
        for (uint32_t idx = 0; idx < gnbDevices.GetN(); idx++)
        {
            NS_TEST_EXPECT_MSG_NE(association->GetBestBfv(idx).rowAng,
                                  90.0,
                                  "Every gNB should report its own best beam");
        }
    });
    Simulator::Stop(MilliSeconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * Run the initial association of a UE with the 60 and 120 degree beam grid.
 *
 * @param ueDevice The UE device.
 * @param gnbDevices The gNB devices to scan.
 * @return The initial association object, after its search.
 */
static Ptr<NrInitialAssociation>
Associate(Ptr<NetDevice> ueDevice, const NetDeviceContainer& gnbDevices)
{
    const std::vector<double> beamAngles{60, 120};
    auto association = CreateObject<NrInitialAssociation>();
    association->AssignStreams(1);
    association->SetUeDevice(ueDevice);
    association->SetGnbDevices(gnbDevices);
    association->SetRowBeamAngles(beamAngles);
    association->SetColBeamAngles(beamAngles);
    association->FindAssociatedGnb();
    return association;
}

/**
 * @ingroup nr-test
 *
 * @brief Verifies that a UE with two panels ends up on the best panel of the gNB it associates
 * with.
 *
 * The UE has one panel facing +x (panel 0) and one facing -x (panel 1). The near gNB, 20 m away
 * in +x, faces away from the UE, so its beams are weak but its path loss is small: it has the
 * highest RSRP, through panel 0. The far gNB, 1 km away in -x, faces the UE, so it has the
 * strongest beam before the path loss, through panel 1.
 */
class NrInitialAssociationActivePanelTestCase : public TestCase
{
  public:
    NrInitialAssociationActivePanelTestCase()
        : TestCase("Two-panel UE: active panel of the associated gNB")
    {
    }

  private:
    void DoRun() override;

    /**
     * Check the active panel of the UE after associating it with each gNB alone, then with both.
     *
     * @param ue The UE device.
     * @param gnbDevices The near and far gNB devices.
     */
    void CheckPanels(Ptr<NetDevice> ue, const NetDeviceContainer& gnbDevices);

    static constexpr uint32_t m_nearIdx{0}; ///< Index of the near gNB
    static constexpr uint32_t m_farIdx{1};  ///< Index of the far gNB
};

void
NrInitialAssociationActivePanelTestCase::CheckPanels(Ptr<NetDevice> ue,
                                                     const NetDeviceContainer& gnbDevices)
{
    // Check the setup: the best panel of each gNB, when it is the only one
    auto nearAssociation = Associate(ue, NetDeviceContainer(gnbDevices.Get(m_nearIdx)));
    NS_TEST_ASSERT_MSG_EQ(nearAssociation->GetUeActivePanel(),
                          0,
                          "The near gNB should be best received by panel 0");
    auto farAssociation = Associate(ue, NetDeviceContainer(gnbDevices.Get(m_farIdx)));
    NS_TEST_ASSERT_MSG_EQ(farAssociation->GetUeActivePanel(),
                          1,
                          "The far gNB should be best received by panel 1");

    auto association = Associate(ue, gnbDevices);
    NS_TEST_ASSERT_MSG_GT(association->GetMaxRsrp(m_nearIdx),
                          association->GetMaxRsrp(m_farIdx),
                          "The near gNB should have the highest RSRP");
    NS_TEST_EXPECT_MSG_EQ(association->GetAssociatedGnb(),
                          gnbDevices.Get(m_nearIdx),
                          "The UE should associate with the near gNB");
    NS_TEST_EXPECT_MSG_EQ(association->GetUeActivePanel(),
                          0,
                          "The UE should activate the best panel of its associated gNB");

    // Changing the gNBs of an association object discards the results of its search
    auto reused = Associate(ue, NetDeviceContainer(gnbDevices.Get(m_farIdx)));
    reused->SetGnbDevices(gnbDevices);
    const auto [reusedGnb, reusedRsrp] = reused->FindAssociatedGnb();
    NS_TEST_EXPECT_MSG_EQ(reusedGnb,
                          gnbDevices.Get(m_nearIdx),
                          "A search with new gNBs should associate with the near gNB");
    NS_TEST_EXPECT_MSG_EQ_TOL(reusedRsrp,
                              association->GetMaxRsrp(m_nearIdx),
                              1e-9,
                              "A search with new gNBs should return the RSRP of the near gNB");
    NS_TEST_EXPECT_MSG_EQ(reused->GetUeActivePanel(),
                          0,
                          "A search with new gNBs should activate the panel of the near gNB");
}

void
NrInitialAssociationActivePanelTestCase::DoRun()
{
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes;
    gnbNodes.Create(2);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    auto positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(20, 0, 25));
    positions->Add(Vector(-1000, 0, 25));
    positions->Add(Vector(0, 0, 1.5));
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    // Both gNBs keep the default bearing angle, facing +x: the near one faces away from the UE
    // and the far one faces it
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    // The first UE panel faces +x, and the second one -x
    nrHelper->SetUeSpectrumAttribute("NumAntennaPanel", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<ThreeGppAntennaModel>()));

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    auto band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    auto gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    Simulator::Schedule(MilliSeconds(1),
                        &NrInitialAssociationActivePanelTestCase::CheckPanels,
                        this,
                        ueDevices.Get(0),
                        gnbDevices);
    Simulator::Stop(MilliSeconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 *
 * @brief Verifies that the RSRP of each gNB follows its transmit power.
 *
 * Three co-located gNBs with the same single-element antenna only differ in their transmit
 * power, so their RSRPs should differ by their transmit power difference, up to the fading of
 * their independent channels. One of them transmits below 0 dBm. GetCellRsrps() should return the
 * same RSRPs, and the search is then repeated on the same object.
 */
class NrInitialAssociationTxPowerTestCase : public TestCase
{
  public:
    NrInitialAssociationTxPowerTestCase()
        : TestCase("RSRP follows the gNB transmit power")
    {
    }

  private:
    void DoRun() override;

    /**
     * Check the RSRP of each gNB found by the initial association of the UE.
     *
     * @param ue The UE device.
     * @param gnbDevices The gNB devices, in the order of m_txPowersDbm.
     */
    void CheckRsrps(Ptr<NetDevice> ue, const NetDeviceContainer& gnbDevices);

    const std::vector<double> m_txPowersDbm{40, 1, -10}; ///< Transmit power of each gNB
};

void
NrInitialAssociationTxPowerTestCase::CheckRsrps(Ptr<NetDevice> ue,
                                                const NetDeviceContainer& gnbDevices)
{
    auto association = CreateObject<NrInitialAssociation>();
    association->AssignStreams(1);
    association->SetUeDevice(ue);
    association->SetGnbDevices(gnbDevices);
    association->SetRowBeamAngles({90});
    association->SetColBeamAngles({90});
    const auto [associatedGnb, associatedRsrp] = association->FindAssociatedGnb();

    NS_TEST_EXPECT_MSG_EQ(associatedGnb,
                          gnbDevices.Get(0),
                          "The UE should associate with the strongest gNB");
    for (size_t i = 1; i < m_txPowersDbm.size(); i++)
    {
        NS_TEST_EXPECT_MSG_EQ_TOL(association->GetMaxRsrp(0) - association->GetMaxRsrp(i),
                                  m_txPowersDbm[0] - m_txPowersDbm[i],
                                  5.0,
                                  "The RSRP difference to gNB "
                                      << i << " should be the transmit power difference");
    }

    // GetCellRsrps() runs the same beam search
    auto cellRsrps = association->GetCellRsrps();
    for (size_t i = 0; i < m_txPowersDbm.size(); i++)
    {
        const auto cellId = DynamicCast<NrGnbNetDevice>(gnbDevices.Get(i))->GetCellId();
        NS_TEST_EXPECT_MSG_EQ_TOL(cellRsrps[cellId],
                                  association->GetMaxRsrp(i),
                                  1e-9,
                                  "GetCellRsrps() should return the RSRP of gNB " << i);
    }

    // With one main interferer, the strongest of the others, the rest of the interference is
    // the weakest gNB
    association->InitializeIntfSet(1, false, 0);
    const auto interferers = association->GetInterferingGnbs();
    NS_TEST_ASSERT_MSG_EQ(interferers.GetN(), 1, "There should be one main interferer");
    NS_TEST_EXPECT_MSG_EQ(interferers.Get(0),
                          gnbDevices.Get(1),
                          "The main interferer should be the second strongest gNB");
    NS_TEST_EXPECT_MSG_EQ_TOL(
        association->GetRelativeRsrpRatio(),
        std::pow(10.0, (association->GetMaxRsrp(2) - association->GetMaxRsrp(1)) / 10.0),
        1e-9,
        "The RSRP ratio should be the weakest gNB over the main interferer");

    // A second search reuses the RSRPs of the first one
    const auto [gnbAgain, rsrpAgain] = association->FindAssociatedGnb();
    NS_TEST_EXPECT_MSG_EQ(gnbAgain, associatedGnb, "A second search should not change the gNB");
    NS_TEST_EXPECT_MSG_EQ(rsrpAgain, associatedRsrp, "A second search should not change RSRP");
}

void
NrInitialAssociationTxPowerTestCase::DoRun()
{
    Config::Reset();
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes;
    gnbNodes.Create(m_txPowersDbm.size());
    NodeContainer ueNodes;
    ueNodes.Create(1);

    auto positions = CreateObject<ListPositionAllocator>();
    for (size_t i = 0; i < m_txPowersDbm.size(); i++)
    {
        positions->Add(Vector(0, 0, 25));
    }
    positions->Add(Vector(100, 0, 1.5));
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
    auto nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));

    auto channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    auto band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    auto allBwps = CcBwpCreator::GetAllBwps({band});

    auto gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    auto ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    nrHelper->AssignStreams({.ueDevs = ueDevices});
    for (size_t i = 0; i < m_txPowersDbm.size(); i++)
    {
        DynamicCast<NrGnbNetDevice>(gnbDevices.Get(i))->GetPhy(0)->SetTxPower(m_txPowersDbm[i]);
    }

    Simulator::Schedule(MilliSeconds(1),
                        &NrInitialAssociationTxPowerTestCase::CheckRsrps,
                        this,
                        ueDevices.Get(0),
                        gnbDevices);
    Simulator::Stop(MilliSeconds(2));
    Simulator::Run();
    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite for NrInitialAssociation.
 */
class NrInitialAssociationTestSuite : public TestSuite
{
  public:
    NrInitialAssociationTestSuite()
        : TestSuite("nr-initial-association", Type::UNIT)
    {
        AddTestCase(new NrInitialAssociationOwnBeamTestCase(true), Duration::QUICK);
        AddTestCase(new NrInitialAssociationOwnBeamTestCase(false), Duration::QUICK);
        AddTestCase(new NrInitialAssociationActivePanelTestCase(), Duration::QUICK);
        AddTestCase(new NrInitialAssociationTxPowerTestCase(), Duration::QUICK);
    }
};

/// Static instance of the test suite
static NrInitialAssociationTestSuite g_nrInitialAssociationTestSuite;
