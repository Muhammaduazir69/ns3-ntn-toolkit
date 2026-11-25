// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-model.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRealisticBeamformingTest");

/**
 * @ingroup test
 * @file nr-realistic-beamforming-test.cc
 *
 * @brief This test tests how different levels of received SINR SRS
 * affect the realistic beamforming algorithm performance. What is expected
 * is that when SINR is high that realistic beamforming algorithm will
 * select the same beamforming vector pair as it would ideal beamforming
 * algorithm that has the perfect knowledge of the channel.
 * On the other hand, when SINR is low it is expected that the error in
 * estimation of the channel is high, thus the selected beamforming pair
 * is expected to be different from those that are selected by the ideal
 * beamforming algorithm.
 * Note that as the ideal and realistic beamforming algorithms are not exactly
 * the same, i.e., ideal beamforming algorithm assumes perfect knowledge
 * of the full channel (including long-term component of the fading,
 * the Doppler, and frequency-selectivity) while realistic beamforming
 * algorithm only estimates the long-term component of the fading.
 * Hence, then slight variations on the best beam selection may appear.
 */

class NrRealisticBeamformingTestSuite : public TestSuite
{
  public:
    NrRealisticBeamformingTestSuite();
};

class NrRealisticBeamformingTestCase : public TestCase
{
  public:
    NrRealisticBeamformingTestCase(std::string name, Duration duration);
    ~NrRealisticBeamformingTestCase() override;

  private:
    void DoRun() override;

    Duration m_testDuration; //!< the test execution mode type
};

/**
 * @brief Test that the quality of the realistic beam selection improves
 * monotonically with the SRS SINR.
 *
 * For a set of scenarios with asymmetric gNB/UE antenna arrays (which also
 * exercises the per-device zenith scan step), the beam pair selected by the
 * realistic algorithm is compared against the one selected by the ideal
 * (perfect channel knowledge) cell-scan algorithm at low (-10 dB), mid
 * (10 dB) and high (40 dB) SRS SINR. The match rate must not decrease with
 * the SRS SINR, must be high at high SINR, and low at low SINR.
 */
class NrRealisticBeamformingSrsSinrTestCase : public TestCase
{
  public:
    NrRealisticBeamformingSrsSinrTestCase(std::string name, Duration duration);

  private:
    void DoRun() override;

    Duration m_testDuration; //!< the test execution mode type
};

/**
 * TestSuite
 */
NrRealisticBeamformingTestSuite::NrRealisticBeamformingTestSuite()
    : TestSuite("nr-realistic-beamforming-test", Type::SYSTEM)
{
    NS_LOG_INFO("Creating NrRealisticBeamformingTestSuite");

    auto durationQuick = Duration::QUICK;
    auto durationExtensive = Duration::EXTENSIVE;

    AddTestCase(
        new NrRealisticBeamformingTestCase("RealisticBeamforming basic test case", durationQuick),
        durationQuick);
    AddTestCase(new NrRealisticBeamformingTestCase("RealisticBeamforming basic test case",
                                                   durationExtensive),
                durationExtensive);
    AddTestCase(new NrRealisticBeamformingSrsSinrTestCase(
                    "RealisticBeamforming beam-match rate vs SRS SINR",
                    durationQuick),
                durationQuick);
    AddTestCase(new NrRealisticBeamformingSrsSinrTestCase(
                    "RealisticBeamforming beam-match rate vs SRS SINR",
                    durationExtensive),
                durationExtensive);
}

/**
 * TestCase
 */

NrRealisticBeamformingTestCase::NrRealisticBeamformingTestCase(std::string name, Duration duration)
    : TestCase(name)
{
    m_testDuration = duration;
}

NrRealisticBeamformingTestCase::~NrRealisticBeamformingTestCase()
{
}

void
NrRealisticBeamformingTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);

    std::list<Vector> uePositionsExtensive = {Vector(10, -10, 1.5),
                                              Vector(0, 10, 1.5),
                                              Vector(0, -10, 1.5)};

    uint16_t totalCounter = 0;
    uint16_t highSinrCounter = 0;
    uint16_t lowSinrCounter = 0;

    std::list<uint16_t> rngList = (m_testDuration == Duration::EXTENSIVE)
                                      ? std::list<uint16_t>({2, 3})
                                      : std::list<uint16_t>({1});

    std::list<Vector> uePositions =
        (m_testDuration == Duration::EXTENSIVE)
            ? uePositionsExtensive
            : std::list<Vector>({Vector(10, 10, 1.5), Vector(-10, 10, 1.5)});

    std::list<uint16_t> antennaConfList = (m_testDuration == Duration::EXTENSIVE)
                                              ? std::list<uint16_t>({3, 4})
                                              : std::list<uint16_t>({2});

    for (auto rng : rngList)
    {
        RngSeedManager::SetRun(rng);

        for (const auto& pos : uePositions)
        {
            for (const auto& antennaConf : antennaConfList)
            {
                for (auto iso : {false, true})
                {
                    totalCounter++;

                    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
                    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
                    channelHelper->ConfigureFactories("UMa", "LOS");
                    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
                    // Create Nodes: gNB and UE
                    NodeContainer gnbNodes;
                    NodeContainer ueNodes;
                    gnbNodes.Create(1);
                    ueNodes.Create(1);
                    NodeContainer allNodes = NodeContainer(gnbNodes, ueNodes);

                    // Install Mobility Model
                    Ptr<ListPositionAllocator> positionAlloc =
                        CreateObject<ListPositionAllocator>();
                    positionAlloc->Add(Vector(0, 0.0, 10)); // gNB
                    positionAlloc->Add(pos);                // UE

                    MobilityHelper mobility;
                    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
                    mobility.SetPositionAllocator(positionAlloc);
                    mobility.Install(allNodes);

                    // Create Devices and install them in the Nodes (gNB and UE)
                    NetDeviceContainer gnbDevs;
                    NetDeviceContainer ueDevs;

                    CcBwpCreator::SimpleOperationBandConf bandConf(29e9, 100e6, 1);
                    CcBwpCreator ccBwpCreator;
                    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
                    // Initialize channel and pathloss, plus other things inside band.
                    channelHelper->AssignChannelsToBands({band});

                    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

                    // Antennas for the gNbs
                    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(antennaConf));
                    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(antennaConf));

                    // Antennas for the UEs
                    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(antennaConf));
                    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(antennaConf));

                    // Antenna element type for both gNB and UE
                    if (iso)
                    {
                        nrHelper->SetGnbAntennaAttribute(
                            "AntennaElement",
                            PointerValue(CreateObject<IsotropicAntennaModel>()));
                        nrHelper->SetUeAntennaAttribute(
                            "AntennaElement",
                            PointerValue(CreateObject<IsotropicAntennaModel>()));
                    }
                    else
                    {
                        nrHelper->SetGnbAntennaAttribute(
                            "AntennaElement",
                            PointerValue(CreateObject<ThreeGppAntennaModel>()));
                        nrHelper->SetUeAntennaAttribute(
                            "AntennaElement",
                            PointerValue(CreateObject<ThreeGppAntennaModel>()));
                    }

                    nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());

                    gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
                    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

                    // This test requires that the NrGnbNetDevice::ConfigureCell()
                    // is called before GetBeamformingVectors() is called below.
                    // Usually NrHelper::AttachToGnb() or NrGnbNetDevice::DoInitialize()
                    // takes care of this, but in this test we need to manually
                    // call it here.
                    for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
                    {
                        DynamicCast<NrGnbNetDevice>(*it)->ConfigureCell();
                    }
                    uint32_t stream = 1;
                    stream += nrHelper->AssignStreams(gnbDevs, stream);
                    stream += nrHelper->AssignStreams(ueDevs, stream);

                    Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevs.Get(0), 0);

                    Ptr<NrSpectrumPhy> txSpectrumPhy =
                        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->GetSpectrumPhy();

                    double sinrSrsHighLineal = std::pow(10.0, 0.1 * 40);
                    double sinrSrsLowLineal = std::pow(10.0, 0.1 * (-10));

                    Ptr<CellScanBeamforming> cellScanBeamforming =
                        CreateObject<CellScanBeamforming>();

                    BeamformingVectorPair bfPairIdeal =
                        cellScanBeamforming->GetBeamformingVectors(txSpectrumPhy,
                                                                   uePhy->GetSpectrumPhy());

                    Ptr<RealisticBeamformingAlgorithm> realisticBeamforming =
                        CreateObject<RealisticBeamformingAlgorithm>();
                    realisticBeamforming->Install(
                        txSpectrumPhy,
                        uePhy->GetSpectrumPhy(),
                        DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0))->GetScheduler(0));
                    stream += realisticBeamforming->AssignStreams(stream);

                    // directly update max SINR SRS to a high value, skipping other set functions of
                    // the algorithm
                    realisticBeamforming->m_maxSrsSinrPerSlot = sinrSrsHighLineal;

                    BeamformingVectorPair bfPairReal1 =
                        realisticBeamforming->GetBeamformingVectors();

                    // directly update max SINR SRS to a new lower value, skipping other set
                    // functions of the algorithm,
                    realisticBeamforming->m_maxSrsSinrPerSlot = sinrSrsLowLineal;

                    BeamformingVectorPair bfPairReal2 =
                        realisticBeamforming->GetBeamformingVectors();

                    if ((bfPairIdeal.first.second == bfPairReal1.first.second) &&
                        (bfPairIdeal.second.second == bfPairReal1.second.second))
                    {
                        highSinrCounter++;
                    }

                    if (!((bfPairIdeal.first.second == bfPairReal2.first.second) &&
                          (bfPairIdeal.second.second == bfPairReal2.second.second)))
                    {
                        lowSinrCounter++;
                    }
                }
            }
        }
    }

    double tolerance = 0.21;
    if (m_testDuration == Duration::EXTENSIVE)
    {
        tolerance = 0.21;
    }
    else
    {
        tolerance =
            0.3; // relax tolerance for QUICK mode since there are only 4 test configurations, e.g.,
                 // if 3 results of 4 are as expected that is already enough, but that gives 0.75
                 // thus it needs larger tolerance than 0.2 which is fine for EXTENSIVE mode
    }

    NS_TEST_ASSERT_MSG_EQ_TOL(highSinrCounter / (double)totalCounter,
                              1,
                              tolerance,
                              "The pair of beamforming vectors should be equal in most of the "
                              "cases when SINR is high, and they are not");
    NS_TEST_ASSERT_MSG_EQ_TOL(lowSinrCounter / (double)totalCounter,
                              1,
                              tolerance,
                              "The pair of beamforming vectors should not be equal in most of the "
                              "cases when SINR is low, and they are");

    NS_LOG_INFO("The result is as expected when high SINR in " << highSinrCounter << " out of "
                                                               << totalCounter << " total cases.");
    NS_LOG_INFO("The result is as expected when low SINR in " << lowSinrCounter << " out of "
                                                              << totalCounter << " total cases.");

    Simulator::Destroy();
}

NrRealisticBeamformingSrsSinrTestCase::NrRealisticBeamformingSrsSinrTestCase(std::string name,
                                                                             Duration duration)
    : TestCase(name)
{
    m_testDuration = duration;
}

void
NrRealisticBeamformingSrsSinrTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);

    // low / mid / high SRS SINR, in dB. Note that the estimation-error
    // variance includes the 9 dB time-domain filtering gain Delta, so the
    // low point must be well below 0 dB for the estimation error to be
    // large enough to change the selected beam pair in most of the cases.
    const std::vector<double> srsSinrsDb = {-20.0, 10.0, 40.0};
    std::vector<uint16_t> matchCounters(srsSinrsDb.size(), 0);
    uint16_t totalCounter = 0;

    std::list<uint16_t> rngList = (m_testDuration == Duration::EXTENSIVE)
                                      ? std::list<uint16_t>({1, 2, 3, 4})
                                      : std::list<uint16_t>({1, 2, 3});

    std::list<Vector> uePositions =
        (m_testDuration == Duration::EXTENSIVE)
            ? std::list<Vector>({Vector(10, 10, 1.5), Vector(-10, 10, 1.5), Vector(0, -10, 1.5)})
            : std::list<Vector>({Vector(10, 10, 1.5), Vector(-10, 10, 1.5)});

    for (auto rng : rngList)
    {
        RngSeedManager::SetRun(rng);

        for (const auto& pos : uePositions)
        {
            for (auto iso : {false, true})
            {
                totalCounter++;

                Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
                Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
                channelHelper->ConfigureFactories("UMa", "LOS");
                channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
                // Create Nodes: gNB and UE
                NodeContainer gnbNodes;
                NodeContainer ueNodes;
                gnbNodes.Create(1);
                ueNodes.Create(1);
                NodeContainer allNodes = NodeContainer(gnbNodes, ueNodes);

                // Install Mobility Model
                Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
                positionAlloc->Add(Vector(0, 0.0, 10)); // gNB
                positionAlloc->Add(pos);                // UE

                MobilityHelper mobility;
                mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
                mobility.SetPositionAllocator(positionAlloc);
                mobility.Install(allNodes);

                // Create Devices and install them in the Nodes (gNB and UE)
                NetDeviceContainer gnbDevs;
                NetDeviceContainer ueDevs;

                CcBwpCreator::SimpleOperationBandConf bandConf(29e9, 100e6, 1);
                CcBwpCreator ccBwpCreator;
                OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
                // Initialize channel and pathloss, plus other things inside band.
                channelHelper->AssignChannelsToBands({band});

                BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

                // Asymmetric arrays: a larger gNB panel and a smaller UE panel, so
                // that the gNB and UE zenith scan steps differ (this also acts as a
                // regression test for the per-device zenith step in the beam scan).
                nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
                nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
                nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
                nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));

                // Antenna element type for both gNB and UE
                if (iso)
                {
                    nrHelper->SetGnbAntennaAttribute(
                        "AntennaElement",
                        PointerValue(CreateObject<IsotropicAntennaModel>()));
                    nrHelper->SetUeAntennaAttribute(
                        "AntennaElement",
                        PointerValue(CreateObject<IsotropicAntennaModel>()));
                }
                else
                {
                    nrHelper->SetGnbAntennaAttribute(
                        "AntennaElement",
                        PointerValue(CreateObject<ThreeGppAntennaModel>()));
                    nrHelper->SetUeAntennaAttribute(
                        "AntennaElement",
                        PointerValue(CreateObject<ThreeGppAntennaModel>()));
                }

                nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());

                gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
                ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

                // This test requires that the NrGnbNetDevice::ConfigureCell()
                // is called before GetBeamformingVectors() is called below.
                for (auto it = gnbDevs.Begin(); it != gnbDevs.End(); ++it)
                {
                    DynamicCast<NrGnbNetDevice>(*it)->ConfigureCell();
                }
                uint32_t stream = 1;
                stream += nrHelper->AssignStreams(gnbDevs, stream);
                stream += nrHelper->AssignStreams(ueDevs, stream);

                Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevs.Get(0), 0);

                Ptr<NrSpectrumPhy> txSpectrumPhy =
                    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->GetSpectrumPhy();

                Ptr<CellScanBeamforming> cellScanBeamforming = CreateObject<CellScanBeamforming>();

                BeamformingVectorPair bfPairIdeal =
                    cellScanBeamforming->GetBeamformingVectors(txSpectrumPhy,
                                                               uePhy->GetSpectrumPhy());

                Ptr<RealisticBeamformingAlgorithm> realisticBeamforming =
                    CreateObject<RealisticBeamformingAlgorithm>();
                realisticBeamforming->Install(
                    txSpectrumPhy,
                    uePhy->GetSpectrumPhy(),
                    DynamicCast<NrGnbNetDevice>(gnbDevs.Get(0))->GetScheduler(0));
                stream += realisticBeamforming->AssignStreams(stream);

                for (size_t i = 0; i < srsSinrsDb.size(); i++)
                {
                    // directly update max SINR SRS, skipping other set functions of
                    // the algorithm
                    realisticBeamforming->m_maxSrsSinrPerSlot =
                        std::pow(10.0, srsSinrsDb[i] / 10.0);

                    BeamformingVectorPair bfPairReal =
                        realisticBeamforming->GetBeamformingVectors();

                    if ((bfPairIdeal.first.second == bfPairReal.first.second) &&
                        (bfPairIdeal.second.second == bfPairReal.second.second))
                    {
                        matchCounters[i]++;
                    }
                }
            }
        }
    }

    std::vector<double> matchRates(srsSinrsDb.size());
    for (size_t i = 0; i < srsSinrsDb.size(); i++)
    {
        matchRates[i] = matchCounters[i] / static_cast<double>(totalCounter);
        NS_LOG_INFO("SRS SINR " << srsSinrsDb[i] << " dB: beam pair matched the ideal one in "
                                << matchCounters[i] << " out of " << totalCounter << " cases.");
    }

    // Allow a small statistical slack in the monotonicity check, since the
    // number of scenarios per duration mode is limited.
    const double slack = 1.0 / totalCounter;
    for (size_t i = 1; i < matchRates.size(); i++)
    {
        NS_TEST_ASSERT_MSG_GT_OR_EQ(matchRates[i] + slack,
                                    matchRates[i - 1],
                                    "The beam-match rate with the ideal selection should not "
                                    "decrease when the SRS SINR increases");
    }
    NS_TEST_ASSERT_MSG_GT_OR_EQ(matchRates.back(),
                                0.7,
                                "At high SRS SINR the realistic algorithm should select the same "
                                "beam pair as the ideal algorithm in most of the cases");
    NS_TEST_ASSERT_MSG_LT_OR_EQ(matchRates.front(),
                                0.3,
                                "At low SRS SINR the realistic algorithm should rarely select the "
                                "same beam pair as the ideal algorithm");

    Simulator::Destroy();
}

// Do not forget to allocate an instance of this TestSuite
static NrRealisticBeamformingTestSuite nrTestSuite;

} // namespace ns3
