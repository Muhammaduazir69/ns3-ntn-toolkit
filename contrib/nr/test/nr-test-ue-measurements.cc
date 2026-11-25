// Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
//
// Author: Manuel Requena <manuel.requena@cttc.es>
//         Nicola Baldo <nbaldo@cttc.es>
//         Marco Miozzo <mmiozzo@cttc.es>
//              adapt lte-test-interference.cc to lte-ue-measurements.cc
//         Budiarto Herman <budiarto.herman@magister.fi>

/**
 * @ingroup test
 * @file nr-test-ue-measurements.cc
 *
 * @brief Implementation of the UE measurement test suites. `nr-ue-measurements` places two
 * gNB/UE pairs at parametric distances and compares the RSRP/RSRQ reported by the UE PHY and
 * carried in RRC measurement reports against precomputed reference values. The piecewise suites
 * (`nr-ue-measurements-piecewise-1/2/3`) run one UE against one, two or three gNBs, teleporting
 * the UE (or a gNB) between near and far positions, and assert that reports triggered by events
 * A1-A5 arrive exactly at the expected times and carry the expected RSRP range values.
 * `nr-ue-measurements-handover` switches the UE measurement configuration through a
 * mid-simulation handover between two gNBs and verifies the reports produced under the target
 * cell's configuration.
 */

#include "nr-test-ue-measurements.h"

#include "ns3/boolean.h"
#include "ns3/callback.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/nr-channel-helper.h"
#include "ns3/nr-common.h"
#include "ns3/nr-epc-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-ue-phy.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrUeMeasurementsTest");

// ===== LTE-UE-MEASUREMENTS TEST SUITE ==================================== //

void
ReportUeMeasurementsCallback(NrUeMeasurementsTestCase* testcase,
                             std::string path,
                             uint16_t rnti,
                             uint16_t cellId,
                             double rsrp,
                             double rsrq,
                             bool servingCell,
                             uint8_t componentCarrierId)
{
    testcase->ReportUeMeasurements(rnti, cellId, rsrp, rsrq, servingCell);
}

void
RecvMeasurementReportCallback(NrUeMeasurementsTestCase* testcase,
                              std::string path,
                              uint64_t imsi,
                              uint16_t cellId,
                              uint16_t rnti,
                              NrRrcSap::MeasurementReport meas)
{
    testcase->RecvMeasurementReport(imsi, cellId, rnti, meas);
}

/*
 * Test Suite
 */

NrUeMeasurementsTestSuite::NrUeMeasurementsTestSuite()
    : TestSuite("nr-ue-measurements", Type::SYSTEM)
{
    // RSRQ expectations are the NR fully-loaded RSRQ: RSSI is the total received
    // power over all 12 subcarriers per RB (NrSpectrumValueHelper::SUBCARRIERS_PER_RB),
    // so a dominant cell reads RSRP / (12 * RSRP) = 1/12 ~ -10.8 dB. These differ from
    // the legacy LTE placeholders (which assumed an unloaded cell, max RSRQ -3 dB) by a
    // fixed 10*log10(12/2) = 7.78 dB for the dominant-cell cases.
    AddTestCase(new NrUeMeasurementsTestCase("d1=10, d2=10000",
                                             10.000000,
                                             10000.000000,
                                             -56.33,
                                             -116.33,
                                             -10.7918,
                                             -70.7918),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=20, d2=10000",
                                             20.000000,
                                             10000.000000,
                                             -62.35,
                                             -116.33,
                                             -10.7918,
                                             -64.7712),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=50, d2=10000",
                                             50.000000,
                                             10000.000000,
                                             -70.31,
                                             -116.33,
                                             -10.7919,
                                             -56.8125),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=100, d2=10000",
                                             100.000000,
                                             10000.000000,
                                             -76.33,
                                             -116.33,
                                             -10.7923,
                                             -50.7923),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=200, d2=10000",
                                             200.000000,
                                             10000.000000,
                                             -82.35,
                                             -116.33,
                                             -10.7937,
                                             -44.7731),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=100, d2=10000",
                                             100.000000,
                                             10000.000000,
                                             -76.33,
                                             -116.33,
                                             -10.7923,
                                             -50.7923),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=200, d2=10000",
                                             200.000000,
                                             10000.000000,
                                             -82.35,
                                             -116.33,
                                             -10.7937,
                                             -44.7731),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=500, d2=10000",
                                             500.000000,
                                             10000.000000,
                                             -90.32,
                                             -116.33,
                                             -10.8035,
                                             -36.8241),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=1000, d2=10000",
                                             1000.000000,
                                             10000.000000,
                                             -96.33,
                                             -116.33,
                                             -10.8385,
                                             -30.8385),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=2000, d2=10000",
                                             2000.000000,
                                             10000.000000,
                                             -102.35,
                                             -116.33,
                                             -10.9758,
                                             -24.9552),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=5000, d2=10000",
                                             5000.000000,
                                             10000.000000,
                                             -110.31,
                                             -116.33,
                                             -11.8312,
                                             -17.8518),
                TestCase::Duration::QUICK);
    AddTestCase(new NrUeMeasurementsTestCase("d1=10000, d2=10000",
                                             10000.000000,
                                             10000.000000,
                                             -116.33,
                                             -116.33,
                                             -13.9758,
                                             -13.9758),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=20000, d2=10000",
                                             20000.000000,
                                             10000.000000,
                                             -122.35,
                                             -116.33,
                                             -18.0562,
                                             -12.0356),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=50000, d2=10000",
                                             50000.000000,
                                             10000.000000,
                                             -130.31,
                                             -116.33,
                                             -25.2697,
                                             -11.2903),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=100000, d2=10000",
                                             100000.000000,
                                             10000.000000,
                                             -136.33,
                                             -116.33,
                                             -31.1725,
                                             -11.1725),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=200000, d2=10000",
                                             200000.000000,
                                             10000.000000,
                                             -142.35,
                                             -116.33,
                                             -37.1632,
                                             -11.1426),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=500000, d2=10000",
                                             500000.000000,
                                             10000.000000,
                                             -150.31,
                                             -116.33,
                                             -45.1136,
                                             -11.1342),
                TestCase::Duration::EXTENSIVE);
    AddTestCase(new NrUeMeasurementsTestCase("d1=1000000, d2=10000",
                                             1000000.000000,
                                             10000.000000,
                                             -156.33,
                                             -116.33,
                                             -51.1330,
                                             -11.1330),
                TestCase::Duration::EXTENSIVE);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrUeMeasurementsTestSuite nrUeMeasurementsTestSuite;

/*
 * Test Case
 */

NrUeMeasurementsTestCase::NrUeMeasurementsTestCase(std::string name,
                                                   double d1,
                                                   double d2,
                                                   double rsrpDbmUe1,
                                                   double rsrpDbmUe2,
                                                   double rsrqDbUe1,
                                                   double rsrqDbUe2)
    : TestCase(name),
      m_d1(d1),
      m_d2(d2),
      m_rsrpDbmUeServingCell(rsrpDbmUe1),
      m_rsrpDbmUeNeighborCell(rsrpDbmUe2),
      m_rsrqDbUeServingCell(rsrqDbUe1),
      m_rsrqDbUeNeighborCell(rsrqDbUe2)
{
    NS_LOG_INFO("Test UE Measurements d1 = " << d1 << " m. and d2 = " << d2 << " m.");
}

NrUeMeasurementsTestCase::~NrUeMeasurementsTestCase()
{
}

void
NrUeMeasurementsTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Config::SetDefault("ns3::NrSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrAmc::AmcModel", EnumValue(NrAmc::ShannonModel));
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    // todo: switch back to false when RRC real is working
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));

    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // LogComponentEnable ("NrUeMeasurementsTest", LOG_LEVEL_ALL);

    // Create Nodes: gNB and UE
    NodeContainer nrNodes;
    NodeContainer ueNodes1;
    NodeContainer ueNodes2;
    nrNodes.Create(2);
    ueNodes1.Create(1);
    ueNodes2.Create(1);
    NodeContainer allNodes = NodeContainer(nrNodes, ueNodes1, ueNodes2);

    // the topology is the following:
    //         d2
    //  UE1-----------gNB2
    //   |             |
    // d1|             |d1
    //   |     d2      |
    //  gNB1----------UE2
    //
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // gNB1
    positionAlloc->Add(Vector(m_d2, m_d1, 0.0)); // gNB2
    positionAlloc->Add(Vector(0.0, m_d1, 0.0));  // UE1
    positionAlloc->Add(Vector(m_d2, 0.0, 0.0));  // UE2
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(allNodes);

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (gNB and UE)
    NetDeviceContainer nrDevs;
    NetDeviceContainer ueDevs1;
    NetDeviceContainer ueDevs2;
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrDevs = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    ueDevs1 = nrHelper->InstallUeDevice(ueNodes1, allBwps);
    ueDevs2 = nrHelper->InstallUeDevice(ueNodes2, allBwps);

    // Attach UEs to gNBs
    for (uint32_t i = 0; i < ueDevs1.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueDevs1.Get(i), nrDevs.Get(0));
    }
    for (uint32_t i = 0; i < ueDevs2.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueDevs2.Get(i), nrDevs.Get(1));
    }

    // Activate an QoS flow
    NrQosFlow::FiveQi q = NrQosFlow::GBR_CONV_VOICE;
    NrQosFlow flow(q);
    nrHelper->ActivateDataRadioBearer(ueDevs1, flow);
    nrHelper->ActivateDataRadioBearer(ueDevs2, flow);

    Config::Connect("/NodeList/2/DeviceList/0/$ns3::NrNetDevice/$ns3::NrUeNetDevice/"
                    "ComponentCarrierMapUe/*/NrUePhy/ReportUeMeasurements",
                    MakeBoundCallback(&ReportUeMeasurementsCallback, this));
    Config::Connect("/NodeList/0/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
                    MakeBoundCallback(&RecvMeasurementReportCallback, this));

    Config::Connect("/NodeList/3/DeviceList/0/$ns3::NrNetDevice/$ns3::NrUeNetDevice/"
                    "ComponentCarrierMapUe/*/NrUePhy/ReportUeMeasurements",
                    MakeBoundCallback(&ReportUeMeasurementsCallback, this));
    Config::Connect("/NodeList/1/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
                    MakeBoundCallback(&RecvMeasurementReportCallback, this));

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.gnbDevs = nrDevs, .ueDevs = ueDevs1});
    nrHelper->AssignStreams({.ueDevs = ueDevs2, .ueDevStream = 7000});

    // need to allow for RRC connection establishment + SRS
    Simulator::Stop(Seconds(0.800));
    Simulator::Run();

    Simulator::Destroy();
}

void
NrUeMeasurementsTestCase::ReportUeMeasurements(uint16_t rnti,
                                               uint16_t cellId,
                                               double rsrp,
                                               double rsrq,
                                               bool servingCell)
{
    // need to allow for RRC connection establishment + CQI feedback reception + UE measurements
    // filtering (200 ms)
    if (Simulator::Now() > MilliSeconds(400))
    {
        if (servingCell)
        {
            NS_LOG_DEBUG("UE serving cellId " << cellId << " Rxed RSRP " << rsrp << " thr "
                                              << m_rsrpDbmUeServingCell << " RSRQ " << rsrq
                                              << " thr " << m_rsrqDbUeServingCell);
            NS_TEST_ASSERT_MSG_EQ_TOL(rsrp, m_rsrpDbmUeServingCell, 0.2, "Wrong RSRP UE 1");
            NS_TEST_ASSERT_MSG_EQ_TOL(rsrq, m_rsrqDbUeServingCell, 0.2, "Wrong RSRQ UE 1");
        }
        else
        {
            NS_LOG_DEBUG("UE neighbor cellId " << cellId << " Rxed RSRP " << rsrp << " thr "
                                               << m_rsrpDbmUeNeighborCell << " RSRQ " << rsrq
                                               << " thr " << m_rsrqDbUeNeighborCell);
            NS_TEST_ASSERT_MSG_EQ_TOL(rsrp, m_rsrpDbmUeNeighborCell, 0.2, "Wrong RSRP UE 2");
            NS_TEST_ASSERT_MSG_EQ_TOL(rsrq, m_rsrqDbUeNeighborCell, 0.2, "Wrong RSRQ UE 2");
        }
    }
}

void
NrUeMeasurementsTestCase::RecvMeasurementReport(uint64_t imsi,
                                                uint16_t cellId,
                                                uint16_t rnti,
                                                NrRrcSap::MeasurementReport meas)
{
    // need to allow for RRC connection establishment + CQI feedback reception + UE measurements
    // filtering (200 ms)
    if (Simulator::Now() > MilliSeconds(400))
    {
        if (cellId == imsi)
        {
            NS_LOG_DEBUG(this << "Serving Cell: received IMSI " << imsi << " CellId " << cellId
                              << " RNTI " << rnti << " thr "
                              << (uint16_t)nr::EutranMeasurementMapping::Dbm2RsrpRange(
                                     m_rsrpDbmUeServingCell)
                              << " RSRP " << (uint16_t)meas.measResults.measResultPCell.rsrpResult
                              << " RSRQ " << (uint16_t)meas.measResults.measResultPCell.rsrqResult
                              << " thr "
                              << (uint16_t)nr::EutranMeasurementMapping::Db2RsrqRange(
                                     m_rsrqDbUeServingCell));
            NS_TEST_ASSERT_MSG_EQ(
                meas.measResults.measResultPCell.rsrpResult,
                nr::EutranMeasurementMapping::Dbm2RsrpRange(m_rsrpDbmUeServingCell),
                "Wrong RSRP ");
            NS_TEST_ASSERT_MSG_EQ(meas.measResults.measResultPCell.rsrqResult,
                                  nr::EutranMeasurementMapping::Db2RsrqRange(m_rsrqDbUeServingCell),
                                  "Wrong RSRQ ");
        }
        else
        {
            NS_LOG_DEBUG(this << "Neighbor cell: received IMSI " << imsi << " CellId " << cellId
                              << " RNTI " << rnti << " thr "
                              << (uint16_t)nr::EutranMeasurementMapping::Dbm2RsrpRange(
                                     m_rsrpDbmUeNeighborCell)
                              << " RSRP " << (uint16_t)meas.measResults.measResultPCell.rsrpResult
                              << " RSRQ " << (uint16_t)meas.measResults.measResultPCell.rsrqResult
                              << " thr "
                              << (uint16_t)nr::EutranMeasurementMapping::Db2RsrqRange(
                                     m_rsrqDbUeNeighborCell));
            NS_TEST_ASSERT_MSG_EQ(
                meas.measResults.measResultPCell.rsrpResult,
                nr::EutranMeasurementMapping::Dbm2RsrpRange(m_rsrpDbmUeNeighborCell),
                "Wrong RSRP ");
            NS_TEST_ASSERT_MSG_EQ(
                meas.measResults.measResultPCell.rsrqResult,
                nr::EutranMeasurementMapping::Db2RsrqRange(m_rsrqDbUeNeighborCell),
                "Wrong RSRQ ");
        }
    }
}

// ===== LTE-UE-MEASUREMENTS-PIECEWISE-1 TEST SUITE ======================== //

/*
 * Overloaded operators, for the convenience of defining test cases
 */

std::vector<Time>&
operator<<(std::vector<Time>& v, const uint64_t& ms)
{
    /*
     * Prior attempt to use seconds as unit of choice resunrd in precision lost.
     * Therefore milliseconds are used now instead.
     */
    v.push_back(MilliSeconds(ms) + NR_UE_MEASUREMENT_REPORT_DELAY);
    return v;
}

std::vector<uint8_t>&
operator<<(std::vector<uint8_t>& v, const uint8_t& range)
{
    v.push_back(range);
    return v;
}

/*
 * Test Suite
 */

NrUeMeasurementsPiecewiseTestSuite1::NrUeMeasurementsPiecewiseTestSuite1()
    : TestSuite("nr-ue-measurements-piecewise-1", Type::SYSTEM)
{
    std::vector<Time> expectedTime;
    std::vector<uint8_t> expectedRsrp;

    // === Event A1 (serving cell becomes better than threshold) ===

    // With very low threshold
    NrRrcSap::ReportConfigEutra config;
    config.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    config.threshold1.choice = NrRrcSap::ThresholdEutra::THRESHOLD_RSRP;
    config.threshold1.range = 0;
    config.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    config.reportInterval = NrRrcSap::ReportConfigEutra::MS120;

    expectedTime.clear();
    expectedTime << 200 << 320 << 440 << 560 << 680 << 800 << 920 << 1040 << 1160 << 1280 << 1400
                 << 1520 << 1640 << 1760 << 1880 << 2000 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 80 << 80 << 77 << 77 << 80 << 62 << 62 << 80 << 80 << 71 << 65 << 65 << 60 << 60
                 << 65 << 71 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with very low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With normal threshold
    config.threshold1.range = 70;
    expectedTime.clear();
    expectedTime << 200 << 320 << 440 << 560 << 680 << 1000 << 1120 << 1240 << 1360 << 2000 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 80 << 80 << 77 << 77 << 80 << 80 << 80 << 71 << 71 << 71 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With short time-to-trigger
    config.timeToTrigger = 64;
    expectedTime.clear();
    expectedTime << 264 << 384 << 504 << 624 << 744 << 1064 << 1184 << 1304 << 1424 << 2064 << 2184;
    expectedRsrp.clear();
    expectedRsrp << 80 << 80 << 77 << 80 << 80 << 80 << 80 << 71 << 65 << 71 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with short time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With long time-to-trigger
    config.timeToTrigger = 128;
    expectedTime.clear();
    expectedTime << 328 << 448 << 568 << 688 << 808 << 1128 << 1248 << 1368 << 1488 << 2128;
    expectedRsrp.clear();
    expectedRsrp << 80 << 77 << 77 << 80 << 62 << 80 << 71 << 71 << 65 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with long time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With super time-to-trigger
    config.timeToTrigger = 256;
    expectedTime.clear();
    expectedTime << 456 << 576 << 696 << 816 << 936 << 1056 << 1176 << 1296 << 1416 << 1536;
    expectedRsrp.clear();
    expectedRsrp << 77 << 77 << 80 << 62 << 62 << 80 << 80 << 71 << 65 << 65;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with super time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With hysteresis
    config.hysteresis = 8;
    config.timeToTrigger = 0;
    expectedTime.clear();
    expectedTime << 200 << 320 << 440 << 560 << 680 << 1000 << 1120 << 1240 << 1360 << 2200;
    expectedRsrp.clear();
    expectedRsrp << 80 << 80 << 77 << 77 << 80 << 80 << 80 << 71 << 71 << 80;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase1("Piecewise test case 1 - Event A1 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::QUICK);

    // With very high threshold
    config.threshold1.range = 113;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A1 with very high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // === Event A2 (serving cell becomes worse than threshold) ===

    // With very low threshold
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    config.threshold1.range = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with very low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With normal threshold
    config.threshold1.range = 70;
    expectedTime.clear();
    expectedTime << 800 << 920 << 1400 << 1520 << 1640 << 1760 << 1880;
    expectedRsrp.clear();
    expectedRsrp << 62 << 62 << 65 << 65 << 60 << 60 << 65;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With short time-to-trigger
    config.timeToTrigger = 64;
    expectedTime.clear();
    expectedTime << 864 << 984 << 1464 << 1584 << 1704 << 1824 << 1944;
    expectedRsrp.clear();
    expectedRsrp << 62 << 62 << 65 << 65 << 60 << 65 << 65;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with short time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With long time-to-trigger
    config.timeToTrigger = 128;
    expectedTime.clear();
    expectedTime << 928 << 1048 << 1528 << 1648 << 1768 << 1888 << 2008;
    expectedRsrp.clear();
    expectedRsrp << 62 << 80 << 65 << 60 << 60 << 65 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with long time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With super time-to-trigger
    config.timeToTrigger = 256;
    expectedTime.clear();
    expectedTime << 1656 << 1776 << 1896 << 2016 << 2136;
    expectedRsrp.clear();
    expectedRsrp << 60 << 60 << 65 << 71 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with super time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With hysteresis
    config.hysteresis = 8;
    config.timeToTrigger = 0;
    expectedTime.clear();
    expectedTime << 800 << 920 << 1400 << 1520 << 1640 << 1760 << 1880 << 2000 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 62 << 62 << 65 << 65 << 60 << 60 << 65 << 71 << 71;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase1("Piecewise test case 1 - Event A2 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::EXTENSIVE);

    // With very high threshold
    config.threshold1.range = 113;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedTime << 200 << 320 << 440 << 560 << 680 << 800 << 920 << 1040 << 1160 << 1280 << 1400
                 << 1520 << 1640 << 1760 << 1880 << 2000 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 80 << 80 << 77 << 77 << 80 << 62 << 62 << 80 << 80 << 71 << 65 << 65 << 60 << 60
                 << 65 << 71 << 71;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1(
                    "Piecewise test case 1 - Event A2 with very high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    /*
     * Event A3, A4, and A5 are not tested intensively here because they depend on
     * the existence of at least one neighbouring cell, which is not available in
     * this configuration. Piecewise configuration #2 includes a neighbouring
     * cell, hence more thorough tests on these events are performed there.
     */

    expectedTime.clear();
    expectedRsrp.clear();

    // === Event A3 (neighbour becomes offset better than PCell) ===

    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    config.a3Offset = 0;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1("Piecewise test case 1 - Event A3",
                                                       config,
                                                       expectedTime,
                                                       expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // === Event A4 (neighbour becomes better than threshold) ===

    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    config.threshold1.range = 70;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1("Piecewise test case 1 - Event A4",
                                                       config,
                                                       expectedTime,
                                                       expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // === Event A5 (PCell becomes worse than absolute threshold1 AND neighbour becomes better than
    // another absolute threshold2) ===

    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    config.threshold2.range = 74;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase1("Piecewise test case 1 - Event A5",
                                                       config,
                                                       expectedTime,
                                                       expectedRsrp),
                TestCase::Duration::EXTENSIVE);

} // end of NrUeMeasurementsPiecewiseTestSuite1::NrUeMeasurementsPiecewiseTestSuite1

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrUeMeasurementsPiecewiseTestSuite1 nrUeMeasurementsPiecewiseTestSuite1;

/*
 * Test Case
 */

NrUeMeasurementsPiecewiseTestCase1::NrUeMeasurementsPiecewiseTestCase1(
    std::string name,
    NrRrcSap::ReportConfigEutra config,
    std::vector<Time> expectedTime,
    std::vector<uint8_t> expectedRsrp)
    : TestCase(name),
      m_config(config),
      m_expectedTime(expectedTime),
      m_expectedRsrp(expectedRsrp)
{
    // input sanity check
    uint16_t size = m_expectedTime.size();

    if (size != m_expectedRsrp.size())
    {
        NS_FATAL_ERROR("Vectors of expected results are not of the same size");
    }

    m_itExpectedTime = m_expectedTime.begin();
    m_itExpectedRsrp = m_expectedRsrp.begin();

    NS_LOG_INFO(this << " name=" << name);
}

NrUeMeasurementsPiecewiseTestCase1::~NrUeMeasurementsPiecewiseTestCase1()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeMeasurementsPiecewiseTestCase1::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));

    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Create Nodes: gNB and UE
    NodeContainer nrNodes;
    NodeContainer ueNodes;
    nrNodes.Create(1);
    ueNodes.Create(1);

    /*
     * The topology is the following:
     *
     * gNB     UE
     *    |       |
     *    x ----- x --------- x --------------- x ------------------- x
     *      100 m |   200 m   |      300 m      |        400 m        |
     *            |           |                 |                     |
     *         VeryNear      Near              Far                 VeryFar
     */

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // gNB
    positionAlloc->Add(Vector(100.0, 0.0, 0.0)); // UE
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(nrNodes);
    mobility.Install(ueNodes);
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    // Disable layer-3 filtering
    Config::SetDefault("ns3::NrGnbRrc::RsrpFilterCoefficient", UintegerValue(0));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (gNB and UE)
    NetDeviceContainer nrDevs;
    NetDeviceContainer ueDevs;
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrDevs = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Setup UE measurement configuration
    Ptr<NrGnbRrc> nrRrc = nrDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
    m_expectedMeasId = nrRrc->AddUeMeasReportConfig(m_config).at(0);

    // Attach UE to gNB
    nrHelper->AttachToGnb(ueDevs.Get(0), nrDevs.Get(0));

    // Activate an QoS flow
    NrQosFlow::FiveQi q = NrQosFlow::GBR_CONV_VOICE;
    NrQosFlow flow(q);
    nrHelper->ActivateDataRadioBearer(ueDevs, flow);

    // Connect to trace sources
    Config::Connect(
        "/NodeList/0/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
        MakeCallback(&NrUeMeasurementsPiecewiseTestCase1::RecvMeasurementReportCallback, this));

    /*
     * Schedule "teleports"
     *          0                   1                   2
     *          +-------------------+-------------------+---------> time
     * VeryNear |------  ----    ----                    --------
     *     Near |                    ----            ----
     *      Far |                        ----    ----
     *  VeryFar |      --    ----            ----
     */
    Simulator::Schedule(MilliSeconds(301),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(401),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryNear,
                        this);
    Simulator::Schedule(MilliSeconds(601),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(801),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryNear,
                        this);
    Simulator::Schedule(MilliSeconds(1001),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportNear,
                        this);
    Simulator::Schedule(MilliSeconds(1201), &NrUeMeasurementsPiecewiseTestCase1::TeleportFar, this);
    Simulator::Schedule(MilliSeconds(1401),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(1601), &NrUeMeasurementsPiecewiseTestCase1::TeleportFar, this);
    Simulator::Schedule(MilliSeconds(1801),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportNear,
                        this);
    Simulator::Schedule(MilliSeconds(2001),
                        &NrUeMeasurementsPiecewiseTestCase1::TeleportVeryNear,
                        this);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.gnbDevs = nrDevs, .ueDevs = ueDevs});

    // Run simulation
    Simulator::Stop(Seconds(2.201));
    Simulator::Run();
    Simulator::Destroy();

} // end of void NrUeMeasurementsPiecewiseTestCase1::DoRun ()

void
NrUeMeasurementsPiecewiseTestCase1::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    bool hasEnded = m_itExpectedTime == m_expectedTime.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "Reporting should have occurred at " << m_itExpectedTime->As(Time::S));
    hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "List still contains "
                              << m_expectedRsrp.size() -
                                     std::distance(m_expectedRsrp.begin(), m_itExpectedRsrp)
                              << " samples");
}

void
NrUeMeasurementsPiecewiseTestCase1::RecvMeasurementReportCallback(
    std::string context,
    uint64_t imsi,
    uint16_t cellId,
    uint16_t rnti,
    NrRrcSap::MeasurementReport report)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(rnti == 1);
    NS_ASSERT(cellId == 1);

    if (report.measResults.measId == m_expectedMeasId)
    {
        // verifying the report completeness
        NrRrcSap::MeasResults measResults = report.measResults;
        NS_LOG_DEBUG(this << " rsrp=" << (uint16_t)measResults.measResultPCell.rsrpResult << " ("
                          << nr::EutranMeasurementMapping::RsrpRange2Dbm(
                                 measResults.measResultPCell.rsrpResult)
                          << " dBm)"
                          << " rsrq=" << (uint16_t)measResults.measResultPCell.rsrqResult << " ("
                          << nr::EutranMeasurementMapping::RsrqRange2Db(
                                 measResults.measResultPCell.rsrqResult)
                          << " dB)");
        NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                              false,
                              "Report should not have neighboring cells information");
        NS_TEST_ASSERT_MSG_EQ(measResults.measResultListEutra.size(), 0, "Unexpected report size");

        bool hasEnded = m_itExpectedTime == m_expectedTime.end();
        NS_TEST_ASSERT_MSG_EQ(hasEnded,
                              false,
                              "Reporting should not have occurred at "
                                  << Simulator::Now().As(Time::S));
        if (!hasEnded)
        {
            hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
            NS_ASSERT(!hasEnded);

            // using milliseconds to avoid floating-point comparison
            uint64_t timeNowMs = Simulator::Now().GetMilliSeconds();
            uint64_t timeExpectedMs = m_itExpectedTime->GetMilliSeconds();
            m_itExpectedTime++;

            uint16_t observedRsrp = measResults.measResultPCell.rsrpResult;
            uint16_t referenceRsrp = *m_itExpectedRsrp;
            m_itExpectedRsrp++;

            NS_TEST_ASSERT_MSG_EQ(timeNowMs,
                                  timeExpectedMs,
                                  "Reporting should not have occurred at this time");
            NS_TEST_ASSERT_MSG_EQ(observedRsrp,
                                  referenceRsrp,
                                  "The RSRP observed differs with the reference RSRP");
        } // end of if (!hasEnded)

    } // end of if (measResults.measId == m_expectedMeasId)

} // end of NrUeMeasurementsPiecewiseTestCase1::RecvMeasurementReportCallback

void
NrUeMeasurementsPiecewiseTestCase1::TeleportVeryNear()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(100.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase1::TeleportNear()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(300.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase1::TeleportFar()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(600.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase1::TeleportVeryFar()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(1000.0, 0.0, 0.0));
}

// ===== LTE-UE-MEASUREMENTS-PIECEWISE-2 TEST SUITE ======================== //

/*
 * Test Suite
 */

NrUeMeasurementsPiecewiseTestSuite2::NrUeMeasurementsPiecewiseTestSuite2()
    : TestSuite("nr-ue-measurements-piecewise-2", Type::SYSTEM)
{
    std::vector<Time> expectedTime;
    std::vector<uint8_t> expectedRsrp;

    /*
     * Higher level of fullness/duration are given to Event A1 and A2 because they
     * are supposed to be more intensively tested in Piecewise configuration #1.
     */

    // === Event A1 (serving cell becomes better than threshold) ===

    // With very low threshold
    NrRrcSap::ReportConfigEutra config;
    config.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    config.threshold1.choice = NrRrcSap::ThresholdEutra::THRESHOLD_RSRP;
    config.threshold1.range = 0;
    config.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    config.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1160 << 1400 << 1640 << 1880 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 67 << 86 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A1 with very low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With normal threshold
    config.threshold1.range = 74;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 1000 << 2200;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 86 << 86;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A1 with normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With hysteresis
    config.hysteresis = 6;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 1000 << 1240 << 2200;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 86 << 73 << 86;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A1 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::EXTENSIVE);

    // With very high threshold
    config.threshold1.range = 97;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A1 with very high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // === Event A2 (serving cell becomes worse than threshold) ===

    // With very low threshold
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    config.threshold1.range = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A2 with very low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With normal threshold
    config.threshold1.range = 74;
    expectedTime.clear();
    expectedTime << 800 << 1200 << 1440 << 1680 << 1920 << 2160;
    expectedRsrp.clear();
    expectedRsrp << 67 << 73 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A2 with normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With hysteresis
    config.hysteresis = 6;
    expectedTime.clear();
    expectedTime << 800 << 1400 << 1640 << 1880 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 67 << 69 << 65 << 69 << 72;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A2 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::EXTENSIVE);

    // With very high threshold
    config.threshold1.range = 97;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1160 << 1400 << 1640 << 1880 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 67 << 86 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A2 with very high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // === Event A3 (neighbour becomes offset better than PCell) ===

    // With positive offset
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    config.threshold1.range = 0;
    config.a3Offset = 7;
    expectedTime.clear();
    expectedTime << 800 << 1600 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 67 << 65 << 69;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A3 with positive offset",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With zero offset
    config.a3Offset = 0;
    expectedTime.clear();
    expectedTime << 800 << 1400 << 1640 << 1880;
    expectedRsrp.clear();
    expectedRsrp << 67 << 69 << 65 << 69;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A3 with zero offset",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::EXTENSIVE);

    // With short time-to-trigger
    config.timeToTrigger = 160;
    expectedTime.clear();
    expectedTime << 960 << 1560 << 1800 << 2040;
    expectedRsrp.clear();
    expectedRsrp << 67 << 69 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A3 with short time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With super time-to-trigger
    config.timeToTrigger = 320;
    expectedTime.clear();
    expectedTime << 1720 << 1960 << 2200;
    expectedRsrp.clear();
    expectedRsrp << 65 << 69 << 86;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A3 with super time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With hysteresis and reportOnLeave
    config.hysteresis = 6;
    config.reportOnLeave = true;
    config.timeToTrigger = 0;
    expectedTime.clear();
    expectedTime << 800 << 1000 << 1600 << 1840 << 2080 << 2200;
    expectedRsrp.clear();
    expectedRsrp << 67 << 86 << 65 << 69 << 72 << 86;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A3 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::QUICK);

    // With negative offset
    config.a3Offset = -7;
    config.hysteresis = 0;
    config.reportOnLeave = false;
    expectedTime.clear();
    expectedTime << 400 << 800 << 1200 << 1440 << 1680 << 1920 << 2160;
    expectedRsrp.clear();
    expectedRsrp << 83 << 67 << 73 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A3 with negative offset",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // === Event A4 (neighbour becomes better than threshold) ===

    // With very low threshold
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    config.threshold1.range = 0;
    config.a3Offset = 0;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1160 << 1400 << 1640 << 1880 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 67 << 86 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A4 with very low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With normal threshold
    config.threshold1.range = 74;
    expectedTime.clear();
    expectedTime << 400 << 800 << 1600 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 83 << 67 << 65 << 69;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A4 with normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With short time-to-trigger
    config.timeToTrigger = 160;
    expectedTime.clear();
    expectedTime << 560 << 960 << 1760 << 2000;
    expectedRsrp.clear();
    expectedRsrp << 83 << 67 << 65 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A4 with short time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With super time-to-trigger
    config.timeToTrigger = 320;
    expectedTime.clear();
    expectedRsrp.clear();
    expectedTime.clear();
    expectedTime << 1920 << 2160;
    expectedRsrp.clear();
    expectedRsrp << 69 << 72;

    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A4 with super time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With hysteresis
    config.hysteresis = 6;
    config.timeToTrigger = 0;
    expectedTime.clear();
    expectedTime << 400 << 640 << 880 << 1120 << 1600 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 83 << 86 << 67 << 86 << 65 << 69;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A4 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::QUICK);

    // With very high threshold
    config.threshold1.range = 97;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A4 with very high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // === Event A5 (PCell becomes worse than absolute threshold1 AND neighbour becomes better than
    // another absolute threshold2) ===

    // With low-low threshold
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    config.threshold1.range = 0;
    config.threshold2.range = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with low-low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With low-normal threshold
    config.threshold2.range = 58;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with low-normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With low-high threshold
    config.threshold2.range = 97;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with low-high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With normal-low threshold
    config.threshold1.range = 74;
    config.threshold2.range = 0;
    expectedTime.clear();
    expectedTime << 800 << 1200 << 1440 << 1680 << 1920 << 2160;
    expectedRsrp.clear();
    expectedRsrp << 67 << 73 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with normal-low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With normal-normal threshold
    config.threshold2.range = 74;
    expectedTime.clear();
    expectedTime << 800 << 1600 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 67 << 65 << 69;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with normal-normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With short time-to-trigger
    config.timeToTrigger = 160;
    expectedTime.clear();
    expectedTime << 960 << 1760 << 2000;
    expectedRsrp.clear();
    expectedRsrp << 67 << 65 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with short time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With super time-to-trigger
    config.timeToTrigger = 320;
    expectedTime.clear();
    // expectedTime << 1720 << 1960 << 2200;
    expectedRsrp.clear();
    // expectedRsrp << 68 << 72 << 72;    expectedTime.clear();
    expectedTime << 1920 << 2160;
    expectedRsrp.clear();
    expectedRsrp << 69 << 72;

    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with super time-to-trigger",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::QUICK);

    // With hysteresis
    config.hysteresis = 6;
    config.timeToTrigger = 0;
    expectedTime.clear();
    expectedTime << 800 << 1600 << 1840; //<< 2080;
    expectedRsrp.clear();
    expectedRsrp << 67 << 65 << 69; // << 59;
    AddTestCase(
        new NrUeMeasurementsPiecewiseTestCase2("Piecewise test case 2 - Event A5 with hysteresis",
                                               config,
                                               expectedTime,
                                               expectedRsrp),
        TestCase::Duration::QUICK);

    // With normal-high threshold
    config.threshold2.range = 97;
    config.hysteresis = 0;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with normal-high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With high-low threshold
    config.threshold1.range = 97;
    config.threshold2.range = 0;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1160 << 1400 << 1640 << 1880 << 2120;
    expectedRsrp.clear();
    expectedRsrp << 86 << 83 << 86 << 67 << 86 << 69 << 65 << 69 << 72;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with high-low threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

    // With high-normal threshold
    config.threshold2.range = 74;
    expectedTime.clear();
    expectedTime << 400 << 800 << 1600 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 83 << 67 << 65 << 69;
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with high-normal threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::TAKES_FOREVER);

    // With high-high threshold
    config.threshold2.range = 113;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsPiecewiseTestCase2(
                    "Piecewise test case 2 - Event A5 with high-high threshold",
                    config,
                    expectedTime,
                    expectedRsrp),
                TestCase::Duration::EXTENSIVE);

} // end of NrUeMeasurementsPiecewiseTestSuite2::NrUeMeasurementsPiecewiseTestSuite2

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrUeMeasurementsPiecewiseTestSuite2 nrUeMeasurementsPiecewiseTestSuite2;

/*
 * Test Case
 */

NrUeMeasurementsPiecewiseTestCase2::NrUeMeasurementsPiecewiseTestCase2(
    std::string name,
    NrRrcSap::ReportConfigEutra config,
    std::vector<Time> expectedTime,
    std::vector<uint8_t> expectedRsrp)
    : TestCase(name),
      m_config(config),
      m_expectedTime(expectedTime),
      m_expectedRsrp(expectedRsrp)
{
    // input sanity check
    uint16_t size = m_expectedTime.size();

    if (size != m_expectedRsrp.size())
    {
        NS_FATAL_ERROR("Vectors of expected results are not of the same size");
    }

    m_itExpectedTime = m_expectedTime.begin();
    m_itExpectedRsrp = m_expectedRsrp.begin();

    NS_LOG_INFO(this << " name=" << name);
}

NrUeMeasurementsPiecewiseTestCase2::~NrUeMeasurementsPiecewiseTestCase2()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeMeasurementsPiecewiseTestCase2::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Create Nodes: gNB and UE
    NodeContainer nrNodes;
    NodeContainer ueNodes;
    nrNodes.Create(2);
    ueNodes.Create(1);

    /*
     * The topology is the following:
     *
     * gNB    UE                                                gNB
     *    |      |                                                    |
     *    x ---- x --------------- x ------- x --------------- x ---- x
     *      50 m |      200 m      |  100 m  |      200 m      | 50 m
     *           |                 |         |                 |
     *        VeryNear            Near      Far             VeryFar
     */

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Serving gNB
    positionAlloc->Add(Vector(600.0, 0.0, 0.0)); // Neighbour gNB
    positionAlloc->Add(Vector(50.0, 0.0, 0.0));  // UE
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(nrNodes);
    mobility.Install(ueNodes);
    m_ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();

    // Disable layer-3 filtering
    Config::SetDefault("ns3::NrGnbRrc::RsrpFilterCoefficient", UintegerValue(0));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (gNB and UE)
    NetDeviceContainer nrDevs;
    NetDeviceContainer ueDevs;
    nrDevs = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    // Setup UE measurement configuration in serving cell
    Ptr<NrGnbRrc> nrRrc1 = nrDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
    m_expectedMeasId = nrRrc1->AddUeMeasReportConfig(m_config).at(0);

    // Disable handover in neighbour cell
    Ptr<NrGnbRrc> nrRrc2 = nrDevs.Get(1)->GetObject<NrGnbNetDevice>()->GetRrc();
    nrRrc2->SetAttribute("AdmitHandoverRequest", BooleanValue(false));

    // Attach UE to serving gNB
    nrHelper->AttachToGnb(ueDevs.Get(0), nrDevs.Get(0));

    // Activate an QoS flow
    NrQosFlow::FiveQi q = NrQosFlow::GBR_CONV_VOICE;
    NrQosFlow flow(q);
    nrHelper->ActivateDataRadioBearer(ueDevs, flow);

    // Connect to trace sources in serving gNB
    Config::Connect(
        "/NodeList/0/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
        MakeCallback(&NrUeMeasurementsPiecewiseTestCase2::RecvMeasurementReportCallback, this));

    /*
     * Schedule "teleports"
     *          0                   1                   2
     *          +-------------------+-------------------+---------> time
     * VeryNear |------  ----    ----                    --------
     *     Near |                    ----            ----
     *      Far |                        ----    ----
     *  VeryFar |      --    ----            ----
     */
    Simulator::Schedule(MilliSeconds(301),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(401),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryNear,
                        this);
    Simulator::Schedule(MilliSeconds(601),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(801),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryNear,
                        this);
    Simulator::Schedule(MilliSeconds(1001),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportNear,
                        this);
    Simulator::Schedule(MilliSeconds(1201), &NrUeMeasurementsPiecewiseTestCase2::TeleportFar, this);
    Simulator::Schedule(MilliSeconds(1401),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryFar,
                        this);
    Simulator::Schedule(MilliSeconds(1601), &NrUeMeasurementsPiecewiseTestCase2::TeleportFar, this);
    Simulator::Schedule(MilliSeconds(1801),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportNear,
                        this);
    Simulator::Schedule(MilliSeconds(2001),
                        &NrUeMeasurementsPiecewiseTestCase2::TeleportVeryNear,
                        this);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.gnbDevs = nrDevs, .ueDevs = ueDevs});

    // Run simulation
    Simulator::Stop(Seconds(2.201));
    Simulator::Run();
    Simulator::Destroy();

} // end of void NrUeMeasurementsPiecewiseTestCase2::DoRun ()

void
NrUeMeasurementsPiecewiseTestCase2::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    bool hasEnded = m_itExpectedTime == m_expectedTime.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "Reporting should have occurred at " << m_itExpectedTime->As(Time::S));
    hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "List still contains "
                              << m_expectedRsrp.size() -
                                     std::distance(m_expectedRsrp.begin(), m_itExpectedRsrp)
                              << " samples");
}

void
NrUeMeasurementsPiecewiseTestCase2::RecvMeasurementReportCallback(
    std::string context,
    uint64_t imsi,
    uint16_t cellId,
    uint16_t rnti,
    NrRrcSap::MeasurementReport report)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(rnti == 1);
    NS_ASSERT(cellId == 1);

    if (report.measResults.measId == m_expectedMeasId)
    {
        // verifying the report completeness
        NrRrcSap::MeasResults measResults = report.measResults;
        NS_LOG_DEBUG(this << " Serving cellId=" << cellId
                          << " rsrp=" << (uint16_t)measResults.measResultPCell.rsrpResult << " ("
                          << nr::EutranMeasurementMapping::RsrpRange2Dbm(
                                 measResults.measResultPCell.rsrpResult)
                          << " dBm)"
                          << " rsrq=" << (uint16_t)measResults.measResultPCell.rsrqResult << " ("
                          << nr::EutranMeasurementMapping::RsrqRange2Db(
                                 measResults.measResultPCell.rsrqResult)
                          << " dB)");

        // verifying reported best cells
        if (measResults.measResultListEutra.empty())
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  false,
                                  "Unexpected report content");
        }
        else
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  true,
                                  "Unexpected report content");
            auto it = measResults.measResultListEutra.begin();
            NS_ASSERT(it != measResults.measResultListEutra.end());
            NS_ASSERT(it->physCellId == 2);
            NS_TEST_ASSERT_MSG_EQ(it->haveCgiInfo,
                                  false,
                                  "Report contains cgi-info, which is not supported");
            NS_TEST_ASSERT_MSG_EQ(it->haveRsrpResult,
                                  true,
                                  "Report does not contain measured RSRP result");
            NS_TEST_ASSERT_MSG_EQ(it->haveRsrqResult,
                                  true,
                                  "Report does not contain measured RSRQ result");
            NS_LOG_DEBUG(
                this << " Neighbour cellId=" << it->physCellId
                     << " rsrp=" << (uint16_t)it->rsrpResult << " ("
                     << nr::EutranMeasurementMapping::RsrpRange2Dbm(it->rsrpResult) << " dBm)"
                     << " rsrq=" << (uint16_t)it->rsrqResult << " ("
                     << nr::EutranMeasurementMapping::RsrqRange2Db(it->rsrqResult) << " dB)");

        } // end of else of if (measResults.measResultListEutra.size () == 0)

        // verifying the report timing
        bool hasEnded = m_itExpectedTime == m_expectedTime.end();
        NS_TEST_ASSERT_MSG_EQ(hasEnded,
                              false,
                              "Reporting should not have occurred at "
                                  << Simulator::Now().As(Time::S));
        if (!hasEnded)
        {
            hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
            NS_ASSERT(!hasEnded);

            // using milliseconds to avoid floating-point comparison
            uint64_t timeNowMs = Simulator::Now().GetMilliSeconds();
            uint64_t timeExpectedMs = m_itExpectedTime->GetMilliSeconds();
            m_itExpectedTime++;

            uint16_t observedRsrp = measResults.measResultPCell.rsrpResult;
            uint16_t referenceRsrp = *m_itExpectedRsrp;
            m_itExpectedRsrp++;

            NS_TEST_ASSERT_MSG_EQ(timeNowMs,
                                  timeExpectedMs,
                                  "Reporting should not have occurred at this time");
            NS_TEST_ASSERT_MSG_EQ(observedRsrp,
                                  referenceRsrp,
                                  "The RSRP observed differs with the reference RSRP");

        } // end of if (!hasEnded)

    } // end of if (report.measResults.measId == m_expectedMeasId)

} // end of void NrUeMeasurementsPiecewiseTestCase2::RecvMeasurementReportCallback

void
NrUeMeasurementsPiecewiseTestCase2::TeleportVeryNear()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(50.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase2::TeleportNear()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(250.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase2::TeleportFar()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(350.0, 0.0, 0.0));
}

void
NrUeMeasurementsPiecewiseTestCase2::TeleportVeryFar()
{
    NS_LOG_FUNCTION(this);
    m_ueMobility->SetPosition(Vector(550.0, 0.0, 0.0));
}

// ===== LTE-UE-MEASUREMENTS-PIECEWISE-3 TEST SUITE ======================== //

/*
 * Test Suite
 */

NrUeMeasurementsPiecewiseTestSuite3::NrUeMeasurementsPiecewiseTestSuite3()
    : TestSuite("nr-ue-measurements-piecewise-3", Type::SYSTEM)
{
    std::vector<Time> expectedTime;

    // === Event A4 (neighbor becomes better than threshold) ===

    // The threshold value was chosen to achieve the following:
    // 1. Neighbor 1 (gNB2) RSRP would be above the chosen threshold, hence,
    // the UE will include it in its reports to its gNB (gNB1) from the beginning
    // of the simulation.
    // 2. When neighbor 2 (gNB3) is placed at a very far position, its RSRP would
    // be less than the chosen threshold, hence, UE will not include it in its
    // initial report(s) to its gNB.
    // 3. When neighbor 2 (gNB3) is placed at a near position, its RSRP would
    // always be above the chosen threshold, hence, the UE will include it in its
    // reports to its gNB (gNB1).
    NrRrcSap::ReportConfigEutra config;
    config.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    config.eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    config.threshold1.choice = NrRrcSap::ThresholdEutra::THRESHOLD_RSRP;
    config.threshold1.range = 6;
    config.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    config.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1160 << 1400 << 1640 << 1880 << 2120;

    AddTestCase(new NrUeMeasurementsPiecewiseTestCase3("Piecewise test case 3 - Event A4",
                                                       config,
                                                       expectedTime),
                TestCase::Duration::QUICK);
} // end of NrUeMeasurementsPiecewiseTestSuite3::NrUeMeasurementsPiecewiseTestSuite3

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrUeMeasurementsPiecewiseTestSuite3 nrUeMeasurementsPiecewiseTestSuite3;

/*
 * Test Case
 */

NrUeMeasurementsPiecewiseTestCase3::NrUeMeasurementsPiecewiseTestCase3(
    std::string name,
    NrRrcSap::ReportConfigEutra config,
    std::vector<Time> expectedTime)
    : TestCase(name),
      m_config(config),
      m_expectedTime(expectedTime)
{
    m_expectedMeasId = std::numeric_limits<uint8_t>::max();

    m_itExpectedTime = m_expectedTime.begin();

    NS_LOG_INFO(this << " name=" << name);
}

NrUeMeasurementsPiecewiseTestCase3::~NrUeMeasurementsPiecewiseTestCase3()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeMeasurementsPiecewiseTestCase3::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));
    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Create Nodes: gNB and UE
    NodeContainer nrNodes;
    NodeContainer ueNodes;
    nrNodes.Create(3);
    ueNodes.Create(1);

    /*
     * The topology is the following:
     *
     * We place the 3rd gNB initially very far so it does not fulfills
     * the entry condition to be reported.
     *
     * gNB    UE              gNB                                  gNB
     *    |      |                 |                                       |
     *    x ---- x --------------- x -------------- x ---------------------x
     *      50 m         100 m             500      |         1000000
     *                                             Near
     */

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));       // Serving gNB
    positionAlloc->Add(Vector(200.0, 0.0, 0.0));     // Neighbour gNB1
    positionAlloc->Add(Vector(1000700.0, 0.0, 0.0)); // Neighbour gNB2
    positionAlloc->Add(Vector(50.0, 0.0, 0.0));      // UE
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(nrNodes);
    mobility.Install(ueNodes);
    m_gnbMobility = nrNodes.Get(2)->GetObject<MobilityModel>();

    // Disable layer-3 filtering
    Config::SetDefault("ns3::NrGnbRrc::RsrpFilterCoefficient", UintegerValue(0));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (gNB and UE)
    NetDeviceContainer nrDevs;
    NetDeviceContainer ueDevs;
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrDevs = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Setup UE measurement configuration in serving cell
    Ptr<NrGnbRrc> nrRrc1 = nrDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
    m_expectedMeasId = nrRrc1->AddUeMeasReportConfig(m_config).at(0);

    // Disable handover in neighbour cells
    Ptr<NrGnbRrc> nrRrc2 = nrDevs.Get(1)->GetObject<NrGnbNetDevice>()->GetRrc();
    nrRrc2->SetAttribute("AdmitHandoverRequest", BooleanValue(false));
    Ptr<NrGnbRrc> nrRrc3 = nrDevs.Get(2)->GetObject<NrGnbNetDevice>()->GetRrc();
    nrRrc3->SetAttribute("AdmitHandoverRequest", BooleanValue(false));

    // Attach UE to serving gNB
    nrHelper->AttachToGnb(ueDevs.Get(0), nrDevs.Get(0));

    // Activate an QoS flow
    NrQosFlow::FiveQi q = NrQosFlow::GBR_CONV_VOICE;
    NrQosFlow flow(q);
    nrHelper->ActivateDataRadioBearer(ueDevs, flow);

    // Connect to trace sources in serving gNB
    Config::Connect(
        "/NodeList/0/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
        MakeCallback(&NrUeMeasurementsPiecewiseTestCase3::RecvMeasurementReportCallback, this));
    /*
     * Schedule "teleport" for the 2nd neighbour
     *
     * We bring the 2nd neighbour near once the UE has already scheduled the periodic
     * reporting after detecting the 1st neighbour, which ideally should be at
     * 200 ms.
     */
    Simulator::Schedule(MilliSeconds(301),
                        &NrUeMeasurementsPiecewiseTestCase3::TeleportGnbNear,
                        this);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.gnbDevs = nrDevs, .ueDevs = ueDevs});

    // Run simulation
    Simulator::Stop(Seconds(2.201));
    Simulator::Run();
    Simulator::Destroy();

} // end of void NrUeMeasurementsPiecewiseTestCase3::DoRun ()

void
NrUeMeasurementsPiecewiseTestCase3::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    bool hasEnded = m_itExpectedTime == m_expectedTime.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "Reporting should have occurred at " << m_itExpectedTime->GetSeconds()
                                                               << "s");
}

void
NrUeMeasurementsPiecewiseTestCase3::RecvMeasurementReportCallback(
    std::string context,
    uint64_t imsi,
    uint16_t cellId,
    uint16_t rnti,
    NrRrcSap::MeasurementReport report)
{
    NS_LOG_FUNCTION(this << context);
    NS_ASSERT(rnti == 1);
    NS_ASSERT(cellId == 1);

    if (report.measResults.measId == m_expectedMeasId)
    {
        // verifying the report completeness
        NrRrcSap::MeasResults measResults = report.measResults;
        NS_LOG_DEBUG(this << " Serving cellId=" << cellId
                          << " rsrp=" << (uint16_t)measResults.measResultPCell.rsrpResult << " ("
                          << nr::EutranMeasurementMapping::RsrpRange2Dbm(
                                 measResults.measResultPCell.rsrpResult)
                          << " dBm)"
                          << " rsrq=" << (uint16_t)measResults.measResultPCell.rsrqResult << " ("
                          << nr::EutranMeasurementMapping::RsrqRange2Db(
                                 measResults.measResultPCell.rsrqResult)
                          << " dB)");

        // verifying reported best cells
        if (measResults.measResultListEutra.empty())
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  false,
                                  "Unexpected report content");
        }
        else
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  true,
                                  "Unexpected report content");
            auto it = measResults.measResultListEutra.begin();
            NS_ASSERT(it != measResults.measResultListEutra.end());
            for (const auto& it : measResults.measResultListEutra)
            {
                NS_ASSERT(it.physCellId == 2 || it.physCellId == 3);
                NS_TEST_ASSERT_MSG_EQ(it.haveCgiInfo,
                                      false,
                                      "Report contains cgi-info, which is not supported");
                NS_TEST_ASSERT_MSG_EQ(it.haveRsrpResult,
                                      true,
                                      "Report does not contain measured RSRP result");
                NS_TEST_ASSERT_MSG_EQ(it.haveRsrqResult,
                                      true,
                                      "Report does not contain measured RSRQ result");
                NS_LOG_DEBUG(
                    this << " Neighbour cellId=" << it.physCellId
                         << " rsrp=" << (uint16_t)it.rsrpResult << " ("
                         << nr::EutranMeasurementMapping::RsrpRange2Dbm(it.rsrpResult) << " dBm)"
                         << " rsrq=" << (uint16_t)it.rsrqResult << " ("
                         << nr::EutranMeasurementMapping::RsrqRange2Db(it.rsrqResult) << " dB)");
            }

        } // end of else of if (measResults.measResultListEutra.size () == 0)

        // verifying the report timing
        bool hasEnded = m_itExpectedTime == m_expectedTime.end();
        NS_TEST_ASSERT_MSG_EQ(hasEnded,
                              false,
                              "Reporting should not have occurred at "
                                  << Simulator::Now().GetSeconds() << "s");
        if (!hasEnded)
        {
            // using milliseconds to avoid floating-point comparison
            uint64_t timeNowMs = Simulator::Now().GetMilliSeconds();
            uint64_t timeExpectedMs = m_itExpectedTime->GetMilliSeconds();
            m_itExpectedTime++;

            NS_TEST_ASSERT_MSG_EQ(timeNowMs,
                                  timeExpectedMs,
                                  "Reporting should not have occurred at this time");

        } // end of if (!hasEnded)

    } // end of if (report.measResults.measId == m_expectedMeasId)

} // end of void NrUeMeasurementsPiecewiseTestCase3::RecvMeasurementReportCallback

void
NrUeMeasurementsPiecewiseTestCase3::TeleportGnbNear()
{
    NS_LOG_FUNCTION(this);
    m_gnbMobility->SetPosition(Vector(700.0, 0.0, 0.0));
}

// ===== LTE-UE-MEASUREMENTS-HANDOVER TEST SUITE =========================== //

/*
 * Test Suite
 */

NrUeMeasurementsHandoverTestSuite::NrUeMeasurementsHandoverTestSuite()
    : TestSuite("nr-ue-measurements-handover", Type::SYSTEM)
{
    std::list<NrRrcSap::ReportConfigEutra> sourceConfigList;
    std::list<NrRrcSap::ReportConfigEutra> targetConfigList;
    std::vector<Time> expectedTime;
    std::vector<uint8_t> expectedRsrp;

    NrRrcSap::ReportConfigEutra sourceConfig;
    sourceConfig.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    sourceConfig.eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    sourceConfig.threshold1.choice = NrRrcSap::ThresholdEutra::THRESHOLD_RSRP;
    sourceConfig.threshold1.range = 0;
    sourceConfig.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    sourceConfig.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    sourceConfigList.push_back(sourceConfig);

    NrRrcSap::ReportConfigEutra targetConfig;
    targetConfig.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    targetConfig.eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    targetConfig.threshold1.choice = NrRrcSap::ThresholdEutra::THRESHOLD_RSRP;
    targetConfig.threshold1.range = 0;
    targetConfig.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    targetConfig.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    targetConfigList.push_back(targetConfig);

    // === Report interval difference ===

    // decreasing report interval
    sourceConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS480;
    targetConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    expectedTime.clear();
    expectedTime << 200 << 680 << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 66 << 66 << 66 << 66;
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - decreasing report interval",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::TAKES_FOREVER);

    // increasing report interval
    sourceConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS120;
    targetConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS640;
    expectedTime.clear();
    expectedTime << 200 << 320 << 440 << 560 << 680 << 800 << 920 << 1200 << 1840;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68 << 68 << 68 << 68 << 66 << 66;
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - increasing report interval",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::QUICK);

    // === Event difference ===

    sourceConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    targetConfigList.front().reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    sourceConfigList.front().threshold1.range = 70;
    sourceConfigList.front().threshold2.range = 70;
    sourceConfigList.front().a3Offset = 1;
    targetConfigList.front().threshold1.range = 70;
    targetConfigList.front().threshold2.range = 70;
    targetConfigList.front().a3Offset = 1;

    // Event A1 to Event A2
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    expectedTime.clear();
    expectedTime << 1200 << 1440 << 1680 << 1920; // << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 66 << 66 << 66 << 66; // << 53 << 53 << 53 << 53;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A1 to Event A2",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::EXTENSIVE);

    // Event A2 to Event A1
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920; // << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68; // << 53 << 53 << 53 << 53;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A2 to Event A1",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::TAKES_FOREVER);

    // Event A3 to Event A4
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A3 to Event A4",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::TAKES_FOREVER);

    // Event A4 to Event A3
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    expectedTime.clear();
    expectedTime << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 66 << 66 << 66 << 66;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A4 to Event A3",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::QUICK);

    // Event A2 to Event A3
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920 << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68 << 66 << 66 << 66 << 66;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A2 to Event A3",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::EXTENSIVE);

    // Event A3 to Event A2
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    expectedTime.clear();
    expectedTime << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 66 << 66 << 66 << 66;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A3 to Event A2",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::TAKES_FOREVER);

    // Event A4 to Event A5
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A4 to Event A5",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::TAKES_FOREVER);

    // Event A5 to Event A4
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - Event A5 to Event A4",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::EXTENSIVE);

    // === Threshold/offset difference ===

    sourceConfigList.front().threshold1.range = 68;
    targetConfigList.front().threshold1.range = 70;

    // Event A1
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68;
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - Event A1 threshold difference",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::EXTENSIVE);

    // Event A2
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A2;
    expectedTime.clear();
    expectedTime << 1200 << 1440 << 1680 << 1920;
    expectedRsrp.clear();
    expectedRsrp << 66 << 66 << 66 << 66;
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - Event A2 threshold difference",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::QUICK);

    // Event A3
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    sourceConfigList.front().a3Offset = -30;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    targetConfigList.front().a3Offset = 30;
    expectedTime.clear();
    expectedTime << 200 << 440 << 680 << 920;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68;
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - Event A3 offset difference",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::QUICK);

    // Event A4
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A4;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - Event A4 threshold difference",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::EXTENSIVE);

    // Event A5
    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    sourceConfigList.front().threshold2.range = 68;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A5;
    targetConfigList.front().threshold2.range = 72;
    expectedTime.clear();
    expectedRsrp.clear();
    AddTestCase(
        new NrUeMeasurementsHandoverTestCase("Handover test case - Event A5 threshold difference",
                                             sourceConfigList,
                                             targetConfigList,
                                             expectedTime,
                                             expectedRsrp,
                                             Seconds(2)),
        TestCase::Duration::EXTENSIVE);

    // === Time-to-trigger (TTT) difference ===

    sourceConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    sourceConfigList.front().a3Offset = 1;
    sourceConfigList.front().threshold1.range = 0;
    sourceConfigList.front().threshold2.range = 0;
    targetConfigList.front().eventId = NrRrcSap::ReportConfigEutra::EVENT_A1;
    targetConfigList.front().a3Offset = 1;
    targetConfigList.front().threshold1.range = 0;
    targetConfigList.front().threshold2.range = 0;

    // decreasing time-to-trigger (short duration)
    sourceConfigList.front().timeToTrigger = 1024;
    targetConfigList.front().timeToTrigger = 100;
    expectedTime.clear();
    expectedTime << 1300 << 1540 << 1780;
    expectedRsrp.clear();
    expectedRsrp << 66 << 66 << 66;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - decreasing TTT (short)",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(2)),
                TestCase::Duration::QUICK);

    // decreasing time-to-trigger (longer duration)
    sourceConfigList.front().timeToTrigger = 1024;
    targetConfigList.front().timeToTrigger = 640;
    expectedTime.clear();
    expectedTime << 1224 << 1464 << 1704 << 1944 << 2840 << 3080 << 3320 << 3560 << 3800 << 4040;
    expectedRsrp.clear();
    expectedRsrp << 68 << 68 << 68 << 68 << 66 << 66 << 66 << 66 << 66 << 66;
    AddTestCase(new NrUeMeasurementsHandoverTestCase("Handover test case - decreasing TTT (long)",
                                                     sourceConfigList,
                                                     targetConfigList,
                                                     expectedTime,
                                                     expectedRsrp,
                                                     Seconds(4.2)),
                TestCase::Duration::EXTENSIVE);

} // end of NrUeMeasurementsHandoverTestSuite::NrUeMeasurementsHandoverTestSuite

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrUeMeasurementsHandoverTestSuite nrUeMeasurementsHandoverTestSuite;

/*
 * Test Case
 */

NrUeMeasurementsHandoverTestCase::NrUeMeasurementsHandoverTestCase(
    std::string name,
    std::list<NrRrcSap::ReportConfigEutra> sourceConfigList,
    std::list<NrRrcSap::ReportConfigEutra> targetConfigList,
    std::vector<Time> expectedTime,
    std::vector<uint8_t> expectedRsrp,
    Time duration)
    : TestCase(name),
      m_sourceConfigList(sourceConfigList),
      m_targetConfigList(targetConfigList),
      m_expectedTime(expectedTime),
      m_expectedRsrp(expectedRsrp),
      m_duration(duration)
{
    // input sanity check
    uint16_t size = m_expectedTime.size();

    if (size != m_expectedRsrp.size())
    {
        NS_FATAL_ERROR("Vectors of expected results are not of the same size");
    }

    m_itExpectedTime = m_expectedTime.begin();
    m_itExpectedRsrp = m_expectedRsrp.begin();

    NS_LOG_INFO(this << " name=" << name);
}

NrUeMeasurementsHandoverTestCase::~NrUeMeasurementsHandoverTestCase()
{
    NS_LOG_FUNCTION(this);
}

void
NrUeMeasurementsHandoverTestCase::DoRun()
{
    NS_LOG_INFO(this << " " << GetName());

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(nrEpcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(30));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23));

    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    // Create Nodes: gNB and UE
    NodeContainer nrNodes;
    NodeContainer ueNodes;
    nrNodes.Create(2);
    ueNodes.Create(1);

    /*
     * The topology is the following:
     *
     * gNB                   UE                     gNB
     *    |                     |                         |
     *    x ------------------- x ----------------------- x
     *             400 m                   500 m
     */

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Source gNB
    positionAlloc->Add(Vector(900.0, 0.0, 0.0)); // Target gNB
    positionAlloc->Add(Vector(400.0, 0.0, 0.0)); // UE
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(nrNodes);
    mobility.Install(ueNodes);

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

    // Enable layer-3 filtering
    Config::SetDefault("ns3::NrGnbRrc::RsrpFilterCoefficient", UintegerValue(4));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assign it to band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    // Create and set the channel with the band
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 5e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    // Create bandwidth part from band
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Create Devices and install them in the Nodes (eNB and UE)
    NetDeviceContainer nrDevs;
    NetDeviceContainer ueDevs;
    nrDevs = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Setup UE measurement configuration in gNBs
    uint8_t measId;
    Ptr<NrGnbRrc> nrRrc1 = nrDevs.Get(0)->GetObject<NrGnbNetDevice>()->GetRrc();
    Ptr<NrGnbRrc> nrRrc2 = nrDevs.Get(1)->GetObject<NrGnbNetDevice>()->GetRrc();

    for (auto itReportConfig = m_sourceConfigList.begin();
         itReportConfig != m_sourceConfigList.end();
         itReportConfig++)
    {
        measId = nrRrc1->AddUeMeasReportConfig(*itReportConfig).at(0);
        m_expectedSourceCellMeasId.insert(measId);
    }

    for (auto itReportConfig = m_targetConfigList.begin();
         itReportConfig != m_targetConfigList.end();
         itReportConfig++)
    {
        measId = nrRrc2->AddUeMeasReportConfig(*itReportConfig).at(0);
        m_expectedTargetCellMeasId.insert(measId);
    }

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Attach UE to serving gNB
    nrHelper->AttachToGnb(ueDevs.Get(0), nrDevs.Get(0));

    // Add X2 interface
    nrHelper->AddX2Interface(nrNodes);

    // Connect to trace sources in source gNB
    Config::Connect(
        "/NodeList/3/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
        MakeCallback(&NrUeMeasurementsHandoverTestCase::RecvMeasurementReportCallback, this));

    // Connect to trace sources in target gNB
    Config::Connect(
        "/NodeList/4/DeviceList/0/NrGnbRrc/RecvMeasurementReport",
        MakeCallback(&NrUeMeasurementsHandoverTestCase::RecvMeasurementReportCallback, this));

    // Schedule handover
    nrHelper->HandoverRequest(MilliSeconds(m_duration.GetMilliSeconds() / 2),
                              ueDevs.Get(0),
                              nrDevs.Get(0),
                              nrDevs.Get(1));

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);
    nrHelper->AssignStreams({.assignEpc = true,
                             .remoteHostNodes = remoteHostContainer,
                             .ueNodes = ueNodes,
                             .gnbDevs = nrDevs,
                             .ueDevs = ueDevs});

    // Run simulation
    Simulator::Stop(m_duration);
    Simulator::Run();
    Simulator::Destroy();

} // end of void NrUeMeasurementsHandoverTestCase::DoRun ()

void
NrUeMeasurementsHandoverTestCase::DoTeardown()
{
    NS_LOG_FUNCTION(this);
    bool hasEnded = m_itExpectedTime == m_expectedTime.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "Reporting should have occurred at " << m_itExpectedTime->As(Time::S));
    hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
    NS_TEST_ASSERT_MSG_EQ(hasEnded,
                          true,
                          "List still contains "
                              << m_expectedRsrp.size() -
                                     std::distance(m_expectedRsrp.begin(), m_itExpectedRsrp)
                              << " samples");
}

void
NrUeMeasurementsHandoverTestCase::RecvMeasurementReportCallback(std::string context,
                                                                uint64_t imsi,
                                                                uint16_t cellId,
                                                                uint16_t rnti,
                                                                NrRrcSap::MeasurementReport report)
{
    uint8_t measId = report.measResults.measId;
    NS_LOG_FUNCTION(this << context << (uint16_t)measId);

    bool isCorrectMeasId;
    if (cellId == 1)
    {
        auto itMeasId = m_expectedSourceCellMeasId.find(measId);
        isCorrectMeasId = (itMeasId != m_expectedSourceCellMeasId.end());
    }
    else if (cellId == 2)
    {
        auto itMeasId = m_expectedTargetCellMeasId.find(measId);
        isCorrectMeasId = (itMeasId != m_expectedTargetCellMeasId.end());
    }
    else
    {
        NS_FATAL_ERROR("Invalid cell ID " << cellId);
    }

    if (isCorrectMeasId)
    {
        // verifying the report completeness
        NrRrcSap::MeasResults measResults = report.measResults;
        NS_LOG_DEBUG(this << " Serving cellId=" << cellId
                          << " rsrp=" << (uint16_t)measResults.measResultPCell.rsrpResult << " ("
                          << nr::EutranMeasurementMapping::RsrpRange2Dbm(
                                 measResults.measResultPCell.rsrpResult)
                          << " dBm)"
                          << " rsrq=" << (uint16_t)measResults.measResultPCell.rsrqResult << " ("
                          << nr::EutranMeasurementMapping::RsrqRange2Db(
                                 measResults.measResultPCell.rsrqResult)
                          << " dB)");

        // verifying reported best cells
        if (measResults.measResultListEutra.empty())
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  false,
                                  "Unexpected report content");
        }
        else
        {
            NS_TEST_ASSERT_MSG_EQ(measResults.haveMeasResultNeighCells,
                                  true,
                                  "Unexpected report content");
            auto it = measResults.measResultListEutra.begin();
            NS_ASSERT(it != measResults.measResultListEutra.end());
            NS_ASSERT(it->physCellId != cellId);
            NS_ASSERT(it->physCellId <= 2);
            NS_TEST_ASSERT_MSG_EQ(it->haveCgiInfo,
                                  false,
                                  "Report contains cgi-info, which is not supported");
            NS_TEST_ASSERT_MSG_EQ(it->haveRsrpResult,
                                  true,
                                  "Report does not contain measured RSRP result");
            NS_TEST_ASSERT_MSG_EQ(it->haveRsrqResult,
                                  true,
                                  "Report does not contain measured RSRQ result");
            NS_LOG_DEBUG(
                this << " Neighbour cellId=" << it->physCellId
                     << " rsrp=" << (uint16_t)it->rsrpResult << " ("
                     << nr::EutranMeasurementMapping::RsrpRange2Dbm(it->rsrpResult) << " dBm)"
                     << " rsrq=" << (uint16_t)it->rsrqResult << " ("
                     << nr::EutranMeasurementMapping::RsrqRange2Db(it->rsrqResult) << " dB)");

        } // end of else of if (measResults.measResultListEutra.size () == 0)

        // verifying the report timing
        bool hasEnded = m_itExpectedTime == m_expectedTime.end();
        NS_TEST_ASSERT_MSG_EQ(hasEnded,
                              false,
                              "Reporting should not have occurred at "
                                  << Simulator::Now().As(Time::S));
        if (!hasEnded)
        {
            hasEnded = m_itExpectedRsrp == m_expectedRsrp.end();
            NS_ASSERT(!hasEnded);

            // using milliseconds to avoid floating-point comparison
            uint64_t timeNowMs = Simulator::Now().GetMilliSeconds();
            uint64_t timeExpectedMs = m_itExpectedTime->GetMilliSeconds();
            m_itExpectedTime++;

            uint16_t observedRsrp = measResults.measResultPCell.rsrpResult;
            uint16_t referenceRsrp = *m_itExpectedRsrp;
            m_itExpectedRsrp++;

            NS_TEST_ASSERT_MSG_EQ(timeNowMs,
                                  timeExpectedMs,
                                  "Reporting should not have occurred at this time");
            NS_TEST_ASSERT_MSG_EQ(observedRsrp,
                                  referenceRsrp,
                                  "The RSRP observed differs with the reference RSRP");

        } // end of if (!hasEnded)

    } // end of if (report.measResults.measId == correctMeasId)

} // end of void NrUeMeasurementsHandoverTestCase::RecvMeasurementReportCallback
