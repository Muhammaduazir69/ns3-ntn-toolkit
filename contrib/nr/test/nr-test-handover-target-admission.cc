// Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-handover-target-admission.cc
 *
 * @brief Test suite `nr-handover-target-admission`: verifies the target-admission floors
 * (`MinTargetRsrpDbm` and `MinTargetRsrqDb`) of NrA3RsrpHandoverAlgorithm. A single UE moves along
 * a line from a strong serving gNB towards a weaker neighbour gNB and would normally hand over to
 * it. With the floors at their disabling defaults the handover occurs, but when the RSRP floor is
 * raised above the weak neighbour's reported RSRP the neighbour is rejected as a handover target,
 * so the handover does not happen. The test counts NrGnbRrc/HandoverEndOk events across runs and
 * asserts that raising the floor strictly reduces (and, at the extreme, eliminates) handovers.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrHandoverTargetAdmissionTest");

/**
 * @ingroup nr-test
 *
 * @brief Runs a single moving-UE, two-gNB scenario with a configurable RSRP/RSRQ target-admission
 * floor and reports the number of successful handovers observed, so different floor settings can be
 * compared.
 */
class NrHandoverTargetAdmissionTestCase : public TestCase
{
  public:
    /**
     * Constructor.
     *
     * @param name descriptive name of the test case
     * @param minTargetRsrpDbm value for the `MinTargetRsrpDbm` attribute (dBm)
     * @param minTargetRsrqDb value for the `MinTargetRsrqDb` attribute (dB)
     * @param expectHandover true if at least one handover is expected, false if none is expected
     */
    NrHandoverTargetAdmissionTestCase(std::string name,
                                      double minTargetRsrpDbm,
                                      double minTargetRsrqDb,
                                      bool expectHandover);

    /**
     * @brief Runs the scenario and returns the number of successful handovers observed.
     * @return the number of NrGnbRrc/HandoverEndOk events fired during the run
     */
    uint32_t CountHandovers();

  private:
    void DoRun() override;

    /**
     * @brief Callback connected to NrGnbRrc/HandoverEndOk that increments the handover counter.
     * @param context the trace context string
     * @param imsi the IMSI of the UE that completed the handover
     * @param cellId the target cell ID
     * @param rnti the RNTI of the UE at the target cell
     */
    void HandoverEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);

    double m_minTargetRsrpDbm;   ///< value for the MinTargetRsrpDbm attribute (dBm)
    double m_minTargetRsrqDb;    ///< value for the MinTargetRsrqDb attribute (dB)
    bool m_expectHandover;       ///< whether at least one handover is expected
    uint32_t m_handoverCount{0}; ///< number of successful handovers observed
};

NrHandoverTargetAdmissionTestCase::NrHandoverTargetAdmissionTestCase(std::string name,
                                                                     double minTargetRsrpDbm,
                                                                     double minTargetRsrqDb,
                                                                     bool expectHandover)
    : TestCase(name),
      m_minTargetRsrpDbm(minTargetRsrpDbm),
      m_minTargetRsrqDb(minTargetRsrqDb),
      m_expectHandover(expectHandover)
{
}

void
NrHandoverTargetAdmissionTestCase::HandoverEndOkCallback(std::string /* context */,
                                                         uint64_t /* imsi */,
                                                         uint16_t /* cellId */,
                                                         uint16_t /* rnti */)
{
    ++m_handoverCount;
}

uint32_t
NrHandoverTargetAdmissionTestCase::CountHandovers()
{
    m_handoverCount = 0;

    Config::Reset();
    Config::SetDefault("ns3::NrGnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(MilliSeconds(200)));
    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(20));
    // Disable Uplink Power Control
    Config::SetDefault("ns3::NrUePhy::EnableUplinkPowerControl", BooleanValue(false));

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));

    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    // Small hysteresis and time-to-trigger so the marginal neighbour would normally be selected.
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(128)));
    nrHelper->SetHandoverAlgorithmAttribute("MinTargetRsrpDbm", DoubleValue(m_minTargetRsrpDbm));
    nrHelper->SetHandoverAlgorithmAttribute("MinTargetRsrqDb", DoubleValue(m_minTargetRsrqDb));

    double distance = 1000.0; // m
    double speed = 150;       // m/s
    uint32_t nGnbs = 2;

    NodeContainer gnbNodes;
    gnbNodes.Create(nGnbs);
    NodeContainer ueNodes;
    ueNodes.Create(1);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    // gNBs are located along a line in the X axis
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nGnbs; i++)
    {
        gnbPositionAlloc->Add(Vector(distance * (i + 1), 0, 0));
    }
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // UE moves with a constant speed along the X axis, from before gNB 0 towards gNB 1
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed, 0, 0));

    // Override the default antenna model with IsotropicAntennaModel
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    // Configure Friis propagation loss model before assigning it to the band
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    for (auto it = gnbDevices.Begin(); it != gnbDevices.End(); ++it)
    {
        Ptr<NrGnbRrc> gnbRrc = (*it)->GetObject<NrGnbNetDevice>()->GetRrc();
        gnbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(true));
    }

    // Weaken the neighbour (gNB 1) so it becomes only a marginal handover target: the UE crosses a
    // region where gNB 1 is barely stronger than gNB 0. Lowering its TxPower keeps its RSRP modest,
    // so an RSRP floor above that level rejects it as a target.
    Ptr<NrGnbPhy> gnb1Phy = NrHelper::GetGnbPhy(gnbDevices.Get(1), 0);
    gnb1Phy->SetTxPower(2.0);

    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    nrHelper->AssignStreams({.assignEpc = true,
                             .ueNodes = ueNodes,
                             .gnbNodes = gnbNodes,
                             .ueNodeStream = 3000,
                             .gnbNodeStream = 2000});

    // All UEs attached to gNB 0 at the beginning
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueDevices.Get(i), gnbDevices.Get(0));
    }

    nrHelper->AddX2Interface(gnbNodes);

    // Count successful handovers observed at the gNB RRC.
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NrHandoverTargetAdmissionTestCase::HandoverEndOkCallback, this));

    Simulator::Stop(Seconds(15));
    Simulator::Run();
    Simulator::Destroy();

    return m_handoverCount;
}

void
NrHandoverTargetAdmissionTestCase::DoRun()
{
    uint32_t handovers = CountHandovers();
    if (m_expectHandover)
    {
        NS_TEST_ASSERT_MSG_GT_OR_EQ(handovers, 1u, "expected at least one handover but saw none");
    }
    else
    {
        NS_TEST_ASSERT_MSG_EQ(handovers, 0u, "expected no handover into the rejected weak target");
    }
}

/**
 * @ingroup nr-test
 *
 * @brief NR handover target-admission test suite. Runs the same moving-UE scenario with the
 * admission floors disabled (baseline handover) and raised (weak target rejected), and asserts that
 * raising the floor strictly reduces the number of handovers.
 */
class NrHandoverTargetAdmissionTestSuite : public TestSuite
{
  public:
    NrHandoverTargetAdmissionTestSuite();
};

NrHandoverTargetAdmissionTestSuite::NrHandoverTargetAdmissionTestSuite()
    : TestSuite("nr-handover-target-admission", Type::SYSTEM)
{
    // Baseline: floors at their disabling defaults, handover into the weak neighbour occurs.
    auto baseline =
        new NrHandoverTargetAdmissionTestCase("baseline: default floors (-140 and -100), "
                                              "handover occurs",
                                              -140.0,
                                              -100.0,
                                              true);

    // Extreme RSRP floor: essentially require a very strong target, so the weak neighbour is
    // rejected and no handover happens. -44 dBm is the maximum of the MinTargetRsrpDbm range.
    auto flooredRsrp = new NrHandoverTargetAdmissionTestCase("RSRP floor -44 dBm rejects weak "
                                                             "target, no handover",
                                                             -44.0,
                                                             -100.0,
                                                             false);

    // RSRQ floor: identical rejection code path as the RSRP floor. -3 dB is the maximum of the
    // MinTargetRsrqDb range; a UE at the cell edge reports RSRQ well below this, so the weak target
    // is rejected on the RSRQ criterion.
    auto flooredRsrq = new NrHandoverTargetAdmissionTestCase("RSRQ floor -3 dB rejects weak "
                                                             "target, no handover",
                                                             -140.0,
                                                             -3.0,
                                                             false);

    AddTestCase(baseline, Duration::QUICK);
    AddTestCase(flooredRsrp, Duration::QUICK);
    AddTestCase(flooredRsrq, Duration::QUICK);
}

/**
 * @ingroup nr-test
 * Static variable for test initialization
 */
static NrHandoverTargetAdmissionTestSuite g_nrHandoverTargetAdmissionTestSuiteInstance;
