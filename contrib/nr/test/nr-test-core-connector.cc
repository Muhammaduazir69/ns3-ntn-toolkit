// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup test
 * @file nr-test-core-connector.cc
 *
 * @brief Test suite `nr-core-connector`: checks that the core network can be replaced by an
 * NrEpcHelper subclass without NrHelper knowing about it.
 *
 * NrHelper only calls NrEpcHelper::AddGnb() and leaves to the EPC helper the connection of the
 * gNB RRC to the entity implementing NrEpcGnbS1SapProvider. The test installs an EPC helper
 * that interposes its own NrEpcGnbS1SapProvider between the RRC and the built-in core, as an
 * external core connector would, and checks that the UEs go through it to reach the ACTIVE
 * NAS state.
 */

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <memory>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrCoreConnectorTest");

namespace
{

/**
 * @ingroup nr-test
 *
 * @brief Core-facing gNB entity that counts the procedures started by the RRC and forwards them
 * to the built-in core.
 */
class NrTestCoreConnector : public NrEpcGnbS1SapProvider
{
  public:
    /**
     * Constructor.
     * @param core the core-facing entity the procedures are forwarded to
     */
    explicit NrTestCoreConnector(NrEpcGnbS1SapProvider* core)
        : m_core(core)
    {
    }

    void InitialUeMessage(uint64_t imsi, uint16_t rnti) override
    {
        m_initialUeMessages++;
        m_core->InitialUeMessage(imsi, rnti);
    }

    void DoSendReleaseIndication(uint64_t imsi, uint16_t rnti, uint8_t qfi) override
    {
        m_core->DoSendReleaseIndication(imsi, rnti, qfi);
    }

    void PathSwitchRequest(PathSwitchRequestParameters params) override
    {
        m_core->PathSwitchRequest(params);
    }

    void UeContextRelease(uint16_t rnti) override
    {
        m_core->UeContextRelease(rnti);
    }

    uint32_t m_initialUeMessages{0}; ///< number of InitialUeMessage procedures seen

  private:
    NrEpcGnbS1SapProvider* m_core; ///< built-in core-facing entity
};

/**
 * @ingroup nr-test
 *
 * @brief EPC helper connecting each gNB RRC to an NrTestCoreConnector.
 */
class NrTestCoreConnectorEpcHelper : public NrPointToPointEpcHelper
{
  public:
    /**
     * Register this type.
     * @return The object TypeId.
     */
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::NrTestCoreConnectorEpcHelper")
                                .SetParent<NrPointToPointEpcHelper>()
                                .SetGroupName("Nr")
                                .AddConstructor<NrTestCoreConnectorEpcHelper>();
        return tid;
    }

    void AddGnb(Ptr<Node> gnbNode, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId) override
    {
        NrPointToPointEpcHelper::AddGnb(gnbNode, nrGnbNetDevice, cellId);
        auto gnbApp = gnbNode->GetApplication(0)->GetObject<NrEpcGnbApplication>();
        auto connector = std::make_unique<NrTestCoreConnector>(gnbApp->GetS1SapProvider());
        DynamicCast<NrGnbNetDevice>(nrGnbNetDevice)->GetRrc()->SetS1SapProvider(connector.get());
        m_connectors.push_back(std::move(connector));
    }

    /**
     * @return the total number of InitialUeMessage procedures seen by the connectors
     */
    uint32_t GetInitialUeMessages() const
    {
        uint32_t count = 0;
        for (const auto& connector : m_connectors)
        {
            count += connector->m_initialUeMessages;
        }
        return count;
    }

  private:
    std::vector<std::unique_ptr<NrTestCoreConnector>> m_connectors; ///< one connector per gNB
};

NS_OBJECT_ENSURE_REGISTERED(NrTestCoreConnectorEpcHelper);

} // namespace

/**
 * @ingroup nr-test
 *
 * @brief Attaches UEs through an EPC helper with its own core connector and checks that the
 * connector is used and that the UEs become active.
 */
class NrCoreConnectorTestCase : public TestCase
{
  public:
    NrCoreConnectorTestCase();

  private:
    void DoRun() override;

    static constexpr uint32_t m_nUes{3}; ///< number of UEs in the scenario
};

NrCoreConnectorTestCase::NrCoreConnectorTestCase()
    : TestCase("Custom NrEpcHelper connects the gNB RRC to its own core connector")
{
}

void
NrCoreConnectorTestCase::DoRun()
{
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(m_nUes);

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
    positions->Add(Vector(0, 0, 10));
    for (uint32_t i = 0; i < m_nUes; i++)
    {
        positions->Add(Vector(10.0 + 5.0 * i, 0, 1.5));
    }
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positions);
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    auto epcHelper = CreateObject<NrTestCoreConnectorEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(true));
    nrHelper->SetUeAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());
    nrHelper->SetGnbAntennaTypeId(IsotropicAntennaModel::GetTypeId().GetName());

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigurePropagationFactory(FriisPropagationLossModel::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(2.8e9, 10e6, static_cast<uint8_t>(1));
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    nrHelper->AssignStreams({.gnbDevs = gnbDevices});
    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    nrHelper->AssignStreams({.ueDevs = ueDevices});

    InternetStackHelper internet;
    internet.Install(ueNodes);
    epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevices));

    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueDevices.Get(i), gnbDevices.Get(0));
    }

    Simulator::Stop(MilliSeconds(500));
    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(epcHelper->GetInitialUeMessages(),
                          m_nUes,
                          "The UEs did not reach the core through the custom connector");
    for (uint32_t i = 0; i < ueDevices.GetN(); i++)
    {
        auto nas = DynamicCast<NrUeNetDevice>(ueDevices.Get(i))->GetNas();
        NS_TEST_EXPECT_MSG_EQ(nas->GetState(),
                              NrEpcUeNas::ACTIVE,
                              "UE " << i << " did not become active");
    }

    Simulator::Destroy();
}

/**
 * @ingroup nr-test
 *
 * @brief Test suite `nr-core-connector`.
 */
class NrCoreConnectorTestSuite : public TestSuite
{
  public:
    NrCoreConnectorTestSuite()
        : TestSuite("nr-core-connector", Type::SYSTEM)
    {
        AddTestCase(new NrCoreConnectorTestCase(), Duration::QUICK);
    }
};

/// Static instance of the test suite.
static NrCoreConnectorTestSuite g_nrCoreConnectorTestSuite;
