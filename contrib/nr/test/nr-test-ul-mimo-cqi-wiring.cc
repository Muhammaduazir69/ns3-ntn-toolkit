// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>

/**
 * @file nr-test-ul-mimo-cqi-wiring.cc
 * @ingroup test
 *
 * @brief Check that NrHelper generates the PUSCH CQI from the MIMO SINR. Multi-port
 * reception requires Eigen, so this suite is only built with it.
 */
namespace ns3
{

/**
 * @ingroup test
 * @brief Check that NrHelper feeds the PUSCH CQI with the MIMO SINR.
 *
 * Two neighboring cells serve one cell-edge UE each, with saturated UL traffic, so that
 * each gNB receives its UE together with the UE of the other cell. The 2x2 array of the
 * gNBs is split into 1 or 4 receive ports. With 4 ports, the post-MMSE MIMO SINR rejects
 * the interfering UE, so the SINR reported for the PUSCH CQI of the first cell must be
 * much higher than with 1 port. The SISO SINR does not see this gain, so a PUSCH CQI
 * generated from it would barely change with the number of ports.
 */
class NrUlMimoCqiWiringTestCase : public TestCase
{
  public:
    NrUlMimoCqiWiringTestCase()
        : TestCase("PUSCH CQI follows the receive ports of the gNB")
    {
    }

  private:
    /**
     * Run a short UL simulation and return the mean SINR reported for the PUSCH CQI.
     * @param ports number of receive ports of the gNB (1 or 4)
     * @return the mean linear SINR over the RBs with signal
     */
    double RunAndGetMeanUlSinr(uint32_t ports)
    {
        // Both runs start from the same defaults and random streams
        Config::Reset();
        RngSeedManager::SetSeed(1);
        RngSeedManager::SetRun(1);

        // Accumulators for the SINR values reported for the PUSCH CQI
        double sinrSum = 0;
        uint64_t sinrCount = 0;

        // Helpers for the RAN, the core network and the channel

        auto epcHelper = CreateObject<NrPointToPointEpcHelper>();
        auto beamformingHelper = CreateObject<IdealBeamformingHelper>();
        auto nrHelper = CreateObject<NrHelper>();
        auto channelHelper = CreateObject<NrChannelHelper>();
        // The 3GPP channel provides the MIMO channel matrices.
        // Without it, NrHelper does not install the MIMO chunk processor.
        channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
        channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
        nrHelper->SetBeamformingHelper(beamformingHelper);
        nrHelper->SetEpcHelper(epcHelper);
        // Deterministic beams pointing at the UE, independent of the channel realization
        beamformingHelper->SetAttribute("BeamformingMethod",
                                        TypeIdValue(DirectPathBeamforming::GetTypeId()));

        // A single 10 MHz band at 3.5 GHz, shared by both cells
        CcBwpCreator ccBwpCreator;
        CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 10e6, 1);
        auto band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
        channelHelper->AssignChannelsToBands({band});
        auto allBwps = CcBwpCreator::GetAllBwps({band});

        // Two gNBs 100 m apart, each with a UE near the cell edge.
        // Each UE is roughly as close to the other gNB as to its own.
        NodeContainer gnbNodes;
        NodeContainer ueNodes;
        gnbNodes.Create(2);
        ueNodes.Create(2);
        MobilityHelper mobility;
        auto positions = CreateObject<ListPositionAllocator>();
        positions->Add(Vector(0, 0, 25));
        positions->Add(Vector(100, 0, 25));
        positions->Add(Vector(45, 10, 1.5));
        positions->Add(Vector(55, -10, 1.5));
        mobility.SetPositionAllocator(positions);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(gnbNodes);
        mobility.Install(ueNodes);

        // 2x2 isotropic gNB array, as 1 port (analog combining) or 4 ports (one per element)
        nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(2));
        nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));
        nrHelper->SetGnbAntennaAttribute("NumHorizontalPorts", UintegerValue(ports == 1 ? 1 : 2));
        nrHelper->SetGnbAntennaAttribute("NumVerticalPorts", UintegerValue(ports == 1 ? 1 : 2));
        nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                         PointerValue(CreateObject<IsotropicAntennaModel>()));
        // Single-element UEs, so the UL is rank 1
        nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
        nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
        nrHelper->SetUeAntennaAttribute("AntennaElement",
                                        PointerValue(CreateObject<IsotropicAntennaModel>()));

        // Full UE power makes the link interference-limited rather than noise-limited
        nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23));
        auto gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
        auto ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

        // UlSinrTrace fires with the per-RB SINR sent to the MAC for each PUSCH CQI.
        // Only the first cell is observed; RBs without signal report 0 and are skipped.
        NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
            ->TraceConnectWithoutContext(
                "UlSinrTrace",
                Callback<void, uint64_t, SpectrumValue&, SpectrumValue&>(
                    [&](uint64_t, SpectrumValue& sinr, SpectrumValue&) {
                        for (auto it = sinr.ConstValuesBegin(); it != sinr.ConstValuesEnd(); ++it)
                        {
                            if (*it > 0)
                            {
                                sinrSum += *it;
                                sinrCount++;
                            }
                        }
                    }));

        // Core network: a remote host behind the PGW receives the UL traffic
        NodeContainer remoteHostContainer;
        remoteHostContainer.Create(1);
        auto remoteHost = remoteHostContainer.Get(0);
        InternetStackHelper internet;
        internet.Install(remoteHostContainer);
        internet.Install(ueNodes);
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        auto internetDevices = p2ph.Install(epcHelper->GetPgwNode(), remoteHost);
        Ipv4AddressHelper ipv4h;
        ipv4h.SetBase("1.0.0.0", "255.0.0.0");
        auto internetIpIfaces = ipv4h.Assign(internetDevices);
        Ipv4StaticRoutingHelper routingHelper;
        routingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>())
            ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
        // Each UE attaches to its own cell, so the other UE is interference
        epcHelper->AssignUeIpv4Address(ueDevs);
        nrHelper->AttachToGnb(ueDevs.Get(0), gnbDevs.Get(0));
        nrHelper->AttachToGnb(ueDevs.Get(1), gnbDevs.Get(1));

        // UDP from both UEs to the remote host
        uint16_t ulPort = 1234;
        UdpServerHelper server(ulPort);
        auto serverApps = server.Install(remoteHost);
        UdpClientHelper client(internetIpIfaces.GetAddress(1), ulPort);
        client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        // Saturate the UL of both cells, so that they interfere on every UL symbol
        client.SetAttribute("PacketSize", UintegerValue(1000));
        client.SetAttribute("Interval", TimeValue(MicroSeconds(100)));
        auto clientApps = client.Install(ueNodes);
        serverApps.Start(MilliSeconds(100));
        clientApps.Start(MilliSeconds(100));

        // Fixed streams, so that both runs see the same channel realization
        nrHelper->AssignStreams({.assignEpc = true,
                                 .remoteHostNodes = remoteHostContainer,
                                 .ueNodes = ueNodes,
                                 .gnbDevs = gnbDevs,
                                 .ueDevs = ueDevs});

        // 300 ms of traffic produce thousands of CQI reports
        Simulator::Stop(MilliSeconds(400));
        Simulator::Run();
        Simulator::Destroy();

        NS_TEST_EXPECT_MSG_GT(sinrCount, 0, "No PUSCH CQI was generated");

        // Linear mean over all reported RBs
        return sinrCount ? sinrSum / sinrCount : 0;
    }

    void DoRun() override
    {
        // Same scenario, only the number of gNB receive ports changes
        const double oneSinr = RunAndGetMeanUlSinr(1);
        const double fourSinr = RunAndGetMeanUlSinr(4);
        // The MIMO SINR gains more than 13 dB from the 4 ports, the SISO SINR less than 1.5 dB
        NS_TEST_EXPECT_MSG_GT(10 * std::log10(fourSinr / oneSinr),
                              6.0,
                              "The PUSCH CQI must include the interference rejection of the "
                              "4 receive ports");
    }
};

/**
 * @ingroup test
 * @brief Test suite for the PUSCH CQI wiring of NrHelper.
 */
class NrUlMimoCqiWiringTestSuite : public TestSuite
{
  public:
    NrUlMimoCqiWiringTestSuite()
        : TestSuite("nr-ul-mimo-cqi-wiring", Type::SYSTEM)
    {
        AddTestCase(new NrUlMimoCqiWiringTestCase(), Duration::QUICK);
    }
};

static NrUlMimoCqiWiringTestSuite
    g_nrUlMimoCqiWiringTestSuite; //!< Static variable for test initialization

} // namespace ns3
