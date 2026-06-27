// SPDX-License-Identifier: GPL-2.0-only
//
// NtnNrStackHelper — 5G-LENA (nr) FR1 NTN radio spine. See the header for the
// rationale (closes boundary A5(i): FR1 15/30 kHz SCS at S-band, which the
// FR2-locked mmwave NtnRealStackHelper cannot produce).

#include "ntn-nr-stack-helper.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnNrStackHelper");

NtnNrStackHelper::NtnNrStackHelper() = default;

NtnNrStackHelper::~NtnNrStackHelper()
{
    delete m_flowmonHelper;
}

void
NtnNrStackHelper::Build(NodeContainer gnbNodes, NodeContainer ueNodes)
{
    NS_ABORT_MSG_IF(m_built, "NtnNrStackHelper::Build() called twice");
    NS_ABORT_MSG_IF(gnbNodes.GetN() == 0 || ueNodes.GetN() == 0,
                    "NtnNrStackHelper::Build() needs at least one gNB and one UE");
    m_gnb = gnbNodes;
    m_ue = ueNodes;

    // Big RLC buffers so a saturating DL flow is not buffer-limited.
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ---- Core network + helpers (cttc-nr-demo recipe) -------------------
    m_epc = CreateObject<NrPointToPointEpcHelper>();
    m_beamforming = CreateObject<IdealBeamformingHelper>();
    m_nr = CreateObject<NrHelper>();
    m_nr->SetBeamformingHelper(m_beamforming);
    m_nr->SetEpcHelper(m_epc);

    // ---- Single operational band -> 1 CC -> 1 FR1 BWP -------------------
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(m_freqHz,
                                                   m_bwHz,
                                                   numCcPerBand,
                                                   BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Quasi-static channel, no shadowing -> clean first link over the slant.
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    m_nr->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    m_nr->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    // Large-scale loss = free-space (Friis), NOT the 3GPP terrestrial pathloss.
    // The 3GPP UMa/UMi/RMa pathloss formulas assume a LOCAL ENU frame (z is the
    // antenna height, the 2D distance is horizontal); fed the ECEF satellite/UE
    // coordinates that the real NTN mobility models (SGP4, TR 38.811) produce,
    // their height/2D split is meaningless and the loss collapses (yielding a
    // physically impossible ~80 dB SINR). Friis depends only on the true 3D
    // slant range, so it is frame-independent and valid at LEO range — the same
    // choice the mmwave NtnRealStackHelper makes. We override only the
    // large-scale loss (bwp->m_propagation); InitializeOperationBand still
    // builds the 3GPP spatial channel (bwp->m_3gppChannel) so the UPA array gain
    // / DirectPath beamforming (which need that PhasedArray spectrum model) keep
    // working, and the angles it uses are computed from the real geometry.
    {
        BandwidthPartInfoPtr& bwp0 = band.GetBwpAt(0, 0);
        Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel>();
        friis->SetAttribute("Frequency", DoubleValue(m_freqHz));
        bwp0->m_propagation = friis;
    }

    m_nr->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Direct-path ideal beamforming (no SRS sounding needed for a first link).
    m_beamforming->SetAttribute("BeamformingMethod",
                                TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Zero the S1-U EPC latency; the NTN one-way delay is modelled on the
    // PGW<->remote-host backhaul instead (SetBackhaulDelay).
    m_epc->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // ---- Antennas: gNB 4x8 UPA, UE 1x2 UPA ------------------------------
    m_nr->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    m_nr->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    m_nr->SetUeAntennaAttribute("AntennaElement",
                                PointerValue(CreateObject<IsotropicAntennaModel>()));
    m_nr->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    m_nr->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    m_nr->SetGnbAntennaAttribute("AntennaElement",
                                 PointerValue(CreateObject<IsotropicAntennaModel>()));

    // ---- Install devices ------------------------------------------------
    m_gnbDevs = m_nr->InstallGnbDevice(m_gnb, allBwps);
    m_ueDevs = m_nr->InstallUeDevice(m_ue, allBwps);

    int64_t stream = 1;
    stream += m_nr->AssignStreams(m_gnbDevs, stream);
    stream += m_nr->AssignStreams(m_ueDevs, stream);

    // ---- Per-device PHY config: FR1 numerology + powers -----------------
    for (uint32_t i = 0; i < m_gnbDevs.GetN(); ++i)
    {
        m_nr->GetGnbPhy(m_gnbDevs.Get(i), 0)
            ->SetAttribute("Numerology", UintegerValue(m_numerology));
        m_nr->GetGnbPhy(m_gnbDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(m_satEirpDbm));
    }
    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        m_nr->GetUePhy(m_ueDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(m_ueTxDbm));
    }

    m_nr->UpdateDeviceConfigs(m_gnbDevs);
    m_nr->UpdateDeviceConfigs(m_ueDevs);

    // ---- Internet + remote host + backhaul ------------------------------
    Ptr<Node> pgw = m_epc->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    m_remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_backhaulDelay));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, m_remoteHost);

    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(m_remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(m_ue);
    Ipv4InterfaceContainer ueIpIface = m_epc->AssignUeIpv4Address(NetDeviceContainer(m_ueDevs));
    for (uint32_t i = 0; i < ueIpIface.GetN(); ++i)
    {
        m_ueAddrs.push_back(ueIpIface.GetAddress(i));
    }

    for (uint32_t j = 0; j < m_ue.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(m_ue.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(m_epc->GetUeDefaultGatewayAddress(), 1);
    }

    // ---- Attach + connect the measured-KPI PHY sink ---------------------
    m_nr->AttachToClosestGnb(m_ueDevs, m_gnbDevs);

    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        Ptr<NrSpectrumPhy> sp = m_nr->GetUePhy(m_ueDevs.Get(i), 0)->GetSpectrumPhy();
        sp->TraceConnectWithoutContext("RxPacketTraceUe",
                                       MakeCallback(&NtnNrStackHelper::DlRxTrace, this));
    }

    m_built = true;
    NS_LOG_INFO("NtnNrStackHelper built: " << m_gnbDevs.GetN() << " gNB(s), " << m_ueDevs.GetN()
                                           << " UE(s), fc=" << m_freqHz / 1e9 << " GHz, BW="
                                           << m_bwHz / 1e6 << " MHz, numerology=" << m_numerology
                                           << " (" << GetScsKhz() << " kHz SCS)");
}

void
NtnNrStackHelper::InstallTraffic(Time start, Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "NtnNrStackHelper::InstallTraffic() before Build()");
    m_trafficStart = start;
    m_trafficStop = stop;

    const uint16_t dlPort = 1234;

    // DL sink (UdpServer) on every UE.
    UdpServerHelper dlPacketSink(dlPort);
    m_serverApps.Add(dlPacketSink.Install(m_ue));

    // Saturating DL UDP client per UE on the remote host. 1400 B at a high
    // packet rate so the radio link (not the source) is the bottleneck.
    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);

    for (uint32_t i = 0; i < m_ueDevs.GetN(); ++i)
    {
        UdpClientHelper dlClient;
        dlClient.SetAttribute("RemoteAddress", AddressValue(m_ueAddrs[i]));
        dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1400));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / 40000.0)));
        m_clientApps.Add(dlClient.Install(m_remoteHost));

        Ptr<NrEpcTft> tft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        tft->Add(dlpf);
        m_nr->ActivateDedicatedEpsBearer(m_ueDevs.Get(i), bearer, tft);
    }

    m_serverApps.Start(start);
    m_clientApps.Start(start);
    m_serverApps.Stop(stop);
    m_clientApps.Stop(stop);

    // FlowMonitor over remote host + UEs -> measured throughput.
    m_flowmonHelper = new FlowMonitorHelper();
    NodeContainer endpointNodes;
    endpointNodes.Add(m_remoteHost);
    endpointNodes.Add(m_ue);
    m_monitor = m_flowmonHelper->Install(endpointNodes);
    m_monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    m_monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    m_monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));
}

void
NtnNrStackHelper::DlRxTrace(RxPacketTraceParams params)
{
    // Measured DL PHY: SINR (linear -> dB) and TBLER from the real error model.
    if (params.m_tbSize == 0)
    {
        return;
    }
    m_dlGlobal.sumSinrDb += 10.0 * std::log10(params.m_sinr);
    m_dlGlobal.sumTbler += params.m_tbler;
    m_dlGlobal.n += 1;
    m_phyRxTb += 1;
}

void
NtnNrStackHelper::Collect()
{
    // PHY-measured SINR / TBLER.
    if (m_dlGlobal.n > 0)
    {
        m_dlSinrDbMean = m_dlGlobal.sumSinrDb / m_dlGlobal.n;
        m_dlTblerMean = m_dlGlobal.sumTbler / m_dlGlobal.n;
    }

    // FlowMonitor-measured DL throughput (sum over DL flows to the UEs).
    if (m_monitor && m_flowmonHelper)
    {
        m_monitor->CheckForLostPackets();
        Ptr<Ipv4FlowClassifier> classifier =
            DynamicCast<Ipv4FlowClassifier>(m_flowmonHelper->GetClassifier());
        FlowMonitor::FlowStatsContainer stats = m_monitor->GetFlowStats();

        double durationS = (m_trafficStop - m_trafficStart).GetSeconds();
        if (durationS <= 0.0)
        {
            durationS = m_simTime.GetSeconds();
        }

        double rxBitsTotal = 0.0;
        for (const auto& kv : stats)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
            // DL = flows originating at the remote host (1.0.0.0/8).
            if ((t.sourceAddress.Get() & 0xFF000000) == 0x01000000)
            {
                rxBitsTotal += kv.second.rxBytes * 8.0;
            }
        }
        m_rxThroughputMbps = rxBitsTotal / durationS / 1e6;
    }

    NS_LOG_INFO("NtnNrStackHelper measured: SINR=" << m_dlSinrDbMean << " dB, TBLER="
                                                   << m_dlTblerMean << ", DL=" << m_rxThroughputMbps
                                                   << " Mbps, PHY RX TBs=" << m_phyRxTb);
}

} // namespace ns3
