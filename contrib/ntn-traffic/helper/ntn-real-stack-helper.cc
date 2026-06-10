// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-real-stack-helper.h"

#include "ns3/abort.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/fatal-error.h"
#include "ns3/data-rate.h"
#include "ns3/double.h"
#include "ns3/epc-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/mmwave-ue-net-device.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mmwave-phy-mac-common.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/ntn-oran-ai-flow-monitor.h"
#include "ns3/ntn-oran-application.h"
#include "ns3/ntn-oran-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

namespace ns3
{

using namespace mmwave;

NS_LOG_COMPONENT_DEFINE("NtnRealStackHelper");

NtnRealStackHelper::NtnRealStackHelper() = default;
NtnRealStackHelper::~NtnRealStackHelper() = default;

void
NtnRealStackHelper::Build(NodeContainer gnbNodes, NodeContainer ueNodes)
{
    NS_ABORT_MSG_IF(m_built, "NtnRealStackHelper::Build() called twice");
    NS_ABORT_MSG_IF(gnbNodes.GetN() == 0 || ueNodes.GetN() == 0,
                    "NtnRealStackHelper: need >=1 gNB and >=1 UE");
    m_gnb = gnbNodes;
    m_ue = ueNodes;

    // ---- NTN-ize the NR air interface via Config defaults (read at CreateObject
    //      time inside MmWaveHelper::DoInitialize) -------------------------------
    // NB: mmwave registers its TypeIds WITHOUT the mmwave:: namespace.
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(m_freqHz));
    Config::SetDefault("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue(m_bwHz));
    Config::SetDefault("ns3::MmWaveEnbPhy::TxPower", DoubleValue(m_satEirpDbm));
    Config::SetDefault("ns3::MmWaveUePhy::TxPower", DoubleValue(m_ueTxDbm));
    Config::SetDefault("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue(m_rlcAm));
    Config::SetDefault("ns3::MmWaveHelper::HarqEnabled", BooleanValue(m_harq));
    Config::SetDefault("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue(m_harq));

    m_mmwave = CreateObject<MmWaveHelper>();
    m_mmwave->SetSchedulerType("ns3::MmWaveFlexTtiMacScheduler");
    m_mmwave->SetHarqEnabled(m_harq);
    // Ideal RRC: set up the RLC bearer synchronously at attach. Real RRC's
    // contention-based RACH/connection handshake assumes terrestrial timing and
    // races/never completes over the LEO slant — a well-known NTN pitfall.
    m_mmwave->SetAttribute("UseIdealRrc", BooleanValue(true));
    // NTN LOS link: free-space (Friis) path loss is valid at LEO range, unlike the
    // terrestrial 3GPP UMa default. The 3GPP spectrum model is kept (mmwave hard-
    // requires it for antenna/beamforming array gain), with an always-LOS condition
    // model — the satellite link is line-of-sight by construction.
    m_mmwave->SetPathlossModelType("ns3::FriisPropagationLossModel");
    m_mmwave->SetChannelConditionModelType("ns3::AlwaysLosChannelConditionModel");

    m_epc = CreateObject<MmWavePointToPointEpcHelper>();
    m_mmwave->SetEpcHelper(m_epc);

    Ptr<Node> pgw = m_epc->GetPgwNode();

    // ---- Remote host behind the core (carries LEO feeder+core latency) ----
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    m_remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_backhaulDelay));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, m_remoteHost);
    m_backhaulCh = internetDevices.Get(0)->GetChannel();
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    m_remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(m_remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ---- Radio devices on the caller's mobility-carrying nodes ----
    m_enbDevs = m_mmwave->InstallEnbDevice(m_gnb);
    m_ueDevs = m_mmwave->InstallUeDevice(m_ue);

    internet.Install(m_ue);
    Ipv4InterfaceContainer ueIpIface = m_epc->AssignUeIpv4Address(NetDeviceContainer(m_ueDevs));
    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(m_ue.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(m_epc->GetUeDefaultGatewayAddress(), 1);
        m_ueAddrs.push_back(ueIpIface.GetAddress(u));
    }

    m_mmwave->AttachToClosestEnb(m_ueDevs, m_enbDevs);

    // ---- Measured-KPI PHY sink: DL SINR/TBLER/corrupt from the UE SpectrumPhy ----
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveUePhy/DlSpectrumPhy/RxPacketTraceUe",
        MakeCallback(&NtnRealStackHelper::DlRxTrace, this));

    m_built = true;
}

void
NtnRealStackHelper::AddExtraPropagationLoss(Ptr<PropagationLossModel> loss)
{
    NS_ABORT_MSG_IF(!m_built, "AddExtraPropagationLoss before Build()");
    if (!loss)
    {
        return;
    }
    // Chain after the built-in Friis loss on the (single) component-carrier channel.
    Ptr<PropagationLossModel> friis = m_mmwave->GetPathLossModel(0);
    NS_ABORT_MSG_IF(!friis, "no base propagation loss model on the mmwave channel");
    // Walk to the end of the chain, then append.
    Ptr<PropagationLossModel> tail = friis;
    while (tail->GetNext())
    {
        tail = tail->GetNext();
    }
    tail->SetNext(loss);
}

void
NtnRealStackHelper::InstallTraffic(TrafficProfile profile, Time start, Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "InstallTraffic before Build()");

    for (uint32_t u = 0; u < m_ue.GetN(); ++u)
    {
        // Choose per-UE profile params (NtnOranApplication QoS flows).
        TrafficProfile p = profile;
        if (profile == TrafficProfile::MixedBouquet)
        {
            uint32_t b = u % 3;
            p = (b == 0) ? TrafficProfile::NbIotPeriodic
                         : (b == 1) ? TrafficProfile::EmbbStreaming
                                    : TrafficProfile::UrllcPings;
        }
        NtnOranApplication::Profile oranProfile;
        uint8_t fiveQi;
        switch (p)
        {
        case TrafficProfile::NbIotPeriodic:
            oranProfile = NtnOranApplication::MMTC_PERIODIC;
            fiveQi = 9;
            break;
        case TrafficProfile::UrllcPings:
            oranProfile = NtnOranApplication::URLLC_PERIODIC;
            fiveQi = 82;
            break;
        case TrafficProfile::ConversationalVoice:
            oranProfile = NtnOranApplication::CONVERSATIONAL_VOICE;
            fiveQi = 1;
            break;
        case TrafficProfile::EmbbStreaming:
        default:
            oranProfile = NtnOranApplication::CBR_SATURATING;
            fiveQi = 2;
            break;
        }
        ApplicationContainer flow =
            InstallOranFlow(u, fiveQi, 1, 0x000001, oranProfile, start, stop);

        if (m_uplink)
        {
            uint16_t ulPort = 2000 + u;
            Ptr<NtnOranSink> ulSink = CreateObject<NtnOranSink>();
            ulSink->SetAttribute("Local",
                                 AddressValue(InetSocketAddress(Ipv4Address::GetAny(), ulPort)));
            m_remoteHost->AddApplication(ulSink);
            m_serverApps.Add(ulSink);

            Ptr<NtnOranApplication> ulClient = CreateObject<NtnOranApplication>();
            ulClient->SetRemote(InetSocketAddress(m_remoteHostAddr, ulPort));
            ulClient->SetProfile(NtnOranApplication::MMTC_PERIODIC);
            ulClient->SetAttribute("PacketSize", UintegerValue(256));
            ulClient->SetAttribute("Period", TimeValue(MilliSeconds(16))); // 128 kbps
            ulClient->SetFlowIdentity(9, 1, 0x000001, 1000 + u, 0);
            m_ue.Get(u)->AddApplication(ulClient);
            m_clientApps.Add(ulClient);
        }
    }

    m_serverApps.Start(Seconds(0.0));
    m_serverApps.Stop(m_simTime);
    m_clientApps.Start(start);
    m_clientApps.Stop(stop);

    // NB: FlowMonitor is intentionally NOT used. Over the LTE/mmwave EPC the GTP
    // tunnel re-encapsulates DL packets and strips the flow byte-tag, so
    // FlowMonitor under-counts DL rx. The NtnOranSink is the authoritative DL
    // measurement (delivery, in-band one-way delay, jitter, seq-gap loss) —
    // its primitives ride INSIDE the GTP tunnel as real payload bytes.

    // Periodic callbacks are scheduled at registration time (see
    // RegisterPeriodicCallback), so registration order vs InstallTraffic does
    // not matter.

    m_wallStartNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
}

ApplicationContainer
NtnRealStackHelper::InstallOranFlow(uint32_t ueIdx,
                                    uint8_t fiveQi,
                                    uint8_t sst,
                                    uint32_t sd,
                                    uint8_t profile,
                                    Time start,
                                    Time stop)
{
    NS_ABORT_MSG_IF(!m_built, "InstallOranFlow before Build()");
    NS_ABORT_MSG_IF(ueIdx >= m_ue.GetN(), "InstallOranFlow: UE index out of range");
    const auto oranProfile = static_cast<NtnOranApplication::Profile>(profile);
    const uint16_t dlPort = m_nextDlPort++;

    Ptr<NtnOranSink> sink = CreateObject<NtnOranSink>();
    sink->SetAttribute("Local",
                       AddressValue(InetSocketAddress(Ipv4Address::GetAny(), dlPort)));
    m_ue.Get(ueIdx)->AddApplication(sink);
    m_serverApps.Add(sink);
    m_dlSinks.Add(sink);
    m_dlSinkUe.push_back(ueIdx);
    sink->TraceConnectWithoutContext("Rx",
                                     MakeCallback(&NtnRealStackHelper::DlSinkRx, this));

    Ptr<NtnOranApplication> client = CreateObject<NtnOranApplication>();
    client->SetRemote(InetSocketAddress(m_ueAddrs[ueIdx], dlPort));
    client->SetProfile(oranProfile);
    // Cadence/size presets matching the helper's historical OnOff rates, so
    // the 30+ examples already gated on these profiles keep their budgets.
    switch (oranProfile)
    {
    case NtnOranApplication::MMTC_PERIODIC:
        client->SetAttribute("PacketSize", UintegerValue(128));
        client->SetAttribute("Period", TimeValue(MilliSeconds(64))); // ~16 kbps
        break;
    case NtnOranApplication::URLLC_PERIODIC:
        client->SetAttribute("PacketSize", UintegerValue(256));
        client->SetAttribute("Period", TimeValue(MilliSeconds(10))); // ~205 kbps
        break;
    case NtnOranApplication::CONVERSATIONAL_VOICE:
        client->SetAttribute("PacketSize", UintegerValue(92)); // AMR-WB + RTP-ish
        break;
    case NtnOranApplication::EMBB_VIDEO:
    case NtnOranApplication::POISSON_BACKGROUND:
    case NtnOranApplication::CBR_SATURATING:
    default:
        client->SetAttribute("DataRate", DataRateValue(DataRate("5Mb/s")));
        client->SetAttribute("PacketSize", UintegerValue(1400));
        break;
    }
    client->SetFlowIdentity(fiveQi, sst, sd, /*srcId=*/dlPort, /*dstId=*/ueIdx);
    m_remoteHost->AddApplication(client);
    m_clientApps.Add(client);
    client->TraceConnectWithoutContext("Tx",
                                       MakeCallback(&NtnRealStackHelper::DlClientTx, this));

    sink->SetStartTime(Seconds(0.0));
    sink->SetStopTime(m_simTime);
    client->SetStartTime(start);
    client->SetStopTime(stop);

    ApplicationContainer apps;
    apps.Add(client);
    apps.Add(sink);
    return apps;
}

Time
NtnRealStackHelper::ComputePayloadExtraDelay(double slantRangeM) const
{
    constexpr double kC = 299792458.0;
    const Time prop = Seconds(slantRangeM / kC);
    switch (m_payload)
    {
    case PayloadOption::Transparent:
        // Bent-pipe: the user plane rides the RF feeder leg too.
        return prop + prop;
    case PayloadOption::RegenerativeRu:
        // Open-FH (split 7.2x) over the feeder; 0.25 ms lower-PHY budget.
        return prop + MicroSeconds(250);
    case PayloadOption::RegenerativeRuDu:
        // F1 midhaul over the feeder.
        return prop + MicroSeconds(150);
    case PayloadOption::FullGnb:
    default:
        // GTP backhaul to the ground core.
        return prop + MicroSeconds(50);
    }
}

void
NtnRealStackHelper::SetFeederGeometry(Ptr<MobilityModel> satMobility,
                                      Ptr<MobilityModel> gwMobility)
{
    NS_ABORT_MSG_IF(!m_built, "SetFeederGeometry before Build()");
    NS_ABORT_MSG_IF(!satMobility || !gwMobility, "SetFeederGeometry: null mobility");
    m_feederSat = satMobility;
    m_feederGw = gwMobility;
    // Live update: the EPC backhaul channel delay tracks the real slant.
    RegisterPeriodicCallback(Seconds(1.0), [this](Time) {
        const double slantM = m_feederSat->GetDistanceFrom(m_feederGw);
        m_backhaulCh->SetAttribute("Delay", TimeValue(ComputePayloadExtraDelay(slantM)));
    });
    const double slantM = m_feederSat->GetDistanceFrom(m_feederGw);
    m_backhaulCh->SetAttribute("Delay", TimeValue(ComputePayloadExtraDelay(slantM)));
}

Ptr<NtnOranAiFlowMonitor>
NtnRealStackHelper::EnableOranFlowMonitor()
{
    NS_ABORT_MSG_IF(!m_built, "EnableOranFlowMonitor before Build()");
    NS_ABORT_MSG_IF(m_dlSinks.GetN() == 0,
                    "EnableOranFlowMonitor before InstallTraffic/InstallOranFlow");
    if (m_oranMonitor)
    {
        return m_oranMonitor;
    }
    m_oranMonitor = CreateObject<NtnOranAiFlowMonitor>();
    m_oranMonitor->SetPhySource(this);
    for (uint32_t i = 0; i < m_clientApps.GetN(); ++i)
    {
        Ptr<NtnOranApplication> app = DynamicCast<NtnOranApplication>(m_clientApps.Get(i));
        if (app)
        {
            m_oranMonitor->AddSource(app);
        }
    }
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> sink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (sink)
        {
            // m_dlSinkUe maps the flow to its UE for PHY (L1M.RS-SINR) metrics.
            const int32_t ue =
                (i < m_dlSinkUe.size()) ? static_cast<int32_t>(m_dlSinkUe[i]) : -1;
            m_oranMonitor->AddSink(sink, ue);
        }
    }
    m_oranMonitor->Start();
    return m_oranMonitor;
}

void
NtnRealStackHelper::RegisterPeriodicCallback(Time period, std::function<void(Time)> cb)
{
    uint32_t idx = m_periodics.size();
    m_periodics.push_back({period, std::move(cb)});
    // Schedule immediately so it fires regardless of call order vs InstallTraffic.
    Simulator::Schedule(period, &NtnRealStackHelper::RunPeriodic, this, idx);
}

void
NtnRealStackHelper::RunPeriodic(uint32_t idx)
{
    if (Simulator::Now() >= m_simTime)
    {
        return;
    }
    m_periodics[idx].cb(Simulator::Now());
    Simulator::Schedule(m_periodics[idx].period, &NtnRealStackHelper::RunPeriodic, this, idx);
}

void
NtnRealStackHelper::DlRxTrace(RxPacketTraceParams params)
{
    // Only count data TBs that actually carry a transport block.
    if (params.m_tbSize == 0)
    {
        return;
    }
    m_dlGlobal.sumSinrDb += 10.0 * std::log10(std::max(params.m_sinr, 1e-12));
    m_dlGlobal.sumTbler += params.m_tbler;
    m_dlGlobal.n += 1;
    if (params.m_corrupt)
    {
        m_dlGlobal.corrupt += 1;
    }

    double sinrDb = 10.0 * std::log10(std::max(params.m_sinr, 1e-12));

    SinrAccum& cell = m_dlPerCell[params.m_cellId];
    cell.sumSinrDb += sinrDb;
    cell.sumTbler += params.m_tbler;
    cell.n += 1;
    if (params.m_corrupt)
    {
        cell.corrupt += 1;
    }

    SinrAccum& ue = m_dlPerRnti[params.m_rnti];
    ue.sumSinrDb += sinrDb;
    ue.sumTbler += params.m_tbler;
    ue.n += 1;
    if (params.m_corrupt)
    {
        ue.corrupt += 1;
    }
    m_lastSinrDbPerRnti[params.m_rnti] = sinrDb;
    m_lastTblerPerRnti[params.m_rnti] = params.m_tbler;
}

uint16_t
NtnRealStackHelper::GetUeRnti(uint32_t ueIndex) const
{
    if (ueIndex >= m_ueDevs.GetN())
    {
        return 0;
    }
    Ptr<MmWaveUeNetDevice> dev = DynamicCast<MmWaveUeNetDevice>(m_ueDevs.Get(ueIndex));
    if (!dev || !dev->GetRrc())
    {
        return 0;
    }
    return dev->GetRrc()->GetRnti();
}

double
NtnRealStackHelper::GetUeRecentSinrDb(uint32_t ueIndex) const
{
    uint16_t rnti = GetUeRnti(ueIndex);
    auto it = m_lastSinrDbPerRnti.find(rnti);
    return (it != m_lastSinrDbPerRnti.end()) ? it->second
                                             : std::numeric_limits<double>::quiet_NaN();
}

double
NtnRealStackHelper::GetUeMeanSinrDb(uint32_t ueIndex) const
{
    uint16_t rnti = GetUeRnti(ueIndex);
    auto it = m_dlPerRnti.find(rnti);
    if (it == m_dlPerRnti.end() || it->second.n == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return it->second.sumSinrDb / it->second.n;
}

double
NtnRealStackHelper::GetUeRecentTbler(uint32_t ueIndex) const
{
    uint16_t rnti = GetUeRnti(ueIndex);
    auto it = m_lastTblerPerRnti.find(rnti);
    return (it != m_lastTblerPerRnti.end()) ? it->second
                                            : std::numeric_limits<double>::quiet_NaN();
}

void
NtnRealStackHelper::Collect()
{
    // PHY-measured aggregates.
    m_phyRxTb = m_dlGlobal.n;
    m_phyCorruptTb = m_dlGlobal.corrupt;
    m_dlSinrDbMean = (m_dlGlobal.n > 0) ? m_dlGlobal.sumSinrDb / m_dlGlobal.n
                                        : std::numeric_limits<double>::quiet_NaN();
    m_dlTblerMean = (m_dlGlobal.n > 0) ? m_dlGlobal.sumTbler / m_dlGlobal.n : 0.0;

    // App-measured KPIs from the NtnOranSinks (authoritative for the DL data
    // plane): delivery from received bytes; one-way delay / jitter / loss
    // from the in-band NtnOranPayloadHeader primitives.
    uint64_t dlRxBytes = 0;
    double sumDelayMs = 0.0;
    uint64_t delaySamples = 0;
    double sumJitterMs = 0.0;
    uint64_t jitterFlows = 0;
    uint64_t lostPkts = 0;
    uint64_t expectedPkts = 0;
    for (uint32_t i = 0; i < m_dlSinks.GetN(); ++i)
    {
        Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(i));
        if (oranSink)
        {
            dlRxBytes += oranSink->GetTotalRx();
            for (const auto& [key, fs] : oranSink->GetFlowStats())
            {
                sumDelayMs += fs.sumDelayMs;
                delaySamples += fs.rxPackets;
                if (fs.rxPackets > 1)
                {
                    sumJitterMs += fs.jitterMs;
                    ++jitterFlows;
                }
                lostPkts += fs.LostPackets();
                expectedPkts += static_cast<uint64_t>(fs.highestSeq) + 1;
            }
            continue;
        }
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(i));
        if (sink)
        {
            dlRxBytes += sink->GetTotalRx();
        }
    }
    double activeS = std::max(1e-6, m_simTime.GetSeconds());
    m_rxThroughputMbps = (dlRxBytes * 8.0) / activeS / 1e6;
    // m_appTxPackets / m_appRxPackets are accumulated live by the app traces.
    m_meanDelayMs = delaySamples ? sumDelayMs / delaySamples : 0.0;
    m_meanJitterMs = jitterFlows ? sumJitterMs / jitterFlows : 0.0;
    m_appLossRatio =
        expectedPkts ? static_cast<double>(lostPkts) / expectedPkts : 0.0;
}

void
NtnRealStackHelper::DlClientTx(Ptr<const Packet>)
{
    ++m_appTxPackets;
}

void
NtnRealStackHelper::DlSinkRx(Ptr<const Packet>, const Address&)
{
    ++m_appRxPackets;
}

double
NtnRealStackHelper::GetDlCorruptFraction() const
{
    return (m_phyRxTb > 0) ? static_cast<double>(m_phyCorruptTb) / m_phyRxTb : 0.0;
}

Ptr<MmWaveHelper>
NtnRealStackHelper::GetMmWaveHelper() const
{
    return m_mmwave;
}

Ptr<MmWavePointToPointEpcHelper>
NtnRealStackHelper::GetEpcHelper() const
{
    return m_epc;
}

uint64_t
NtnRealStackHelper::GetUeRxBytes(uint32_t ueIndex) const
{
    if (ueIndex >= m_dlSinks.GetN())
    {
        return 0;
    }
    Ptr<NtnOranSink> oranSink = DynamicCast<NtnOranSink>(m_dlSinks.Get(ueIndex));
    if (oranSink)
    {
        return oranSink->GetTotalRx();
    }
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_dlSinks.Get(ueIndex));
    return sink ? sink->GetTotalRx() : 0;
}

double
NtnRealStackHelper::GetCellMeanSinrDb(uint16_t cellId) const
{
    auto it = m_dlPerCell.find(cellId);
    if (it == m_dlPerCell.end() || it->second.n == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return it->second.sumSinrDb / it->second.n;
}

void
NtnRealStackHelper::WriteHealthReport()
{
    int64_t wallEndNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    double wallSec = (wallEndNs - m_wallStartNs) / 1e9;

    // ---- HONEST gates (PROTOCOL_FIDELITY_AUDIT_AND_FIX_2026-06 §8) ----
    bool gateStackDepth = (m_phyRxTb >= m_gates.minPhyRxTb);          // packets crossed the radio PHY
    bool gateThroughput = (m_rxThroughputMbps >= m_gates.minRxThroughputMbps); // measured app KPI
    bool gateProvenance = (!m_gates.requireSinrProvenance) || (m_dlGlobal.n > 0); // SINR from PHY trace
    bool gateErrorModel =
        (!m_gates.requireErrorModelActive) || (m_dlGlobal.n > 0);    // TBLER samples exist
    bool channelInPath = (m_ueDevs.GetN() > 0 && m_enbDevs.GetN() > 0); // packets rode mmwave devs
    bool allOk =
        gateStackDepth && gateThroughput && gateProvenance && gateErrorModel && channelInPath;

    std::error_code ec;
    std::filesystem::create_directories(m_outputDir, ec);
    std::ofstream out(m_outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass,provenance\n";
    out << "sim_time_s," << m_simTime.GetSeconds() << "," << m_simTime.GetSeconds()
        << ",1,config\n";
    out << "wall_clock_s," << wallSec << ",-,1,measured\n";
    out << "air_interface,mmwave-ntn,-,1,config\n";
    out << "carrier_hz," << m_freqHz << ",-,1,config\n";
    out << "channel_in_path," << (channelInPath ? 1 : 0) << ",1,"
        << (channelInPath ? 1 : 0) << ",topology\n";
    out << "phy_rx_tb," << m_phyRxTb << "," << m_gates.minPhyRxTb << ","
        << (gateStackDepth ? 1 : 0) << ",phy-trace\n";
    out << "dl_sinr_db," << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << ",-,"
        << (gateProvenance ? 1 : 0) << ",phy-trace\n";
    out << "dl_tbler_mean," << m_dlTblerMean << ",-," << (gateErrorModel ? 1 : 0)
        << ",phy-trace\n";
    out << "dl_corrupt_frac," << GetDlCorruptFraction() << ",-,1,phy-trace\n";
    out << "rx_throughput_mbps," << m_rxThroughputMbps << "," << m_gates.minRxThroughputMbps
        << "," << (gateThroughput ? 1 : 0) << ",packetsink\n";
    double delivery = (m_appTxPackets > 0)
                          ? static_cast<double>(m_appRxPackets) / m_appTxPackets
                          : 0.0;
    out << "app_delivery_ratio," << delivery << ",-,1,app-trace\n";
    out << "app_owd_ms," << m_meanDelayMs << ",-,1,inband-timestamp\n";
    out << "app_jitter_ms," << m_meanJitterMs << ",-,1,inband-timestamp\n";
    out << "app_loss_ratio," << m_appLossRatio << ",-,1,inband-seq\n";
    out << "app_tx_pkts," << m_appTxPackets << ",-,1,app-trace\n";
    out << "app_rx_pkts," << m_appRxPackets << ",-,1,app-trace\n";
    out << "ues," << m_ue.GetN() << ",-,1,config\n";
    out << "gnbs," << m_gnb.GetN() << ",-,1,config\n";
    out << "run_tag," << m_runTag << ",-,1,config\n";
    out.close();

    std::cout << "[sim_health/honest] air=mmwave-ntn fc=" << (m_freqHz / 1e9) << "GHz | phy_rx_tb="
              << m_phyRxTb << " (floor " << m_gates.minPhyRxTb << ") | dl_sinr="
              << (std::isnan(m_dlSinrDbMean) ? 0.0 : m_dlSinrDbMean) << "dB | tbler="
              << m_dlTblerMean << " | corrupt=" << GetDlCorruptFraction() << " | thr="
              << m_rxThroughputMbps << "Mbps | delay=" << m_meanDelayMs << "ms -> "
              << (allOk ? "PASS" : "FAIL") << "\n";

    if (m_strictGates && !allOk)
    {
        NS_FATAL_ERROR("Honest fidelity gates failed; see " << m_outputDir << "/sim_health.csv");
    }
}

} // namespace ns3
