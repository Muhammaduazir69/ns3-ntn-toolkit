// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-realistic-traffic-helper.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/fatal-error.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnRealisticTrafficHelper");

NtnRealisticTrafficHelper::NtnRealisticTrafficHelper() = default;

NodeContainer
NtnRealisticTrafficHelper::InstallUes(uint32_t numUes)
{
    NS_ABORT_MSG_IF(numUes == 0, "InstallUes(0) requested — refusing");
    NodeContainer ues;
    ues.Create(numUes);
    m_ues = ues;
    return ues;
}

void
NtnRealisticTrafficHelper::AttachExistingUes(NodeContainer ues)
{
    m_ues = ues;
}

void
NtnRealisticTrafficHelper::Wire()
{
    NS_ABORT_MSG_IF(m_wired, "NtnRealisticTrafficHelper::Wire() called twice");
    NS_ABORT_MSG_IF(m_ues.GetN() == 0, "NtnRealisticTrafficHelper: no UEs attached");

    // Gateway and remote host
    m_gateway.Create(1);
    m_remoteHost.Create(1);

    NodeContainer everyone(m_ues, m_gateway, m_remoteHost);
    InternetStackHelper inet;
    inet.Install(everyone);

    // ---- Gateway <-> remote host backbone (fast) ----------------------
    PointToPointHelper bb;
    bb.SetDeviceAttribute("DataRate", StringValue(m_gwRate));
    bb.SetChannelAttribute("Delay", StringValue("2ms"));
    m_gwToRemoteDevices = bb.Install(m_gateway.Get(0), m_remoteHost.Get(0));

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer bbIfaces = addr.Assign(m_gwToRemoteDevices);
    Ipv4Address remoteAddr = bbIfaces.GetAddress(1);

    // ---- Per-UE access links (variable rate / delay later) ------------
    PointToPointHelper access;
    access.SetDeviceAttribute("DataRate", StringValue(m_ueRate));
    access.SetChannelAttribute("Delay", StringValue("15ms")); // typical LEO RTT/2

    for (uint32_t i = 0; i < m_ues.GetN(); i++)
    {
        NetDeviceContainer devs = access.Install(m_ues.Get(i), m_gateway.Get(0));
        m_gwToUeDevices.Add(devs);
        std::ostringstream subnet;
        subnet << "10.1." << (i + 1) << ".0";
        addr.SetBase(subnet.str().c_str(), "255.255.255.0");
        addr.Assign(devs);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // ---- Traffic apps -------------------------------------------------
    // PacketSink on the remote host
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), m_port));
    m_serverApps = sinkHelper.Install(m_remoteHost.Get(0));
    m_serverApps.Start(Seconds(0.0));
    m_serverApps.Stop(m_simTime + Seconds(1.0));

    // Trace the sink so we count delivered packets
    for (uint32_t i = 0; i < m_serverApps.GetN(); i++)
    {
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(m_serverApps.Get(i));
        if (sink)
        {
            sink->TraceConnectWithoutContext(
                "Rx", MakeCallback(&NtnRealisticTrafficHelper::TraceRx, this));
        }
    }

    // Per-UE OnOff toward the remote host. Vary parameters per UE so we
    // don't get a thundering-herd that compresses all events to t=0.
    Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
    jitter->SetAttribute("Min", DoubleValue(0.0));
    jitter->SetAttribute("Max", DoubleValue(0.5));

    for (uint32_t i = 0; i < m_ues.GetN(); i++)
    {
        std::string rate;
        uint32_t pkt;
        std::string onTime;
        std::string offTime;
        switch (m_profile)
        {
        case TrafficProfile::NbIotPeriodic:
            rate = "8kbps"; pkt = 128; onTime = "ns3::ConstantRandomVariable[Constant=0.1]";
            offTime = "ns3::ConstantRandomVariable[Constant=0.9]"; break;
        case TrafficProfile::EmbbStreaming:
            rate = "2Mbps"; pkt = 1400; onTime = "ns3::ConstantRandomVariable[Constant=1.0]";
            offTime = "ns3::ConstantRandomVariable[Constant=0.0]"; break;
        case TrafficProfile::UrllcPings:
            rate = "200kbps"; pkt = 256; onTime = "ns3::ConstantRandomVariable[Constant=1.0]";
            offTime = "ns3::ConstantRandomVariable[Constant=0.0]"; break;
        case TrafficProfile::DigitalTwinTelemetry:
            rate = "40kbps"; pkt = 512; onTime = "ns3::ConstantRandomVariable[Constant=0.2]";
            offTime = "ns3::ConstantRandomVariable[Constant=0.8]"; break;
        case TrafficProfile::MixedBouquet:
        default:
        {
            uint32_t bucket = i % 3;
            if (bucket == 0) { rate = "8kbps"; pkt = 128;
                onTime = "ns3::ConstantRandomVariable[Constant=0.1]";
                offTime = "ns3::ConstantRandomVariable[Constant=0.9]"; }
            else if (bucket == 1) { rate = "2Mbps"; pkt = 1400;
                onTime = "ns3::ConstantRandomVariable[Constant=1.0]";
                offTime = "ns3::ConstantRandomVariable[Constant=0.0]"; }
            else { rate = "200kbps"; pkt = 256;
                onTime = "ns3::ConstantRandomVariable[Constant=1.0]";
                offTime = "ns3::ConstantRandomVariable[Constant=0.0]"; }
            break;
        }
        }

        OnOffHelper onoff("ns3::UdpSocketFactory",
                          InetSocketAddress(remoteAddr, m_port));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate(rate)));
        onoff.SetAttribute("PacketSize", UintegerValue(pkt));
        onoff.SetAttribute("OnTime", StringValue(onTime));
        onoff.SetAttribute("OffTime", StringValue(offTime));
        ApplicationContainer apps = onoff.Install(m_ues.Get(i));
        Time startJitter = Seconds(0.5 + jitter->GetValue());
        apps.Start(startJitter);
        apps.Stop(m_simTime - Seconds(0.5));
        m_clientApps.Add(apps);

        for (uint32_t a = 0; a < apps.GetN(); a++)
        {
            apps.Get(a)->TraceConnectWithoutContext(
                "Tx", MakeCallback(&NtnRealisticTrafficHelper::TraceTx, this));
        }
    }

    // Schedule periodic callbacks
    for (uint32_t k = 0; k < m_periodics.size(); k++)
    {
        Simulator::Schedule(m_periodics[k].period,
                            &NtnRealisticTrafficHelper::RunPeriodic, this, k);
    }
    Simulator::Schedule(m_heartbeat, &NtnRealisticTrafficHelper::RunHeartbeat, this);

    m_wallStartNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

    m_wired = true;
}

void
NtnRealisticTrafficHelper::RegisterPeriodicCallback(Time period,
                                                    std::function<void(Time)> cb)
{
    m_periodics.push_back({period, std::move(cb)});
}

void
NtnRealisticTrafficHelper::RunPeriodic(uint32_t cbIdx)
{
    if (Simulator::Now() >= m_simTime) return;
    auto& entry = m_periodics[cbIdx];
    entry.cb(Simulator::Now());
    ++m_analyticalEvents;
    Simulator::Schedule(entry.period, &NtnRealisticTrafficHelper::RunPeriodic,
                        this, cbIdx);
}

void
NtnRealisticTrafficHelper::RunHeartbeat()
{
    if (Simulator::Now() >= m_simTime) return;
    ++m_heartbeatTicks;
    Simulator::Schedule(m_heartbeat, &NtnRealisticTrafficHelper::RunHeartbeat,
                        this);
}

void
NtnRealisticTrafficHelper::TraceTx(Ptr<const Packet>)
{
    ++m_txPackets;
}

void
NtnRealisticTrafficHelper::TraceRx(Ptr<const Packet>, const Address&)
{
    ++m_rxPackets;
}

static void
EnsureDir(const std::string& dir)
{
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
}

void
NtnRealisticTrafficHelper::WriteHealthReport()
{
    int64_t wallEndNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    double wallSec = (wallEndNs - m_wallStartNs) / 1e9;
    double simSec = m_simTime.GetSeconds();

    double minWall = m_gates.minWallClockPerSec * simSec;
    uint64_t minTx = static_cast<uint64_t>(
        std::ceil(m_gates.minPacketsPerSimSec * simSec));
    double rxRatio = (m_txPackets > 0)
                         ? static_cast<double>(m_rxPackets) / m_txPackets
                         : 0.0;
    double txPerSec = m_txPackets / std::max(simSec, 1e-6);

    bool gateWall = (wallSec >= minWall);
    bool gateTx = (m_txPackets >= minTx);
    bool gateRx = (rxRatio >= m_gates.minRxOverTxRatio) || m_txPackets == 0;
    bool gateAnalytical = (m_analyticalEvents >= m_gates.minAnalyticalCallbacks);
    bool allOk = gateWall && gateTx && gateRx && gateAnalytical;

    EnsureDir(m_outputDir);
    std::ofstream out(m_outputDir + "/sim_health.csv");
    out << "metric,value,floor,pass\n"
        << "sim_time_s," << simSec << "," << simSec << ",1\n"
        << "wall_clock_s," << wallSec << "," << minWall << ","
            << (gateWall ? 1 : 0) << "\n"
        << "packets_tx," << m_txPackets << "," << minTx << ","
            << (gateTx ? 1 : 0) << "\n"
        << "packets_rx," << m_rxPackets << ",-,1\n"
        << "tx_per_sim_sec," << txPerSec << ","
            << m_gates.minPacketsPerSimSec << ","
            << (gateTx ? 1 : 0) << "\n"
        << "rx_over_tx," << rxRatio << "," << m_gates.minRxOverTxRatio
            << "," << (gateRx ? 1 : 0) << "\n"
        << "analytical_events," << m_analyticalEvents << ","
            << m_gates.minAnalyticalCallbacks << ","
            << (gateAnalytical ? 1 : 0) << "\n"
        << "heartbeat_ticks," << m_heartbeatTicks << ",-,1\n"
        << "ues," << m_ues.GetN() << ",-,1\n"
        << "run_tag," << m_runTag << ",-,1\n";
    out.close();

    std::cout << "[sim_health] wall=" << wallSec << "s (floor " << minWall
              << "s) | tx=" << m_txPackets << " (" << txPerSec
              << "/s, floor " << m_gates.minPacketsPerSimSec
              << "/s) | rx/tx=" << rxRatio
              << " | analytical=" << m_analyticalEvents
              << " -> " << (allOk ? "PASS" : "FAIL") << "\n";

    if (m_strictGates && !allOk)
    {
        NS_FATAL_ERROR("Simulation health gates failed; see "
                       << m_outputDir << "/sim_health.csv");
    }
}

} // namespace ns3
