// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-command-and-control-app.h"

#include "ntn-oran-payload-header.h"

#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnCommandAndControlApp");
NS_OBJECT_ENSURE_REGISTERED(NtnCommandAndControlApp);

TypeId
NtnCommandAndControlApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnCommandAndControlApp")
            .SetParent<Application>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnCommandAndControlApp>()
            .AddAttribute("Remote",
                          "SMO endpoint address (InetSocketAddress).",
                          AddressValue(),
                          MakeAddressAccessor(&NtnCommandAndControlApp::m_remote),
                          MakeAddressChecker())
            .AddAttribute("Period",
                          "Telemetry reporting period.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&NtnCommandAndControlApp::m_period),
                          MakeTimeChecker(MilliSeconds(1)))
            .AddAttribute("SrcId",
                          "Platform id reported in the payload header.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NtnCommandAndControlApp::m_srcId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("DstId",
                          "SMO endpoint id.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NtnCommandAndControlApp::m_dstId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Sst",
                          "S-NSSAI slice/service type for the C&C flow.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NtnCommandAndControlApp::m_sst),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("Sd",
                          "S-NSSAI slice differentiator for the C&C flow.",
                          UintegerValue(0x000001),
                          MakeUintegerAccessor(&NtnCommandAndControlApp::m_sd),
                          MakeUintegerChecker<uint32_t>(0, 0xFFFFFF))
            .AddAttribute("BatteryCapacityWh",
                          "Platform battery capacity (Wh); 0 = unconstrained.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnCommandAndControlApp::m_batteryCapacityWh),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("PowerDrawW",
                          "Mean platform power draw (W) for the battery model.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnCommandAndControlApp::m_powerDrawW),
                          MakeDoubleChecker<double>(0.0))
            .AddTraceSource("Tx",
                            "A telemetry packet has been sent.",
                            MakeTraceSourceAccessor(&NtnCommandAndControlApp::m_txTrace),
                            "ns3::Packet::TracedCallback");
    return tid;
}

NtnCommandAndControlApp::NtnCommandAndControlApp() = default;
NtnCommandAndControlApp::~NtnCommandAndControlApp() = default;

void
NtnCommandAndControlApp::DoDispose()
{
    m_socket = nullptr;
    Application::DoDispose();
}

double
NtnCommandAndControlApp::GetBatteryFraction() const
{
    if (m_batteryCapacityWh <= 0.0 || m_powerDrawW <= 0.0)
    {
        return 1.0;
    }
    const double elapsedH = (Simulator::Now() - m_startTime).GetSeconds() / 3600.0;
    const double usedWh = m_powerDrawW * elapsedH;
    return std::max(0.0, 1.0 - usedWh / m_batteryCapacityWh);
}

void
NtnCommandAndControlApp::StartApplication()
{
    m_startTime = Simulator::Now();
    if (!m_socket)
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Bind();
        m_socket->Connect(m_remote);
    }
    m_sendEvent =
        Simulator::Schedule(m_period, &NtnCommandAndControlApp::SendTelemetry, this);
}

void
NtnCommandAndControlApp::StopApplication()
{
    m_sendEvent.Cancel();
    if (m_socket)
    {
        m_socket->Close();
    }
}

void
NtnCommandAndControlApp::SendTelemetry()
{
    Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
    NS_ABORT_MSG_IF(!mob, "NtnCommandAndControlApp: node has no MobilityModel — the "
                          "telemetry MUST come from a real mobility source");
    const Vector pos = mob->GetPosition();
    const Vector vel = mob->GetVelocity();

    NtnCncTelemetry t;
    t.posX = pos.x;
    t.posY = pos.y;
    t.posZ = pos.z;
    t.velX = vel.x;
    t.velY = vel.y;
    t.velZ = vel.z;
    // Attitude from the velocity frame (nadir-pointing platform assumption):
    // yaw = ground-track heading, pitch = flight-path angle, roll = 0 (no
    // lateral-acceleration model).
    const double horiz = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    t.yawDeg = std::atan2(vel.y, vel.x) * 180.0 / M_PI;
    t.pitchDeg = std::atan2(vel.z, std::max(horiz, 1e-9)) * 180.0 / M_PI;
    t.rollDeg = 0.0;
    t.batteryFraction = GetBatteryFraction();
    t.uptimeNs = (Simulator::Now() - m_startTime).GetNanoSeconds();

    uint8_t buf[NtnCncTelemetry::SERIALIZED_SIZE];
    t.WriteTo(buf);
    Ptr<Packet> p = Create<Packet>(buf, NtnCncTelemetry::SERIALIZED_SIZE);

    NtnOranPayloadHeader hdr;
    hdr.SetPayloadType(NtnOranPayloadHeader::CNC_TELEMETRY);
    hdr.SetSeq(m_seq++);
    hdr.SetTxTimestampNs(Simulator::Now().GetNanoSeconds());
    hdr.SetFiveQi(69); // TS 23.501: mission-critical delay-sensitive signalling
    hdr.SetSnssai(m_sst, m_sd);
    hdr.SetQfi(69 & 0x3F);
    hdr.SetSrcId(m_srcId);
    hdr.SetDstId(m_dstId);
    p->AddHeader(hdr);

    m_txTrace(p);
    m_socket->Send(p);
    m_sendEvent =
        Simulator::Schedule(m_period, &NtnCommandAndControlApp::SendTelemetry, this);
}

} // namespace ns3
