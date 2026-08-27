// SPDX-License-Identifier: GPL-2.0-only
#include "ntn-oran-application.h"

#include "ns3/abort.h"
#include "ns3/address-utils.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnOranApplication");
NS_OBJECT_ENSURE_REGISTERED(NtnOranApplication);

TypeId
NtnOranApplication::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnOranApplication")
            .SetParent<Application>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnOranApplication>()
            .AddAttribute("Remote",
                          "Destination address (InetSocketAddress).",
                          AddressValue(),
                          MakeAddressAccessor(&NtnOranApplication::m_remote),
                          MakeAddressChecker())
            .AddAttribute("Profile",
                          "3GPP traffic profile.",
                          EnumValue(NtnOranApplication::CBR_SATURATING),
                          MakeEnumAccessor<NtnOranApplication::Profile>(
                              &NtnOranApplication::m_profile),
                          MakeEnumChecker(NtnOranApplication::CONVERSATIONAL_VOICE,
                                          "ConversationalVoice",
                                          NtnOranApplication::EMBB_VIDEO,
                                          "EmbbVideo",
                                          NtnOranApplication::URLLC_PERIODIC,
                                          "UrllcPeriodic",
                                          NtnOranApplication::MMTC_PERIODIC,
                                          "MmtcPeriodic",
                                          NtnOranApplication::POISSON_BACKGROUND,
                                          "PoissonBackground",
                                          NtnOranApplication::CBR_SATURATING,
                                          "CbrSaturating"))
            .AddAttribute("FiveQi",
                          "3GPP 5QI for the flow (0 = preset from profile).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NtnOranApplication::m_fiveQi),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("Sst",
                          "S-NSSAI slice/service type.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&NtnOranApplication::m_sst),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("Sd",
                          "S-NSSAI slice differentiator (24 bits).",
                          UintegerValue(0x000001),
                          MakeUintegerAccessor(&NtnOranApplication::m_sd),
                          MakeUintegerChecker<uint32_t>(0, 0xFFFFFF))
            .AddAttribute("SrcId",
                          "Application-level source endpoint id.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NtnOranApplication::m_srcId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("DstId",
                          "Application-level destination endpoint id.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NtnOranApplication::m_dstId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("DataRate",
                          "Mean rate for EmbbVideo/PoissonBackground/CbrSaturating.",
                          DataRateValue(DataRate("5Mb/s")),
                          MakeDataRateAccessor(&NtnOranApplication::m_dataRate),
                          MakeDataRateChecker())
            .AddAttribute("PacketSize",
                          "Application packet size incl. NtnOranPayloadHeader (bytes).",
                          UintegerValue(1400),
                          MakeUintegerAccessor(&NtnOranApplication::m_packetSize),
                          MakeUintegerChecker<uint32_t>(
                              NtnOranPayloadHeader::SERIALIZED_SIZE + 1, 65000),
                          TypeId::SupportLevel::SUPPORTED)
            .AddAttribute("Period",
                          "Send period for periodic profiles (0 = profile preset).",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&NtnOranApplication::m_period),
                          MakeTimeChecker())
            .AddAttribute("FrameRate",
                          "Video frame rate for EmbbVideo (frames/s).",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&NtnOranApplication::m_frameRate),
                          MakeDoubleChecker<double>(1.0, 240.0))
            .AddTraceSource("Tx",
                            "A packet has been sent.",
                            MakeTraceSourceAccessor(&NtnOranApplication::m_txTrace),
                            "ns3::Packet::TracedCallback");
    return tid;
}

NtnOranApplication::NtnOranApplication() = default;
NtnOranApplication::~NtnOranApplication() = default;

void
NtnOranApplication::SetFlowIdentity(uint8_t fiveQi,
                                    uint8_t sst,
                                    uint32_t sd,
                                    uint16_t srcId,
                                    uint16_t dstId)
{
    m_fiveQi = fiveQi;
    m_sst = sst;
    m_sd = sd & 0xFFFFFF;
    m_srcId = srcId;
    m_dstId = dstId;
}

void
NtnOranApplication::DoDispose()
{
    m_socket = nullptr;
    Application::DoDispose();
}

void
NtnOranApplication::ResolveProfile()
{
    // (preset 5QI, payload type, cadence) per TS 23.501 QoS class.
    uint8_t presetQi = 9;
    Time presetPeriod = Seconds(1.0);
    switch (m_profile)
    {
    case CONVERSATIONAL_VOICE:
        presetQi = 1;
        presetPeriod = MilliSeconds(20); // vocoder frame cadence
        m_payloadType = NtnOranPayloadHeader::VOICE;
        break;
    case EMBB_VIDEO:
        presetQi = 2;
        NS_ABORT_MSG_IF(m_frameRate <= 0,
                        "NtnOranApplication: FrameRate must be > 0 for EmbbVideo");
        presetPeriod = Seconds(1.0 / m_frameRate);
        m_payloadType = NtnOranPayloadHeader::EMBB_VIDEO;
        break;
    case URLLC_PERIODIC:
        presetQi = 82;
        presetPeriod = MilliSeconds(10);
        m_payloadType = NtnOranPayloadHeader::URLLC_CMD;
        break;
    case MMTC_PERIODIC:
        presetQi = 9;
        presetPeriod = Seconds(1.0);
        m_payloadType = NtnOranPayloadHeader::MMTC_READING;
        break;
    case POISSON_BACKGROUND:
        presetQi = 9;
        m_payloadType = NtnOranPayloadHeader::BACKGROUND;
        presetPeriod = Seconds(static_cast<double>(m_packetSize) * 8.0 /
                               std::max<double>(m_dataRate.GetBitRate(), 1.0));
        break;
    case CBR_SATURATING:
        presetQi = 2;
        m_payloadType = NtnOranPayloadHeader::EMBB_VIDEO;
        presetPeriod = Seconds(static_cast<double>(m_packetSize) * 8.0 /
                               std::max<double>(m_dataRate.GetBitRate(), 1.0));
        break;
    }
    if (m_fiveQi == 0)
    {
        m_fiveQi = presetQi;
    }
    if (m_qfi == 0)
    {
        m_qfi = m_fiveQi & 0x3F;
    }
    m_resolvedPeriod = (m_period.IsZero()) ? presetPeriod : m_period;
    if (m_profile == POISSON_BACKGROUND)
    {
        m_expVar = CreateObject<ExponentialRandomVariable>();
        m_expVar->SetAttribute("Mean", DoubleValue(m_resolvedPeriod.GetSeconds()));
    }
}

void
NtnOranApplication::StartApplication()
{
    ResolveProfile();
    if (!m_socket)
    {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        if (InetSocketAddress::IsMatchingType(m_remote))
        {
            m_socket->Bind();
        }
        else
        {
            m_socket->Bind6();
        }
        m_socket->Connect(m_remote);
        m_socket->SetAllowBroadcast(true);
    }
    ScheduleNext();
}

void
NtnOranApplication::StopApplication()
{
    m_sendEvent.Cancel();
    if (m_socket)
    {
        m_socket->Close();
    }
}

void
NtnOranApplication::ScheduleNext()
{
    // Rate-driven profiles recompute the period each send, so a live
    // DataRate attribute change (e.g. an SMO quota actuation) takes effect
    // immediately on the running flow.
    if (m_profile == CBR_SATURATING || m_profile == POISSON_BACKGROUND ||
        m_profile == EMBB_VIDEO)
    {
        const double bitRate = std::max<double>(m_dataRate.GetBitRate(), 1.0);
        if (m_profile == EMBB_VIDEO)
        {
            NS_ABORT_MSG_IF(m_frameRate <= 0,
                            "NtnOranApplication: FrameRate must be > 0 for EmbbVideo");
            m_resolvedPeriod = (m_period.IsZero()) ? Seconds(1.0 / m_frameRate) : m_period;
        }
        else if (m_period.IsZero())
        {
            m_resolvedPeriod = Seconds(static_cast<double>(m_packetSize) * 8.0 / bitRate);
        }
        if (m_profile == POISSON_BACKGROUND)
        {
            m_expVar->SetAttribute("Mean", DoubleValue(m_resolvedPeriod.GetSeconds()));
        }
    }
    Time next = m_resolvedPeriod;
    if (m_profile == POISSON_BACKGROUND)
    {
        next = Seconds(m_expVar->GetValue());
    }
    if (m_profile == EMBB_VIDEO)
    {
        m_sendEvent = Simulator::Schedule(next, &NtnOranApplication::SendFrameBurst, this);
    }
    else
    {
        m_sendEvent =
            Simulator::Schedule(next, &NtnOranApplication::SendOne, this, m_packetSize);
    }
}

void
NtnOranApplication::SendOne(uint32_t bytes)
{
    // Runtime gate: stay scheduled but emit nothing. Returning before the
    // header is built also leaves m_seq untouched, so the receiver does not
    // record a sequence gap for packets the control plane chose not to send.
    if (!m_txEnabled)
    {
        ScheduleNext();
        return;
    }
    NtnOranPayloadHeader hdr;
    hdr.SetPayloadType(m_payloadType);
    hdr.SetSeq(m_seq++);
    hdr.SetTxTimestampNs(Simulator::Now().GetNanoSeconds());
    hdr.SetFiveQi(m_fiveQi);
    hdr.SetSnssai(m_sst, m_sd);
    hdr.SetQfi(m_qfi);
    hdr.SetSrcId(m_srcId);
    hdr.SetDstId(m_dstId);

    const uint32_t body = std::max<uint32_t>(bytes, hdr.GetSerializedSize() + 1) -
                          hdr.GetSerializedSize();
    Ptr<Packet> p;
    if (!m_payloadBuilder.IsNull() && body > 0)
    {
        // Let the application fill the body (e.g. a J2735 BSM). The in-band KPI
        // header is still added on top, so the sink's measured delay/jitter/loss
        // are unchanged — only the opaque padding becomes real content.
        Buffer buf;
        buf.AddAtStart(body);
        m_payloadBuilder(buf.Begin(), body);
        p = Create<Packet>(buf.PeekData(), body);
    }
    else
    {
        p = Create<Packet>(body);
    }
    p->AddHeader(hdr);
    m_txBytes += p->GetSize();
    m_txTrace(p);
    m_socket->Send(p);

    if (m_profile != EMBB_VIDEO) // frame bursts reschedule themselves
    {
        ScheduleNext();
    }
}

void
NtnOranApplication::SendFrameBurst()
{
    // One video frame = DataRate / FrameRate bits, fragmented at PacketSize.
    NS_ABORT_MSG_IF(m_frameRate <= 0,
                    "NtnOranApplication: FrameRate must be > 0 for EmbbVideo");
    NS_ABORT_MSG_IF(m_dataRate.GetBitRate() == 0,
                    "NtnOranApplication: DataRate must be > 0 for EmbbVideo");
    uint64_t frameBytes =
        static_cast<uint64_t>(m_dataRate.GetBitRate() / m_frameRate / 8.0);
    frameBytes = std::max<uint64_t>(frameBytes, NtnOranPayloadHeader::SERIALIZED_SIZE + 1);
    while (frameBytes > 0)
    {
        const uint32_t chunk =
            static_cast<uint32_t>(std::min<uint64_t>(frameBytes, m_packetSize));
        SendOne(std::max<uint32_t>(chunk, NtnOranPayloadHeader::SERIALIZED_SIZE + 1));
        frameBytes -= std::min<uint64_t>(frameBytes, chunk);
    }
    ScheduleNext();
}

} // namespace ns3
