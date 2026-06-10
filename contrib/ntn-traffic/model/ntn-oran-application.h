// SPDX-License-Identifier: GPL-2.0-only
//
// NtnOranApplication — QoS-flow-aware traffic source of the AI-Native
// ORAN-NTN suite (AI_NATIVE_ORAN_NTN_ADOPTION_PLAN WS1). Replaces bare
// OnOffHelper in every toolkit example: each packet carries an
// NtnOranPayloadHeader (5QI / S-NSSAI / QFI / seq / TX timestamp as real
// bytes), so the receiving NtnOranSink measures one-way delay, RFC 3550
// jitter and seq-gap loss from the payload itself, end-to-end through the
// GTP tunnel of the real EPC.
//
// Attribute-driven 3GPP traffic profiles (TS 23.501 Table 5.7.4-1 classes):
//   ConversationalVoice  5QI 1   deterministic 20 ms vocoder cadence
//   EmbbVideo            5QI 2   frame bursts at FrameRate, bytes from DataRate
//   UrllcPeriodic        5QI 82  deterministic small-period command/measurement
//   MmtcPeriodic         5QI 9*  NB-IoT-style periodic sensor report
//   PoissonBackground    5QI 9   exponential inter-arrival best effort
//   CbrSaturating        5QI 2   back-to-back CBR at DataRate (eMBB full-buffer)
// The profile presets 5QI/QFI/payload type; explicit attributes override.

#ifndef NTN_ORAN_APPLICATION_H
#define NTN_ORAN_APPLICATION_H

#include "ntn-oran-payload-header.h"

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"

#include <cstdint>

namespace ns3
{

class Socket;
class Packet;

class NtnOranApplication : public Application
{
  public:
    enum Profile : uint8_t
    {
        CONVERSATIONAL_VOICE = 0,
        EMBB_VIDEO = 1,
        URLLC_PERIODIC = 2,
        MMTC_PERIODIC = 3,
        POISSON_BACKGROUND = 4,
        CBR_SATURATING = 5,
    };

    static TypeId GetTypeId();
    NtnOranApplication();
    ~NtnOranApplication() override;

    void SetRemote(const Address& addr) { m_remote = addr; }
    void SetProfile(Profile p) { m_profile = p; }
    void SetFlowIdentity(uint8_t fiveQi, uint8_t sst, uint32_t sd, uint16_t srcId, uint16_t dstId);

    uint32_t GetTxPackets() const { return m_seq; }
    uint64_t GetTxBytes() const { return m_txBytes; }

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /// Apply the profile's 5QI / payload-type / cadence presets where the
    /// user left attributes at their sentinel defaults.
    void ResolveProfile();
    void ScheduleNext();
    void SendOne(uint32_t bytes);
    void SendFrameBurst();

    Address m_remote;
    Profile m_profile{CBR_SATURATING};
    uint8_t m_fiveQi{0};   // 0 = auto from profile
    uint8_t m_sst{1};
    uint32_t m_sd{0x000001};
    uint8_t m_qfi{0};      // 0 = auto (= 5QI)
    uint16_t m_srcId{0};
    uint16_t m_dstId{0};
    NtnOranPayloadHeader::PayloadType m_payloadType{NtnOranPayloadHeader::EMBB_VIDEO};

    DataRate m_dataRate{DataRate("5Mb/s")};
    uint32_t m_packetSize{1400};
    Time m_period{Seconds(0)}; // 0 = auto from profile
    double m_frameRate{30.0};

    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    uint32_t m_seq{0};
    uint64_t m_txBytes{0};
    Time m_resolvedPeriod{Seconds(0)};
    Ptr<ExponentialRandomVariable> m_expVar;

    TracedCallback<Ptr<const Packet>> m_txTrace;
};

} // namespace ns3

#endif // NTN_ORAN_APPLICATION_H
