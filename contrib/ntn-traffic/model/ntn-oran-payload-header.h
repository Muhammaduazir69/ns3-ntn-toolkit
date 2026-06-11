// SPDX-License-Identifier: GPL-2.0-only
//
// NtnOranPayloadHeader — the in-band user-plane header of the AI-Native
// ORAN-NTN application suite (AI-Native ORAN-NTN adoption WS1).
//
// Every NtnOranApplication packet carries this header as REAL serialized
// bytes, so QoS/slice identity (5QI, S-NSSAI, QFI) and the measurement
// primitives (sequence number, TX timestamp) ride INSIDE the GTP tunnel of
// the EPC. Delay, jitter and loss are then computed by NtnOranSink from the
// received bytes themselves — immune to the documented FlowMonitor byte-tag
// stripping over GTP re-encapsulation.
//
// Wire format (24 bytes, network order):
//   version      u8   (NTN_ORAN_PAYLOAD_VERSION)
//   payloadType  u8   (PayloadType)
//   seq          u32  per-flow sequence number
//   txTimestampNs u64 Simulator time at send (ns)
//   fiveQi       u8   3GPP TS 23.501 5QI
//   sst          u8   S-NSSAI slice/service type (TS 23.003)
//   sd           u24  S-NSSAI slice differentiator
//   qfi          u8   QoS flow identifier (6 bits used, TS 38.415)
//   srcId        u16  application-level source endpoint id
//   dstId        u16  application-level destination endpoint id

#ifndef NTN_ORAN_PAYLOAD_HEADER_H
#define NTN_ORAN_PAYLOAD_HEADER_H

#include "ns3/header.h"

#include <cstdint>

namespace ns3
{

class NtnOranPayloadHeader : public Header
{
  public:
    static constexpr uint8_t NTN_ORAN_PAYLOAD_VERSION = 1;
    static constexpr uint32_t SERIALIZED_SIZE = 24;

    /// Paper-aligned payload classes (Deng 2026 use cases + O-RAN planes).
    enum PayloadType : uint8_t
    {
        EMBB_VIDEO = 0,    ///< eMBB streaming / video frames (5QI 2/80)
        URLLC_CMD = 1,     ///< URLLC deterministic command (5QI 82/83)
        MMTC_READING = 2,  ///< mMTC / NB-IoT periodic sensor reading
        CNC_TELEMETRY = 3, ///< platform command-and-control telemetry (paper Sec. III-C)
        KPM_REPORT = 4,    ///< E2SM-KPM-style measurement report
        FH_SAMPLE = 5,     ///< fronthaul IQ/sample transport (split Option 7.2x)
        VOICE = 6,         ///< conversational voice (5QI 1)
        BACKGROUND = 7,    ///< Poisson best-effort background
    };

    NtnOranPayloadHeader() = default;

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    void Print(std::ostream& os) const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;

    void SetPayloadType(PayloadType t) { m_payloadType = t; }
    void SetSeq(uint32_t s) { m_seq = s; }
    void SetTxTimestampNs(uint64_t t) { m_txTimestampNs = t; }
    void SetFiveQi(uint8_t q) { m_fiveQi = q; }
    void SetSnssai(uint8_t sst, uint32_t sd) { m_sst = sst; m_sd = sd & 0xFFFFFF; }
    void SetQfi(uint8_t qfi) { m_qfi = qfi & 0x3F; }
    void SetSrcId(uint16_t id) { m_srcId = id; }
    void SetDstId(uint16_t id) { m_dstId = id; }

    uint8_t GetVersion() const { return m_version; }
    PayloadType GetPayloadType() const { return static_cast<PayloadType>(m_payloadType); }
    uint32_t GetSeq() const { return m_seq; }
    uint64_t GetTxTimestampNs() const { return m_txTimestampNs; }
    uint8_t GetFiveQi() const { return m_fiveQi; }
    uint8_t GetSst() const { return m_sst; }
    uint32_t GetSd() const { return m_sd; }
    uint8_t GetQfi() const { return m_qfi; }
    uint16_t GetSrcId() const { return m_srcId; }
    uint16_t GetDstId() const { return m_dstId; }

  private:
    uint8_t m_version{NTN_ORAN_PAYLOAD_VERSION};
    uint8_t m_payloadType{EMBB_VIDEO};
    uint32_t m_seq{0};
    uint64_t m_txTimestampNs{0};
    uint8_t m_fiveQi{9};
    uint8_t m_sst{1};
    uint32_t m_sd{0x000001};
    uint8_t m_qfi{9};
    uint16_t m_srcId{0};
    uint16_t m_dstId{0};
};

} // namespace ns3

#endif // NTN_ORAN_PAYLOAD_HEADER_H
