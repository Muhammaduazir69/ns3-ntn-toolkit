/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
#ifndef NTN_UE_SIB19_CONSUMER_H
#define NTN_UE_SIB19_CONSUMER_H

#include "ntn-sib19.h"
#include "ntn-timing-advance.h"
#include "ntn-ul-timing-precompensator.h"

#include <ns3/nstime.h>
#include <ns3/object.h>

#include <cstdint>
#include <vector>

namespace ns3
{
namespace ntnrrc
{

/**
 * \ingroup ntn-rrc
 * UE-side SIB19 consumer (Wave 2 M1, P0-7).
 *
 * Takes the SIB19 wire image a UE received, parses it (Sib19Codec), and hands the
 * decoded block to the UE's own NtnUlTimingPrecompensator, which forms the UE's
 * TA from its GNSS fix, the broadcast ephemeris and ta-Common (TS 38.213 4.2).
 * Each Sample() compares that estimate with the true UE-gNB round trip from the
 * network-side NtnTimingAdvance and accumulates the error statistics against
 * Te_NTN (TS 38.133 Table 7.1C.2-1) and the UL sync state (ul-SyncValidityDuration,
 * TS 38.331 5.2.2.6).
 *
 * Nothing is read from the broadcaster's in-memory struct: what the UE knows is
 * what it decoded.
 */
class NtnUeSib19Consumer : public Object
{
  public:
    static TypeId GetTypeId();
    NtnUeSib19Consumer();
    ~NtnUeSib19Consumer() override;

    /// UE mobility (sampled as GNSS fixes), network-side truth, GNSS error.
    void Setup(Ptr<MobilityModel> ue,
               Ptr<NtnTimingAdvance> truth,
               double gnssSigmaM = 0.0,
               Time gnssPeriod = Seconds(1));
    /// Te_NTN used for the violation count (default 716.15 ns, FR1 30/30 kHz).
    void SetTimingErrorLimit(Time te) { m_te = te; }

    /// A received SIB19 wire image. Returns true if it parsed.
    bool ReceiveWire(const uint8_t* data, std::size_t len);
    bool ReceiveWire(const std::vector<uint8_t>& bytes)
    {
        return ReceiveWire(bytes.data(), bytes.size());
    }

    struct SampleResult
    {
        bool valid{false};     //!< false until a SIB19 was decoded
        double estUs{0.0};     //!< UE TA estimate
        double truthUs{0.0};   //!< true UE-gNB round trip
        double errNs{0.0};     //!< estimate - truth
        double ageMs{0.0};     //!< SIB19 information age
        bool synced{false};    //!< within ul-SyncValidityDuration
    };

    /// Evaluate now and accumulate statistics.
    SampleResult Sample();

    Ptr<NtnUlTimingPrecompensator> GetPrecompensator() const { return m_pc; }
    uint32_t GetDecodedCount() const { return m_decoded; }
    uint32_t GetSampleCount() const { return m_samples; }
    uint32_t GetViolationCount() const { return m_viol; }
    uint32_t GetSyncedCount() const { return m_synced; }
    double GetMaxAbsErrorNs() const { return m_maxAbsErrNs; }
    /// The last decoded block (as the UE holds it).
    const Sib19Content& GetHeld() const { return m_held; }

  protected:
    void DoDispose() override;

  private:
    Ptr<NtnUlTimingPrecompensator> m_pc;
    Ptr<NtnTimingAdvance> m_truth;
    Sib19Content m_held;
    Time m_te{NanoSeconds(716.15)};
    uint32_t m_decoded{0};
    uint32_t m_samples{0};
    uint32_t m_viol{0};
    uint32_t m_synced{0};
    double m_maxAbsErrNs{0.0};
};

} // namespace ntnrrc
} // namespace ns3

#endif // NTN_UE_SIB19_CONSUMER_H
