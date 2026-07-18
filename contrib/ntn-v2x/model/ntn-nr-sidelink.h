/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_NR_SIDELINK_H
#define NTN_NR_SIDELINK_H

#include "ns3/callback.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \defgroup ntn-nr-sidelink NR PC5 sidelink (V1)
 * \ingroup ntn-v2x
 *
 * A faithful-but-tractable abstraction of NR V2X PC5 sidelink Mode 2
 * (autonomous, sensing-based resource selection), per 3GPP TS 38.321 §5.22 and
 * the SL-ResourcePool of TS 38.331, evaluated with the TS 38.885 V2X sidelink
 * methodology (PRR vs distance). This replaces the earlier Uu/P2P "V2X relay":
 * vehicles now exchange BSMs directly over PC5 with no gNB in the loop.
 *
 * Modelled: a time-slotted (slot x subchannel) resource grid; per-UE Mode-2
 * selection (candidate set in the selection window, exclusion of resources a
 * sensed neighbour has RESERVED above an SL-RSRP threshold, the >=20% remaining
 * rule with the +3 dB threshold step, random pick, periodic reservation with a
 * reselection counter); the half-duplex constraint (a UE cannot receive in a
 * slot it transmits in); and co-channel collisions. NOT modelled: PSCCH/PSSCH
 * bit-level decoding, HARQ combining, 2nd-stage SCI formats — this is a MAC/
 * resource-pool abstraction, not a PHY.
 */

/**
 * \ingroup ntn-nr-sidelink
 * \brief SL-ResourcePool abstraction (TS 38.331 SL-ResourcePool).
 */
struct NtnSlResourcePool
{
    uint32_t numSubchannels{5};   //!< sl-NumSubchannel
    uint32_t subchannelSizeRb{10};//!< sl-SubchannelSize (RBs); PSSCH occupies whole subchannels
    Time slotDuration{MilliSeconds(1)}; //!< one SL slot (numerology-dependent; mu=0 -> 1 ms)

    /// Number of single-subchannel resources available per slot.
    uint32_t ResourcesPerSlot() const { return numSubchannels; }
};

/**
 * \ingroup ntn-nr-sidelink
 * \brief A selected single-slot sidelink resource (a PSCCH+PSSCH grant).
 */
struct NtnSlGrant
{
    uint64_t slot{0};       //!< absolute SL slot index
    uint32_t startSubch{0}; //!< first subchannel
    uint32_t lenSubch{1};   //!< contiguous subchannels
    bool valid{false};
};

class NtnSlChannel;

/**
 * \ingroup ntn-nr-sidelink
 * \brief One vehicle's NR sidelink MAC with Mode-2 autonomous selection.
 */
class NtnSlUeMac : public Object
{
  public:
    /// rx(fromUeId, bytes) — a decoded BSM from a peer.
    typedef Callback<void, uint32_t, uint32_t> SlRxCallback;

    static TypeId GetTypeId();
    NtnSlUeMac();
    ~NtnSlUeMac() override;

    void SetUeId(uint32_t id) { m_ueId = id; }
    uint32_t GetUeId() const { return m_ueId; }
    void SetMobility(Ptr<MobilityModel> m) { m_mobility = m; }
    Ptr<MobilityModel> GetMobility() const { return m_mobility; }
    void SetRxCallback(SlRxCallback cb) { m_rx = cb; }

    /// TS 38.321 §5.22.1.1 config.
    void SetSelectionWindow(uint32_t t1Slots, uint32_t t2Slots);
    void SetReservationPeriod(uint32_t prsvpSlots) { m_prsvpSlots = prsvpSlots; }
    void SetRsrpThresholdDbm(double dbm) { m_rsrpThreshDbm = dbm; }
    void SetPacketBytes(uint32_t bytes) { m_pktBytes = bytes; }

    int64_t AssignStreams(int64_t stream);

    // ---- counters (KPIs) ----
    uint32_t GetTxCount() const { return m_txCount; }
    uint32_t GetReselections() const { return m_reselections; }

  private:
    friend class NtnSlChannel;

    /// Run Mode-2 sensing-based selection for the next transmission. Returns the grant.
    NtnSlGrant SelectResource(uint64_t nowSlot);
    /// Called by the channel when this UE decodes a peer's SCI reservation.
    void SenseReservation(uint64_t slot, uint32_t subch, double rsrpDbm);
    /// Called by the channel to deliver a decoded data packet.
    void DeliverRx(uint32_t fromUeId, uint32_t bytes);
    /// Drop sensed reservations older than the sensing window.
    void PruneSensing(uint64_t nowSlot);

    uint32_t m_ueId{0};
    Ptr<MobilityModel> m_mobility;
    SlRxCallback m_rx;

    // Mode-2 config
    uint32_t m_t1Slots{1};    //!< selection window start offset
    uint32_t m_t2Slots{100};  //!< selection window end offset (<= remaining PDB)
    uint32_t m_prsvpSlots{100};//!< resource reservation interval (e.g. 100 ms @ mu=0)
    double m_rsrpThreshDbm{-110.0};
    uint32_t m_pktBytes{190};

    // sensing memory: (slot,subch) -> best sensed SL-RSRP (dBm), for reservations
    // announced by peers within the sensing window.
    std::map<std::pair<uint64_t, uint32_t>, double> m_sensed;

    Ptr<UniformRandomVariable> m_rand;
    NtnSlResourcePool m_pool;
    uint32_t m_txCount{0};
    uint32_t m_reselections{0};
};

/**
 * \ingroup ntn-nr-sidelink
 * \brief The shared PC5 medium: steps SL slots, resolves half-duplex + collisions,
 *        computes PRR (TS 38.885 methodology).
 */
class NtnSlChannel : public Object
{
  public:
    static TypeId GetTypeId();
    NtnSlChannel();
    ~NtnSlChannel() override;

    void SetResourcePool(const NtnSlResourcePool& pool) { m_pool = pool; }
    const NtnSlResourcePool& GetResourcePool() const { return m_pool; }

    /// Register a UE MAC (adopts the pool + a shared slot clock).
    void AddUe(Ptr<NtnSlUeMac> ue);

    /// Simple log-distance SL path loss; SL-RSRP (dBm) at a receiver.
    void SetTxPowerDbm(double dbm) { m_txPowerDbm = dbm; }
    void SetPathLossExponent(double n) { m_plExp = n; }
    void SetRefLossDb(double db) { m_refLossDb = db; }
    void SetDecodeThresholdDbm(double dbm) { m_decodeThreshDbm = dbm; }
    /// Two decoded neighbours whose SL-RSRP are within this margin collide.
    void SetCollisionMarginDb(double db) { m_collisionMarginDb = db; }

    /// Begin stepping slots; each UE transmits its periodic BSM on its grant.
    void Start(Time firstTx, Time stop);

    // ---- KPIs (TS 38.885) ----
    uint64_t GetTxTotal() const { return m_txTotal; }
    uint64_t GetRxSuccessTotal() const { return m_rxSuccess; }
    uint64_t GetRxAttemptTotal() const { return m_rxAttempt; }
    /// Packet Reception Ratio over all (tx, in-range rx) pairs.
    double GetPrr() const { return m_rxAttempt ? double(m_rxSuccess) / m_rxAttempt : 0.0; }
    /// PRR restricted to receivers within \p rangeM of the transmitter.
    double GetPrrWithinRange(double rangeM) const;

    double SlRsrpDbm(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const;

  private:
    void SlotTick();

    NtnSlResourcePool m_pool;
    std::vector<Ptr<NtnSlUeMac>> m_ues;
    uint64_t m_slot{0};
    Time m_stop;
    bool m_running{false};

    // PHY
    double m_txPowerDbm{23.0};
    double m_plExp{2.2};
    double m_refLossDb{40.0}; // loss at 1 m reference
    double m_decodeThreshDbm{-110.0};
    double m_collisionMarginDb{6.0};

    // pending grants per UE for the current period
    std::map<uint32_t, NtnSlGrant> m_grants;
    // reselection counter per UE (TS 38.321 SL_RESOURCE_RESELECTION_COUNTER)
    std::map<uint32_t, int32_t> m_reselCounter;

    // KPI accumulators
    uint64_t m_txTotal{0};
    uint64_t m_rxAttempt{0};
    uint64_t m_rxSuccess{0};
    // range-bucketed
    struct RangeBucket { double range; uint64_t attempt; uint64_t success; };
    mutable std::vector<RangeBucket> m_rangeBuckets;
};

} // namespace ns3

#endif // NTN_NR_SIDELINK_H
