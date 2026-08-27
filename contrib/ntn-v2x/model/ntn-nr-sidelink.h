/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_NR_SIDELINK_H
#define NTN_NR_SIDELINK_H

#include "ns3/callback.h"
#include "ns3/mobility-model.h"
#include "ns3/packet.h"
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
    /// rx(fromUeId, packet) - a decoded PC5 packet from a peer.
    ///
    /// V2X-3: this used to be Callback<void, uint32_t, uint32_t>, i.e. a
    /// (sender id, byte count) pair. No Ptr<Packet> existed anywhere in the
    /// sidelink, so the SAE J2735 basic safety message the README says
    /// vehicles broadcast over PC5 never crossed PC5: NtnV2xBsmHeader appeared
    /// only in the test suite and on the Uu path. Nothing at PDCP, RLC or IP
    /// could attach either, so the sidelink could not compose with FlowMonitor,
    /// NtnOranSink, the in-band QoS header or anything else in the toolkit, and
    /// the content of a BSM influenced nothing.
    typedef Callback<void, uint32_t, Ptr<Packet>> SlRxCallback;

    /// Build the payload for one transmission opportunity.
    ///
    /// Called at the transmit slot so the sender can stamp current kinematics
    /// and an incrementing message count, exactly as a real BSM generator
    /// would. When unset, the MAC synthesises an opaque packet of
    /// SetPacketBytes() bytes so byte-count-only scenarios behave as before.
    typedef Callback<Ptr<Packet>, uint32_t> SlTxPacketCallback;

    static TypeId GetTypeId();
    NtnSlUeMac();
    ~NtnSlUeMac() override;

    void SetUeId(uint32_t id) { m_ueId = id; }
    uint32_t GetUeId() const { return m_ueId; }
    void SetMobility(Ptr<MobilityModel> m) { m_mobility = m; }
    Ptr<MobilityModel> GetMobility() const { return m_mobility; }
    void SetRxCallback(SlRxCallback cb) { m_rx = cb; }
    /// V2X-3: supply the packet each transmission carries. See SlTxPacketCallback.
    void SetTxPacketCallback(SlTxPacketCallback cb) { m_txPacket = cb; }

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
    void DeliverRx(uint32_t fromUeId, Ptr<Packet> pkt);
    /// V2X-3: build (or synthesise) the packet for this transmission.
    Ptr<Packet> BuildTxPacket();
    /// Drop sensed reservations older than the sensing window.
    void PruneSensing(uint64_t nowSlot);

    uint32_t m_ueId{0};
    Ptr<MobilityModel> m_mobility;
    SlRxCallback m_rx;
    SlTxPacketCallback m_txPacket;

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

    // ---- V2X-4: TR 37.885 Highway propagation and an SINR decode ----
    //
    // The sidelink PHY was `PL = 40 + 22*log10(d)` with decode as a bare RSRP
    // threshold: no noise, no SINR, no shadowing, no vehicle blockage. At the
    // example's 23 dBm and -115 dBm threshold that solves to a decode range of
    // 27.4 km, so every vehicle in a 475 m platoon decoded everything it was
    // not half-duplex-blocked from. PRR-vs-distance - the single KPI this
    // module advertises - had an inert distance axis: the reported degradation
    // came entirely from half-duplex and same-subchannel collisions.
    //
    // TR 37.885 Table 6.2.1-1 Highway LOS is
    //   PL = 32.4 + 20 log10(d_3D[m]) + 20 log10(fc[GHz])
    // with log-normal shadowing (sigma 3 dB LOS), plus an NLOSv additional
    // blockage loss when another vehicle obstructs the path.

    /// Carrier frequency (Hz). TR 37.885 evaluates V2X at 5.9 GHz.
    void SetCarrierFrequencyHz(double hz) { m_carrierHz = hz; }
    double GetCarrierFrequencyHz() const { return m_carrierHz; }

    /// Use the TR 37.885 Highway model instead of the log-distance fit.
    /// DEFAULT ON: the log-distance fit cannot produce a realistic
    /// PRR-vs-distance curve, which is what this module is for.
    void SetUseTr37885(bool on) { m_useTr37885 = on; }
    bool GetUseTr37885() const { return m_useTr37885; }

    /// Log-normal shadowing standard deviation (dB). TR 37.885: 3 dB LOS.
    /// Reconfigures the draw, so calling this after construction takes effect.
    void SetShadowingSigmaDb(double db);

    /// Mean and standard deviation of the NLOSv additional blockage loss (dB).
    /// Applied when a third vehicle lies within m_blockerLateralM of the
    /// tx-rx line and between the two, which is the geometry TR 37.885 calls
    /// NLOSv.
    void SetNlosvBlockage(double meanDb, double sigmaDb);

    /// Receiver noise figure (dB) and per-subchannel bandwidth (Hz), used for
    /// the thermal-noise floor of the SINR test.
    void SetNoiseFigureDb(double db) { m_noiseFigureDb = db; }
    void SetSubchannelBandwidthHz(double hz) { m_subchBwHz = hz; }

    /// Minimum post-detection SINR for a PSSCH decode (dB). TS 38.214: the
    /// lowest MCS (QPSK, R=120/1024) needs roughly -2 dB at 10% BLER.
    void SetDecodeSinrDb(double db) { m_decodeSinrDb = db; }
    double GetDecodeSinrDb() const { return m_decodeSinrDb; }

    /// Thermal noise + noise figure over one subchannel (dBm).
    double NoiseFloorDbm() const;

    /// Test seams (V2X-4): evaluate the propagation model and register a
    /// blocker without standing up a full resource pool. The physics is what
    /// this finding is about, and it should be checkable in isolation.
    double SlRsrpDbmForTest(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const
    {
        return SlRsrpDbm(tx, rx);
    }
    bool AddBlockerForTest(Ptr<MobilityModel> m);

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

    /// V2X-5: mean PC5 one-way delay in ms over every successful reception.
    ///
    /// Reception used to be scheduled in the same event as the transmission, so
    /// this quantity was identically zero and no accessor for it existed. A
    /// transmission occupies its slot, so the floor is one slot duration
    /// (1 ms at mu=0) plus propagation, which at V2X ranges is under a
    /// microsecond.
    double GetMeanDelayMs() const
    {
        return m_delayCount ? (m_delaySumS / m_delayCount) * 1e3 : 0.0;
    }
    /// V2X-5: worst observed PC5 one-way delay in ms.
    double GetMaxDelayMs() const { return m_delayMaxS * 1e3; }
    /// V2X-5: number of receptions the delay statistics cover.
    uint64_t GetDelaySampleCount() const { return m_delayCount; }

    double SlRsrpDbm(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const;
    /// TR 37.885 Highway path loss including shadowing and NLOSv blockage.
    double Tr37885PathLossDb(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const;
    /// Whether a third registered vehicle blocks the tx-rx line (NLOSv).
    bool IsNlosv(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const;

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
    // V2X-4
    double m_carrierHz{5.9e9};      //!< TR 37.885 evaluates V2X at 5.9 GHz
    bool m_useTr37885{true};
    double m_shadowSigmaDb{3.0};    //!< TR 37.885 Highway LOS
    double m_nlosvMeanDb{5.0};      //!< NLOSv additional blockage loss
    double m_nlosvSigmaDb{4.0};
    double m_blockerLateralM{2.0};  //!< how close to the line counts as blocking
    double m_noiseFigureDb{9.0};    //!< vehicular UE receiver
    double m_subchBwHz{10.0 * 12.0 * 15e3}; //!< 10 RB x 12 SC x 15 kHz
    double m_decodeSinrDb{-2.0};    //!< TS 38.214 lowest MCS at 10% BLER
    Ptr<NormalRandomVariable> m_shadowRv;
    Ptr<NormalRandomVariable> m_nlosvRv;
    std::vector<Ptr<MobilityModel>> m_extraBlockers;

    // pending grants per UE for the current period
    std::map<uint32_t, NtnSlGrant> m_grants;
    // reselection counter per UE (TS 38.321 SL_RESOURCE_RESELECTION_COUNTER)
    std::map<uint32_t, int32_t> m_reselCounter;

    // KPI accumulators
    uint64_t m_txTotal{0};
    uint64_t m_rxAttempt{0};
    uint64_t m_rxSuccess{0};
    // V2X-5: PC5 one-way delay accumulators.
    double m_delaySumS{0.0};
    double m_delayMaxS{0.0};
    uint64_t m_delayCount{0};
    // range-bucketed
    struct RangeBucket { double range; uint64_t attempt; uint64_t success; };
    mutable std::vector<RangeBucket> m_rangeBuckets;
};

} // namespace ns3

#endif // NTN_NR_SIDELINK_H
