/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-nr-sidelink.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnNrSidelink");

// ===========================================================================
//  NtnSlUeMac
// ===========================================================================
NS_OBJECT_ENSURE_REGISTERED(NtnSlUeMac);

TypeId
NtnSlUeMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnSlUeMac")
                            .SetParent<Object>()
                            .SetGroupName("NtnV2x")
                            .AddConstructor<NtnSlUeMac>();
    return tid;
}

NtnSlUeMac::NtnSlUeMac()
{
    m_rand = CreateObject<UniformRandomVariable>();
}

NtnSlUeMac::~NtnSlUeMac()
{
}

void
NtnSlUeMac::SetSelectionWindow(uint32_t t1Slots, uint32_t t2Slots)
{
    NS_ASSERT_MSG(t2Slots > t1Slots, "selection window T2 must exceed T1");
    m_t1Slots = t1Slots;
    m_t2Slots = t2Slots;
}

int64_t
NtnSlUeMac::AssignStreams(int64_t stream)
{
    m_rand->SetStream(stream);
    return 1;
}

void
NtnSlUeMac::SenseReservation(uint64_t slot, uint32_t subch, double rsrpDbm)
{
    auto key = std::make_pair(slot, subch);
    auto it = m_sensed.find(key);
    if (it == m_sensed.end() || rsrpDbm > it->second)
    {
        m_sensed[key] = rsrpDbm;
    }
}

void
NtnSlUeMac::PruneSensing(uint64_t nowSlot)
{
    for (auto it = m_sensed.begin(); it != m_sensed.end();)
    {
        if (it->first.first < nowSlot)
        {
            it = m_sensed.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void
NtnSlUeMac::DeliverRx(uint32_t fromUeId, Ptr<Packet> pkt)
{
    if (!m_rx.IsNull())
    {
        m_rx(fromUeId, pkt);
    }
}

Ptr<Packet>
NtnSlUeMac::BuildTxPacket()
{
    // V2X-3. A real packet crosses PC5 now. When the scenario supplies a
    // builder it stamps its own payload (a J2735 BSM, in the shipped example);
    // otherwise synthesise an opaque packet of the configured size so
    // byte-count-only scenarios are unaffected.
    if (!m_txPacket.IsNull())
    {
        Ptr<Packet> p = m_txPacket(m_ueId);
        if (p)
        {
            return p;
        }
    }
    return Create<Packet>(m_pktBytes);
}

NtnSlGrant
NtnSlUeMac::SelectResource(uint64_t nowSlot)
{
    PruneSensing(nowSlot);
    ++m_reselections;

    const uint32_t C = m_pool.ResourcesPerSlot();
    const uint64_t winStart = nowSlot + m_t1Slots;
    const uint64_t winEnd = nowSlot + m_t2Slots; // inclusive
    const uint32_t windowSlots = static_cast<uint32_t>(winEnd - winStart + 1);
    const uint32_t total = windowSlots * C;
    NS_ASSERT_MSG(total > 0, "empty selection window");

    // TS 38.321 §5.22.1.2: build candidate single-slot resources S_A, then exclude
    // any resource a sensed neighbour has RESERVED (periodically, mod Prsvp) with
    // SL-RSRP above the threshold. If < 20% remain, raise the threshold by 3 dB
    // and repeat. Then pick uniformly at random from the remaining set S_B.
    double thresh = m_rsrpThreshDbm;
    std::vector<NtnSlGrant> candidates;
    for (int guard = 0; guard < 40; ++guard) // threshold can climb at most 40*3 dB
    {
        candidates.clear();
        for (uint64_t s = winStart; s <= winEnd; ++s)
        {
            for (uint32_t c = 0; c < C; ++c)
            {
                // Exclude if a sensed reservation collides (periodic projection):
                // any sensed (slot', c) with slot' % Prsvp == s % Prsvp and rsrp>thresh.
                bool excluded = false;
                for (const auto& [key, rsrp] : m_sensed)
                {
                    if (key.second == c && rsrp > thresh &&
                        (m_prsvpSlots == 0 ||
                         (key.first % m_prsvpSlots) == (s % m_prsvpSlots)))
                    {
                        excluded = true;
                        break;
                    }
                }
                if (!excluded)
                {
                    NtnSlGrant g;
                    g.slot = s;
                    g.startSubch = c;
                    g.lenSubch = 1;
                    g.valid = true;
                    candidates.push_back(g);
                }
            }
        }
        if (candidates.size() * 5 >= total) // >= 20%
        {
            break;
        }
        thresh += 3.0; // TS 38.321 step 7-8
    }

    if (candidates.empty())
    {
        // Degenerate: threshold escaped its guard (should not happen). Fall back
        // to the whole window so the UE still transmits (never silently drops).
        for (uint64_t s = winStart; s <= winEnd; ++s)
        {
            for (uint32_t c = 0; c < C; ++c)
            {
                NtnSlGrant g{s, c, 1, true};
                candidates.push_back(g);
            }
        }
    }

    uint32_t pick = static_cast<uint32_t>(m_rand->GetInteger(0, candidates.size() - 1));
    return candidates[pick];
}

// ===========================================================================
//  NtnSlChannel
// ===========================================================================
NS_OBJECT_ENSURE_REGISTERED(NtnSlChannel);

TypeId
NtnSlChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnSlChannel")
            .SetParent<Object>()
            .SetGroupName("NtnV2x")
            .AddConstructor<NtnSlChannel>()
            .AddAttribute("TxPowerDbm", "PC5 transmit power (dBm)", DoubleValue(23.0),
                          MakeDoubleAccessor(&NtnSlChannel::m_txPowerDbm), MakeDoubleChecker<double>())
            .AddAttribute("PathLossExponent", "Log-distance path-loss exponent", DoubleValue(2.2),
                          MakeDoubleAccessor(&NtnSlChannel::m_plExp), MakeDoubleChecker<double>());
    return tid;
}

NtnSlChannel::NtnSlChannel()
{
    // V2X-4: shadowing and blockage draws. Seeded from ns-3's stream manager,
    // so a run stays reproducible under RngSeedManager like everything else.
    m_shadowRv = CreateObject<NormalRandomVariable>();
    m_shadowRv->SetAttribute("Mean", DoubleValue(0.0));
    m_shadowRv->SetAttribute("Variance", DoubleValue(m_shadowSigmaDb * m_shadowSigmaDb));
    m_nlosvRv = CreateObject<NormalRandomVariable>();
    m_nlosvRv->SetAttribute("Mean", DoubleValue(m_nlosvMeanDb));
    m_nlosvRv->SetAttribute("Variance", DoubleValue(m_nlosvSigmaDb * m_nlosvSigmaDb));
}

NtnSlChannel::~NtnSlChannel()
{
}

void
NtnSlChannel::AddUe(Ptr<NtnSlUeMac> ue)
{
    ue->m_pool = m_pool;
    m_ues.push_back(ue);
}

double
NtnSlChannel::SlRsrpDbm(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const
{
    if (m_useTr37885)
    {
        return m_txPowerDbm - Tr37885PathLossDb(tx, rx);
    }
    double d = tx->GetDistanceFrom(rx);
    d = std::max(d, 1.0);
    double pl = m_refLossDb + 10.0 * m_plExp * std::log10(d);
    return m_txPowerDbm - pl;
}

bool
NtnSlChannel::IsNlosv(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const
{
    // NLOSv per TR 37.885: another VEHICLE obstructs the tx-rx path. A third
    // registered UE counts as a blocker when it lies between the two and close
    // to the line joining them. This is geometry the module already has; the
    // old model had no notion of blockage at all.
    const Vector a = tx->GetPosition();
    const Vector b = rx->GetPosition();
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double abLen2 = abx * abx + aby * aby;
    if (abLen2 < 1.0)
    {
        return false;
    }
    std::vector<Ptr<MobilityModel>> candidates;
    candidates.reserve(m_ues.size() + m_extraBlockers.size());
    for (const auto& ue : m_ues)
    {
        candidates.push_back(ue->GetMobility());
    }
    for (const auto& e : m_extraBlockers)
    {
        candidates.push_back(e);
    }
    for (const auto& m : candidates)
    {
        if (!m || m == tx || m == rx)
        {
            continue;
        }
        const Vector c = m->GetPosition();
        // Projection parameter of c onto the segment ab.
        const double t = ((c.x - a.x) * abx + (c.y - a.y) * aby) / abLen2;
        if (t <= 0.0 || t >= 1.0)
        {
            continue; // not between them
        }
        const double px = a.x + t * abx;
        const double py = a.y + t * aby;
        const double lateral = std::hypot(c.x - px, c.y - py);
        if (lateral <= m_blockerLateralM)
        {
            return true;
        }
    }
    return false;
}

double
NtnSlChannel::Tr37885PathLossDb(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx) const
{
    // TR 37.885 Table 6.2.1-1, Highway LOS:
    //   PL = 32.4 + 20 log10(d_3D [m]) + 20 log10(fc [GHz])
    double d = tx->GetDistanceFrom(rx);
    d = std::max(d, 1.0);
    const double fcGhz = m_carrierHz / 1e9;
    double pl = 32.4 + 20.0 * std::log10(d) + 20.0 * std::log10(fcGhz);

    // Log-normal shadowing, sigma 3 dB in LOS.
    if (m_shadowSigmaDb > 0.0 && m_shadowRv)
    {
        pl += m_shadowRv->GetValue();
    }
    // sigma == 0 contributes nothing, which keeps the closed form checkable.

    // NLOSv: an additional blockage loss when a vehicle obstructs the path.
    // Clamped at zero because blockage cannot improve the link.
    if (IsNlosv(tx, rx) && m_nlosvRv)
    {
        pl += std::max(0.0, m_nlosvRv->GetValue());
    }
    return pl;
}

void
NtnSlChannel::SetShadowingSigmaDb(double db)
{
    // The RNGs are built in the constructor from the defaults, so a setter
    // that only wrote the member left the DRAW at its old variance - the
    // configured value would have been silently ignored.
    m_shadowSigmaDb = db;
    if (m_shadowRv)
    {
        m_shadowRv->SetAttribute("Variance", DoubleValue(db * db));
    }
}

void
NtnSlChannel::SetNlosvBlockage(double meanDb, double sigmaDb)
{
    m_nlosvMeanDb = meanDb;
    m_nlosvSigmaDb = sigmaDb;
    if (m_nlosvRv)
    {
        m_nlosvRv->SetAttribute("Mean", DoubleValue(meanDb));
        m_nlosvRv->SetAttribute("Variance", DoubleValue(sigmaDb * sigmaDb));
    }
}

bool
NtnSlChannel::AddBlockerForTest(Ptr<MobilityModel> m)
{
    if (!m)
    {
        return false;
    }
    m_extraBlockers.push_back(m);
    return true;
}

double
NtnSlChannel::NoiseFloorDbm() const
{
    // kTB at 290 K plus the receiver noise figure, over one subchannel.
    return -174.0 + 10.0 * std::log10(std::max(1.0, m_subchBwHz)) + m_noiseFigureDb;
}

void
NtnSlChannel::Start(Time firstTx, Time stop)
{
    m_stop = stop;
    m_running = true;
    m_slot = 0;
    // Give every UE an initial grant (first selection at slot 0).
    for (auto& ue : m_ues)
    {
        NtnSlGrant g = ue->SelectResource(0);
        // Offset the first transmission into the selection window.
        m_grants[ue->GetUeId()] = g;
        m_reselCounter[ue->GetUeId()] =
            static_cast<int32_t>(ue->m_rand->GetInteger(5, 15)); // SL_RESOURCE_RESELECTION_COUNTER
    }
    Simulator::Schedule(firstTx, &NtnSlChannel::SlotTick, this);
}

void
NtnSlChannel::SlotTick()
{
    if (!m_running || Simulator::Now() > m_stop)
    {
        m_running = false;
        return;
    }

    // 1. Collect transmissions scheduled for THIS slot.
    struct Tx
    {
        Ptr<NtnSlUeMac> ue;
        uint32_t subch;
        /// V2X-3: the packet this transmission carries. Built ONCE per
        /// transmission, so every receiver in the slot decodes the same bytes -
        /// building per receiver would give each one its own BSM and quietly
        /// break any continuity check on the sequence.
        Ptr<Packet> pkt;
    };
    std::vector<Tx> txs;
    for (auto& ue : m_ues)
    {
        auto it = m_grants.find(ue->GetUeId());
        if (it != m_grants.end() && it->second.valid && it->second.slot == m_slot)
        {
            txs.push_back({ue, it->second.startSubch, ue->BuildTxPacket()});
        }
    }

    // 2. Deliver: for each receiver NOT transmitting this slot (half-duplex),
    //    attempt every transmitter; a co-channel neighbour within the collision
    //    margin corrupts the packet.
    for (auto& rxUe : m_ues)
    {
        bool rxIsTx = std::any_of(txs.begin(), txs.end(),
                                  [&](const Tx& t) { return t.ue == rxUe; });
        if (rxIsTx)
        {
            continue; // half-duplex: cannot receive while transmitting
        }
        for (const auto& t : txs)
        {
            double rsrp = SlRsrpDbm(t.ue->GetMobility(), rxUe->GetMobility());
            double dist = t.ue->GetMobility()->GetDistanceFrom(rxUe->GetMobility());
            ++m_rxAttempt;
            // range bucket
            {
                bool placed = false;
                for (auto& b : m_rangeBuckets)
                {
                    if (std::abs(b.range - dist) < 25.0)
                    {
                        ++b.attempt;
                        placed = true;
                        break;
                    }
                }
                if (!placed)
                {
                    m_rangeBuckets.push_back({dist, 1, 0});
                }
            }

            bool decodable;
            if (m_useTr37885)
            {
                // V2X-4: decode is an SINR test against a TS 38.214 threshold,
                // not a bare RSRP threshold. Same-subchannel transmitters are
                // summed as INTERFERENCE in linear power rather than compared
                // one at a time against a margin, so two weak interferers can
                // jointly break a link that neither breaks alone - which is the
                // behaviour that produces a real PRR-vs-distance curve.
                const double noiseMw = std::pow(10.0, NoiseFloorDbm() / 10.0);
                double interfMw = 0.0;
                for (const auto& other : txs)
                {
                    if (other.ue == t.ue || other.subch != t.subch)
                    {
                        continue;
                    }
                    const double rsrpOther =
                        SlRsrpDbm(other.ue->GetMobility(), rxUe->GetMobility());
                    interfMw += std::pow(10.0, rsrpOther / 10.0);
                }
                const double sigMw = std::pow(10.0, rsrp / 10.0);
                const double sinrDb = 10.0 * std::log10(sigMw / (noiseMw + interfMw));
                decodable = (sinrDb > m_decodeSinrDb);
            }
            else
            {
                decodable = rsrp > m_decodeThreshDbm;
                // Co-channel collision: another transmitter on the same
                // subchannel heard within the margin.
                if (decodable)
                {
                    for (const auto& other : txs)
                    {
                        if (other.ue == t.ue || other.subch != t.subch)
                        {
                            continue;
                        }
                        double rsrpOther =
                            SlRsrpDbm(other.ue->GetMobility(), rxUe->GetMobility());
                        if (rsrpOther > rsrp - m_collisionMarginDb)
                        {
                            decodable = false;
                            break;
                        }
                    }
                }
            }

            if (decodable)
            {
                ++m_rxSuccess;
                for (auto& b : m_rangeBuckets)
                {
                    if (std::abs(b.range - dist) < 25.0)
                    {
                        ++b.success;
                        break;
                    }
                }
                // V2X-5. Delivery used to happen synchronously inside SlotTick,
                // in the same event as the transmission, so the one-way delay of
                // every PC5 message was identically zero and the module exposed
                // no delay KPI at all. A V2X safety study whose latency is zero
                // by construction cannot say anything about latency.
                //
                // A transmission occupies its slot: the receiver cannot decode
                // before the slot ends. Add the propagation time on top, which
                // at V2X ranges is sub-microsecond and so is dominated by the
                // slot, but is the term that makes the number a delay rather
                // than a constant.
                const double propS = dist / 299792458.0;
                const Time owd = m_pool.slotDuration + Seconds(propS);
                m_delaySumS += owd.GetSeconds();
                m_delayMaxS = std::max(m_delayMaxS, owd.GetSeconds());
                ++m_delayCount;
                Ptr<NtnSlUeMac> dst = rxUe;
                const uint32_t src = t.ue->GetUeId();
                Ptr<Packet> copy = t.pkt->Copy();
                Simulator::Schedule(owd, [dst, src, copy]() { dst->DeliverRx(src, copy); });
            }
        }
    }

    // 3. Post-TX bookkeeping: announce the periodic reservation to peers (they
    //    sense it), advance the reselection counter, and reselect when it hits 0.
    for (const auto& t : txs)
    {
        ++t.ue->m_txCount;
        ++m_txTotal;
        uint32_t uid = t.ue->GetUeId();
        uint64_t nextSlot = m_slot + t.ue->m_prsvpSlots;

        // Peers within sensing range decode the SCI reserving nextSlot@subch.
        for (auto& peer : m_ues)
        {
            if (peer == t.ue)
            {
                continue;
            }
            double rsrp = SlRsrpDbm(t.ue->GetMobility(), peer->GetMobility());
            if (rsrp > m_decodeThreshDbm)
            {
                peer->SenseReservation(nextSlot, t.subch, rsrp);
            }
        }

        int32_t& ctr = m_reselCounter[uid];
        --ctr;
        if (ctr > 0)
        {
            // Keep the same subchannel; periodic reservation.
            NtnSlGrant g = m_grants[uid];
            g.slot = nextSlot;
            m_grants[uid] = g;
        }
        else
        {
            NtnSlGrant g = t.ue->SelectResource(m_slot);
            m_grants[uid] = g;
            m_reselCounter[uid] = static_cast<int32_t>(t.ue->m_rand->GetInteger(5, 15));
        }
    }

    ++m_slot;
    Simulator::Schedule(m_pool.slotDuration, &NtnSlChannel::SlotTick, this);
}

double
NtnSlChannel::GetPrrWithinRange(double rangeM) const
{
    uint64_t att = 0;
    uint64_t suc = 0;
    for (const auto& b : m_rangeBuckets)
    {
        if (b.range <= rangeM)
        {
            att += b.attempt;
            suc += b.success;
        }
    }
    return att ? double(suc) / att : 0.0;
}

} // namespace ns3
