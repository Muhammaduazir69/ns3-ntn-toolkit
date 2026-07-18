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
NtnSlUeMac::DeliverRx(uint32_t fromUeId, uint32_t bytes)
{
    if (!m_rx.IsNull())
    {
        m_rx(fromUeId, bytes);
    }
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
    double d = tx->GetDistanceFrom(rx);
    d = std::max(d, 1.0);
    double pl = m_refLossDb + 10.0 * m_plExp * std::log10(d);
    return m_txPowerDbm - pl;
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
    };
    std::vector<Tx> txs;
    for (auto& ue : m_ues)
    {
        auto it = m_grants.find(ue->GetUeId());
        if (it != m_grants.end() && it->second.valid && it->second.slot == m_slot)
        {
            txs.push_back({ue, it->second.startSubch});
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

            bool decodable = rsrp > m_decodeThreshDbm;
            // Co-channel collision: another transmitter on the same subchannel
            // heard within the margin.
            if (decodable)
            {
                for (const auto& other : txs)
                {
                    if (other.ue == t.ue || other.subch != t.subch)
                    {
                        continue;
                    }
                    double rsrpOther = SlRsrpDbm(other.ue->GetMobility(), rxUe->GetMobility());
                    if (rsrpOther > rsrp - m_collisionMarginDb)
                    {
                        decodable = false;
                        break;
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
                rxUe->DeliverRx(t.ue->GetUeId(), t.ue->m_pktBytes);
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
