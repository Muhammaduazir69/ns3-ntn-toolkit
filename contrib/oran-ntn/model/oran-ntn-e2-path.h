/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnE2Path - the E2 transport path between an E2 node and the Near-RT
// RIC as an ordered list of hops, each with its own delay from live geometry.
//
// Why this exists (Wave 6B). The E2 node carried ONE "FeederLinkDelay": the
// slant range from the serving satellite to a gateway. That is the whole E2
// path only when the RIC sits at a gateway the serving satellite can see.
// With a regenerative payload (gNB on board, TS 38.300 16.14) and a ground
// RIC, E2 leaves the satellite over zero or more inter-satellite links to a
// satellite that sees the RIC's gateway, then the feeder link, then
// terrestrial backhaul to the RIC site; with a transparent payload and an
// on-board (space) RIC the direction reverses over the feeder link. The O-RAN
// NTN white paper (Apr 2025) asks for the control-loop timing to be compared
// against "the expected delays" of the NTN deployment, and E2GAP 5.1 bounds
// near-RT loops to 10 ms - 1 s, so every hop has to be in the loop and each
// has to be reportable on its own.
//
// Each hop is either GEOMETRIC (one-way propagation |a - b| / c between two
// live mobility models, recomputed every refresh) or FIXED (terrestrial
// backhaul, processing). BindToE2Node keeps the E2 node's one-way transport
// delay (the FeederLinkDelay attribute, applied once uplink to every RIC
// INDICATION and once downlink to every RIC CONTROL) equal to the sum, and
// keeps per-hop statistics. The control loop then costs per hop twice its
// one-way delay, plus the wait for the RIC control-loop tick.

#ifndef ORAN_NTN_E2_PATH_H
#define ORAN_NTN_E2_PATH_H

#include "oran-ntn-e2-interface.h"

#include <ns3/event-id.h>
#include <ns3/mobility-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>

#include <ostream>
#include <string>
#include <vector>

namespace ns3
{

class OranNtnE2Path : public Object
{
  public:
    enum class HopKind : uint8_t
    {
        SERVICE_LINK = 0, //!< UE - satellite (Uu), when E2 measurement rides it
        FEEDER_LINK = 1,  //!< satellite - gateway (NTN-GW)
        ISL = 2,          //!< satellite - satellite
        TERRESTRIAL = 3,  //!< gateway - RIC site backhaul
        PROCESSING = 4,   //!< E2 termination / SCTP / encoding at a node
    };
    static const char* KindName(HopKind k);

    static TypeId GetTypeId();
    OranNtnE2Path() = default;

    /// One-way hop whose delay is the live distance between two mobility models over c.
    void AddGeometricHop(const std::string& name,
                         HopKind kind,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b);
    /// One-way hop with a fixed delay.
    void AddFixedHop(const std::string& name, HopKind kind, Time delay);
    size_t GetNHops() const { return m_hops.size(); }

    struct HopSample
    {
        std::string name;
        HopKind kind;
        double rangeKm; //!< NaN for a fixed hop
        Time delay;
    };
    /// One-way E2 transport delay now; per-hop breakdown in \p out if given.
    Time ComputeOneWay(std::vector<HopSample>* out = nullptr) const;

    /**
     * \brief Keep \p e2's one-way E2 transport delay equal to this path.
     *
     * Applies immediately and then every \p refresh (100 ms by default, so a
     * LEO path at 7.5 km/s is never more than ~2.5 us stale), and records
     * per-hop statistics on every refresh.
     */
    void BindToE2Node(Ptr<OranNtnE2Node> e2, Time refresh = MilliSeconds(100));
    void Unbind();

    struct HopStats
    {
        std::string name;
        HopKind kind;
        uint64_t n{0};
        double sumMs{0.0};
        double minMs{0.0};
        double maxMs{0.0};
        double sumRangeKm{0.0};
        double MeanMs() const { return n ? sumMs / n : 0.0; }
        double MeanRangeKm() const { return n ? sumRangeKm / n : 0.0; }
    };
    const std::vector<HopStats>& GetHopStats() const { return m_stats; }
    /// Mean one-way total over the refreshes so far (ms).
    double GetMeanOneWayMs() const;

    /**
     * \brief Per-hop control-loop budget, CSV.
     *
     * Columns: hop,kind,range_km_mean,one_way_ms_mean,one_way_ms_min,
     * one_way_ms_max,loop_ms_mean (= 2 x one-way: KPM up, control down).
     * Then a "ric_tick_wait" row with \p ricWaitMeanMs (measured or half the
     * control-loop period), and a "loop_total" row.
     */
    void WriteLoopBudgetCsv(std::ostream& os, double ricWaitMeanMs) const;

  protected:
    void DoDispose() override;

  private:
    struct Hop
    {
        std::string name;
        HopKind kind;
        Ptr<MobilityModel> a;
        Ptr<MobilityModel> b;
        Time fixed;
    };
    void Refresh();

    std::vector<Hop> m_hops;
    std::vector<HopStats> m_stats;
    Ptr<OranNtnE2Node> m_e2;
    Time m_refresh{MilliSeconds(100)};
    EventId m_event;
    double m_sumTotalMs{0.0};
    uint64_t m_nTotal{0};
};

} // namespace ns3

#endif // ORAN_NTN_E2_PATH_H
