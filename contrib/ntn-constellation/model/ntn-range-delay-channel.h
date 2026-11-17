/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NtnRangeDelayChannel: a point-to-point channel whose propagation delay is
// computed for every packet from the exact orbital state of its two endpoints.
//
// Why. The constellation and SAGIN examples set the Delay attribute of a plain
// PointToPointChannel from a periodic tick (1 s, later 100 ms), and the
// satellites' GetPosition() is itself cached on a 100 ms grain. The delay a
// packet saw was therefore a staircase: constant between refreshes, then a
// jump of up to |range rate| x refresh / c (about 25 us per second on a LEO
// ground link). A constant-rate flow then reports a jitter that is the step
// size averaged over the packets between steps, an artifact of the refresh
// period rather than of the orbit.
//
// What this channel does instead, per packet:
//   1. the emission instant of the last bit is t_e = now + txTime (the ns-3
//      point-to-point convention: Receive() is scheduled for the last bit);
//   2. the endpoint positions at t_e come from the exact propagator: an
//      Sgp4MobilityModel is evaluated with GetEcefStateAt() (no position
//      cache); any other MobilityModel is read now and extrapolated linearly
//      with its velocity to t_e;
//   3. the light-time equation c tau = |r_rx(t_e + tau) - r_tx(t_e)| is solved
//      by fixed-point iteration (two passes; the residual is of order
//      (v/c)^2 tau, below a picosecond), with the receiver moved along its
//      velocity during tau and, in the inertial frame frozen at t_e, rotated
//      by omega_E tau about the Earth axis (Sagnac term, IERS
//      omega_E = 7.2921159e-5 rad/s), because the endpoint positions are ECEF.
//
// The delay mode "Fixed" falls back to the base class Delay attribute, so an
// existing scenario can switch back to the staircase for comparison.
//
// Known simplifications, stated plainly: no tropospheric or ionospheric excess
// path delay (ITU-R P.531 / P.834; a few ns to tens of ns at Ka band), and no
// NetAnim tx/rx animation trace (the base class keeps it private).

#ifndef NTN_RANGE_DELAY_CHANNEL_H
#define NTN_RANGE_DELAY_CHANNEL_H

#include "ns3/data-rate.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/traced-callback.h"

#include <array>
#include <cstdint>
#include <string>

namespace ns3
{
namespace ntncon
{

class NtnRangeDelayChannel : public PointToPointChannel
{
  public:
    /// How the per-packet propagation delay is obtained.
    enum DelayMode
    {
        EXACT = 0, //!< light-time solution on the exact endpoint state (default)
        FIXED = 1  //!< the base class Delay attribute (legacy staircase)
    };

    static TypeId GetTypeId();
    NtnRangeDelayChannel();

    /// Endpoint mobility for device index 0 and 1 (attach order). When unset,
    /// the MobilityModel aggregated to each device's node is used.
    void SetEndpointMobility(Ptr<MobilityModel> m0, Ptr<MobilityModel> m1);

    /// One-way delay (s) of a signal whose last bit leaves endpoint `from`
    /// `emitOffset` after now, to the other endpoint. Public so tests and
    /// examples can query the same physics the packets see.
    double ComputeDelayS(uint32_t from, Time emitOffset) const;

    bool TransmitStart(Ptr<const Packet> p, Ptr<PointToPointNetDevice> src, Time txTime) override;

    uint64_t GetPacketCount() const { return m_packets; }
    double GetLastDelayS() const { return m_lastDelayS; }
    double GetMinDelayS() const { return m_minDelayS; }
    double GetMaxDelayS() const { return m_maxDelayS; }

    /// Build two PointToPointNetDevices (DropTail queue of `queueSize`, same
    /// layout as PointToPointHelper::Install with flow control) on a new
    /// NtnRangeDelayChannel. `chOut` receives the channel.
    static NetDeviceContainer Connect(Ptr<Node> a,
                                      Ptr<Node> b,
                                      DataRate rate,
                                      const std::string& queueSize,
                                      Ptr<NtnRangeDelayChannel>& chOut);

  private:
    Ptr<MobilityModel> Endpoint(uint32_t i) const;
    static void StateAt(Ptr<MobilityModel> m, Time offset, Vector& p, Vector& v);

    DelayMode m_mode{EXACT};
    bool m_sagnac{true};
    std::array<Ptr<MobilityModel>, 2> m_mob;
    uint64_t m_packets{0};
    double m_lastDelayS{0.0};
    double m_minDelayS{1e300};
    double m_maxDelayS{0.0};
    TracedCallback<Ptr<const Packet>, Time> m_delayTrace;
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_RANGE_DELAY_CHANNEL_H
