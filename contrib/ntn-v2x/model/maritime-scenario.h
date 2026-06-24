/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * Maritime mobility (DEPRECATED / offline-test-only).
 *
 * This model uses synthetic billiard-ball box-bounce motion: a vessel drifts
 * at a fixed speed and reflects off a bounding box. It is NOT a real vessel
 * track and is retained only for the bounded-area unit test.
 *
 * The REAL maritime NTN path now reuses ntn-sagin's AisMobilityModel, which
 * replays a recorded Danish Maritime AIS track (lat/lon/SOG/COG) — see the
 * example ntn-v2x-maritime-ais. Prefer that for any measured study. Do not
 * use MaritimeMobilityModel for results; it fabricates motion.
 */
#ifndef NTN_V2X_MARITIME_SCENARIO_H
#define NTN_V2X_MARITIME_SCENARIO_H

#include "ns3/box.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"

namespace ns3
{
namespace ntnv2x
{

class MaritimeMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    MaritimeMobilityModel();
    ~MaritimeMobilityModel() override;

    /// Bounded sea area; vessels bounce on box edges.
    void SetSeaArea(const Box& area);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void Tick();

    Box m_area{-50000, 50000, -50000, 50000, 0, 0};
    double m_speedMps{8.0};
    Time m_tickInterval{Seconds(1.0)};

    Vector m_position;
    Vector m_velocity;
    Ptr<UniformRandomVariable> m_rand;
    EventId m_tickEvent;
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_MARITIME_SCENARIO_H
