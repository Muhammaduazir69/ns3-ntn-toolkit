/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 */
#ifndef NTN_SAGIN_HAPS_MOBILITY_H
#define NTN_SAGIN_HAPS_MOBILITY_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * @brief HAPS station-keeping mobility for stratospheric platforms (~20 km).
 *
 * Loon-style high-altitude platforms hold a nominal station by drifting on
 * a small horizontal "racetrack" (figure-8) within a station-keeping radius.
 * This model produces a horizontally-bounded slow drift (≤ a few m/s) at a
 * configurable altitude (default 20 km), so position stays within
 * `StationKeepRadius` of the centre indefinitely.
 *
 * Defaults from ITU-R F.1500 / ITU-R M.[HAPS_CHAR]:
 *  - altitude       20 000 m
 *  - radius          5 000 m
 *  - horizontal speed   15 m/s   (≪ 50 m/s wind)
 *
 * The vertical channel uses a small Ornstein-Uhlenbeck-style perturbation
 * with a long correlation time so altitude stays within the
 * `MaxVerticalDeviation` envelope (default ±50 m, matching the W5
 * validation gate).
 */
class HapsMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    HapsMobilityModel();
    ~HapsMobilityModel() override;

    /// Set the (lat, lon) anchor point converted to an ECEF-like local frame.
    void SetCenter(const Vector& centerXyz);
    Vector GetCenter() const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void Tick();

    Vector m_center;
    double m_altitudeM{20000.0};
    double m_radiusM{5000.0};
    double m_speedMps{15.0};
    double m_maxVerticalDeviationM{50.0};
    Time m_tickInterval{Seconds(1.0)};

    // Phase along the figure-8 (parametric `t`).
    double m_phase{0.0};
    double m_verticalOffset{0.0};
    Vector m_currentPosition{};
    Vector m_currentVelocity{};

    Ptr<NormalRandomVariable> m_verticalNoise;
    EventId m_tickEvent;
};

} // namespace ns3

#endif // NTN_SAGIN_HAPS_MOBILITY_H
