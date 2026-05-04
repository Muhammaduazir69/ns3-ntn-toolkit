/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W5)
 *
 * Aeronautical mobility — commercial flight at cruise altitude (10-12 km),
 * speed ~250 m/s. Emulates a great-circle straight-leg trajectory between
 * a departure and arrival airport over a flat-Earth local frame.
 */
#ifndef NTN_SAGIN_AERONAUTICAL_SCENARIO_H
#define NTN_SAGIN_AERONAUTICAL_SCENARIO_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3
{

class AeronauticalMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    AeronauticalMobilityModel();
    ~AeronauticalMobilityModel() override;

    void SetFlightPlan(const Vector& departure, const Vector& arrival,
                       double cruiseAltitudeM, double cruiseSpeedMps);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    void Tick();

    Vector m_dep;
    Vector m_arr;
    double m_cruiseAltM{11000.0};
    double m_speedMps{250.0};
    double m_legLengthM{0.0};

    Vector m_current;
    Vector m_velocity;
    double m_progressM{0.0};
    Time m_tickInterval{Seconds(1.0)};
    EventId m_tickEvent;
};

} // namespace ns3

#endif // NTN_SAGIN_AERONAUTICAL_SCENARIO_H
