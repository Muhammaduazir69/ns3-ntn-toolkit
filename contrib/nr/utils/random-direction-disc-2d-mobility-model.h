// Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef RANDOM_DIRECTION_DISC_2D_MOBILITY_MODEL_H
#define RANDOM_DIRECTION_DISC_2D_MOBILITY_MODEL_H

#include "ns3/constant-velocity-helper.h"
#include "ns3/event-id.h"
#include "ns3/mobility-model.h"
#include "ns3/nr-export.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{

/**
 * @ingroup nr-utils
 * @brief Random-direction 2D mobility model bounded by a disc.
 *
 * Mirrors ns3::RandomDirection2dMobilityModel but uses a circular boundary instead of a
 * rectangle. Useful for TR 36.839-style handover deployments where the simulated UEs are
 * meant to roam inside a disc centred on the central site.
 *
 * Behavior: at each leg the model picks a uniformly random direction, moves at a random
 * speed until it hits the disc boundary, pauses for a random time, then picks a new
 * inward-pointing direction (uniform over the half-plane that faces the disc interior).
 *
 * Attributes:
 *   - CenterX, CenterY: disc centre coordinates (metres).
 *   - Radius:           disc radius (metres). Must be > 0.
 *   - Speed:            RandomVariableStream for leg speed (m/s).
 *   - Pause:            RandomVariableStream for boundary pause (s).
 */
class NR_EXPORT RandomDirectionDisc2dMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    RandomDirectionDisc2dMobilityModel();
    ~RandomDirectionDisc2dMobilityModel() override;

    Ptr<MobilityModel> Copy() const override
    {
        return CreateObject<RandomDirectionDisc2dMobilityModel>(*this);
    }

  private:
    void DoDispose() override;
    void DoInitialize() override;
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;
    int64_t DoAssignStreams(int64_t stream) override;

    /// Pick a fresh random direction (uniform over [0, 2pi)) and start a new leg.
    void StartLeg();
    /// Pause at the current position for a random time, then start a new inward leg.
    void BeginPause();
    /// At a boundary point: pick a uniform direction in the inward half-plane and go.
    void RestartInwardLeg();
    /// Run a leg in the given direction (radians, CCW from +x axis).
    void RunLeg(double direction);
    /// Time (seconds) until the constant-velocity ray from position with given velocity
    /// hits the disc boundary. Returns 0 if velocity has zero length.
    double TimeToBoundary(const Vector& position, const Vector& velocity) const;

    mutable ConstantVelocityHelper m_helper;
    Ptr<UniformRandomVariable> m_direction;
    Ptr<RandomVariableStream> m_speed;
    Ptr<RandomVariableStream> m_pause;
    EventId m_event;

    double m_centerX{0.0};
    double m_centerY{0.0};
    double m_radius{1.0};
};

} // namespace ns3

#endif // RANDOM_DIRECTION_DISC_2D_MOBILITY_MODEL_H
