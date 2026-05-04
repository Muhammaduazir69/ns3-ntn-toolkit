/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * Vehicle-to-LEO direct uplink helper.
 *
 * One-shot computation: given a vehicle position and a serving LEO
 * satellite mobility, compute slant range and free-space path loss. Used
 * by `V2xLeoRelay` to decide which vehicle has the best uplink budget,
 * and by Grafana panels to visualise per-vehicle SNR over a pass.
 */
#ifndef NTN_V2X_LEO_DIRECT_H
#define NTN_V2X_LEO_DIRECT_H

#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

namespace ns3
{
namespace ntnv2x
{

struct V2xLinkBudget
{
    double slantRangeKm{0.0};
    double freeSpacePlDb{0.0};   ///< 20·log10(d) + 20·log10(fc) + 32.45
    double rxPowerDbm{0.0};
    double snrDb{0.0};
    double elevationDeg{0.0};
    bool aboveHorizon{true};
};

class V2xLeoDirect : public Object
{
  public:
    static TypeId GetTypeId();
    V2xLeoDirect();

    /// Carrier frequency (GHz). 2 GHz default — NTN UE-link Ka/Ku-band-equivalent.
    void SetFrequencyGHz(double fcGHz);

    /// EIRP at the LEO downlink (dBm). 50 dBm = 10 W EIRP, typical for Starlink.
    void SetSatEirpDbm(double dbm);

    /// Receiver noise floor at the vehicle (dBm). −110 dBm ≈ 10 MHz.
    void SetNoiseFloorDbm(double dbm);

    /// Compute one link-budget snapshot.
    V2xLinkBudget Compute(Ptr<MobilityModel> vehicle,
                          Ptr<MobilityModel> sat) const;

    /// Static helper — same math, no Object overhead.
    static V2xLinkBudget ComputeStatic(const Vector& vehicle, const Vector& sat,
                                       double fcGHz, double satEirpDbm,
                                       double noiseFloorDbm);

  private:
    double m_fcGHz{2.0};
    double m_satEirpDbm{50.0};
    double m_noiseFloorDbm{-110.0};
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_LEO_DIRECT_H
