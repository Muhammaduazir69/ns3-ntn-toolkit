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

    /// Carrier frequency (GHz). 2 GHz default: the S-band (n255/n256) NTN service link.
    void SetFrequencyGHz(double fcGHz);

    /// EIRP at the LEO downlink (dBm). Default 74 dBm: TR 38.821 Table 6.1.1.1-1
    /// Set-1 S-band LEO-600 EIRP density 34 dBW/MHz over the 10 MHz the noise
    /// floor below assumes (34 + 10 + 30). The old 50 dBm default had no source.
    void SetSatEirpDbm(double dbm);

    /// Receiver noise floor at the vehicle (dBm). Default -97 dBm: kTB over
    /// 10 MHz (-174 + 70 = -104 dBm) plus the 7 dB UE noise figure of TR 38.821
    /// Table 6.1.1.1-3. The old -110 dBm was 13 dB below thermal noise alone.
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
    double m_satEirpDbm{74.0};    // TR 38.821 Set-1, 34 dBW/MHz x 10 MHz
    double m_noiseFloorDbm{-97.0}; // kTB(10 MHz) + NF 7 dB
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_LEO_DIRECT_H
