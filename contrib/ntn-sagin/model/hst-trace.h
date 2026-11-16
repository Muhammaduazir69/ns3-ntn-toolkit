/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_HST_TRACE_H
#define NTN_SAGIN_HST_TRACE_H

// High-Speed Train (HST) mobility, 3GPP HST geometry (Roadmap §4.4.3).
//
// Unlike 4.4.1 (OpenSky ADS-B) and 4.4.2 (AIS), HST is not a bulk-data
// import: the 3GPP HST case defines a deterministic synthetic motion: the train
// moves along a straight rail corridor at a chosen speed, the gNB sits
// at a perpendicular distance Dmin from the rail, and the cell handover
// pattern is fixed by the inter-site distance Ds = 2 * Dmin.
//
// HstTrace exposes the synthetic trajectory as a sample list (compatible
// with the OpenSky / AIS replay pattern); HstMobilityModel drives a node
// along the corridor under Simulator::Run() and exposes a Doppler-shift
// helper for downstream PHY consumers.
//
// Reference scenarios. NOTE: TR 38.901 has no HST table (its Section 7.5 is
// fast fading); the 3GPP HST geometry (Ds, Dmin, v) is defined for conformance
// in TS 36.104 / TS 38.104 Annex B and TS 38.101-4 Annex B. The presets below
// are module conventions loosely after those (500 km/h with Dmin = 150 m is the
// NR FR1 HST case), and Ds = 2*Dmin is this module's choice, not a 3GPP value:
//   HST-A 500 km/h Dmin=150 m  Ds=300 m (open corridor)
//   HST-B 300 km/h Dmin=10 m   Ds=20 m  (tunnel / station approach)
//   HST-C 350 km/h Dmin=100 m  Ds=200 m (typical mainline)

#include <cstdint>
#include <vector>

namespace ns3
{
namespace sagin
{

/// One snapshot of the train state.
struct HstSample
{
    double time_s;
    double x_m;       //!< along-track distance from t=0 origin
    double y_m;       //!< lateral (perpendicular to rail) — always 0
    double z_m;       //!< altitude (typically 0)
    double speed_mps; //!< constant in the 3GPP HST scenario
};

struct HstTrace
{
    double speed_kmh{500.0};
    double dmin_m{150.0};   //!< gNB perpendicular distance from rail
    double cellSpacing_m{300.0}; //!< Ds = 2*Dmin (module convention)
    double startPos_m{0.0}; //!< train along-track position at t=0
    std::vector<HstSample> samples;

    /// Speed in m/s (kmh / 3.6).
    double SpeedMps() const { return speed_kmh / 3.6; }

    /// Carrier-Doppler shift in Hz given (gNB position) and (carrier Hz).
    /// Sign convention: positive when train moves toward gNB.
    /// gNB sits at (gnb_x_m, dmin_m, 0) as in the 3GPP HST geometry.
    double DopplerHzAt(double t_s,
                        double gnb_x_m,
                        double carrierHz) const;
};

/// Builds an HST trace.
class HstTraceGenerator
{
  public:
    /// Generate `numSamples` evenly spaced over `duration_s` for the given
    /// (speed_kmh, dmin_m). Train starts at along-track position 0.
    static HstTrace Generate(double speed_kmh,
                              double dmin_m,
                              double duration_s,
                              uint32_t numSamples = 100);

    /// Preset: module HST-A (500 km/h, Dmin = 150 m).
    static HstTrace PresetTR38901_A(double duration_s,
                                      uint32_t numSamples = 100);
    /// Preset: module HST-B (300 km/h, Dmin = 10 m).
    static HstTrace PresetTR38901_B(double duration_s,
                                      uint32_t numSamples = 100);
    /// Preset: module HST-C (350 km/h, Dmin = 100 m).
    static HstTrace PresetTR38901_C(double duration_s,
                                      uint32_t numSamples = 100);
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_HST_TRACE_H
