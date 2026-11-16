// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
//
// NtnEarthBlockageLossModel: Earth occultation and elevation mask on any
// radio link, as a chainable ns-3 PropagationLossModel.
//
// Friis and the TR 38.811 excess terms are frame independent: a satellite
// below a terminal's horizon, or two satellites whose line of sight crosses
// the Earth, still exchanged power through the planet. This model closes that
// gap with geometry only:
//   - ground or low-altitude end (below GroundAltitudeThreshold): the link is
//     blocked when the elevation of the other end, measured about the WGS-84
//     ellipsoid normal at the ground end, is below MinElevationDeg
//     (TR 38.821 clause 6.1 evaluates service links above a minimum
//     elevation; physically the link is blocked below 0 deg);
//   - both ends above that altitude (ISL, HAPS to satellite): the link is
//     blocked when the straight segment between them passes closer to the
//     ellipsoid than MinGrazingAltitude (limb test).
// Links whose both ends are below that altitude (terminal to terminal) are
// terrestrial paths and are not judged.
// Endpoints are read in ECEF: GetGeocentricPosition() for geocentric models
// (Sgp4MobilityModel, NtnTr38811MobilityModel, ns-3 Geocentric), GetPosition()
// otherwise. A scenario whose positions are not geocentric (an endpoint less
// than half an Earth radius from the origin) is passed through unchanged and
// counted, so flat-frame scenarios are never blocked by accident.

#ifndef NTN_EARTH_BLOCKAGE_LOSS_MODEL_H
#define NTN_EARTH_BLOCKAGE_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/vector.h"

#include <cstdint>

namespace ns3
{

/**
 * \ingroup ntn-traffic
 * \brief Earth-occultation / elevation-mask loss for NTN links (geometry only).
 */
class NtnEarthBlockageLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    NtnEarthBlockageLossModel();
    ~NtnEarthBlockageLossModel() override;

    /// Elevation (deg) of \p other seen from \p ground about the WGS-84
    /// ellipsoid normal at \p ground (both ECEF metres). Negative below the
    /// horizon.
    static double GeodeticElevationDeg(const Vector& groundEcef, const Vector& otherEcef);

    /// True when the link between the two ECEF points is blocked under this
    /// model's current mask settings. \p frameOk is set false when the points
    /// are not geocentric (pass-through case).
    bool IsBlocked(const Vector& aEcef, const Vector& bEcef, bool& frameOk) const;

    /// Number of link evaluations that were blocked / passed / not geocentric.
    uint64_t GetBlockedCount() const { return m_blocked; }
    uint64_t GetClearCount() const { return m_clear; }
    uint64_t GetNonGeocentricCount() const { return m_nonGeocentric; }

    /// ECEF position of a mobility model (geocentric accessor when available).
    static Vector EcefOf(Ptr<MobilityModel> m);

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    double m_minElevationDeg{0.0};     //!< elevation mask at a ground end (deg)
    double m_groundAltitudeM{5000.0};  //!< ends below this altitude use the mask
    double m_minGrazingAltM{0.0};      //!< limb clearance for air/space links (m)
    double m_blockedLossDb{300.0};     //!< loss applied to a blocked link (dB)
    mutable uint64_t m_blocked{0};
    mutable uint64_t m_clear{0};
    mutable uint64_t m_nonGeocentric{0};
};

} // namespace ns3

#endif // NTN_EARTH_BLOCKAGE_LOSS_MODEL_H
