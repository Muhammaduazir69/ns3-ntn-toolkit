/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NtnTr38811MobilityModel — a REAL ns-3 MobilityModel implementing the 3GPP
 * TR 38.811 §6.1.1.1 NTN UE classes (handheld static/pedestrian, vehicular,
 * HST, maritime vessel, commercial aviation, fixed IoT).
 *
 * This closes the toolkit's last mobility-architecture gap: the per-class
 * motion physics existed only as a struct generator (NtnRealisticMobilityHelper
 * / RealisticUe), which could log trajectories but could NOT drive an ns-3
 * radio stack. As a real MobilityModel the TR 38.811 motion now feeds the
 * SpectrumChannel path-loss geometry, the per-pass Doppler, the TTE estimator,
 * and every other consumer of node mobility — exactly like the satellite side's
 * Sgp4MobilityModel.
 *
 * Frame: position is WGS84 geodetic internally and returned in ECEF metres
 * (DoGetPosition), the same frame ntn-constellation's Sgp4MobilityModel uses,
 * so a UE under this model and a satellite under SGP4 share one geometry and
 * Friis/elevation/Doppler all come out physically right. Velocity is the
 * class's topocentric ENU velocity rotated into ECEF.
 *
 * Propagation is lazy (like Sgp4MobilityModel): each DoGetPosition() advances
 * the TR 38.811 motion state from the last evaluation to Simulator::Now() in
 * sub-steps of at most UpdateStep (default 1 s).
 *
 * References: 3GPP TR 38.811 §6.1.1.1, TR 38.821 §6.1.1, TR 38.901 §7.2 (HST),
 * TR 36.763 (IoT-NTN), ITU-R M.1371-5 (AIS), ICAO Doc 4444 (airways).
 */

#ifndef NTN_TR38811_MOBILITY_MODEL_H
#define NTN_TR38811_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ntn-realistic-mobility.h"

#include <memory>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief Shared ECEF geometry utilities for NTN examples (UE under
 *        NtnTr38811MobilityModel + satellite under Sgp4MobilityModel).
 */
namespace ntngeo
{
/// WGS84 geodetic (deg, deg, m) -> ECEF metres.
Vector GeodeticToEcef(double latDeg, double lonDeg, double altM);

/// ECEF metres -> WGS84 geodetic (deg, deg, m).
void EcefToGeodetic(const Vector& ecef, double& latDeg, double& lonDeg, double& altM);

/// Topocentric ENU velocity (m/s) at (lat,lon) -> ECEF velocity (m/s).
Vector EnuVelocityToEcef(double latDeg, double lonDeg, double vEast, double vNorth, double vUp);

/// Elevation angle (deg) of `satEcef` as seen from `ueEcef` (local-up = ue/|ue|).
double ElevationDeg(const Vector& ueEcef, const Vector& satEcef);

/// Slant range (m) between two ECEF points.
double SlantRangeM(const Vector& a, const Vector& b);

/// Doppler shift (Hz) of a carrier `freqHz` for the relative motion of
/// sat (pos/vel) seen from ue (pos/vel); positive while approaching.
double DopplerHz(const Vector& ueEcef,
                 const Vector& ueVelEcef,
                 const Vector& satEcef,
                 const Vector& satVelEcef,
                 double freqHz);

/// ECEF point -> local ENU metres about the geodetic origin (lat0, lon0, alt0).
Vector EcefToEnu(const Vector& ecef, double lat0Deg, double lon0Deg, double alt0M);

/// Local ENU metres about (lat0, lon0, alt0) -> ECEF point.
Vector EnuToEcef(const Vector& enu, double lat0Deg, double lon0Deg, double alt0M);
} // namespace ntngeo

/**
 * \ingroup ntn-cho
 * \brief Projects any ECEF mobility model (e.g. ntn-constellation's
 *        Sgp4MobilityModel) into a local ENU frame about a geodetic origin.
 *
 * Scenarios whose platform mobility lives in a local tangent plane (the SAGIN
 * OpenSky/AIS/HST traces, SUMO vehicle replays, city-scale layouts) cannot mix
 * frames with an ECEF satellite. This wrapper gives them REAL SGP4 orbital
 * dynamics: position/velocity of the wrapped model are rotated into the ENU
 * frame at the scenario origin, so the satellite rises, passes and sets with
 * genuine orbital geometry while the rest of the scenario stays in metres.
 */
class NtnEnuProjectionMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    NtnEnuProjectionMobilityModel();
    ~NtnEnuProjectionMobilityModel() override;

    /// The ECEF source model to project (e.g. a Sgp4MobilityModel).
    void SetSource(Ptr<MobilityModel> source) { m_source = source; }
    /// The geodetic origin of the scenario's local ENU frame.
    void SetReference(double lat0Deg, double lon0Deg, double alt0M);

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    Ptr<MobilityModel> m_source;
    double m_lat0{0.0};
    double m_lon0{0.0};
    double m_alt0{0.0};
};

/**
 * \ingroup ntn-cho
 * \brief 3GPP TR 38.811 §6.1.1.1 UE-class mobility as a real ns-3
 *        MobilityModel (ECEF frame, SGP4-compatible).
 */
class NtnTr38811MobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    NtnTr38811MobilityModel();
    ~NtnTr38811MobilityModel() override;

    /**
     * \brief Bind this model to a generated TR 38.811 UE state and the shared
     *        motion engine that advances it (class-specific motion rules).
     */
    void SetUeState(const RealisticUe& ue, std::shared_ptr<NtnRealisticMobilityHelper> engine);

    /// Maximum sub-step used when lazily advancing the motion state.
    void SetUpdateStep(Time step) { m_step = step; }

    NtnUeClass GetUeClass() const { return m_ue.ueClass; }
    const std::string& GetClassName() const { return m_ue.className; }
    /// Current WGS84 geodetic position (advances the state to Now).
    void GetGeodetic(double& latDeg, double& lonDeg, double& altM) const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    /// Advance the TR 38.811 motion state from m_lastUpdate to Simulator::Now().
    void Advance() const;

    mutable RealisticUe m_ue;
    std::shared_ptr<NtnRealisticMobilityHelper> m_engine;
    mutable Time m_lastUpdate{Seconds(0)};
    Time m_step{Seconds(1.0)};
};

/**
 * \ingroup ntn-cho
 * \brief Installs NtnTr38811MobilityModel instances on a NodeContainer from a
 *        TR 38.811 scenario MobilityProfile (class mix), with initial positions
 *        drawn inside a geographic bounding box.
 */
class NtnTr38811MobilityHelper
{
  public:
    explicit NtnTr38811MobilityHelper(uint64_t seed = 1);

    /**
     * \brief Generate one UE per node (class mix from \p profile, positions in
     *        the lat/lon box) and aggregate a NtnTr38811MobilityModel on each.
     * \return the installed models, indexed like \p nodes.
     */
    std::vector<Ptr<NtnTr38811MobilityModel>> Install(NodeContainer nodes,
                                                      const MobilityProfile& profile,
                                                      double minLat,
                                                      double maxLat,
                                                      double minLon,
                                                      double maxLon);

  private:
    std::shared_ptr<NtnRealisticMobilityHelper> m_engine;
};

} // namespace ns3

#endif // NTN_TR38811_MOBILITY_MODEL_H
