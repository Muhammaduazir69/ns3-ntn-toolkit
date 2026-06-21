/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * NtnGeodeticFixedMobilityModel — a ground terminal anchored at a true WGS84
 * geodetic location (lat, lon, alt), reported in the ECEF frame.
 *
 * Why this exists. Across the toolkit, fixed ground terminals, gateways and
 * RIS panels were placed with ConstantPositionMobilityModel at hand-picked
 * local-ENU metres. That is fine for a single-cell radio study but it is wrong
 * for two things the 3D / NetSimulyzer / Cesium work needs:
 *
 *   1. Geometry against ECEF satellites. A terminal placed at arbitrary ENU
 *      metres has no real latitude/longitude, so its elevation angle, slant
 *      range and Doppler against an ECEF Sgp4MobilityModel satellite are not
 *      physically anchored. Anchoring it at a real (lat, lon, alt) and
 *      returning ECEF makes Friis / elevation / Doppler come out right, exactly
 *      like NtnTr38811MobilityModel does for moving UEs.
 *
 *   2. Placement on a globe. A scene recorder cannot put a node on the Earth if
 *      it only knows local metres. A geodetic anchor (GetGeodetic) lets the
 *      Cesium globe and the NetSimulyzer Earth-sphere place the terminal where
 *      it really is.
 *
 * Frame note (deliberately honest). A point at fixed lat/lon/alt is STATIONARY
 * in ECEF — ECEF rotates with the Earth, so the terminal does not move and its
 * ECEF velocity is zero. That is the correct velocity for ECEF-frame Doppler
 * (the satellite's GetEcefVelocity already carries the -omega x r transport
 * term). For inertial-frame tooling or an ECI 3D view, SetReportFrame(Eci)
 * makes DoGetPosition return the diurnal-circle ECI position R_z(omega t)·ECEF
 * and a non-zero ECI velocity; this is OFF by default because the toolkit's
 * satellites report ECEF.
 *
 * References: WGS84 (NIMA TR8350.2), IERS Earth rotation rate
 * omega = 7.2921159e-5 rad/s.
 */

#ifndef NTN_GEODETIC_FIXED_MOBILITY_MODEL_H
#define NTN_GEODETIC_FIXED_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"

namespace ns3
{

/**
 * \ingroup ntn-cho
 * \brief A ground node fixed at a WGS84 geodetic location, reported in ECEF
 *        (default) or ECI metres.
 */
class NtnGeodeticFixedMobilityModel : public MobilityModel
{
  public:
    /// Frame in which DoGetPosition()/DoGetVelocity() report.
    enum ReportFrame
    {
        Ecef, //!< Earth-fixed: position constant, velocity zero (default).
        Eci   //!< Inertial: position traces the diurnal circle, velocity != 0.
    };

    static TypeId GetTypeId();
    NtnGeodeticFixedMobilityModel();
    ~NtnGeodeticFixedMobilityModel() override;

    /// Anchor the terminal at a WGS84 geodetic location.
    void SetGeodetic(double latDeg, double lonDeg, double altM);

    /// Read back the geodetic anchor (for globe placement / scene export).
    void GetGeodetic(double& latDeg, double& lonDeg, double& altM) const;

    /// Choose the reporting frame. Default ECEF (matches the satellites).
    void SetReportFrame(ReportFrame frame) { m_frame = frame; }
    ReportFrame GetReportFrame() const { return m_frame; }

    /// Earth-rotation epoch offset (seconds) used only in ECI mode; the ECI
    /// position is R_z(omega * (epochOffset + Now)) applied to the ECEF anchor.
    void SetEpochOffset(Time offset) { m_epochOffset = offset; }

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    double m_latDeg{0.0};
    double m_lonDeg{0.0};
    double m_altM{0.0};
    Vector m_ecef{0.0, 0.0, 0.0}; //!< cached ECEF anchor
    ReportFrame m_frame{Ecef};
    Time m_epochOffset{Seconds(0)};
};

} // namespace ns3

#endif // NTN_GEODETIC_FIXED_MOBILITY_MODEL_H
