/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Realistic NTN UE mobility helper (3GPP TR 38.811 v15.4.0 §6.1.1.1).
 *
 * Replaces the toolkit's earlier ad-hoc lat/lon/velocity model with
 * a per-class motion generator whose speeds, acceleration ranges, and
 * spatial constraints are sourced from authoritative references:
 *
 *   - 3GPP TR 38.811 §6.1.1.1 (NTN UE classes & speed brackets)
 *   - 3GPP TR 38.821 §6.1.1   (Rel-17 NR-NTN UE refinement)
 *   - 3GPP TR 38.913 §6.1.5   (5G eMBB/URLLC mobility targets)
 *   - 3GPP TR 38.901 §7.2     (HST geometry)
 *   - 3GPP TR 36.763          (NB-IoT NTN — stationary UE assumption)
 *   - ITU-R M.1371-5          (AIS maritime data structure)
 *   - IMO Resolution A.857(20)(Maritime Traffic Separation Schemes)
 *   - ICAO Doc 4444 PANS-ATM  (airway / great-circle waypoint structure)
 *
 * Six UE classes are provided (matching TR 38.811 Table 6.1.1.1):
 *   HANDHELD_PEDESTRIAN, HANDHELD_STATIC, VEHICULAR, HIGH_SPEED_TRAIN,
 *   MARITIME_VESSEL, AVIATION_COMMERCIAL, IOT_FIXED.
 *
 * Each class provides: an initial geographic distribution (regions
 * where that class is plausible — e.g., maritime UEs spawn over
 * shipping lanes, aviation UEs along airway waypoints, HST along
 * straight tracks), a per-class speed PDF, a heading-evolution rule,
 * and a motion-bound clamp.
 */

#ifndef NTN_REALISTIC_MOBILITY_H
#define NTN_REALISTIC_MOBILITY_H

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \brief 3GPP TR 38.811 §6.1.1.1 NTN UE classes.
 */
enum NtnUeClass
{
    /** Static handheld; ~50% of "handheld" population in TR 38.811. */
    HANDHELD_STATIC = 0,
    /** Walking handheld @ 0.5–3 m/s (3GPP baseline 3 km/h ≈ 0.83 m/s). */
    HANDHELD_PEDESTRIAN = 1,
    /** Passenger car / van; TR 38.811 baseline 100 km/h, max 250 km/h. */
    VEHICULAR = 2,
    /** HST (linear track) @ 300 km/h baseline, up to 500 km/h. */
    HIGH_SPEED_TRAIN = 3,
    /** Cargo / tanker @ 12–25 knots (≈6–13 m/s); IMO TSS lanes. */
    MARITIME_VESSEL = 4,
    /** Commercial aircraft @ FL350, ~240 m/s ground speed. */
    AVIATION_COMMERCIAL = 5,
    /** NB-IoT NTN; stationary with sub-mm/s drift (TR 36.763). */
    IOT_FIXED = 6,
};

/**
 * \brief Per-UE realistic-mobility state.
 *
 * Used by Walker-Star handover scenarios where each UE is propagated
 * with class-specific motion. Position is in geodetic (lat, lon),
 * velocities in topocentric (East, North) m/s. Aircraft additionally
 * carry an altitude in metres for elevation-angle accuracy.
 */
struct RealisticUe
{
    uint32_t id{0};

    NtnUeClass ueClass{HANDHELD_STATIC};
    std::string className;        //!< human-readable for trace files

    // --- Geodetic position ---
    double lat{0.0};              //!< degrees
    double lon{0.0};              //!< degrees
    double altitude_m{0.0};       //!< 0 for ground UEs, ~11000 for aviation

    // --- Topocentric ENU velocity ---
    double vEast{0.0};            //!< m/s
    double vNorth{0.0};           //!< m/s
    double vUp{0.0};              //!< m/s (only used for climb/descent)

    // --- Heading-evolution state ---
    double heading_rad{0.0};      //!< current heading (0 = north, +π/2 = east)
    double headingChangeUntil_s{0.0}; //!< time after which heading is re-rolled

    // --- Waypoint state (used by HST, maritime, aviation) ---
    std::vector<std::pair<double, double>> waypoints; //!< (lat, lon)
    size_t nextWaypoint{0};

    // --- Speed bookkeeping ---
    double speed_mps{0.0};        //!< instantaneous |v|
};

/**
 * \brief Configuration knobs for the mobility generator.
 *
 * Distributions over UE classes — populate with values that sum to 1.0.
 * Each scenario (urban handover, maritime backhaul, mixed coverage,
 * etc.) gets its own profile.
 */
struct MobilityProfile
{
    double pStatic{0.30};
    double pPedestrian{0.20};
    double pVehicular{0.30};
    double pHst{0.05};
    double pMaritime{0.05};
    double pAviation{0.05};
    double pIot{0.05};
};

/**
 * \brief Built-in scenario profiles (preset MobilityProfile values).
 *
 * Each preset is calibrated for a typical NTN deployment per the
 * 3GPP NTN study and recent IEEE Access / TVT NTN papers.
 */
class NtnMobilityScenarios
{
  public:
    /**
     * Mixed-coverage continental scenario (default for HO research).
     * Roughly tracks the Iridium / Starlink consumer-mix population
     * model: majority ground (handheld + vehicular), with rare
     * specialty users (aviation, maritime, IoT).
     */
    static MobilityProfile MixedContinental();

    /** Maritime-heavy (open-sea + coastal): cargo, fishing, cruise. */
    static MobilityProfile Maritime();

    /** Aviation corridor (NAT-OTS over Atlantic): aircraft + ground. */
    static MobilityProfile AeronauticalCorridor();

    /** Urban dense (no maritime/aviation): ped + vehicular + IoT. */
    static MobilityProfile UrbanDense();

    /** IoT field (ag/oil-and-gas): mostly static IoT + occasional vehicle. */
    static MobilityProfile IotField();
};

/**
 * \brief Generates UE populations with realistic per-class mobility.
 *
 * Usage:
 * \code{.cpp}
 *   ns3::NtnRealisticMobilityHelper helper(rngSeed);
 *   auto profile = ns3::NtnMobilityScenarios::MixedContinental();
 *   auto ues = helper.GenerateUes(/n_ues=/30, profile,
 *                                 /minLat=/25.0, /maxLat=/65.0,
 *                                 /minLon=/-20.0, /maxLon=/40.0);
 *   for (double t = 0; t < simTime; t += dt) {
 *       for (auto& ue : ues) helper.AdvanceUe(ue, dt);
 *   }
 * \endcode
 */
class NtnRealisticMobilityHelper
{
  public:
    /** \param seed RNG seed for reproducibility (ns-3 RngRun-aligned). */
    explicit NtnRealisticMobilityHelper(uint64_t seed = 1);

    /**
     * \brief Generate a UE population with realistic per-class motion.
     *
     * Class assignment follows the supplied MobilityProfile. Initial
     * positions for maritime UEs are biased toward mid-ocean tiles in
     * the requested bounding box; aviation UEs are placed on
     * great-circle airway waypoints; HST UEs sit on linear track
     * segments. Other classes are uniform over the box.
     */
    std::vector<RealisticUe> GenerateUes(uint32_t n,
                                          const MobilityProfile& profile,
                                          double minLat, double maxLat,
                                          double minLon, double maxLon);

    /**
     * \brief Advance one UE by dt seconds applying its class's motion rule.
     *
     * Static / IoT: no motion (with optional Gaussian sub-mm jitter).
     * Pedestrian: random-walk with heading change every 5–15 s.
     * Vehicular: Gauss-Markov heading with τ_heading ≈ 60 s.
     * HST: constant velocity along linear track (waypoint-driven).
     * Maritime: constant heading (great-circle), turn at waypoint.
     * Aviation: constant velocity at FL350, follows airway waypoints.
     */
    void AdvanceUe(RealisticUe& ue, double dt_s);

  private:
    void InitializeStatic(RealisticUe& ue);
    void InitializePedestrian(RealisticUe& ue);
    void InitializeVehicular(RealisticUe& ue);
    void InitializeHst(RealisticUe& ue);
    void InitializeMaritime(RealisticUe& ue);
    void InitializeAviation(RealisticUe& ue);
    void InitializeIot(RealisticUe& ue);

    void AdvanceWalk(RealisticUe& ue, double dt_s);
    void AdvanceVehicular(RealisticUe& ue, double dt_s);
    void AdvanceWaypoint(RealisticUe& ue, double dt_s);

    static void GeoStep(RealisticUe& ue, double dt_s);

    std::mt19937_64 m_rng;
};

} // namespace ns3

#endif // NTN_REALISTIC_MOBILITY_H
