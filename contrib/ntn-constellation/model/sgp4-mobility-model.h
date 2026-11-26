/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H
#define NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H

// Kepler + J2 secular propagator with a TLE-compatible interface
// (Roadmap §3 T5). Despite the class name, this is NOT a full SGP4
// implementation.
//
// The v2.1 baseline uses an analytic Kepler propagator with secular J2
// corrections to RAAN and argument-of-perigee. This is accurate to within
// a few km over hours of propagation for the typical near-circular LEO
// constellations this module targets (Walker-Delta Starlink shell 1,
// OneWeb, Kuiper). Full Vallado SGP4 — including atmospheric drag — is
// planned for Q4 2026 (T5 follow-on); until then the TLE drag term (B*)
// is parsed by TleRecord but UNUSED by this propagator. The
// classical-element interface is identical so the backend swap is
// transparent to consumers.
//
// Position is cached at 100 ms grain per roadmap §3 T5 to avoid
// re-evaluating Kepler's equation on every DoGetPosition() call from
// densely scheduled events.
//
// Two coordinate-system surfaces:
//   GetPosition() / DoGetPosition() returns ECEF metres (rotates with
//     Earth at omega = 7.2921159e-5 rad/s). This is what ns-3 mobility
//     consumers (link budgets, mobility-aware xApps) expect.
//   GetEciPosition() returns the inertial ECI position for ground-track
//     tooling.

#include "orbital-elements.h"

#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/mobility-model.h"
#include "ns3/nstime.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace ns3
{
namespace ntncon
{

/// Opaque Vallado-SGP4 state (full elsetrec), defined in the .cc so the public
/// header stays free of the satellite-module sgp4 headers.
struct ValladoState;

/// NT-03: derives from GeocentricConstantPositionMobilityModel rather than
/// MobilityModel directly.
///
/// The satellite still propagates its own orbit; nothing about that changes and
/// GetPosition() still returns live ECEF metres. The base class is adopted for
/// its INTERFACE: ns-3's ThreeGppChannelModel refuses to evaluate any of the
/// TR 38.811 NTN scenarios unless both endpoints DynamicCast to
/// GeocentricConstantPositionMobilityModel
/// (three-gpp-channel-model.cc, "Mobility Models needs to be of type Geocentric
/// for NTN scenarios"), and it reads the elevation angle and the
/// satellite-versus-HAPS decision off GetGeographicPosition().z.
///
/// Without this the toolkit could not use the NTN cluster tables at all: the
/// small-scale plane that produces the array gain and the fading behind every
/// measured SINR ran terrestrial UMa/UMi street-canyon statistics at 600 to
/// 2000 km, with hand-rolled TR 38.811 large-scale terms chained on top - two
/// different standards on one link. Setting the scenario string alone would
/// have aborted the run at the first channel realization.
///
/// The geographic accessors below are overridden to derive from the live
/// propagated ECEF position, so they track the orbit instead of the constant
/// the base class would otherwise store.
class Sgp4MobilityModel : public GeocentricConstantPositionMobilityModel
{
  public:
    static TypeId GetTypeId();
    /// ns-3.48: copy for MobilityModel::Copy() (spectrum wraparound); keeps the dynamics.
    Ptr<MobilityModel> Copy() const override
    {
        return CreateObject<Sgp4MobilityModel>(*this);
    }
    Sgp4MobilityModel();
    ~Sgp4MobilityModel() override = default;

    /// Install classical elements directly.
    void SetElements(const KeplerianElements& elements);

    /// Install classical elements and propagate them with Vallado SGP4: the
    /// elements are written as a synthesized TLE (MakeTleFromElements) and
    /// loaded through SetTle(). Use this where a generated shell must run SGP4;
    /// SetElements() keeps the Kepler + J2 path. Returns false (and leaves the
    /// Kepler + J2 elements installed) if the TLE cannot be formed.
    bool SetElementsSgp4(const KeplerianElements& elements);
    /// Convenience: parse a TLE record and install it.
    /// Returns false on parse failure.
    bool SetTle(const TleRecord& tle);

    /// Select the full Vallado SGP4 propagator (drag/B* included, the default since
    /// TWIN-01) or, with false, the Kepler + J2-secular backend. Vallado mode requires a TLE-sourced
    /// orbit (raw lines), so call SetTle() with a real TLE; Walker-generated
    /// classical elements (SetElements) keep the fast Kepler+J2 path. The order
    /// of SetUseVallado()/SetTle() does not matter — init is (re)done as needed.
    void SetUseVallado(bool on);
    /// True once a TLE has been loaded into the Vallado propagator.
    bool IsValladoReady() const;

    /// TWIN-01: which propagator is ACTUALLY running.
    ///
    /// IsValladoReady() answers whether the SGP4 state is initialised, which is
    /// not the same question and reads as though it were - it stays true after
    /// SetUseVallado(false). This one is the propagator in effect: true means
    /// GetPosition() comes from SGP4, false means the analytic Kepler + J2
    /// path. Anything comparing the two must assert on this, or it risks
    /// comparing SGP4 against itself.
    bool IsUsingSgp4() const;

    /// Anchor simulation time zero at an absolute UTC instant (Unix
    /// seconds). By default time zero is the element set's own epoch, which
    /// is what a Walker shell built from one epoch wants. Satellites loaded
    /// from real element sets each carry their own epoch, so a constellation
    /// of them is only consistent when every satellite is anchored to the
    /// same instant.
    void SetSimulationEpochUnix(double unixS) { m_simEpochUnixS = unixS; m_cacheValid = false; }
    double SimBaseUnix() const
    {
        return std::isnan(m_simEpochUnixS) ? m_elements.epoch_unix_s : m_simEpochUnixS;
    }

    /// Read-only access to the installed elements.
    const KeplerianElements& GetElements() const { return m_elements; }
    /// Element-set provenance for datasets (also exposed as read-only attributes
    /// ElementEpochUnix, NoradId and FromRealTle, so helpers that cannot link this
    /// module read it through the attribute system).
    double GetElementEpochUnix() const { return m_elements.epoch_unix_s; }
    uint32_t GetNoradId() const { return m_elements.norad_id; }
    bool GetFromRealTle() const { return m_fromRealTle; }

    /**
     * UT1 - UTC (s, IERS Bulletin A "DUT1", |value| < 0.9 s) used for the Earth rotation angle of
     * the TEME -> Earth-fixed rotation, for every Sgp4MobilityModel of the run. Default 0: the
     * rotation then uses UTC, which leaves an along-rotation error of DUT1 x 7.29e-5 rad/s
     * (about 0.09 s in late 2026: 40 m at LEO, 260 m at GEO). Set it before the run starts;
     * positions already cached keep the old value until their cache expires.
     * @param seconds UT1 - UTC
     */
    static void SetUt1MinusUtc(double seconds);
    /// @return the UT1 - UTC value in use (s)
    static double GetUt1MinusUtc();

    /// Cache granularity for the position lookup. Default 100 ms.
    void SetCacheGranularity(Time grain) { m_cacheGrain = grain; }

    /// Inertial-frame position (ECI metres) at sim time.
    Vector GetEciPosition() const;

    /// Earth-fixed position (ECEF metres) at sim time — what ns-3 link
    /// budget consumers expect.
    Vector GetEcefPosition() const;

    /// ECEF velocity (m/s).
    Vector GetEcefVelocity() const;

    /// ECEF position and velocity at Simulator::Now() + \p offset, from the
    /// same propagator GetPosition() uses, without touching the position cache.
    ///
    /// A study of prediction error needs the orbit the satellite will actually
    /// fly, evaluated at a future instant, to compare against what a predictor
    /// expected. Reading GetPosition() after the simulator has advanced gives
    /// the same answer but only one instant at a time, and only once the
    /// instant has passed. This returns it on demand.
    void GetEcefStateAt(Time offset, Vector& posEcef, Vector& velEcef) const;

    /// Geodetic latitude / longitude / altitude (deg, deg, m above WGS-84
    /// reference ellipsoid).
    void GetGeodetic(double& lat_deg, double& lon_deg, double& alt_m) const;
    /**
     * \brief Which propagator this object will actually use, as a short tag.
     *
     * CVC-12. The class is named Sgp4MobilityModel, but it only runs Vallado
     * SGP4 when it was initialized from a TLE. Initialized from Keplerian
     * elements, which is what every Walker-generated shell does, it propagates
     * with a Kepler + J2 secular model instead. Both are legitimate; naming
     * only the first one in a caption is not. Scenarios and health records read
     * this so the claim in the output matches the code that produced it.
     *
     * \return "sgp4-vallado" or "kepler-j2".
     */
    std::string GetPropagatorName() const;

    /// CVC-12: Vallado propagation calls attempted (0 on the Kepler+J2 path).
    uint64_t GetSgp4CallCount() const { return m_sgp4Calls; }

    /**
     * \brief Times the Vallado path failed mid-run and fell back to Kepler+J2.
     *
     * The fallback used to be an NS_LOG_WARN, which the optimized build
     * compiles out, so a run could silently stop being SGP4 partway through a
     * pass and still report itself as SGP4. Non-zero here means exactly that
     * happened.
     */
    uint64_t GetSgp4FallbackCount() const { return m_sgp4Fallbacks; }


    /// Elevation angle (deg) from a ground station at (lat_deg, lon_deg)
    /// at the current sim time. Returns a value < 0 when the satellite is
    /// below the horizon.
    double GetElevationDeg(double gs_lat_deg, double gs_lon_deg) const;

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    // NT-03: geographic view of the live orbit, for the TR 38.811 NTN channel.
    Vector DoGetGeographicPosition() const override;
    Vector DoGetGeocentricPosition() const override;

    /// Propagate to absolute time `unix_s` and fill ECI position/velocity.
    void Propagate(double unix_s, Vector& pos_eci, Vector& vel_eci) const;

    /// (Re)initialise the Vallado elsetrec from the stored TLE lines.
    void InitVallado();

    /// Convert ECI to ECEF for the given absolute time.
    static Vector EciToEcef(const Vector& eci, double unix_s);

    /// Cache lookup keyed on the rounded-down sim time.
    void EnsureCache() const;

    KeplerianElements m_elements;
    bool m_fromRealTle{false}; //!< true when the elements came from a real TLE (SetTle), not a generated shell
    Time m_cacheGrain{MilliSeconds(100)};

    // Full Vallado SGP4 backend (opaque; reuses the satellite module's proven
    // sgp4unit). Null until SetTle() is called; active only when m_useVallado.
    /// TWIN-01: default TRUE, so a TLE actually gets SGP4.
    ///
    /// This was false, which meant SetTle() parsed the TLE into Keplerian
    /// elements and then propagated them with an analytic Kepler + J2-secular
    /// model unless the caller ALSO remembered SetUseVallado(true). Nothing in
    /// the tree did. So a class named Sgp4MobilityModel, fed a real TLE, was
    /// not running SGP4 - while the Python twin
    /// (ntn_constellation/propagator.py) runs genuine Satrec.sgp4. The digital
    /// twin's stated premise is that "twin and sim propagate identical orbits";
    /// they did not.
    ///
    /// Measured for the ISS TLE over the exporter's 45-minute horizon, the two
    /// propagators separate by 5.2 to 11.0 km - roughly a second of along-track
    /// lag at orbital speed, enough to move an argmax-elevation crossover and
    /// therefore every handover instant the twin exports.
    ///
    /// SetElements() callers are unaffected: with no TLE there is nothing for
    /// SGP4 to initialise from, and the Kepler + J2 path still runs. Use
    /// IsValladoReady() to find out which propagator a given object is on.
    bool m_useVallado{true};
    std::shared_ptr<ValladoState> m_vallado;
    /// CVC-12: how many times the Vallado path was asked and answered.
    mutable uint64_t m_sgp4Calls{0};
    /// CVC-12: how many times it failed mid-run and silently fell back.
    mutable uint64_t m_sgp4Fallbacks{0};

    double m_simEpochUnixS{std::numeric_limits<double>::quiet_NaN()}; //!< see SetSimulationEpochUnix

    // Cache.
    mutable bool m_cacheValid{false};
    mutable Time m_cacheTime;
    /// Epoch anchor (SimBaseUnix) the cache was propagated with (EnsureCache).
    mutable double m_cacheBaseUnixS{std::numeric_limits<double>::quiet_NaN()};
    mutable Vector m_cachedEci;
    mutable Vector m_cachedEciVel;
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_CONSTELLATION_SGP4_MOBILITY_MODEL_H
