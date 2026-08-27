/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-visibility-index - exact argmax-elevation lookup without the O(S x U) scan.
//
// Why this exists (audit WF-10). Serving-satellite selection across the toolkit
// is an unindexed double loop: for every terminal, on every tick, compute the
// elevation to every satellite and keep the largest
// (ntn-scalability-load-regimes.cc:356-380 is the clearest instance). That is
// O(satellites x terminals x ticks). A 1584-satellite shell with 1000 terminals
// over a 600 s run is close to a billion elevation evaluations, and the stress
// report records the consequence: "each sim-second costs ~3-10 s wall".
//
// The fix has to be EXACT. A serving-cell choice that differs from the brute
// force changes every downstream KPI, so an approximate nearest-neighbour index
// would trade a scale limit for a correctness one. This is a branch and bound
// that provably returns the same satellite.
//
// The geometry that makes it work. With the terminal at radius R and unit up
// vector u, a satellite at radius rho and central angle gamma from u:
//
//     sin(el) = (rho*cos(gamma) - R) / sqrt(rho^2 + R^2 - 2*rho*R*cos(gamma))
//
// For fixed rho this decreases monotonically in gamma - a satellite further
// around the Earth is lower in the sky. For fixed gamma it increases in rho - a
// higher satellite over the same ground point is higher in the sky. So for a
// bucket of satellites whose central angles are all at least gammaMin and whose
// radii are all at most rhoMax, sin(el) is bounded above by the value at
// (rhoMax, gammaMin). Visit buckets in descending bound and stop as soon as the
// bound cannot beat the best found: everything skipped was provably worse.

#ifndef NTN_VISIBILITY_INDEX_H
#define NTN_VISIBILITY_INDEX_H

#include "ns3/vector.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ns3
{
namespace ntncon
{

/**
 * \ingroup ntn-constellation
 * \brief Exact highest-elevation satellite lookup over a bucketed sky.
 *
 * Rebuild once per tick after the orbits are propagated, then query per
 * terminal. Returns the same index and elevation the brute-force scan returns,
 * by construction rather than by tolerance.
 */
class NtnVisibilityIndex
{
  public:
    /// Angular bucket resolution in degrees. Smaller buckets prune harder and
    /// cost more to build; 10 degrees is a reasonable balance for LEO shells.
    explicit NtnVisibilityIndex(double bucketDeg = 10.0);

    /// (Re)build from the current ECEF satellite positions. O(S).
    void Build(const std::vector<Vector>& satEcef);

    /// Highest-elevation satellite for a terminal at `ueEcef`.
    ///
    /// \param[out] bestElDeg elevation of the returned satellite, degrees.
    /// \return index into the vector passed to Build(), or SIZE_MAX if empty.
    std::size_t BestElevation(const Vector& ueEcef, double& bestElDeg) const;

    /// Satellites actually evaluated by the last BestElevation() call. Compare
    /// against the satellite count to see how much of the sky was skipped.
    std::size_t LastEvaluated() const { return m_lastEvaluated; }

    std::size_t SatelliteCount() const { return m_positions.size(); }

  private:
    struct Bucket
    {
        // WF-10: the centre is stored as a UNIT VECTOR, not lat/lon.
        //
        // The first version kept lat/lon and rebuilt the direction with three
        // trig calls per bucket per query, then took an acos to get the angle
        // and sorted every bucket. With ~400 buckets that cost about as much as
        // the 1584-satellite scan it replaced: exact, 54x fewer elevation
        // evaluations, and only 1.1x faster in wall clock. The bound is now
        // pure arithmetic on a dot product.
        double cx{0.0};
        double cy{0.0};
        double cz{0.0};
        double maxRadiusM{0.0};   //!< largest |r| in this bucket
        /// cos and sin of the LARGEST angular deviation of any member from the
        /// stored centre, measured at Build().
        ///
        /// The first version assumed members lie within one bucket width of the
        /// centre. They do not: the centre is the centroid of member
        /// directions, not the bin centre, and a lat/lon bin spans more than
        /// one width along its diagonal. That bound was too tight, so buckets
        /// containing the true best were skipped - 12.6x faster and wrong,
        /// caught immediately by the brute-force oracle. Measuring the actual
        /// deviation is exact by construction and tighter than any guess.
        double cosMaxDev{1.0};
        double sinMaxDev{0.0};
        std::vector<std::size_t> members;
    };

    /// sin(elevation) for a satellite at radius rho and central angle gamma
    /// seen from a terminal at radius R. The bound and the exact value come
    /// from the same expression, which is what keeps them consistent.
    static double SinElevation(double rho, double R, double cosGamma);

    double m_bucketRad;
    /// cos and sin of one bucket width, so cos(gammaCentre - width) can be
    /// formed with the angle-difference identity instead of an acos.
    double m_cosHalf{1.0};
    double m_sinHalf{0.0};
    std::vector<Vector> m_positions;
    std::vector<Bucket> m_buckets;
    mutable std::size_t m_lastEvaluated{0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_VISIBILITY_INDEX_H
