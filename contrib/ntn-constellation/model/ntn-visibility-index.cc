/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-visibility-index.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{
namespace ntncon
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
} // namespace

NtnVisibilityIndex::NtnVisibilityIndex(double bucketDeg)
    : m_bucketRad(std::max(1.0, bucketDeg) * kPi / 180.0)
{
    m_cosHalf = std::cos(m_bucketRad);
    m_sinHalf = std::sin(m_bucketRad);
}

double
NtnVisibilityIndex::SinElevation(double rho, double R, double cosGamma)
{
    // sin(el) = (rho*cos(gamma) - R) / |d|, |d|^2 = rho^2 + R^2 - 2*rho*R*cos(gamma)
    const double d2 = rho * rho + R * R - 2.0 * rho * R * cosGamma;
    const double d = std::sqrt(std::max(d2, 1.0));
    return (rho * cosGamma - R) / d;
}

void
NtnVisibilityIndex::Build(const std::vector<Vector>& satEcef)
{
    m_positions = satEcef;
    m_buckets.clear();
    if (m_positions.empty())
    {
        return;
    }

    // Bucket by sub-satellite latitude/longitude. Longitude bands are widened
    // toward the poles so a bucket never spans more than m_bucketRad of arc,
    // which is what keeps the angular bound below tight AND valid.
    const int nLat = std::max(1, static_cast<int>(std::ceil(kPi / m_bucketRad)));
    struct Key
    {
        int iLat;
        int iLon;
        bool operator<(const Key& o) const
        {
            return iLat != o.iLat ? iLat < o.iLat : iLon < o.iLon;
        }
    };
    std::vector<std::pair<Key, std::size_t>> keyed;
    keyed.reserve(m_positions.size());

    for (std::size_t i = 0; i < m_positions.size(); ++i)
    {
        const Vector& p = m_positions[i];
        const double r = std::max(1.0, p.GetLength());
        const double lat = std::asin(std::max(-1.0, std::min(1.0, p.z / r)));
        const double lon = std::atan2(p.y, p.x);
        const int iLat = std::min(nLat - 1,
                                  static_cast<int>((lat + kPi / 2.0) / m_bucketRad));
        // Longitude bins per latitude band, at least one.
        const double bandLat = -kPi / 2.0 + (iLat + 0.5) * m_bucketRad;
        const double cosBand = std::max(0.05, std::cos(bandLat));
        const int nLon =
            std::max(1, static_cast<int>(std::ceil(2.0 * kPi * cosBand / m_bucketRad)));
        int iLon = static_cast<int>((lon + kPi) / (2.0 * kPi) * nLon);
        iLon = std::max(0, std::min(nLon - 1, iLon));
        keyed.push_back({{iLat, iLon}, i});
    }

    std::sort(keyed.begin(), keyed.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (std::size_t k = 0; k < keyed.size();)
    {
        std::size_t j = k;
        Bucket b;
        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        while (j < keyed.size() && !(keyed[k].first < keyed[j].first) &&
               !(keyed[j].first < keyed[k].first))
        {
            const std::size_t idx = keyed[j].second;
            const Vector& p = m_positions[idx];
            const double r = std::max(1.0, p.GetLength());
            sx += p.x / r;
            sy += p.y / r;
            sz += p.z / r;
            b.maxRadiusM = std::max(b.maxRadiusM, r);
            b.members.push_back(idx);
            ++j;
        }
        const double n = std::max(1.0, static_cast<double>(b.members.size()));
        const double cx = sx / n;
        const double cy = sy / n;
        const double cz = sz / n;
        const double cn = std::max(1e-12, std::sqrt(cx * cx + cy * cy + cz * cz));
        b.cx = cx / cn;
        b.cy = cy / cn;
        b.cz = cz / cn;
        // Largest angular deviation of any member from that centre.
        double minDot = 1.0;
        for (const std::size_t idx : b.members)
        {
            const Vector& q = m_positions[idx];
            const double qn = std::max(1.0, q.GetLength());
            const double d = (q.x * b.cx + q.y * b.cy + q.z * b.cz) / qn;
            minDot = std::min(minDot, std::max(-1.0, std::min(1.0, d)));
        }
        b.cosMaxDev = minDot;
        b.sinMaxDev = std::sqrt(std::max(0.0, 1.0 - minDot * minDot));
        m_buckets.push_back(std::move(b));
        k = j;
    }
}

std::size_t
NtnVisibilityIndex::BestElevation(const Vector& ueEcef, double& bestElDeg) const
{
    m_lastEvaluated = 0;
    bestElDeg = -90.0;
    if (m_positions.empty())
    {
        return std::numeric_limits<std::size_t>::max();
    }

    const double R = std::max(1.0, ueEcef.GetLength());
    const double ux = ueEcef.x / R;
    const double uy = ueEcef.y / R;
    const double uz = ueEcef.z / R;

    // Optimistic bound per bucket, as pure arithmetic.
    //
    // A bucket's members lie within one bucket width of its centre direction,
    // so their smallest possible central angle is gammaCentre - width. Rather
    // than take an acos and subtract, use the angle-difference identity:
    //   cos(gammaCentre - w) = cos(gammaCentre)cos(w) + sin(gammaCentre)sin(w)
    // with cos(gammaCentre) the dot product we already have. Pairing that with
    // the bucket's LARGEST radius gives an upper bound on sin(el) that no
    // member can exceed - so any bucket whose bound loses to the running best
    // can be skipped, provably without changing the answer.
    //
    // There is no sort: the nearest bucket is evaluated first to establish a
    // strong incumbent, then the rest are scanned once and mostly skipped.
    double bestSin = -2.0;
    std::size_t bestIdx = std::numeric_limits<std::size_t>::max();

    auto bucketBound = [&](const Bucket& bk) {
        const double cosC = std::max(-1.0, std::min(1.0, ux * bk.cx + uy * bk.cy + uz * bk.cz));
        const double sinC = std::sqrt(std::max(0.0, 1.0 - cosC * cosC));
        // Clamp: once gammaCentre <= width the smallest angle is 0.
        // Smallest central angle any member of this bucket can have.
        //
        // When the centre is FURTHER from zenith than the bucket's own spread,
        // that is gammaCentre - maxDev. When it is closer, a member can sit
        // exactly at zenith, so the answer is 0 and the cosine is 1.
        //
        // Getting this wrong is what broke the first working version: it
        // computed cos(gammaCentre - maxDev) unconditionally, and for a
        // negative argument cos() returns the cosine of the POSITIVE angle -
        // less than 1 - so the bound came out tighter than the truth and
        // pruned buckets containing the best satellite. It showed up as a
        // 2.87 degree error on a single-altitude shell with the true best
        // 0.43 degrees off zenith: precisely the case where gammaCentre is
        // small and maxDev is not.
        const double cosMin = (cosC >= bk.cosMaxDev)
                                  ? 1.0
                                  : std::min(1.0, cosC * bk.cosMaxDev + sinC * bk.sinMaxDev);
        return SinElevation(bk.maxRadiusM, R, cosMin);
    };

    auto scanBucket = [&](const Bucket& bk) {
        for (const std::size_t i : bk.members)
        {
            const Vector& p = m_positions[i];
            const double rho = std::max(1.0, p.GetLength());
            const double cosGamma =
                std::max(-1.0, std::min(1.0, (p.x * ux + p.y * uy + p.z * uz) / rho));
            const double sv = SinElevation(rho, R, cosGamma);
            ++m_lastEvaluated;
            if (sv > bestSin)
            {
                bestSin = sv;
                bestIdx = i;
            }
        }
    };

    // Pass 1: the bucket whose centre is closest to the terminal's zenith.
    std::size_t seed = 0;
    double seedDot = -2.0;
    for (std::size_t b = 0; b < m_buckets.size(); ++b)
    {
        const Bucket& bk = m_buckets[b];
        const double d = ux * bk.cx + uy * bk.cy + uz * bk.cz;
        if (d > seedDot)
        {
            seedDot = d;
            seed = b;
        }
    }
    scanBucket(m_buckets[seed]);

    // Pass 2: everything else, skipped unless its bound can still win.
    for (std::size_t b = 0; b < m_buckets.size(); ++b)
    {
        if (b == seed)
        {
            continue;
        }
        if (bucketBound(m_buckets[b]) <= bestSin)
        {
            continue; // provably cannot contain the best
        }
        scanBucket(m_buckets[b]);
    }

    bestElDeg = std::asin(std::max(-1.0, std::min(1.0, bestSin))) * 180.0 / kPi;
    return bestIdx;
}

} // namespace ntncon
} // namespace ns3
