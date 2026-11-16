/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-screen-blockage-model.h"

#include "thz-ntn-link-geometry.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnScreenBlockageModel");
NS_OBJECT_ENSURE_REGISTERED(NtnScreenBlockageModel);
NS_OBJECT_ENSURE_REGISTERED(NtnScreenBlockageLossModel);
NS_OBJECT_ENSURE_REGISTERED(NtnScreenBlockageSpectrumModel);

namespace
{
constexpr double kC = 299792458.0;
constexpr double kHalfEarth = 0.5 * 6371000.0;

Vector
GeodeticToEcef(double latDeg, double lonDeg, double altM)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double lat = latDeg * M_PI / 180.0;
    const double lon = lonDeg * M_PI / 180.0;
    const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    return Vector((n + altM) * std::cos(lat) * std::cos(lon),
                  (n + altM) * std::cos(lat) * std::sin(lon),
                  (n * (1.0 - e2) + altM) * std::sin(lat));
}

/// Knife-edge term F of TR 38.901 clause 7.6.4.2 (blockage model B).
double
EdgeF(bool shadow, double d1, double d2, double r, double lambda)
{
    const double excess = std::max(d1 + d2 - r, 0.0);
    const double arg = (M_PI / 2.0) * std::sqrt((M_PI / lambda) * excess);
    return std::atan(shadow ? arg : -arg) / M_PI;
}
} // namespace

// ---------------------------------------------------------------------------
TypeId
NtnScreenBlockageModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnScreenBlockageModel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<NtnScreenBlockageModel>()
            .AddAttribute("GroundZ",
                          "Ground height in the blocker frame (m); a screen whose bottom is at "
                          "or below it has no bottom diffraction edge.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NtnScreenBlockageModel::m_groundZ),
                          MakeDoubleChecker<double>())
            .AddAttribute("MaxLoss",
                          "Upper clip of the total blockage loss (dB).",
                          DoubleValue(80.0),
                          MakeDoubleAccessor(&NtnScreenBlockageModel::m_maxLossDb),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

NtnScreenBlockageModel::NtnScreenBlockageModel() = default;
NtnScreenBlockageModel::~NtnScreenBlockageModel() = default;

uint32_t
NtnScreenBlockageModel::AddScreen(Ptr<MobilityModel> bottomCentre,
                                  double widthM,
                                  double heightM,
                                  const std::string& name)
{
    NS_ABORT_MSG_IF(!bottomCentre, "screen needs a mobility model");
    m_screens.push_back(Screen{bottomCentre, widthM, heightM, name});
    return static_cast<uint32_t>(m_screens.size() - 1);
}

void
NtnScreenBlockageModel::SetEnuReference(double latDeg, double lonDeg, double altM)
{
    m_hasRef = true;
    m_refLat = latDeg;
    m_refLon = lonDeg;
    m_refAlt = altM;
    m_refEcef = GeodeticToEcef(latDeg, lonDeg, altM);
}

bool
NtnScreenBlockageModel::ToLocal(const MobilityModel* m, Vector& out) const
{
    const Vector p = ThzNtnLinkGeometry::PositionOf(m);
    const double n = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (n <= kHalfEarth)
    {
        out = p;
        return true;
    }
    if (!m_hasRef)
    {
        return false;
    }
    const double lat = m_refLat * M_PI / 180.0;
    const double lon = m_refLon * M_PI / 180.0;
    const double dx = p.x - m_refEcef.x;
    const double dy = p.y - m_refEcef.y;
    const double dz = p.z - m_refEcef.z;
    out.x = -std::sin(lon) * dx + std::cos(lon) * dy;
    out.y = -std::sin(lat) * std::cos(lon) * dx - std::sin(lat) * std::sin(lon) * dy +
            std::cos(lat) * dz;
    out.z = std::cos(lat) * std::cos(lon) * dx + std::cos(lat) * std::sin(lon) * dy +
            std::sin(lat) * dz;
    return true;
}

double
NtnScreenBlockageModel::ScreenLossDb(const Vector& t,
                                     const Vector& r,
                                     const Vector& c,
                                     double width,
                                     double height,
                                     double groundZ,
                                     double lambda,
                                     bool* crosses)
{
    if (crosses)
    {
        *crosses = false;
    }
    const double hx = r.x - t.x;
    const double hy = r.y - t.y;
    const double rh = std::sqrt(hx * hx + hy * hy);
    if (rh < 1e-6)
    {
        return 0.0; // vertical link: a vertical screen cannot sit across it
    }
    const double ux = hx / rh;
    const double uy = hy / rh;
    // Fraction of the way along the link where it meets the screen plane
    // (plane through c with normal u, model B orientation).
    const double sC = (c.x - t.x) * ux + (c.y - t.y) * uy; // horizontal distance from tx
    if (sC <= 0.0 || sC >= rh)
    {
        return 0.0; // screen not between the ends
    }
    const double frac = sC / rh;
    const Vector p(t.x + frac * (r.x - t.x), t.y + frac * (r.y - t.y), t.z + frac * (r.z - t.z));
    // Lateral offset of the crossing point along the screen (axis l = (-uy, ux)).
    const double q = (p.x - c.x) * (-uy) + (p.y - c.y) * ux;
    const double zTop = c.z + height;
    const double zBot = c.z;

    // --- vertical edges (top view) ---
    auto wEdge = [&](double off, bool shadow) {
        const double ex = c.x + off * (-uy);
        const double ey = c.y + off * ux;
        const double d1 = std::hypot(ex - t.x, ey - t.y);
        const double d2 = std::hypot(r.x - ex, r.y - ey);
        return EdgeF(shadow, d1, d2, rh, lambda);
    };
    const double fw1 = wEdge(-0.5 * width, q > -0.5 * width);
    const double fw2 = wEdge(0.5 * width, q < 0.5 * width);

    // --- horizontal edges (side view in the vertical plane of the link) ---
    const double r3 = std::sqrt(rh * rh + (r.z - t.z) * (r.z - t.z));
    auto hEdge = [&](double ze, bool shadow) {
        const double d1 = std::hypot(sC, ze - t.z);
        const double d2 = std::hypot(rh - sC, r.z - ze);
        return EdgeF(shadow, d1, d2, r3, lambda);
    };
    const double fh1 = hEdge(zTop, p.z < zTop);
    double fh2;
    if (zBot <= groundZ + 1e-9)
    {
        fh2 = (p.z > zBot) ? 0.5 : -0.5; // no bottom edge on a ground-standing screen
    }
    else
    {
        fh2 = hEdge(zBot, p.z > zBot);
    }
    if (crosses)
    {
        *crosses = (std::abs(q) < 0.5 * width) && (p.z < zTop) && (p.z > zBot);
    }
    const double prod = (fh1 + fh2) * (fw1 + fw2);
    const double arg = 1.0 - prod;
    if (arg <= 1e-12)
    {
        return 1e9;
    }
    return -20.0 * std::log10(arg);
}

double
NtnScreenBlockageModel::LossDb(const Vector& tx, const Vector& rx, double freqHz) const
{
    ++m_evals;
    const double lambda = kC / freqHz;
    double total = 0.0;
    bool blocked = false;
    for (const auto& s : m_screens)
    {
        bool cr = false;
        const double l =
            ScreenLossDb(tx, rx, s.mobility->GetPosition(), s.width, s.height, m_groundZ, lambda, &cr);
        blocked = blocked || cr;
        total += std::max(l, 0.0);
    }
    if (blocked)
    {
        ++m_blockedEvals;
    }
    return std::clamp(total, 0.0, m_maxLossDb);
}

double
NtnScreenBlockageModel::LossDb(const MobilityModel* tx, const MobilityModel* rx, double freqHz) const
{
    Vector a;
    Vector b;
    if (!ToLocal(tx, a) || !ToLocal(rx, b))
    {
        ++m_nonLocal;
        return 0.0;
    }
    return LossDb(a, b, freqHz);
}

std::vector<double>
NtnScreenBlockageModel::LossDbBand(const Vector& tx,
                                   const Vector& rx,
                                   const std::vector<double>& freqsHz) const
{
    const size_t n = freqsHz.size();
    std::vector<double> out(n, 0.0);
    if (n == 0)
    {
        return out;
    }
    if (n <= 3)
    {
        for (size_t i = 0; i < n; ++i)
        {
            out[i] = LossDb(tx, rx, freqsHz[i]);
        }
        return out;
    }
    const size_t m = n / 2;
    const double l0 = LossDb(tx, rx, freqsHz[0]);
    const double lm = LossDb(tx, rx, freqsHz[m]);
    const double l1 = LossDb(tx, rx, freqsHz[n - 1]);
    for (size_t i = 0; i < n; ++i)
    {
        if (i <= m)
        {
            const double w = (freqsHz[i] - freqsHz[0]) / std::max(freqsHz[m] - freqsHz[0], 1e-9);
            out[i] = l0 + w * (lm - l0);
        }
        else
        {
            const double w =
                (freqsHz[i] - freqsHz[m]) / std::max(freqsHz[n - 1] - freqsHz[m], 1e-9);
            out[i] = lm + w * (l1 - lm);
        }
    }
    return out;
}

bool
NtnScreenBlockageModel::IsBlocked(const Vector& tx, const Vector& rx) const
{
    for (const auto& s : m_screens)
    {
        bool cr = false;
        ScreenLossDb(tx, rx, s.mobility->GetPosition(), s.width, s.height, m_groundZ, 1e-3, &cr);
        if (cr)
        {
            return true;
        }
    }
    return false;
}

bool
NtnScreenBlockageModel::IsBlocked(const MobilityModel* tx, const MobilityModel* rx) const
{
    Vector a;
    Vector b;
    if (!ToLocal(tx, a) || !ToLocal(rx, b))
    {
        return false;
    }
    return IsBlocked(a, b);
}

// ---------------------------------------------------------------------------
TypeId
NtnScreenBlockageLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnScreenBlockageLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<NtnScreenBlockageLossModel>()
            .AddAttribute("Frequency",
                          "Carrier frequency (Hz) for the knife-edge terms.",
                          DoubleValue(2.0e9),
                          MakeDoubleAccessor(&NtnScreenBlockageLossModel::m_freqHz),
                          MakeDoubleChecker<double>(1e6));
    return tid;
}

double
NtnScreenBlockageLossModel::DoCalcRxPower(double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
    if (!m_model || !a || !b)
    {
        return txPowerDbm;
    }
    m_last = m_model->LossDb(PeekPointer(a), PeekPointer(b), m_freqHz);
    return txPowerDbm - m_last;
}

// ---------------------------------------------------------------------------
TypeId
NtnScreenBlockageSpectrumModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnScreenBlockageSpectrumModel")
                            .SetParent<SpectrumPropagationLossModel>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<NtnScreenBlockageSpectrumModel>();
    return tid;
}

double
NtnScreenBlockageSpectrumModel::GetLastCentreLossDb(const MobilityModel* end) const
{
    auto it = m_lastByEnd.find(end);
    return it == m_lastByEnd.end() ? 0.0 : it->second;
}

Ptr<SpectrumValue>
NtnScreenBlockageSpectrumModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b) const
{
    Ptr<SpectrumValue> psd = params->psd->Copy();
    if (!m_model || !a || !b || m_model->GetScreens().empty())
    {
        return psd;
    }
    Vector pa;
    Vector pb;
    if (!m_model->ToLocal(PeekPointer(a), pa) || !m_model->ToLocal(PeekPointer(b), pb))
    {
        return psd;
    }
    std::vector<double> freqs;
    freqs.reserve(psd->GetValuesN());
    for (auto it = psd->ConstBandsBegin(); it != psd->ConstBandsEnd(); ++it)
    {
        freqs.push_back(it->fc);
    }
    // One evaluation per link and simulated instant (DL data, control and
    // reference signals of a slot share the geometry).
    const Time now = Simulator::Now();
    auto& e = m_cache[std::make_pair(static_cast<const void*>(PeekPointer(a)),
                                     static_cast<const void*>(PeekPointer(b)))];
    if (e.t != now || e.loss.size() != freqs.size())
    {
        e.loss = m_model->LossDbBand(pa, pb, freqs);
        e.t = now;
    }
    size_t i = 0;
    for (auto v = psd->ValuesBegin(); v != psd->ValuesEnd() && i < e.loss.size(); ++v, ++i)
    {
        *v *= std::pow(10.0, -e.loss[i] / 10.0);
    }
    const double centre = e.loss.empty() ? 0.0 : e.loss[e.loss.size() / 2];
    m_lastByEnd[PeekPointer(a)] = centre;
    m_lastByEnd[PeekPointer(b)] = centre;
    return psd;
}

} // namespace ns3
