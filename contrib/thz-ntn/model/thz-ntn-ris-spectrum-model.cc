/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-ris-spectrum-model.h"

#include "thz-ntn-link-geometry.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRisSpectrumModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRisSpectrumModel);

namespace
{
constexpr double kC = 299792458.0;

Vector
Unit(const Vector& v)
{
    const double n = v.GetLength();
    return n > 0.0 ? Vector(v.x / n, v.y / n, v.z / n) : Vector(0, 0, 1);
}

double
Dot(const Vector& a, const Vector& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector
Cross(const Vector& a, const Vector& b)
{
    return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
} // namespace

TypeId
ThzNtnRisSpectrumModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnRisSpectrumModel")
            .SetParent<SpectrumPropagationLossModel>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnRisSpectrumModel>()
            .AddAttribute("UpdatePeriod",
                          "Controller period: the phase profile and the carrier co-phasing are "
                          "recomputed from the geometry this often (CSI period).",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ThzNtnRisSpectrumModel::m_updatePeriod),
                          MakeTimeChecker())
            .AddAttribute("DirectRayGainWhenEngagedDb",
                          "Terminal gain toward the direct ray, relative to its peak, while the "
                          "RIS is engaged and the terminal beam points at the RIS (0 = wide-beam "
                          "terminal; e.g. -20 = the direct ray falls in a sidelobe).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&ThzNtnRisSpectrumModel::m_directWhenEngagedDb),
                          MakeDoubleChecker<double>(-100.0, 0.0))
            .AddAttribute("ArrayFactorCache",
                          "Largest age of the cached array-factor misalignment term.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ThzNtnRisSpectrumModel::m_afCache),
                          MakeTimeChecker());
    return tid;
}

ThzNtnRisSpectrumModel::ThzNtnRisSpectrumModel()
{
    SetPanelNormal(Vector(0.0, 0.0, 1.0));
}

ThzNtnRisSpectrumModel::~ThzNtnRisSpectrumModel() = default;

void
ThzNtnRisSpectrumModel::SetPanelNormal(const Vector& n)
{
    m_normal = Unit(n);
    // In-plane axes: e1 horizontal when possible.
    Vector ref = (std::abs(m_normal.z) < 0.9) ? Vector(0, 0, 1) : Vector(1, 0, 0);
    m_e1 = Unit(Cross(ref, m_normal));
    m_e2 = Cross(m_normal, m_e1);
}

void
ThzNtnRisSpectrumModel::AddTerminal(Ptr<MobilityModel> m)
{
    if (!m)
    {
        return;
    }
    m_terms.insert(PeekPointer(m));
    if (!m_target)
    {
        m_target = PeekPointer(m);
    }
}

void
ThzNtnRisSpectrumModel::SetEngaged(bool e)
{
    if (e && !m_engaged)
    {
        m_lastUpdate = Seconds(-1); // configure at the next evaluation
    }
    m_engaged = e;
}

void
ThzNtnRisSpectrumModel::PanelAngles(const Vector& from, double& thetaDeg, double& phiDeg) const
{
    const Vector r = m_risMob->GetPosition();
    const Vector v = Unit(Vector(from.x - r.x, from.y - r.y, from.z - r.z));
    thetaDeg = std::acos(std::clamp(Dot(v, m_normal), -1.0, 1.0)) * 180.0 / M_PI;
    phiDeg = std::atan2(Dot(v, m_e2), Dot(v, m_e1)) * 180.0 / M_PI;
}

void
ThzNtnRisSpectrumModel::ControllerUpdate(const Vector& src, const Vector& term) const
{
    double ti;
    double pi;
    double to;
    double po;
    PanelAngles(src, ti, pi);
    PanelAngles(term, to, po);
    m_ris->ComputeOptimalPhases(ti, pi, to, po);
    m_afRefDb = m_ris->ComputeRisGain_dB(ti, pi, to, po);
    m_afMisDb = 0.0;
    m_afTime = Simulator::Now();
    const Vector r = m_risMob->GetPosition();
    const double d = CalculateDistance(src, term);
    const double d1 = CalculateDistance(src, r);
    const double d2 = CalculateDistance(r, term);
    const double fc = m_ris->GetWavelength() > 0.0 ? kC / m_ris->GetWavelength() : 0.0;
    m_phi0 = 2.0 * M_PI * fc * (d1 + d2 - d) / kC;
    m_lastUpdate = Simulator::Now();
    ++m_updates;
}

Ptr<SpectrumValue>
ThzNtnRisSpectrumModel::DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                     Ptr<const MobilityModel> a,
                                                     Ptr<const MobilityModel> b) const
{
    Ptr<SpectrumValue> psd = params->psd->Copy();
    if (!a || !b || !m_ris || !m_risMob)
    {
        return psd;
    }
    const bool aT = m_terms.count(PeekPointer(a)) > 0;
    const bool bT = m_terms.count(PeekPointer(b)) > 0;
    if (aT == bT)
    {
        return psd; // not a source-terminal link
    }
    const MobilityModel* termM = aT ? PeekPointer(a) : PeekPointer(b);
    const MobilityModel* srcM = aT ? PeekPointer(b) : PeekPointer(a);
    Vector src;
    Vector term;
    if (m_blockage)
    {
        if (!m_blockage->ToLocal(srcM, src) || !m_blockage->ToLocal(termM, term))
        {
            return psd;
        }
    }
    else
    {
        src = ThzNtnLinkGeometry::PositionOf(srcM);
        term = ThzNtnLinkGeometry::PositionOf(termM);
    }
    const Vector r = m_risMob->GetPosition();
    const Time now = Simulator::Now();
    const double d = CalculateDistance(src, term);
    const double d1 = CalculateDistance(src, r);
    const double d2 = CalculateDistance(r, term);
    const double lambdaC = m_ris->GetWavelength();
    const bool inFront = Dot(Vector(src.x - r.x, src.y - r.y, src.z - r.z), m_normal) > 0.0 &&
                         Dot(Vector(term.x - r.x, term.y - r.y, term.z - r.z), m_normal) > 0.0;

    const bool risActive = m_engaged && inFront && d1 > 0.0 && d2 > 0.0;
    if (risActive && termM == m_target &&
        (m_lastUpdate < Seconds(0) || now - m_lastUpdate >= m_updatePeriod))
    {
        ControllerUpdate(src, term);
    }
    if (risActive && (m_afTime < Seconds(0) || now - m_afTime >= m_afCache))
    {
        double ti;
        double pi;
        double to;
        double po;
        PanelAngles(src, ti, pi);
        PanelAngles(term, to, po);
        m_afMisDb = std::max(0.0, m_afRefDb - m_ris->ComputeRisGain_dB(ti, pi, to, po));
        m_afTime = now;
    }

    double aR = 0.0;
    double risRelDb = -1e9;
    if (risActive)
    {
        const double fsplDirect = 20.0 * std::log10(4.0 * M_PI * d / lambdaC);
        risRelDb = fsplDirect - m_ris->ComputeCascadedPathLoss_dB(src, r, term) - m_afMisDb;
        aR = std::pow(10.0, risRelDb / 20.0);
    }
    const double dTau = (d1 + d2 - d) / kC;

    const size_t n = psd->GetValuesN();
    std::vector<double> freqs;
    freqs.reserve(n);
    for (auto it = psd->ConstBandsBegin(); it != psd->ConstBandsEnd(); ++it)
    {
        freqs.push_back(it->fc);
    }
    std::vector<double> lDirV(n, 0.0);
    std::vector<double> lLegV(n, 0.0);
    if (m_blockage)
    {
        lDirV = m_blockage->LossDbBand(src, term, freqs);
        if (aR > 0.0)
        {
            const auto l1 = m_blockage->LossDbBand(src, r, freqs);
            const auto l2 = m_blockage->LossDbBand(r, term, freqs);
            for (size_t k = 0; k < n; ++k)
            {
                lLegV[k] = l1[k] + l2[k];
            }
        }
    }
    size_t i = 0;
    double sumDb = 0.0;
    double minDb = 1e9;
    double maxDb = -1e9;
    double directC = 0.0;
    double legC = 0.0;
    auto band = psd->ConstBandsBegin();
    for (auto v = psd->ValuesBegin(); v != psd->ValuesEnd(); ++v, ++band, ++i)
    {
        const double f = band->fc;
        const double lDir = lDirV[i];
        double bD = std::pow(10.0, -lDir / 20.0);
        if (aR > 0.0)
        {
            bD *= std::pow(10.0, m_directWhenEngagedDb / 20.0);
        }
        double g = bD * bD;
        double lLeg = 0.0;
        if (aR > 0.0)
        {
            lLeg = lLegV[i];
            const double bR = aR * std::pow(10.0, -lLeg / 20.0);
            const double psi = 2.0 * M_PI * f * dTau - m_phi0;
            g = bD * bD + bR * bR + 2.0 * bD * bR * std::cos(psi);
        }
        g = std::max(g, 1e-30);
        const double gDb = 10.0 * std::log10(g);
        sumDb += gDb;
        minDb = std::min(minDb, gDb);
        maxDb = std::max(maxDb, gDb);
        if (i == n / 2)
        {
            directC = lDir;
            legC = lLeg;
        }
        *v *= g;
    }
    m_state.directBlockDb = directC;
    m_state.risRelDb = risActive ? risRelDb - legC : -1e9;
    m_state.misalignDb = risActive ? m_afMisDb : 0.0;
    m_state.excessDelayNs = dTau * 1e9;
    m_state.meanGainDb = n > 0 ? sumDb / n : 0.0;
    m_state.minGainDb = n > 0 ? minDb : 0.0;
    m_state.maxGainDb = n > 0 ? maxDb : 0.0;
    m_state.inFront = inFront;
    m_state.time = now;
    return psd;
}

} // namespace ns3
