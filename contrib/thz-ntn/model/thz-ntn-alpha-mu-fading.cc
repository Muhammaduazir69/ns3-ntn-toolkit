/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-alpha-mu-fading.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnAlphaMuFading");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnAlphaMuFading);

TypeId
ThzNtnAlphaMuFading::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnAlphaMuFading")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnAlphaMuFading>()
            .AddAttribute("Alpha",
                          "Nonlinearity exponent (α > 0). 2 → Rayleigh/Nakagami.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&ThzNtnAlphaMuFading::SetAlpha,
                                             &ThzNtnAlphaMuFading::GetAlpha),
                          MakeDoubleChecker<double>(1e-3, 50.0))
            .AddAttribute("Mu",
                          "Multipath cluster parameter (μ > 0).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ThzNtnAlphaMuFading::SetMu,
                                             &ThzNtnAlphaMuFading::GetMu),
                          MakeDoubleChecker<double>(1e-3, 50.0))
            .AddAttribute("Omega",
                          "Mean linear power Ω = E[R²]. Default 1.0.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&ThzNtnAlphaMuFading::m_omega),
                          MakeDoubleChecker<double>(1e-30, 1e30));
    return tid;
}

ThzNtnAlphaMuFading::ThzNtnAlphaMuFading()
    : m_gamma(CreateObject<GammaRandomVariable>())
{
    // Gamma defaults: Alpha=shape, Beta=rate=1/scale. With shape=μ and
    // rate=μ, E[X]=1 and X^(1/α) maps onto the α-μ distribution.
    m_gamma->SetAttribute("Alpha", DoubleValue(m_mu));
    m_gamma->SetAttribute("Beta", DoubleValue(1.0));
}

ThzNtnAlphaMuFading::~ThzNtnAlphaMuFading() = default;

void
ThzNtnAlphaMuFading::SetAlpha(double alpha)
{
    m_alpha = std::max(1e-3, alpha);
}

void
ThzNtnAlphaMuFading::SetMu(double mu)
{
    m_mu = std::max(1e-3, mu);
    if (m_gamma)
    {
        m_gamma->SetAttribute("Alpha", DoubleValue(m_mu));
    }
}

double
ThzNtnAlphaMuFading::SampleAmplitude()
{
    // For α-μ, we want E[R^α] = r̂^α. Choose r̂^α such that E[R²] = Ω.
    //   E[R²] = (r̂² / μ^(2/α)) * Γ(μ + 2/α) / Γ(μ)
    // Setting this to Ω gives:
    //   r̂² = Ω * μ^(2/α) * Γ(μ) / Γ(μ + 2/α)
    const double mu2OverAlpha = std::pow(m_mu, 2.0 / m_alpha);
    const double gammaRatio =
        std::tgamma(m_mu) / std::tgamma(m_mu + 2.0 / m_alpha);
    const double rHatSq = m_omega * mu2OverAlpha * gammaRatio;
    const double rHat = std::sqrt(std::max(0.0, rHatSq));

    // Draw X ~ Gamma(shape=μ, rate=μ). ns-3 GammaRandomVariable has
    // mean Alpha/Beta = shape/rate = 1 when shape=μ and rate=μ.
    // GetValue returns one sample.
    // Our gamma is configured with Alpha=μ, Beta=1 (default 1 above) —
    // mean = μ. Rescale by 1/μ to match the "rate=μ" parameterisation.
    const double xBase = m_gamma->GetValue();
    const double x = xBase / m_mu;
    if (x < 0.0)
    {
        return 0.0;
    }
    // R = (x · r̂^α)^(1/α)
    const double powed = std::pow(x * std::pow(rHat, m_alpha), 1.0 / m_alpha);
    return powed;
}

double
ThzNtnAlphaMuFading::SamplePowerLinear()
{
    const double r = SampleAmplitude();
    return r * r;
}

double
ThzNtnAlphaMuFading::SamplePowerDb()
{
    const double p = SamplePowerLinear();
    return 10.0 * std::log10(std::max(p, 1e-30));
}

int64_t
ThzNtnAlphaMuFading::AssignStreams(int64_t stream)
{
    m_gamma->SetStream(stream);
    return 1;
}

} // namespace ns3
