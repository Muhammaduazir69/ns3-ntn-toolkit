/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// thz-ntn-alpha-mu-fading — α-μ generalised fading sample generator.
//
// Reference: A.-A. A. Boulogeorgos and A. Alexiou, "Joint Communication
// and Sensing for 6G — A Vision on Sub-THz and THz", IEEE Trans. Wireless
// Commun. (2021). The α-μ distribution is the canonical fading model in
// sub-THz / THz JCAS analyses because it captures both nonlinear
// propagation (α) and multipath cluster diversity (μ) with only two
// parameters.
//
//   PDF:  f_R(r) = α μ^μ r^(αμ-1) / (Γ(μ) r̂^(αμ)) · exp(−μ (r/r̂)^α)
//   E[R^α] = r̂^α
//
// Special cases:
//   α=2, μ=1                  → Rayleigh
//   α=2, μ=m (integer / half) → Nakagami-m
//   α=1, μ=1                  → Exponential (in power)
//
// Sampling uses the well-known result that if X ~ Gamma(shape=μ, rate=μ),
// then R = (X r̂^α)^(1/α) follows the α-μ distribution. We pin the
// reference value `r̂` so that the linear-power expectation matches the
// caller's configured `Omega` (mean linear power).

#ifndef THZ_NTN_ALPHA_MU_FADING_H
#define THZ_NTN_ALPHA_MU_FADING_H

#include <ns3/object.h>
#include <ns3/random-variable-stream.h>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief α-μ generalised fading sample generator.
 */
class ThzNtnAlphaMuFading : public Object
{
  public:
    static TypeId GetTypeId();

    ThzNtnAlphaMuFading();
    ~ThzNtnAlphaMuFading() override;

    /// Set nonlinearity exponent (α > 0). Default 2.0 (Rayleigh/Nakagami).
    void SetAlpha(double alpha);
    double GetAlpha() const { return m_alpha; }

    /// Set multipath cluster parameter (μ > 0). Default 1.0 (Rayleigh).
    void SetMu(double mu);
    double GetMu() const { return m_mu; }

    /// Set the mean linear power Ω = E[R²]. Default 1.0.
    void SetOmega(double omega) { m_omega = omega; }
    double GetOmega() const { return m_omega; }

    /// One amplitude sample R ≥ 0 from the configured α-μ distribution.
    double SampleAmplitude();

    /// One linear-power sample R². Convenience around SampleAmplitude().
    double SamplePowerLinear();

    /// One power sample in dB: 10*log10(SamplePowerLinear()).
    double SamplePowerDb();

    /// Seed the underlying RNG stream. Returns the number of streams
    /// consumed (always 1).
    int64_t AssignStreams(int64_t stream);

  private:
    double m_alpha{2.0};
    double m_mu{1.0};
    double m_omega{1.0};
    Ptr<GammaRandomVariable> m_gamma;
};

} // namespace ns3

#endif // THZ_NTN_ALPHA_MU_FADING_H
