/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// ntn-tdl-spectrum-loss-model - frequency-selective NTN fading.
//
// Why this exists (audit BOTH-01). Nothing in this toolkit produced
// frequency-selective fading or a delay spread. The small-scale processes that
// existed - the ITU-R P.681-11 Lutz two-state model, an alpha-mu model, and a
// flat elevation-dependent Rician term - are all FLAT: they scale every
// subcarrier by the same factor, so a wideband channel is as smooth as a
// narrowband one and no BLER number reflects intersymbol interference or
// frequency diversity.
//
// This is a tapped-delay-line applied per subcarrier: each tap contributes
// exp(-j*2*pi*f*tau) with its own amplitude, so the transfer function varies
// ACROSS the band with a coherence bandwidth set by the delay spread. The LOS
// tap carries the elevation-dependent Rician K-factor from TR 38.811 section
// 6.7.2, reusing Ntn38811ExcessLossModel's table rather than restating it.
//
// ON THE TAP TABLE, PLAINLY. TR 38.811 section 6.9.2 defines NTN-TDL-A through
// NTN-TDL-D. This class does NOT ship those tables, because their values could
// not be verified against the document here, and attributing invented taps to a
// 3GPP table is the exact defect this audit found elsewhere (a model presented
// as HITRAN-2024 that was ITU-R P.676-13). The built-in profile is a documented
// STAND-IN with a stated delay spread, labelled as such by
// GetProfileProvenance(), and SetTaps() takes the real table when a caller has
// it. What the class provides that the toolkit had none of is the mechanism:
// frequency selectivity, delay spread, and a K-factor that varies with
// elevation.

#ifndef NTN_TDL_SPECTRUM_LOSS_MODEL_H
#define NTN_TDL_SPECTRUM_LOSS_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/spectrum-value.h"

#include <vector>

namespace ns3
{

/// One tapped-delay-line tap.
struct NtnTdlTap
{
    double delayS{0.0};    //!< excess delay, seconds
    double powerDb{0.0};   //!< relative power, dB (0 = strongest)
    bool isLos{false};     //!< carries the Rician specular component
};

/**
 * \ingroup ntn-traffic
 * \brief Frequency-selective NTN fading as a tapped delay line.
 */
class NtnTdlSpectrumLossModel : public Object
{
  public:
    static TypeId GetTypeId();
    NtnTdlSpectrumLossModel();

    /// Replace the tap set. Powers are normalised so the profile has unit
    /// total power: a fading model must not add or remove average energy.
    void SetTaps(std::vector<NtnTdlTap> taps);
    const std::vector<NtnTdlTap>& GetTaps() const { return m_taps; }

    /// Where the current tap set came from, so a results table can say.
    std::string GetProfileProvenance() const { return m_provenance; }
    void SetProfileProvenance(std::string s) { m_provenance = std::move(s); }

    /// RMS delay spread of the current profile, seconds. The coherence
    /// bandwidth is roughly 1/(5*this).
    double GetRmsDelaySpreadS() const;

    /// Elevation-dependent Rician K (dB) applied to the LOS tap.
    /// Interpolated over the TR 38.811 section 6.7.2 elevation grid.
    void SetRicianKTable(std::vector<std::pair<double, double>> elevDegToKdB);
    double RicianKdB(double elevDeg) const;

    /// Per-subcarrier power gain (linear, mean 1) for the given geometry.
    /// \param freqsHz absolute subcarrier frequencies.
    /// \param elevDeg elevation of the satellite from the terminal.
    std::vector<double> ComputeGains(const std::vector<double>& freqsHz, double elevDeg) const;

    /// Convenience: apply ComputeGains to a SpectrumValue in place.
    void ApplyTo(Ptr<SpectrumValue> psd, const std::vector<double>& freqsHz,
                 double elevDeg) const;

  private:
    std::vector<NtnTdlTap> m_taps;
    std::vector<std::pair<double, double>> m_kTable;
    std::string m_provenance;
    Ptr<NormalRandomVariable> m_reRng;
    Ptr<NormalRandomVariable> m_imRng;
};

} // namespace ns3

#endif // NTN_TDL_SPECTRUM_LOSS_MODEL_H
