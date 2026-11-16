/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef SIONNA_CIR_SPECTRUM_LOSS_MODEL_H
#define SIONNA_CIR_SPECTRUM_LOSS_MODEL_H

#include "cir-doppler-synth.h"

#include "ns3/spectrum-propagation-loss-model.h"

#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-sionna
 * \brief Frequency-selective CIR fading per RB (Wave 2 M3, plan item B3).
 *
 * The narrowband SionnaCirPropagationLossModel collapses the CIR to one
 * coherent sum, which drops the tap delays: a 35 to 120 ns delay spread has a
 * coherence bandwidth of 8 to 28 MHz, comparable to the NR carrier. This
 * plugin applies the CIR's transfer function at each RB centre instead:
 *
 *   H(f_k, t) = sum_p a_p(t) exp(-j 2 pi (f_k - f_c) tau_p)
 *   G_k = |H(f_k, t)|^2 / sum_p |a_p|^2      (unit mean power)
 *
 * which is Sionna RT's Paths.cir() -> cir_to_ofdm_channel() convention
 * (baseband frequencies relative to the carrier), with a_p(t) evolved by the
 * per-tap Doppler of CirDopplerSynthesizer (Paths.apply_doppler()). The
 * carrier f_c is c / snapshot.wavelength_m. Installed through
 * NtnRealStackHelper::AddSpectrumChannelLoss on top of the 3GPP spatial
 * channel, which should then run without its own small-scale fading
 * (SetSmallScaleFading(false)) so exactly one multipath process acts.
 * Endpoint velocities are taken from the live mobility models, with the
 * snapshot's satellite and ground sides kept (same rule as the narrowband
 * model).
 */
class SionnaCirSpectrumLossModel : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();
    SionnaCirSpectrumLossModel();
    ~SionnaCirSpectrumLossModel() override;

    void SetSnapshot(const CirSnapshot& snap);
    std::size_t GetTapCount() const { return m_snap.taps.size(); }

    /// Per-RB gain (dB) of the last evaluation, band order of the PSD.
    std::vector<double> GetLastGainDb() const { return m_lastGainDb; }
    double GetLastMeanGainDb() const;
    double GetLastSpreadDb() const;

    /// Transfer-function gain (linear, unit mean) at baseband offsets
    /// \p dfHz for the evolved amplitudes \p amps (exposed for tests).
    std::vector<double> GainsAt(const std::vector<std::complex<double>>& amps,
                                const std::vector<double>& dfHz) const;

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t) override { return 0; }

    CirSnapshot m_snap;
    double m_refGain{1.0};
    mutable std::vector<double> m_lastGainDb;
};

} // namespace ns3

#endif // SIONNA_CIR_SPECTRUM_LOSS_MODEL_H
