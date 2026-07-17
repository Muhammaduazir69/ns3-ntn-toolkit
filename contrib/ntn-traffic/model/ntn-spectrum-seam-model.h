/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SPECTRUM_SEAM_MODEL_H
#define NTN_SPECTRUM_SEAM_MODEL_H

#include "ns3/phased-array-spectrum-propagation-loss-model.h"
#include "ns3/spectrum-propagation-loss-model.h"

#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-traffic
 * \brief Composes module-supplied spectrum transfer functions ON TOP of the
 *        3GPP phased-array channel instead of replacing it (gap S1).
 *
 * WHY THIS EXISTS
 * ---------------
 * `MultiModelSpectrumChannel::StartTx` applies the two spectrum-loss families
 * with an **else-if**:
 *
 *     if (m_spectrumPropagationLoss)            { ... }
 *     else if (m_phasedArraySpectrumPropagationLoss) { ... }
 *
 * 5G-LENA installs the 3GPP channel (array gain, small-scale fading, and the
 * `spectrumChannelMatrix` that NrPmSearchFull needs for rank/PMI adaptation) as
 * the **phased-array** model. So the moment a module installed its own plain
 * `SpectrumPropagationLossModel` — the toolkit's "Enabler B spectrum seam", used
 * to inject THz / Sionna / RIS transfer functions — the entire 3GPP spatial
 * channel was silently switched off: ~18-21 dB of UPA beamforming gain, all
 * fading, and MIMO all vanished from the PSD. The seam and MIMO, the two halves
 * of the same enabler, were mutually destructive.
 *
 * Chaining via `SetNext()` does NOT fix it: the base class's chain walk passes
 * the ORIGINAL params (not the previous model's output) to the next model, so
 * only the last model in a chain survives. This class therefore composes
 * explicitly:
 *
 *   1. run the inner 3GPP phased-array model  -> PSD (+ MIMO channel matrix)
 *   2. run each plugin on that result         -> per-RB excess gain applied
 *   3. rescale the MIMO channel matrix by sqrt(gain) per RB so |H|^2 stays
 *      consistent with the PSD the plugin produced
 *
 * Installed on the channel via `AddPhasedArraySpectrumPropagationLossModel`, so
 * the else-if picks the phased-array branch and everything survives.
 */
class NtnSpectrumSeamModel : public PhasedArraySpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();

    /// The 3GPP (or any other) phased-array model to run FIRST. Typically the
    /// model 5G-LENA already installed on the BWP channel.
    void SetInnerModel(Ptr<PhasedArraySpectrumPropagationLossModel> inner);
    Ptr<PhasedArraySpectrumPropagationLossModel> GetInnerModel() const { return m_inner; }

    /// Add a module-supplied transfer function applied AFTER the inner model.
    /// Plugins are applied in insertion order.
    void AddPlugin(Ptr<SpectrumPropagationLossModel> plugin);
    std::size_t GetNumPlugins() const { return m_plugins.size(); }

  private:
    Ptr<SpectrumSignalParameters> DoCalcRxPowerSpectralDensity(
        Ptr<const SpectrumSignalParameters> params,
        Ptr<const MobilityModel> a,
        Ptr<const MobilityModel> b,
        Ptr<const PhasedArrayModel> aPhasedArrayModel,
        Ptr<const PhasedArrayModel> bPhasedArrayModel) const override;

    int64_t DoAssignStreams(int64_t stream) override;

    void DoDispose() override;

    Ptr<PhasedArraySpectrumPropagationLossModel> m_inner;
    std::vector<Ptr<SpectrumPropagationLossModel>> m_plugins;
};

} // namespace ns3

#endif // NTN_SPECTRUM_SEAM_MODEL_H
