/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * NtnTdlSpectrumPropagationLossModel - the adapter that lets the toolkit's
 * frequency-selective tapped-delay-line actually reach a spectrum channel.
 *
 * NtnTdlSpectrumLossModel computes a per-subcarrier gain and exposes ApplyTo(),
 * but it is a plain Object, not a SpectrumPropagationLossModel, so there was no
 * way to attach it to anything. ApplyTo() had exactly two callers in the whole
 * tree and both were inside its own unit test: the model was correct, complete
 * and unreachable, which is the shape of defect this audit keeps finding.
 *
 * This wraps it in the interface NrHelper and NtnRealStackHelper already accept,
 * so a scenario can opt in and have frequency-selective fading appear in the
 * measured SINR rather than in a test fixture.
 *
 * The stand-in tap table stays a stand-in. TR 38.811 section 6.9.2 defines
 * NTN-TDL-A through NTN-TDL-D and this does not ship those values, so a paper
 * still may not claim "NTN-TDL"; see SCOPE_AND_LIMITATIONS.md A1. What it may
 * claim is a frequency-selective channel with a stated delay spread, which is
 * more than a flat model gives.
 */
#ifndef NTN_TDL_SPECTRUM_PROPAGATION_LOSS_MODEL_H
#define NTN_TDL_SPECTRUM_PROPAGATION_LOSS_MODEL_H

#include "ntn-tdl-spectrum-loss-model.h"

#include "ns3/spectrum-propagation-loss-model.h"

namespace ns3
{

class NtnTdlSpectrumPropagationLossModel : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();
    NtnTdlSpectrumPropagationLossModel();

    /// The tapped-delay-line this adapter drives. Exposed so a scenario can
    /// replace the tap set or read its provenance for a results table.
    Ptr<NtnTdlSpectrumLossModel> GetTdl() const { return m_tdl; }
    void SetTdl(Ptr<NtnTdlSpectrumLossModel> tdl);

    /// How many times the model has been asked for a PSD, and how many of those
    /// it could actually shape. A run that never shaped anything is a run where
    /// this model was attached and did nothing, which the caller should know.
    uint64_t GetAppliedCount() const { return m_applied; }
    uint64_t GetSkippedCount() const { return m_skipped; }

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    /// Elevation of the higher node seen from the lower one, in degrees, taken
    /// from the geocentric up at the ground node. Matches the convention in
    /// Ntn38811ExcessLossModel so the two models cannot disagree about the
    /// geometry they are given.
    static double ElevationDeg(Ptr<const MobilityModel> a, Ptr<const MobilityModel> b);

    Ptr<NtnTdlSpectrumLossModel> m_tdl;
    mutable uint64_t m_applied{0};
    mutable uint64_t m_skipped{0};
};

} // namespace ns3

#endif // NTN_TDL_SPECTRUM_PROPAGATION_LOSS_MODEL_H
