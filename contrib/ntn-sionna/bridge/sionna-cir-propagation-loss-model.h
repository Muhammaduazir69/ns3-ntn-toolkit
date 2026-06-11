/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// SionnaCirPropagationLossModel — re-homes the FULL Sionna RT channel-impulse
// response (multipath taps + per-tap Doppler) as a real ns-3
// PropagationLossModel (Phase 2 of 2026-06 protocol-fidelity audit,
// channel-plugin recipe).
//
// Audit finding for ntn-sionna: the RT result was collapsed to a single scalar
// `path_loss_db` (NtnSionnaChannel) — the multipath structure, the per-tap
// delays, and the Doppler-driven small-scale fading were all thrown away, so the
// "Sionna" link behaved like a static free-space attenuator.
//
// This model keeps the CIR intact. It holds a CIR snapshot (the taps a Sionna RT
// query returns) and, per transmission, coherently combines the taps after
// rotating each by its Doppler phase via CirDopplerSynthesizer (the existing C++
// equivalent of Sionna's paths.apply_doppler()). The resulting narrowband gain
// |Σ a_p·e^{jφ_p(t)}|² fluctuates in time — real constructive/destructive fading
// — and is charged as a fading deviation around the snapshot reference, so when
// chained on the real mmwave Friis channel (NtnRealStackHelper::AddExtraPropaga
// tionLoss) it makes the MEASURED SINR exhibit genuine multipath fading that a
// scalar path_loss_db cannot reproduce.

#ifndef NTN_SIONNA_CIR_PROPAGATION_LOSS_MODEL_H
#define NTN_SIONNA_CIR_PROPAGATION_LOSS_MODEL_H

#include "cir-doppler-synth.h"

#include "ns3/propagation-loss-model.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * \brief Full-CIR multipath fading (Doppler-evolved) as a real
 *        PropagationLossModel, sampled per transmission from the live sim time.
 */
class SionnaCirPropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    SionnaCirPropagationLossModel();
    ~SionnaCirPropagationLossModel() override;

    /// Install the CIR snapshot (the taps from a Sionna RT query). Recomputes
    /// the reference combine so the time-0 fading deviation is 0 dB.
    void SetSnapshot(const CirSnapshot& snap);
    /// Tx/Rx velocities (m/s, ECEF) that drive the per-tap Doppler.
    void SetTxVelocity(const Vector& v) { m_txVel = v; }
    void SetRxVelocity(const Vector& v) { m_rxVel = v; }

    /// Last fading deviation charged on the chained channel (dB; +ve = fade).
    double GetLastFadingDb() const { return m_lastFadingDb; }
    /// Number of taps in the installed CIR (>1 means real multipath).
    std::size_t GetTapCount() const { return m_snap.taps.size(); }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    CirSnapshot m_snap;
    Vector m_txVel{0.0, 0.0, 0.0};
    Vector m_rxVel{0.0, 0.0, 0.0};
    double m_refGain{1.0}; ///< |Σ a_p|² at the snapshot reference time
    mutable double m_lastFadingDb{0.0};
};

} // namespace ns3

#endif // NTN_SIONNA_CIR_PROPAGATION_LOSS_MODEL_H
