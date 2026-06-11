/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * ntn-atmospheric-propagation-loss-model — adapts NtnAtmosphericLossChain
 * (ITU-R P.676 gaseous + P.618/P.838 rain + P.681 LMS) into a real ns-3
 * PropagationLossModel charging PURE ATMOSPHERIC EXCESS (no FSPL inside), so
 * it can be chained onto a real spectrum channel that already applies Friis
 * (e.g. NtnRealStackHelper::AddExtraPropagationLoss) without double-counting.
 *
 * Phase 2 of 2026-06 protocol-fidelity audit (channel-plugin recipe):
 * before, the chain was only queried in user-space probe loops that drove a
 * sigmoid SnrToPer() RateErrorModel; packets never saw the atmosphere. As a
 * PropagationLossModel the ITU-R physics attenuates the transmitted packets
 * and shows up in the MEASURED SINR/TBLER.
 */
#ifndef NTN_ATMOSPHERIC_PROPAGATION_LOSS_MODEL_H
#define NTN_ATMOSPHERIC_PROPAGATION_LOSS_MODEL_H

#include "ntn-atmospheric-loss-chain.h"

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * \ingroup ntn-sionna
 *
 * \brief ITU-R atmospheric excess (gaseous + rain + optional LMS shadowing)
 *        as a real PropagationLossModel, computed per transmission from the
 *        live Tx/Rx geometry. Works in ECEF or local ENU frames (delegated
 *        to NtnAtmosphericLossChain's frame detection).
 */
class NtnAtmosphericPropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    NtnAtmosphericPropagationLossModel();
    ~NtnAtmosphericPropagationLossModel() override;

    /// Use this chain (reconfigurable at runtime — e.g. a rain-event
    /// schedule calls chain->SetRainRateMmH() mid-run and the very next
    /// transmission feels it). If never called, a default chain is created.
    void SetChain(Ptr<NtnAtmosphericLossChain> chain);
    Ptr<NtnAtmosphericLossChain> GetChain() const;

    /// Last total atmospheric excess applied (dB) — for logging/inspection.
    double GetLastLossDb() const { return m_lastLossDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    mutable Ptr<NtnAtmosphericLossChain> m_chain;
    mutable double m_lastLossDb{0.0};
};

} // namespace ns3

#endif // NTN_ATMOSPHERIC_PROPAGATION_LOSS_MODEL_H
