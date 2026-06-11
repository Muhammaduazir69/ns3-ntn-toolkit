// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
//
// NtnStaticExtraLossModel — a runtime-settable scalar excess loss as a real
// ns-3 PropagationLossModel, for chaining onto a real spectrum channel via
// NtnRealStackHelper::AddExtraPropagationLoss().
//
// 2026-06 protocol-fidelity audit (channel-plugin recipe): scenario
// events that used to be folded into closed-form SNR formulas — NLOS blockage
// onset, a RIS engaging/releasing, beam-pointing loss — become live channel
// reconfigurations that real packets feel, so the event shows up in the
// MEASURED SINR/TBLER rather than in a synthetic RateErrorModel.

#ifndef NTN_STATIC_EXTRA_LOSS_MODEL_H
#define NTN_STATIC_EXTRA_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * \ingroup ntn-traffic
 * \brief Settable scalar excess loss (dB) in the real packet path.
 *
 * Negative values model a gain (e.g. a RIS or MIMO array recovering a
 * blocked path); the net is clamped so the model never amplifies beyond the
 * configured floor.
 */
class NtnStaticExtraLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    NtnStaticExtraLossModel();
    ~NtnStaticExtraLossModel() override;

    /// Set the excess loss applied to every transmission (dB; >= floor).
    void SetLossDb(double lossDb) { m_lossDb = lossDb; }
    double GetLossDb() const { return m_lossDb; }

    /// Lowest allowed net loss (default 0 dB: never amplify above the base
    /// channel). Lower it explicitly to model a genuine array gain.
    void SetFloorDb(double floorDb) { m_floorDb = floorDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    double m_lossDb{0.0};
    double m_floorDb{0.0};
};

} // namespace ns3

#endif // NTN_STATIC_EXTRA_LOSS_MODEL_H
