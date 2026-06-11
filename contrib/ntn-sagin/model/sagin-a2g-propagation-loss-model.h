/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 *
 * SaginA2gPropagationLossModel — re-homes the ntn-sagin A2gChannelTr36777
 * CALCULATOR as a real ns-3 PropagationLossModel (Phase 2 of
 * 2026-06 protocol-fidelity audit, channel-plugin pattern).
 *
 * Before, A2gChannelTr36777::PathLossDb() was only invoked from user-space loops
 * and written to CSV; no packet ever saw it. As a PropagationLossModel it can be
 * chained onto a real spectrum channel (NtnRealStackHelper::AddExtraPropagation
 * Loss), so the TR 36.777 air-to-ground path loss (including the NLOS shadowing
 * branch) actually attenuates packets and shows up in the MEASURED SINR.
 *
 * Composition note: NtnRealStackHelper already applies free-space (Friis) loss
 * on the base channel. To avoid double-counting the distance term, this model
 * returns only the EXCESS of the TR 36.777 path loss over free space, so the net
 * channel loss reconstructs TR 36.777 exactly.
 */
#ifndef NTN_SAGIN_A2G_PROPAGATION_LOSS_MODEL_H
#define NTN_SAGIN_A2G_PROPAGATION_LOSS_MODEL_H

#include "a2g-channel-tr36777.h"

#include "ns3/propagation-loss-model.h"

namespace ns3
{

/**
 * \brief 3GPP TR 36.777 air-to-ground path loss (UMa/RMa/UMi-AV, LOS/NLOS) as a
 *        real PropagationLossModel, computed per transmission from Tx/Rx geometry.
 */
class SaginA2gPropagationLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    SaginA2gPropagationLossModel();
    ~SaginA2gPropagationLossModel() override;

    void SetFrequencyGHz(double fcGHz) { m_fcGHz = fcGHz; }
    void SetScenario(A2gScenario s) { m_scenario = s; }
    void SetLink(A2gLink l) { m_link = l; }
    /// Override the UAV (UT) altitude AGL in metres; <=0 derives it from geometry.
    void SetUavAltitudeM(double m) { m_uavAltM = m; }
    /// Last total TR 36.777 path loss applied (dB) — for logging/inspection.
    double GetLastLossDb() const { return m_lastLossDb; }
    /// Last excess-over-free-space actually charged on the chained channel (dB).
    double GetLastExcessDb() const { return m_lastExcessDb; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    double m_fcGHz{2.0};
    A2gScenario m_scenario{A2gScenario::UMa_AV};
    A2gLink m_link{A2gLink::NLOS};
    double m_uavAltM{-1.0};
    mutable double m_lastLossDb{0.0};
    mutable double m_lastExcessDb{0.0};
};

} // namespace ns3

#endif // NTN_SAGIN_A2G_PROPAGATION_LOSS_MODEL_H
