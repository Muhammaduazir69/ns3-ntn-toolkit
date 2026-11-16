/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_RIS_SPECTRUM_MODEL_H
#define THZ_NTN_RIS_SPECTRUM_MODEL_H

#include "ntn-screen-blockage-model.h"
#include "thz-ntn-ris.h"

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/spectrum-propagation-loss-model.h"

#include <map>
#include <set>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Direct ray plus RIS ray, combined coherently per RB from the live
 *        positions of the source, the RIS panel and the terminal
 *        (NtnRealStackHelper::AddSpectrumChannelLoss).
 *
 * The stack already applies the direct-path free-space loss (and the 3GPP
 * array/beam terms). This plugin multiplies each RB by
 *
 *   G(f) = | b_d(f) + a_r b_r(f) exp(-j psi(f)) |^2
 *
 *   b_d(f) = 10^(-L_blk(source->terminal, f)/20)  geometric blockage of the
 *            direct ray (NtnScreenBlockageModel, TR 38.901 7.6.4.2 model B);
 *   a_r    = 10^((FSPL_direct - PL_cascaded - L_mis)/20), PL_cascaded the
 *            far-field Tang et al. 2021 law (ThzNtnRis::ComputeCascadedPathLoss_dB
 *            on the live positions), L_mis the array-factor loss of a phase
 *            profile computed at the last controller update for the geometry of
 *            that time (ThzNtnRis::ComputeRisGain_dB, panel frame);
 *   b_r(f) = blockage of the source->RIS and RIS->terminal legs;
 *   psi(f) = 2 pi f (tau_r - tau_d) - phi0, tau_r - tau_d = (d1 + d2 - d)/c
 *            from the live geometry and phi0 = 2 pi f_c (tau_r - tau_d) at the
 *            last controller update (the controller co-phases the RIS ray with
 *            the direct ray at the carrier).
 *
 * So the RIS gain over the direct link follows the aperture law (no
 * 20 log10 N over the direct link), the excess delay of the RIS ray makes the
 * channel frequency selective across RBs, and a stale configuration (update
 * period longer than the geometry's coherence) loses the coherent sum.
 *
 * The RIS ray exists only when source and terminal are both in front of the
 * panel (positive projection on the panel normal). Links that do not join a
 * registered terminal pass through. While the RIS is disengaged only the
 * direct-ray blockage is applied.
 */
class ThzNtnRisSpectrumModel : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();
    ThzNtnRisSpectrumModel();
    ~ThzNtnRisSpectrumModel() override;

    void SetRis(Ptr<ThzNtnRis> ris) { m_ris = ris; }
    Ptr<ThzNtnRis> GetRis() const { return m_ris; }
    void SetRisMobility(Ptr<MobilityModel> m) { m_risMob = m; }
    /// Outward panel normal (scenario ENU frame); normalised internally.
    void SetPanelNormal(const Vector& n);
    /// Terminal served through the RIS (the controller optimises for the first).
    void AddTerminal(Ptr<MobilityModel> m);
    void SetBlockage(Ptr<NtnScreenBlockageModel> b) { m_blockage = b; }
    Ptr<NtnScreenBlockageModel> GetBlockage() const { return m_blockage; }
    /// Engage or release the RIS (controller / xApp decision).
    void SetEngaged(bool e);
    bool IsEngaged() const { return m_engaged; }

    /// Snapshot of the last evaluation for the terminal link.
    struct State
    {
        double directBlockDb{0.0};  //!< direct-ray blockage at f_c
        double risRelDb{-1e9};      //!< RIS ray vs unblocked direct ray at f_c (incl. leg blockage, misalignment)
        double misalignDb{0.0};     //!< array-factor loss of the stale profile
        double excessDelayNs{0.0};  //!< tau_r - tau_d
        double meanGainDb{0.0};     //!< mean of 10 log10 G(f) over RBs
        double minGainDb{0.0};
        double maxGainDb{0.0};
        bool inFront{false};
        Time time;
    };
    State GetLastState() const { return m_state; }
    uint64_t GetControllerUpdates() const { return m_updates; }

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t) override { return 0; }

    void PanelAngles(const Vector& from, double& thetaDeg, double& phiDeg) const;
    void ControllerUpdate(const Vector& src, const Vector& term) const;

    Ptr<ThzNtnRis> m_ris;
    Ptr<MobilityModel> m_risMob;
    Ptr<NtnScreenBlockageModel> m_blockage;
    Vector m_normal{0.0, 0.0, 1.0};
    Vector m_e1{1.0, 0.0, 0.0};
    Vector m_e2{0.0, 1.0, 0.0};
    std::set<const MobilityModel*> m_terms;
    const MobilityModel* m_target{nullptr};
    bool m_engaged{false};
    Time m_updatePeriod{MilliSeconds(1)};
    double m_directWhenEngagedDb{0.0};
    Time m_afCache{MilliSeconds(1)};

    mutable Time m_lastUpdate{Seconds(-1)};
    mutable double m_phi0{0.0};
    mutable double m_afRefDb{0.0};
    mutable Time m_afTime{Seconds(-1)};
    mutable double m_afMisDb{0.0};
    mutable uint64_t m_updates{0};
    mutable State m_state;
};

} // namespace ns3

#endif // THZ_NTN_RIS_SPECTRUM_MODEL_H
