// SPDX-License-Identifier: GPL-2.0-only
//
// Ntn38811ExcessLossModel — the TR 38.811 large-scale EXCESS-loss terms as a
// real ns-3 PropagationLossModel, so they attenuate the MEASURED mmwave plane.
//
// Rationale (STANDARDS_VALIDATION_GAP_ANALYSIS_2026-06-26 finding G1): the
// NtnRealStackHelper measured plane installs Friis path loss (FSPL, valid as the
// LEO mean) but the TR 38.811 excess terms — elevation-dependent atmospheric gas
// absorption (ITU-R P.676), tropospheric scintillation (ITU-R P.618),
// scenario-dependent clutter loss (TR 38.811 Table 6.6.2-1) and elevation-
// dependent shadow fading (Table 6.6.2-1) — previously lived ONLY in
// ntn-cho/model/ntn-measurement-model.cc, an analytical oracle that extends
// Object (not PropagationLossModel) and therefore never reached the radio. This
// class re-homes that SAME math as a chainable PropagationLossModel so it adds
// real, geometry-correct attenuation on top of Friis and shows up in the
// measured SINR. It applies ONLY the excess terms (FSPL is already applied by
// the Friis model it is chained after).
//
// This is not new physics: it is the existing TR 38.811 excess-loss model wired
// into the measured plane via NtnRealStackHelper::AddExtraPropagationLoss().

#ifndef NTN_TR38811_EXCESS_LOSS_MODEL_H
#define NTN_TR38811_EXCESS_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"

#include <map>

#include <utility>

#include "ns3/nstime.h"

#include "ns3/vector.h"

#include "ns3/simulator.h"

namespace ns3
{

/**
 * \brief TR 38.811 large-scale excess-loss terms (gas + scintillation + clutter
 *        + shadow fading) as a chainable PropagationLossModel. Chain AFTER a
 *        Friis (FSPL) model — this class adds only the excess above free space.
 */
class Ntn38811ExcessLossModel : public PropagationLossModel
{
  public:
    /// TR 38.811 deployment scenario (sets the clutter-loss constant and the
    /// elevation-dependent shadow-fading sigma bins).
    enum NtnScenario
    {
        DenseUrban,
        Urban,
        Suburban,
        Rural,
    };

    static TypeId GetTypeId();
    Ntn38811ExcessLossModel();
    ~Ntn38811ExcessLossModel() override;

    void SetScenario(NtnScenario s) { m_scenario = s; }
    void SetCarrierFrequencyHz(double f) { m_freqHz = f; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

    /// Elevation angle (deg) of the higher node seen from the lower node.
    /// Robust to both ECEF (|pos| > Earth radius) and local-ENU coordinate
    /// frames: in ECEF the local "up" is the radial unit vector; in ENU it is
    /// the +z axis.
    double ElevationDeg(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

    /// Linear-interpolate a per-scenario, elevation-indexed (10..90 deg, 10-deg
    /// step) TR 38.811 S-band LOS table at the given elevation.
    double InterpByElevation(const double (&table)[9], double elevDeg) const;
    /// TR 38.811 Table 6.6.2-x S-band LOS shadow-fading sigma (dB) at elevDeg.
    double ShadowSigmaDb(double elevDeg) const;
    /// TR 38.811 Table 6.7.2-Xa S-band LOS Rician K mean (dB) at elevDeg.
    double RicianKdB(double elevDeg) const;

    NtnScenario m_scenario{Suburban};
    double m_freqHz{2.0e9};        ///< carrier (selects the P.676 gas band)
    bool m_enableShadowFading{true};
    bool m_enableScintillation{true};
    /// Rician small-scale fading (TR 38.811 §6.7/6.9).
    ///
    /// GAP S6: defaults to FALSE. Both radio backends keep the 3GPP phased-array
    /// spectrum model, which ALREADY applies small-scale fading (with Doppler)
    /// on the same link. Running this Rician process as well multiplied two
    /// independent fast-fading realisations onto one link — the measured SINR
    /// scatter was an artifact, not physics. Enable only for a link with no 3GPP
    /// spectrum model in the path.
    bool m_enableFastFading{false};
    Ptr<NormalRandomVariable> m_sfRng;    ///< shadow fading N(0,1)
    Ptr<NormalRandomVariable> m_scintRng; ///< scintillation N(0,1)
    Ptr<NormalRandomVariable> m_fadeIRng; ///< Rician in-phase scatter N(0,1)
    Ptr<NormalRandomVariable> m_fadeQRng; ///< Rician quadrature scatter N(0,1)

    // ---- S6: large-scale-parameter correlation ---------------------------
    // TR 38.811 §6.6.2 shadow fading is a POSITION-CORRELATED large-scale
    // process (correlation distance ~37-50 m), and P.618 scintillation has a
    // seconds-scale coherence time. Both were previously re-drawn i.i.d. on
    // EVERY DoCalcRxPower call — i.e. per transport block, and again for every
    // interference evaluation — turning correlated physics into white noise:
    // the mean SINR was unaffected while the per-sample variance ballooned,
    // which is what forced the TR 38.821 calibration gate to be widened.
    // Cache per node-pair and re-draw only when the link has genuinely
    // decorrelated (pattern: ThreeGppPropagationLossModel::m_shadowingMap).
    struct LargeScaleState
    {
        double shadowDb{0.0};
        double scintDb{0.0};
        Vector lastPos{0, 0, 0}; ///< ground-node position at the last SF draw
        double lastElevDeg{0.0};
        Time lastScintTime{Seconds(-1e9)};
        bool valid{false};
    };
    /// Shadow-fading decorrelation distance (m), TR 38.811 §6.6.2.
    double m_sfCorrDistanceM{50.0};
    /// Elevation change (deg) that also decorrelates the shadowing: for a static
    /// UE the geometry changes because the SATELLITE moves, so pure UE
    /// displacement never decorrelates a LEO pass.
    double m_sfCorrElevDeg{5.0};
    /// Scintillation coherence time (s), ITU-R P.618 §2.4.
    double m_scintCoherenceS{1.0};
    mutable std::map<std::pair<uintptr_t, uintptr_t>, LargeScaleState> m_lssCache;
};

} // namespace ns3

#endif // NTN_TR38811_EXCESS_LOSS_MODEL_H
