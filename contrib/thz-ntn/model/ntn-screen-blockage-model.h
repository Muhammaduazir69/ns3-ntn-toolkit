/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SCREEN_BLOCKAGE_MODEL_H
#define NTN_SCREEN_BLOCKAGE_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/spectrum-propagation-loss-model.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 * \brief Geometric blockage by rectangular screens, TR 38.901 clause 7.6.4.2
 *        (blockage model B), evaluated from the live positions of the link
 *        ends and of the blockers.
 *
 * Each blocker is a vertical rectangular screen of width w and height h whose
 * bottom-centre follows a MobilityModel (a bus, a truck, a building edge, or a
 * fixed obstacle). Following model B the screen is oriented perpendicular to
 * the link (its normal is the horizontal Tx-Rx direction). When the screen
 * plane lies between the two ends, the knife-edge diffraction loss is
 *
 *   L = -20 log10( 1 - (F_h1 + F_h2) (F_w1 + F_w2) )                [dB]
 *   F = atan( +/- (pi/2) sqrt( (pi/lambda) (D1 + D2 - r) ) ) / pi
 *
 * with D1, D2 the projected distances from the Tx and Rx to each edge (top
 * view for the two vertical edges w1, w2, side view for the top and bottom
 * edges h1, h2), r the projected direct distance, and the + sign when the
 * direct path lies in the shadow side of that edge. A screen standing on the
 * ground (bottom at or below \p GroundZ) has no bottom diffraction edge:
 * F_h2 = 1/2 whenever the ray passes above the bottom.
 *
 * The loss is frequency dependent (lambda), so the spectrum form applies it at
 * each RB centre. Several screens add in dB. The result is clipped to
 * [0, MaxLoss] dB (the diffraction ripple outside the shadow, a fraction of a
 * dB, is not reproduced).
 *
 * Frames: blockers live in the scenario's local ENU frame (z up). A link end
 * given in ECEF (norm above half an Earth radius) is converted to that ENU
 * frame when SetEnuReference() was called; otherwise the link is passed
 * through and counted as non-local.
 */
class NtnScreenBlockageModel : public Object
{
  public:
    static TypeId GetTypeId();
    NtnScreenBlockageModel();
    ~NtnScreenBlockageModel() override;

    struct Screen
    {
        Ptr<MobilityModel> mobility; //!< bottom-centre of the screen
        double width{2.5};           //!< m
        double height{3.0};          //!< m
        std::string name;
    };

    /// Add a blocker; returns its index.
    uint32_t AddScreen(Ptr<MobilityModel> bottomCentre,
                       double widthM,
                       double heightM,
                       const std::string& name = "");
    const std::vector<Screen>& GetScreens() const { return m_screens; }

    /// ENU origin (WGS-84) used to map ECEF link ends into the blocker frame.
    void SetEnuReference(double latDeg, double lonDeg, double altM);

    /// Total model-B loss (dB) of the straight path tx -> rx at \p freqHz.
    double LossDb(const Vector& tx, const Vector& rx, double freqHz) const;
    /// Same, from mobility models (ECEF ends converted when a reference is set).
    double LossDb(const MobilityModel* tx, const MobilityModel* rx, double freqHz) const;
    /// Per-band loss for a band set: exact at the first, middle and last
    /// frequency, piecewise linear in between (the model-B loss varies as
    /// sqrt(f) and moves by well under 0.1 dB across one channel).
    std::vector<double> LossDbBand(const Vector& tx,
                                   const Vector& rx,
                                   const std::vector<double>& freqsHz) const;
    /// True if the direct ray crosses at least one screen rectangle.
    bool IsBlocked(const MobilityModel* tx, const MobilityModel* rx) const;
    /// Same for raw positions.
    bool IsBlocked(const Vector& tx, const Vector& rx) const;

    /// Loss of one screen (dB) for a straight path (no clipping).
    static double ScreenLossDb(const Vector& tx,
                               const Vector& rx,
                               const Vector& bottomCentre,
                               double width,
                               double height,
                               double groundZ,
                               double lambda,
                               bool* crossesRectangle = nullptr);

    /// Map a link end into the blocker frame; false if it cannot be mapped.
    bool ToLocal(const MobilityModel* m, Vector& out) const;

    uint64_t GetEvaluations() const { return m_evals; }
    uint64_t GetBlockedEvaluations() const { return m_blockedEvals; }
    uint64_t GetNonLocalEvaluations() const { return m_nonLocal; }

  private:
    std::vector<Screen> m_screens;
    double m_groundZ{0.0};
    double m_maxLossDb{80.0};
    bool m_hasRef{false};
    double m_refLat{0.0};
    double m_refLon{0.0};
    double m_refAlt{0.0};
    Vector m_refEcef;
    mutable uint64_t m_evals{0};
    mutable uint64_t m_blockedEvals{0};
    mutable uint64_t m_nonLocal{0};
};

/**
 * \ingroup thz-ntn
 * \brief Scalar PropagationLossModel form of NtnScreenBlockageModel at one
 *        carrier frequency (AddExtraPropagationLoss, any backend).
 */
class NtnScreenBlockageLossModel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();
    void SetBlockage(Ptr<NtnScreenBlockageModel> b) { m_model = b; }
    Ptr<NtnScreenBlockageModel> GetBlockage() const { return m_model; }
    void SetFrequency(double f) { m_freqHz = f; }
    double GetLastLossDb() const { return m_last; }

  private:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t) override { return 0; }
    Ptr<NtnScreenBlockageModel> m_model;
    double m_freqHz{2.0e9};
    mutable double m_last{0.0};
};

/**
 * \ingroup thz-ntn
 * \brief Per-RB SpectrumPropagationLossModel form of NtnScreenBlockageModel
 *        (NtnRealStackHelper::AddSpectrumChannelLoss).
 *
 * Optionally restricted to links that have one end in a registered set
 * (e.g. the vehicles): other links pass through.
 */
class NtnScreenBlockageSpectrumModel : public SpectrumPropagationLossModel
{
  public:
    static TypeId GetTypeId();
    void SetBlockage(Ptr<NtnScreenBlockageModel> b) { m_model = b; }
    Ptr<NtnScreenBlockageModel> GetBlockage() const { return m_model; }
    /// Per-link loss at the band centre of the last evaluation of that rx end.
    double GetLastCentreLossDb(const MobilityModel* end) const;

  private:
    Ptr<SpectrumValue> DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                    Ptr<const MobilityModel> a,
                                                    Ptr<const MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t) override { return 0; }
    Ptr<NtnScreenBlockageModel> m_model;
    mutable std::map<const MobilityModel*, double> m_lastByEnd;
    struct CacheEntry
    {
        Time t{Seconds(-1)};
        std::vector<double> loss;
    };
    mutable std::map<std::pair<const void*, const void*>, CacheEntry> m_cache;
};

} // namespace ns3

#endif // NTN_SCREEN_BLOCKAGE_MODEL_H
