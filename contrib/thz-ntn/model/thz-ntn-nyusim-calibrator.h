/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// thz-ntn-nyusim-calibrator — drives a ThzNtnChannelModel (or any other
// `PropagationLossModel`) at the NYUSIM-140 reference geometries and
// reports residual statistics (max, mean, std). Used by Roadmap §4.3.4
// to ground the toolkit's sub-THz channel claim against the NYU
// WIRELESS published Close-In CI corpus.
//
// LOS calibration is the meaningful gate: the toolkit's FSPL + atmospheric
// stack should reproduce the NYUSIM CI predictor to within ~1 dB at LOS,
// and the residual envelope at NLOS is documented (no clutter model in
// the v2.x baseline — §4.3.4 deliberately leaves clutter for future work).

#ifndef THZ_NTN_NYUSIM_CALIBRATOR_H
#define THZ_NTN_NYUSIM_CALIBRATOR_H

#include "thz-ntn-nyusim-reference.h"

#include <ns3/object.h>
#include <ns3/propagation-loss-model.h>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Drives a propagation model at NYUSIM-140 reference geometries.
 *
 * Usage:
 * \code
 *   Ptr<ThzNtnNyusimReference> ref = CreateObject<ThzNtnNyusimReference>();
 *   ref->Load();
 *   Ptr<ThzNtnNyusimCalibrator> cal = CreateObject<ThzNtnNyusimCalibrator>();
 *   cal->SetReference(ref);
 *   cal->SetModel(myPathLossModel);
 *   const auto rep = cal->Run("urban", true);
 *   NS_ASSERT(rep.max_abs_dB < 3.0);
 * \endcode
 */
class ThzNtnNyusimCalibrator : public Object
{
  public:
    /// Aggregated residual statistics for one (env, scenario) slice.
    struct Report
    {
        std::size_t samples{0};
        double max_abs_dB{0.0};   //!< max |model - ref| (dB)
        double mean_dB{0.0};       //!< mean (model - ref)
        double std_dB{0.0};        //!< sample std of (model - ref)
    };

    /// One point's prediction + reference for inspection.
    struct Point
    {
        ThzNtnNyusimReference::Entry ref;
        double model_pl_db{0.0};
        double residual_dB{0.0};
    };

    static TypeId GetTypeId();

    ThzNtnNyusimCalibrator();
    ~ThzNtnNyusimCalibrator() override;

    void SetReference(Ptr<ThzNtnNyusimReference> ref) { m_ref = ref; }
    Ptr<ThzNtnNyusimReference> GetReference() const { return m_ref; }

    void SetModel(Ptr<PropagationLossModel> model) { m_model = model; }
    Ptr<PropagationLossModel> GetModel() const { return m_model; }

    /// Drive the model at each NYUSIM entry matching (env, los) and
    /// return the per-point residuals.
    std::vector<Point> RunPoints(const std::string& environment, bool los);

    /// Aggregate stats for the same slice; convenience around RunPoints.
    Report Run(const std::string& environment, bool los);

  private:
    /// Compute the model's predicted path loss at a 3-D distance, holding
    /// the Tx at the origin and the Rx along the +x axis. Used because
    /// NYUSIM CI uses scalar distance.
    double ComputeModelPlDb(double dist_m) const;

    Ptr<ThzNtnNyusimReference> m_ref;
    Ptr<PropagationLossModel> m_model;
};

} // namespace ns3

#endif // THZ_NTN_NYUSIM_CALIBRATOR_H
