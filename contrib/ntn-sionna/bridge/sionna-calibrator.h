/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.3.3)
 *
 * sionna-calibrator — drives a propagation model (any `PropagationLossModel`)
 * at a curated grid of (freq, dist) points alongside a Sionna RT transport
 * and reports residual statistics. The roadmap's §4.3.3 gate is that
 * LOS-only Sionna queries reproduce the closed-form FSPL inside the model
 * to within 1 dB; the multipath envelope is documented per scenario.
 *
 * Two modes:
 *
 *   LOS-only mode: every query carries `los_only=true` so the server only
 *   computes direct paths (or, equivalently, an empty scene returns FSPL).
 *   The expected residual against a pure-FSPL model is ~0 dB.
 *
 *   Multipath mode: the server runs the full path solver with reflections,
 *   refractions, and diffractions. The residual depends on scene geometry
 *   and is not bounded a priori — calibrator reports the empirical envelope
 *   for downstream documentation.
 *
 * The calibrator is decoupled from any specific module — it sees only the
 * abstract SionnaTransport surface. Thz-NTN integration lives in the
 * companion `examples/thz-ntn-vs-sionna-calibration.cc` example.
 */
#ifndef SIONNA_CALIBRATOR_H
#define SIONNA_CALIBRATOR_H

#include "sionna-transport.h"

#include "ns3/object.h"
#include "ns3/propagation-loss-model.h"

#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup ntn-sionna
 *
 * \brief Sionna RT vs. propagation-model residual calibrator.
 */
class SionnaCalibrator : public Object
{
  public:
    /// One (freq, dist) sample point.
    struct GridPoint
    {
        double freq_hz;
        double dist_m;
    };

    /// Per-point calibration result.
    struct PointResult
    {
        GridPoint grid;
        double sionna_pl_db;
        double model_pl_db;
        double residual_dB; //!< model - sionna
        bool sionna_ok;     //!< whether the Sionna query succeeded
    };

    /// Aggregate residual stats over the calibration grid.
    struct Report
    {
        std::size_t samples{0};
        std::size_t sionna_failures{0};
        double max_abs_dB{0.0};
        double mean_dB{0.0};
        double std_dB{0.0};
        bool los_only{false};
        /// SIONNA-07: the largest grid distance, so a reader can see when the
        /// sweep ran past what the reference scene can answer. Sionna's
        /// `simple_reflector` is room-scale; a kilometre-plus probe returns a
        /// free-space fall-through rather than a traced path, and a calibration
        /// report that does not say so reads as agreement with a ray tracer.
        double max_distance_m{0.0};
        /// True when the whole sweep was answered by the model side and the
        /// Sionna side with the SAME closed form, i.e. the report compares free
        /// space against free space and validates the harness rather than the
        /// channel.
        bool both_sides_closed_form{false};
    };

    static TypeId GetTypeId();

    SionnaCalibrator();
    ~SionnaCalibrator() override;

    void SetTransport(Ptr<SionnaTransport> transport) { m_transport = transport; }
    Ptr<SionnaTransport> GetTransport() const { return m_transport; }

    void SetModel(Ptr<PropagationLossModel> model) { m_model = model; }
    Ptr<PropagationLossModel> GetModel() const { return m_model; }

    /// True (default) sends `los_only` requests to the Sionna server so
    /// reflections off the scene are suppressed; false runs the full
    /// multipath solver.
    void SetLosOnly(bool los) { m_losOnly = los; }
    bool GetLosOnly() const { return m_losOnly; }

    /// Build a default grid: 3 freqs × 6 distances per default.
    void UseDefaultGrid();
    void SetGrid(const std::vector<GridPoint>& grid) { m_grid = grid; }
    const std::vector<GridPoint>& GetGrid() const { return m_grid; }

    /// Run the calibration. Returns per-point results.
    std::vector<PointResult> RunPoints();

    /// Aggregate stats; convenience around RunPoints.
    Report Run();

  private:
    double QuerySionnaDb(const GridPoint& g, bool& ok) const;
    double QueryModelDb(const GridPoint& g) const;

    Ptr<SionnaTransport> m_transport;
    Ptr<PropagationLossModel> m_model;
    std::vector<GridPoint> m_grid;
    bool m_losOnly{true};
};

} // namespace ns3

#endif // SIONNA_CALIBRATOR_H
