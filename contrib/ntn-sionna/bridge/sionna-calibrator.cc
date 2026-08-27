/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.3.3)
 */
#include "sionna-calibrator.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaCalibrator");
NS_OBJECT_ENSURE_REGISTERED(SionnaCalibrator);

TypeId
SionnaCalibrator::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaCalibrator")
                            .SetParent<Object>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaCalibrator>();
    return tid;
}

SionnaCalibrator::SionnaCalibrator()
{
    UseDefaultGrid();
}

SionnaCalibrator::~SionnaCalibrator() = default;

void
SionnaCalibrator::UseDefaultGrid()
{
    // 3 carrier frequencies × 6 ground-to-satellite distances.
    const double freqs[] = {2.0e9, 12.0e9, 28.0e9};
    // SIONNA-07: the top of this grid is well outside what the reference scene
    // can answer meaningfully. Sionna's `simple_reflector` scene is a small
    // room-scale geometry; probing it at 1,000,000 m asks the ray tracer about a
    // world that does not exist there, and the answer it returns is a free-space
    // fall-through rather than a traced path. The distances stay, because a
    // slant-range sweep is what a calibration wants, and the caller is told which
    // of them lie outside the scene.
    const double dists[] = {1000.0, 5000.0, 50000.0, 200000.0, 500000.0, 1000000.0};
    m_grid.clear();
    m_grid.reserve(18);
    for (double f : freqs)
    {
        for (double d : dists)
        {
            m_grid.push_back({f, d});
        }
    }
}

double
SionnaCalibrator::QuerySionnaDb(const GridPoint& g, bool& ok) const
{
    ok = false;
    if (!m_transport)
    {
        return 0.0;
    }
    // SIONNA-01: los_only is now an explicit field on the Request and the UDP
    // transport emits it. The comment that used to sit here claimed the
    // transport "always emits this"; it did not, because there was no field to
    // emit, so every query reached the server without the key and took its
    // default. The calibrator wants the matched-scenario LOS reference, so it
    // asks for it rather than relying on a default.
    SionnaTransport::Request req{0.0, 0.0, g.dist_m,
                                  0.0, 0.0, 0.0,
                                  g.freq_hz, 1,
                                  // SIONNA-07: ask for what the report will CLAIM.
                                  //
                                  // This was hardcoded true while the report set
                                  // `rep.los_only = m_losOnly`, so a calibrator
                                  // configured with SetLosOnly(false) sent
                                  // los_only=true on every query and then
                                  // reported los_only=false. The flag on the
                                  // report described a configuration, not the
                                  // run.
                                  /*los_only=*/m_losOnly,
                                  std::nullopt, std::nullopt, std::nullopt};
    SionnaTransport::Response rsp = m_transport->Query(req);
    if (rsp.ok && std::isfinite(rsp.path_loss_db))
    {
        ok = true;
        return rsp.path_loss_db;
    }
    return 0.0;
}

double
SionnaCalibrator::QueryModelDb(const GridPoint& g) const
{
    if (!m_model)
    {
        return 0.0;
    }
    Ptr<ConstantPositionMobilityModel> a =
        CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> b =
        CreateObject<ConstantPositionMobilityModel>();
    a->SetPosition(Vector(0, 0, g.dist_m));
    b->SetPosition(Vector(0, 0, 0));
    const double txDbm = 0.0;
    const double rxDbm = m_model->CalcRxPower(txDbm, a, b);
    return txDbm - rxDbm;
}

std::vector<SionnaCalibrator::PointResult>
SionnaCalibrator::RunPoints()
{
    std::vector<PointResult> out;
    out.reserve(m_grid.size());
    for (const auto& g : m_grid)
    {
        PointResult p;
        p.grid = g;
        p.sionna_pl_db = QuerySionnaDb(g, p.sionna_ok);
        p.model_pl_db = QueryModelDb(g);
        p.residual_dB = p.sionna_ok ? (p.model_pl_db - p.sionna_pl_db) : 0.0;
        out.push_back(p);
    }
    return out;
}

SionnaCalibrator::Report
SionnaCalibrator::Run()
{
    Report rep;
    rep.los_only = m_losOnly;
    // SIONNA-07: record how far the sweep reached, so a report cannot silently
    // include probes the reference scene cannot answer.
    rep.max_distance_m = 0.0;
    for (const auto& g : m_grid)
    {
        rep.max_distance_m = std::max(rep.max_distance_m, g.dist_m);
    }
    const auto pts = RunPoints();
    double sum = 0.0;
    double sumSq = 0.0;
    double maxAbs = 0.0;
    std::size_t n = 0;
    for (const auto& p : pts)
    {
        if (!p.sionna_ok)
        {
            ++rep.sionna_failures;
            continue;
        }
        sum += p.residual_dB;
        sumSq += p.residual_dB * p.residual_dB;
        maxAbs = std::max(maxAbs, std::abs(p.residual_dB));
        ++n;
    }
    rep.samples = n;
    if (n > 0)
    {
        rep.mean_dB = sum / n;
        const double var = sumSq / n - rep.mean_dB * rep.mean_dB;
        rep.std_dB = std::sqrt(std::max(0.0, var));
        rep.max_abs_dB = maxAbs;
    }
    // SIONNA-07: was this a calibration or a harness check?
    //
    // The shipped test drives FsplUdpMockServer on BOTH sides, so the model
    // answer and the "Sionna" answer are the same closed form and the delta is
    // zero by construction. That is a useful harness check and it is not a
    // calibration against a ray tracer, and a report that does not distinguish
    // them invites the second reading. Near-zero spread across the whole sweep
    // is the signature.
    rep.both_sides_closed_form = (n > 2) && (rep.max_abs_dB < 1e-6);

    return rep;
}

} // namespace ns3
