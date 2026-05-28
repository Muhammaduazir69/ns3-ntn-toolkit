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
    SionnaTransport::Request req{0.0, 0.0, g.dist_m,
                                  0.0, 0.0, 0.0,
                                  g.freq_hz, 1,
                                  std::nullopt, std::nullopt, std::nullopt};
    // Note: the los_only flag goes into the JSON `los_only` field on the
    // wire side (the SionnaUdpTransport always emits this); concrete
    // transports decide whether to forward it. For non-UDP transports the
    // hint is harmless.
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
    return rep;
}

} // namespace ns3
