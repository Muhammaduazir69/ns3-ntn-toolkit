/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-nyusim-calibrator.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnNyusimCalibrator");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnNyusimCalibrator);

TypeId
ThzNtnNyusimCalibrator::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThzNtnNyusimCalibrator")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnNyusimCalibrator>();
    return tid;
}

ThzNtnNyusimCalibrator::ThzNtnNyusimCalibrator() = default;
ThzNtnNyusimCalibrator::~ThzNtnNyusimCalibrator() = default;

double
ThzNtnNyusimCalibrator::ComputeModelPlDb(double dist_m) const
{
    if (!m_model)
    {
        return 0.0;
    }
    Ptr<ConstantPositionMobilityModel> tx =
        CreateObject<ConstantPositionMobilityModel>();
    Ptr<ConstantPositionMobilityModel> rx =
        CreateObject<ConstantPositionMobilityModel>();
    tx->SetPosition(Vector(0.0, 0.0, 0.0));
    rx->SetPosition(Vector(dist_m, 0.0, 0.0));
    const double txDbm = 0.0;
    const double rxDbm = m_model->CalcRxPower(txDbm, tx, rx);
    // PathLoss = TxPower - RxPower (PL in dB is the positive attenuation).
    return txDbm - rxDbm;
}

std::vector<ThzNtnNyusimCalibrator::Point>
ThzNtnNyusimCalibrator::RunPoints(const std::string& environment, bool los)
{
    std::vector<Point> out;
    if (!m_ref || !m_model)
    {
        return out;
    }
    const auto entries = m_ref->Select(environment, los);
    out.reserve(entries.size());
    for (const auto& e : entries)
    {
        Point p;
        p.ref = e;
        p.model_pl_db = ComputeModelPlDb(e.dist_m);
        p.residual_dB = p.model_pl_db - e.pl_db;
        out.push_back(p);
    }
    return out;
}

ThzNtnNyusimCalibrator::Report
ThzNtnNyusimCalibrator::Run(const std::string& environment, bool los)
{
    Report rep;
    const auto pts = RunPoints(environment, los);
    if (pts.empty())
    {
        return rep;
    }
    double sum = 0.0;
    double sumSq = 0.0;
    double maxAbs = 0.0;
    for (const auto& p : pts)
    {
        sum += p.residual_dB;
        sumSq += p.residual_dB * p.residual_dB;
        maxAbs = std::max(maxAbs, std::abs(p.residual_dB));
    }
    rep.samples = pts.size();
    rep.mean_dB = sum / pts.size();
    const double var = sumSq / pts.size() - rep.mean_dB * rep.mean_dB;
    rep.std_dB = std::sqrt(std::max(0.0, var));
    rep.max_abs_dB = maxAbs;
    return rep;
}

} // namespace ns3
