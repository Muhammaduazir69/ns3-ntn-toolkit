/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit)
 */
#include "ntn-tdl-spectrum-propagation-loss-model.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnTdlSpectrumPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(NtnTdlSpectrumPropagationLossModel);

TypeId
NtnTdlSpectrumPropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnTdlSpectrumPropagationLossModel")
            .SetParent<SpectrumPropagationLossModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnTdlSpectrumPropagationLossModel>();
    return tid;
}

NtnTdlSpectrumPropagationLossModel::NtnTdlSpectrumPropagationLossModel()
    : m_tdl(CreateObject<NtnTdlSpectrumLossModel>())
{
}

void
NtnTdlSpectrumPropagationLossModel::SetTdl(Ptr<NtnTdlSpectrumLossModel> tdl)
{
    NS_ABORT_MSG_IF(!tdl, "NtnTdlSpectrumPropagationLossModel needs a tapped-delay-line");
    m_tdl = tdl;
}

double
NtnTdlSpectrumPropagationLossModel::ElevationDeg(Ptr<const MobilityModel> a,
                                                 Ptr<const MobilityModel> b)
{
    const Vector pa = a->GetPosition();
    const Vector pb = b->GetPosition();
    const double ra = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    const double rb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    const Vector ground = (ra <= rb) ? pa : pb;
    const Vector sat = (ra <= rb) ? pb : pa;
    const double rGround = std::min(ra, rb);

    const Vector los(sat.x - ground.x, sat.y - ground.y, sat.z - ground.z);
    const double d = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (d < 1.0)
    {
        return 90.0;
    }
    // Geocentric up when the ground node really is on the globe; local +z for a
    // scenario laid out in a flat ENU frame. Same rule as
    // Ntn38811ExcessLossModel, deliberately, so the two cannot disagree about
    // the elevation they were handed.
    Vector up(0.0, 0.0, 1.0);
    if (rGround > 0.5 * 6371000.0)
    {
        up = Vector(ground.x / rGround, ground.y / rGround, ground.z / rGround);
    }
    double sinElev = (los.x * up.x + los.y * up.y + los.z * up.z) / d;
    sinElev = std::max(-1.0, std::min(1.0, sinElev));
    return std::asin(sinElev) * 180.0 / M_PI;
}

Ptr<SpectrumValue>
NtnTdlSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity(
    Ptr<const SpectrumSignalParameters> params,
    Ptr<const MobilityModel> a,
    Ptr<const MobilityModel> b) const
{
    Ptr<SpectrumValue> rx = params->psd->Copy();
    if (!m_tdl || !a || !b || !rx->GetSpectrumModel())
    {
        ++m_skipped;
        return rx;
    }

    // Absolute subcarrier centres, which is what the tapped-delay-line needs to
    // give exp(-j 2 pi f tau) a meaningful f.
    std::vector<double> freqs;
    freqs.reserve(rx->GetSpectrumModel()->GetNumBands());
    for (auto it = rx->GetSpectrumModel()->Begin(); it != rx->GetSpectrumModel()->End(); ++it)
    {
        freqs.push_back(it->fc);
    }
    if (freqs.empty())
    {
        ++m_skipped;
        return rx;
    }

    m_tdl->ApplyTo(rx, freqs, ElevationDeg(a, b));
    ++m_applied;
    return rx;
}

int64_t
NtnTdlSpectrumPropagationLossModel::DoAssignStreams(int64_t stream)
{
    // NtnTdlSpectrumLossModel owns its own normal variates and exposes no
    // stream API, so there is nothing to hand down. Returning 0 is the honest
    // answer rather than pretending a stream was assigned.
    return 0;
}

} // namespace ns3
