/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.4.7)
 */
#include "sagin-slice-router.h"

#include "ns3/log.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SaginSliceRouter");
NS_OBJECT_ENSURE_REGISTERED(SaginSliceRouter);

namespace
{
constexpr double kSpeedOfLightMs = 299792458.0; // m/s
} // namespace

TypeId
SaginSliceRouter::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SaginSliceRouter")
                            .SetParent<Object>()
                            .SetGroupName("NtnSagin")
                            .AddConstructor<SaginSliceRouter>();
    return tid;
}

SaginSliceRouter::SaginSliceRouter()
{
    // Register the three canonical slices by default so RouteForQfi works
    // out-of-the-box.
    AddSlice(ntnslice::DefaultEmbb());
    AddSlice(ntnslice::DefaultUrllc());
    AddSlice(ntnslice::DefaultMmtc());

    // Default QFI → S-NSSAI bands (common 5QI groupings).
    for (uint8_t q = 1; q <= 4; ++q)
    {
        m_qfiMap[q] = {ntnslice::SliceSst::Urllc, ntnslice::kSdUnset};
    }
    for (uint8_t q = 5; q <= 9; ++q)
    {
        m_qfiMap[q] = {ntnslice::SliceSst::Embb, ntnslice::kSdUnset};
    }
    for (uint16_t q = 10; q <= 63; ++q)
    {
        m_qfiMap[static_cast<uint8_t>(q)] =
            {ntnslice::SliceSst::Mmtc, ntnslice::kSdUnset};
    }
}

SaginSliceRouter::~SaginSliceRouter() = default;

void
SaginSliceRouter::AddSlice(const ntnslice::SliceProfile& profile)
{
    m_slices[ntnslice::PackSnssai(profile.snssai)] = profile;
}

void
SaginSliceRouter::SetQfiSlice(uint8_t qfi, const ntnslice::Snssai& snssai)
{
    m_qfiMap[qfi] = snssai;
}

ntnslice::Snssai
SaginSliceRouter::QfiToSnssai(uint8_t qfi) const
{
    auto it = m_qfiMap.find(qfi);
    if (it != m_qfiMap.end())
    {
        return it->second;
    }
    // Unmapped QFI → eMBB by convention.
    return {ntnslice::SliceSst::Embb, ntnslice::kSdUnset};
}

ntnslice::SliceProfile
SaginSliceRouter::GetProfile(const ntnslice::Snssai& s) const
{
    auto it = m_slices.find(ntnslice::PackSnssai(s));
    if (it != m_slices.end())
    {
        return it->second;
    }
    return ntnslice::DefaultEmbb();
}

double
SaginSliceRouter::OneWayLatencyMs(double rangeM)
{
    return (rangeM / kSpeedOfLightMs) * 1000.0;
}

SaginScoreCallback
SaginSliceRouter::MakeSliceScorer(const ntnslice::SliceProfile& profile) const
{
    const double budgetMs = profile.latencyBudgetMs;
    const bool allowGeo = profile.allowGeo;
    const double geoAlt = m_geoAltThresholdM;
    return [budgetMs, allowGeo, geoAlt](const SaginHopCandidate& c) -> double {
        const double neg = -std::numeric_limits<double>::infinity();
        // GEO-equivalent gate: a high-altitude hop is refused by slices
        // that disallow GEO (URLLC).
        const double candAltM = c.node->GetPosition().z;
        if (!allowGeo && candAltM >= geoAlt)
        {
            return neg;
        }
        // Latency-budget gate: the *added* one-way propagation latency of
        // this hop must fit within the slice budget. (Cumulative latency
        // from earlier hops is not double-counted here because lower hops
        // contribute negligibly; the dominant term is the single longest
        // space link, which this gate captures.)
        const double hopLatencyMs = OneWayLatencyMs(c.rangeM);
        if (hopLatencyMs > budgetMs)
        {
            return neg;
        }
        // Feasible: prefer higher elevation (better link budget). Add a
        // small latency-penalty term so that, among equal-elevation
        // candidates, the lower-latency one wins.
        return c.elevationDeg - 0.001 * hopLatencyMs;
    };
}

std::vector<SaginHop>
SaginSliceRouter::RouteForSlice(Ptr<MobilityModel> source,
                                 const ntnslice::Snssai& snssai)
{
    std::vector<SaginHop> empty;
    if (!m_router)
    {
        return empty;
    }
    const ntnslice::SliceProfile profile = GetProfile(snssai);
    m_router->SetScorer(MakeSliceScorer(profile));
    auto path = m_router->Route(source);
    m_router->ClearScorer();
    return path;
}

std::vector<SaginHop>
SaginSliceRouter::RouteForQfi(Ptr<MobilityModel> source, uint8_t qfi)
{
    return RouteForSlice(source, QfiToSnssai(qfi));
}

} // namespace ns3
