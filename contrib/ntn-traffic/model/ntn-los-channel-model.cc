// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Muhammad Uzair
#include "ntn-los-channel-model.h"

#include "ns3/angles.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/phased-array-model.h"
#include "ns3/simulator.h"

#include <cmath>
#include <complex>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnLosChannelModel");
NS_OBJECT_ENSURE_REGISTERED(NtnLosChannelModel);

namespace
{
constexpr double kC = 299792458.0;
}

TypeId
NtnLosChannelModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnLosChannelModel")
            .SetParent<MatrixBasedChannelModel>()
            .SetGroupName("NtnTraffic")
            .AddConstructor<NtnLosChannelModel>()
            .AddAttribute("Frequency",
                          "Carrier frequency (Hz), read by ThreeGppSpectrumPropagationLossModel.",
                          DoubleValue(2.0e9),
                          MakeDoubleAccessor(&NtnLosChannelModel::m_frequency),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("UpdatePeriod",
                          "Geometry refresh period of the LOS ray (0 = every call).",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&NtnLosChannelModel::m_updatePeriod),
                          MakeTimeChecker());
    return tid;
}

NtnLosChannelModel::NtnLosChannelModel() = default;
NtnLosChannelModel::~NtnLosChannelModel() = default;

Ptr<const MatrixBasedChannelModel::ChannelMatrix>
NtnLosChannelModel::GetChannel(Ptr<const MobilityModel> aMob,
                               Ptr<const MobilityModel> bMob,
                               Ptr<const PhasedArrayModel> aAntenna,
                               Ptr<const PhasedArrayModel> bAntenna)
{
    const uint64_t key = GetKey(aAntenna->GetId(), bAntenna->GetId());
    const Time now = Simulator::Now();
    auto it = m_cache.find(key);
    if (it != m_cache.end() && it->second.matrix &&
        (now - it->second.matrix->m_generatedTime) < m_updatePeriod)
    {
        return it->second.matrix;
    }

    // s = a, u = b for this realization; ThreeGppSpectrumPropagationLossModel
    // handles the reverse direction through ChannelMatrix::IsReverse.
    const Vector sPos = aMob->GetPosition();
    const Vector uPos = bMob->GetPosition();
    const Angles sAngle(uPos, sPos); // departure direction at s, towards u
    const Angles uAngle(sPos, uPos); // arrival direction at u, towards s
    const double lambda = kC / m_frequency;
    const double d3 = CalculateDistance(sPos, uPos);
    const std::complex<double> distPhase(std::cos(-2 * M_PI * d3 / lambda),
                                         std::sin(-2 * M_PI * d3 / lambda));

    const size_t uSize = bAntenna->GetNumElems();
    const size_t sSize = aAntenna->GetNumElems();
    Ptr<ChannelMatrix> m = Create<ChannelMatrix>();
    m->m_channel = Complex3DVector(uSize, sSize, 1);
    const double siU = std::sin(uAngle.GetInclination());
    const double coU = std::cos(uAngle.GetInclination());
    const double saU = std::sin(uAngle.GetAzimuth());
    const double caU = std::cos(uAngle.GetAzimuth());
    const double siS = std::sin(sAngle.GetInclination());
    const double coS = std::cos(sAngle.GetInclination());
    const double saS = std::sin(sAngle.GetAzimuth());
    const double caS = std::cos(sAngle.GetAzimuth());
    // TR 38.901 eq. 7.5-29 with Phi_LOS = 0 (a constant phase has no effect).
    for (size_t u = 0; u < uSize; ++u)
    {
        const Vector uLoc = bAntenna->GetElementLocation(u);
        const double rxPh = 2 * M_PI * (siU * caU * uLoc.x + siU * saU * uLoc.y + coU * uLoc.z);
        auto [rxPhi, rxTheta] =
            bAntenna->GetElementFieldPattern(Angles(uAngle.GetAzimuth(), uAngle.GetInclination()),
                                             bAntenna->GetElemPol(u));
        for (size_t s = 0; s < sSize; ++s)
        {
            const Vector sLoc = aAntenna->GetElementLocation(s);
            const double txPh =
                2 * M_PI * (siS * caS * sLoc.x + siS * saS * sLoc.y + coS * sLoc.z);
            auto [txPhi, txTheta] = aAntenna->GetElementFieldPattern(
                Angles(sAngle.GetAzimuth(), sAngle.GetInclination()),
                aAntenna->GetElemPol(s));
            m->m_channel(u, s, 0) = (rxTheta * txTheta - rxPhi * txPhi) * distPhase *
                                    std::complex<double>(std::cos(rxPh), std::sin(rxPh)) *
                                    std::complex<double>(std::cos(txPh), std::sin(txPh));
        }
    }
    m->m_generatedTime = now;
    m->m_antennaPair = std::make_pair(aAntenna->GetId(), bAntenna->GetId());
    const uint32_t aId = aMob->GetObject<Node>()->GetId();
    const uint32_t bId = bMob->GetObject<Node>()->GetId();
    m->m_nodeIds = std::make_pair(aId, bId);

    // One cluster at the LOS angles, zero delay, no scatterer Doppler.
    Ptr<ChannelParams> p = Create<ChannelParams>();
    p->m_generatedTime = now;
    p->m_nodeIds = m->m_nodeIds;
    p->m_delay = DoubleVector{0.0};
    p->m_angle.resize(4);
    p->m_angle[AOA_INDEX] = DoubleVector{uAngle.GetAzimuth() * 180.0 / M_PI};
    p->m_angle[ZOA_INDEX] = DoubleVector{uAngle.GetInclination() * 180.0 / M_PI};
    p->m_angle[AOD_INDEX] = DoubleVector{sAngle.GetAzimuth() * 180.0 / M_PI};
    p->m_angle[ZOD_INDEX] = DoubleVector{sAngle.GetInclination() * 180.0 / M_PI};
    p->m_cachedAngleSincos.resize(4);
    for (size_t d = 0; d < 4; ++d)
    {
        const double r = p->m_angle[d][0] * M_PI / 180.0;
        p->m_cachedAngleSincos[d] = {std::make_pair(std::sin(r), std::cos(r))};
    }
    p->m_alpha = DoubleVector{0.0};
    p->m_D = DoubleVector{0.0};

    m_cache[key] = Entry{m, p};
    m_cache[GetKey(aId, bId) ^ 0x8000000000000000ULL] = Entry{nullptr, p};
    return m;
}

Ptr<const MatrixBasedChannelModel::ChannelParams>
NtnLosChannelModel::GetParams(Ptr<const MobilityModel> aMob, Ptr<const MobilityModel> bMob) const
{
    const uint32_t aId = aMob->GetObject<Node>()->GetId();
    const uint32_t bId = bMob->GetObject<Node>()->GetId();
    auto it = m_cache.find(GetKey(aId, bId) ^ 0x8000000000000000ULL);
    NS_ASSERT_MSG(it != m_cache.end(), "NtnLosChannelModel: GetParams before GetChannel");
    return it->second.params;
}

} // namespace ns3
