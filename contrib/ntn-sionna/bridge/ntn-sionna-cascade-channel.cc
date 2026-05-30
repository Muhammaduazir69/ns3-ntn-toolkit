/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.7)
 */
#include "ntn-sionna-cascade-channel.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/pointer.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSionnaCascadeChannel");
NS_OBJECT_ENSURE_REGISTERED(NtnSionnaCascadeChannel);

TypeId
NtnSionnaCascadeChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnSionnaCascadeChannel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnSionna")
            .AddConstructor<NtnSionnaCascadeChannel>()
            .AddAttribute("Base",
                          "Inner NtnSionnaChannel responsible for multipath.",
                          PointerValue(),
                          MakePointerAccessor(&NtnSionnaCascadeChannel::m_base),
                          MakePointerChecker<NtnSionnaChannel>())
            .AddAttribute("Chain",
                          "Inner NtnAtmosphericLossChain for ITU-R cascade.",
                          PointerValue(),
                          MakePointerAccessor(&NtnSionnaCascadeChannel::m_chain),
                          MakePointerChecker<NtnAtmosphericLossChain>());
    return tid;
}

NtnSionnaCascadeChannel::NtnSionnaCascadeChannel()
    : m_cascadeQueries(0),
      m_lastTotalDb(0.0)
{
}

NtnSionnaCascadeChannel::~NtnSionnaCascadeChannel() = default;

void
NtnSionnaCascadeChannel::EnsureSubModels()
{
    if (!m_base)
    {
        m_base = CreateObject<NtnSionnaChannel>();
    }
    if (!m_chain)
    {
        m_chain = CreateObject<NtnAtmosphericLossChain>();
    }
}

void
NtnSionnaCascadeChannel::SetSionnaChannel(Ptr<NtnSionnaChannel> base)
{
    m_base = base;
}

void
NtnSionnaCascadeChannel::SetAtmosphericChain(Ptr<NtnAtmosphericLossChain> chain)
{
    m_chain = chain;
}

void
NtnSionnaCascadeChannel::SetTransport(Ptr<SionnaTransport> transport)
{
    EnsureSubModels();
    m_base->SetTransport(transport);
}

Ptr<SionnaTransport>
NtnSionnaCascadeChannel::GetTransport() const
{
    return m_base ? m_base->GetTransport() : nullptr;
}

void
NtnSionnaCascadeChannel::SetFrequencyHz(double freqHz)
{
    if (!m_base)
    {
        m_base = CreateObject<NtnSionnaChannel>();
    }
    if (!m_chain)
    {
        m_chain = CreateObject<NtnAtmosphericLossChain>();
    }
    m_base->SetFrequencyHz(freqHz);
    m_chain->SetFrequencyHz(freqHz);
}

double
NtnSionnaCascadeChannel::GetFrequencyHz() const
{
    return m_chain ? m_chain->GetFrequencyHz() : 0.0;
}

NtnAtmosphericLossChain::Components
NtnSionnaCascadeChannel::GetLastComponents() const
{
    if (!m_chain)
    {
        return NtnAtmosphericLossChain::Components{};
    }
    return m_chain->GetLastComponents();
}

double
NtnSionnaCascadeChannel::DoCalcRxPower(double txPowerDbm,
                                        Ptr<MobilityModel> a,
                                        Ptr<MobilityModel> b) const
{
    // Lazy construction so the cascade can be instantiated by typeid
    // (e.g. via Config::Connect) before any attributes are set.
    if (!m_base)
    {
        const_cast<NtnSionnaCascadeChannel*>(this)->m_base =
            CreateObject<NtnSionnaChannel>();
    }
    if (!m_chain)
    {
        const_cast<NtnSionnaCascadeChannel*>(this)->m_chain =
            CreateObject<NtnAtmosphericLossChain>();
    }

    // 1. Base Sionna RT (or FSPL fall-back) path-loss → Rx power.
    const double rxBaseDbm = m_base->CalcRxPower(txPowerDbm, a, b);

    // 2. ITU-R cascade — gaseous + rain (geometry-driven) + LMS (Markov).
    //    Auto-detect which mobility is the ground end: in both ENU and
    //    ECEF frames the satellite sits further from the origin, so we
    //    pass the lower-magnitude position as the ground. This keeps the
    //    cascade commutative with respect to (a, b) ordering, matching
    //    the symmetry that FSPL-style PropagationLossModels rely on.
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();
    const double magA =
        std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
    const double magB =
        std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
    Vector ground = (magA <= magB) ? pa : pb;
    Vector sat = (magA <= magB) ? pb : pa;
    const double atmosDb = m_chain->ComputeAttenuationDb(ground, sat);
    const double lmsDb = m_chain->StepLmsDb();
    m_lastTotalDb = atmosDb + lmsDb;
    ++m_cascadeQueries;

    return rxBaseDbm - atmosDb - lmsDb;
}

int64_t
NtnSionnaCascadeChannel::DoAssignStreams(int64_t stream)
{
    EnsureSubModels();
    return m_chain->AssignStreams(stream);
}

} // namespace ns3
