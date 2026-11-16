/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-ue-sib19-consumer.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>
#include <ns3/simulator.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnUeSib19Consumer");

namespace ntnrrc
{

NS_OBJECT_ENSURE_REGISTERED(NtnUeSib19Consumer);

TypeId
NtnUeSib19Consumer::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ntnrrc::NtnUeSib19Consumer")
                            .SetParent<Object>()
                            .SetGroupName("NtnRrc")
                            .AddConstructor<NtnUeSib19Consumer>();
    return tid;
}

NtnUeSib19Consumer::NtnUeSib19Consumer() = default;
NtnUeSib19Consumer::~NtnUeSib19Consumer() = default;

void
NtnUeSib19Consumer::DoDispose()
{
    m_pc = nullptr;
    m_truth = nullptr;
    Object::DoDispose();
}

void
NtnUeSib19Consumer::Setup(Ptr<MobilityModel> ue, Ptr<NtnTimingAdvance> truth, double gnssSigmaM, Time gnssPeriod)
{
    NS_ABORT_MSG_IF(!ue || !truth, "NtnUeSib19Consumer::Setup needs the UE mobility and the truth TA");
    m_truth = truth;
    m_pc = CreateObject<NtnUlTimingPrecompensator>();
    m_pc->SetUeMobility(ue);
    m_pc->SetGnss(gnssPeriod, gnssSigmaM);
}

bool
NtnUeSib19Consumer::ReceiveWire(const uint8_t* data, std::size_t len)
{
    Sib19Content parsed;
    if (!data || !Sib19Codec::Parse(data, len, parsed))
    {
        return false;
    }
    m_held = parsed;
    ++m_decoded;
    if (m_pc)
    {
        m_pc->ReceiveSib19(parsed); // TS 38.331 5.2.2.4: the UE stores what it acquired
    }
    return true;
}

NtnUeSib19Consumer::SampleResult
NtnUeSib19Consumer::Sample()
{
    SampleResult r;
    if (!m_pc || !m_pc->HasSib19())
    {
        return r;
    }
    r.valid = true;
    const double est = m_pc->ComputeTa().GetSeconds(); // TS 38.213 4.2 UE-side TA
    const double truth = m_truth->ComputeTotalTa().GetSeconds();
    r.estUs = est * 1e6;
    r.truthUs = truth * 1e6;
    r.errNs = (est - truth) * 1e9;
    r.ageMs = m_pc->GetInformationAge().GetSeconds() * 1e3;
    r.synced = m_pc->IsUplinkSynchronized();
    ++m_samples;
    m_synced += r.synced ? 1 : 0;
    m_viol += (std::fabs(r.errNs) > m_te.GetSeconds() * 1e9) ? 1 : 0;
    m_maxAbsErrNs = std::max(m_maxAbsErrNs, std::fabs(r.errNs));
    return r;
}

} // namespace ntnrrc
} // namespace ns3
