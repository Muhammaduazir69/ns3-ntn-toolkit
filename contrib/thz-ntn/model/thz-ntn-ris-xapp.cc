/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-ris-xapp.h"

#include "ns3/callback.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRisXapp");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRisXapp);

TypeId
ThzNtnRisXapp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ThzNtnRisXapp")
                            .SetParent<Object>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnRisXapp>();
    return tid;
}

ThzNtnRisXapp::ThzNtnRisXapp() = default;
ThzNtnRisXapp::~ThzNtnRisXapp() = default;

void
ThzNtnRisXapp::Attach(Ptr<ThzNtnRisServiceModel> sm)
{
    m_sm = sm;
    if (!m_sm)
    {
        return;
    }
    m_sm->SubscribeIndications(
        MakeCallback(&ThzNtnRisXapp::OnIndication, this));
}

void
ThzNtnRisXapp::OnIndication(
    const ThzNtnRisServiceModel::IndicationMessage& msg)
{
    ++m_indSeen;
    if (!m_sm)
    {
        return;
    }
    // Translate the active A1 policy into a controller optimisation
    // method choice on every received Indication.
    OptimizationMethod method =
        OptimizationMethod::CODEBOOK;
    switch (m_sm->GetActivePolicy())
    {
    case ThzNtnRisServiceModel::Policy::RANDOM_SEARCH:
        method = OptimizationMethod::RANDOM_SEARCH;
        break;
    case ThzNtnRisServiceModel::Policy::ALTERNATING_OPT:
        method = OptimizationMethod::ALTERNATING_OPT;
        break;
    case ThzNtnRisServiceModel::Policy::DRL_BASED:
        method = OptimizationMethod::DRL_BASED;
        break;
    case ThzNtnRisServiceModel::Policy::CODEBOOK:
    default:
        method = OptimizationMethod::CODEBOOK;
        break;
    }

    // Build a control message. For the in-process v2.x baseline, the
    // target phase profile is the indication's current profile (the xApp
    // simply confirms the current state). A real controller would issue
    // new phases derived from `method`; the choice of method is what we
    // exercise here.
    ThzNtnRisServiceModel::ControlMessage ctl;
    ctl.risId = msg.risId;
    ctl.targetPhaseProfile_rad = msg.phaseProfile_rad;
    ctl.method = method;
    m_sm->OnControl(ctl);
    ++m_ctlSent;
}

} // namespace ns3
