/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-ris-service-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ThzNtnRisServiceModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnRisServiceModel);

TypeId
ThzNtnRisServiceModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ThzNtnRisServiceModel")
            .SetParent<Object>()
            .SetGroupName("ThzNtn")
            .AddConstructor<ThzNtnRisServiceModel>()
            .AddAttribute("RisId",
                          "Logical RIS identifier in messages.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&ThzNtnRisServiceModel::m_risId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("IndicationPeriod",
                          "Periodicity of Indication messages.",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&ThzNtnRisServiceModel::m_indicationPeriod),
                          MakeTimeChecker(MilliSeconds(1)));
    return tid;
}

ThzNtnRisServiceModel::ThzNtnRisServiceModel() = default;

ThzNtnRisServiceModel::~ThzNtnRisServiceModel()
{
    StopIndications();
}

void
ThzNtnRisServiceModel::StartIndications()
{
    StopIndications();
    m_indicationEvent =
        Simulator::Schedule(m_indicationPeriod,
                            &ThzNtnRisServiceModel::EmitIndication, this);
}

void
ThzNtnRisServiceModel::StopIndications()
{
    if (m_indicationEvent.IsPending())
    {
        Simulator::Cancel(m_indicationEvent);
    }
}

void
ThzNtnRisServiceModel::SubscribeIndications(IndicationCallback cb)
{
    m_subscribers.push_back(cb);
}

void
ThzNtnRisServiceModel::OnControl(const ControlMessage& ctl)
{
    ++m_ctlAccepted;
    if (ctl.risId != m_risId)
    {
        NS_LOG_WARN("Control rejected: risId mismatch (got "
                    << ctl.risId << ", expected " << m_risId << ")");
        return;
    }
    m_currentPhases = ctl.targetPhaseProfile_rad;
    // Future T4 hook: route through controller's optimizer/codebook here.
    // For v2.x we mirror the controller's view of the phases so the next
    // Indication reports the target phase profile.
}

void
ThzNtnRisServiceModel::OnPolicy(Policy p)
{
    ++m_polUpdates;
    m_policy = p;
}

void
ThzNtnRisServiceModel::EmitIndication()
{
    IndicationMessage msg;
    msg.timestamp = Simulator::Now();
    msg.risId = m_risId;
    msg.phaseProfile_rad = m_currentPhases;
    msg.estimatedChannelGain_dB = m_lastChannelGain_dB;
    msg.snr_dB = 0.0; // intentionally unset; downstream observability
                      // wires SNR from the PHY/MAC stack.
    ++m_indEmitted;
    for (auto& cb : m_subscribers)
    {
        if (!cb.IsNull())
        {
            cb(msg);
        }
    }
    // Reschedule.
    m_indicationEvent =
        Simulator::Schedule(m_indicationPeriod,
                            &ThzNtnRisServiceModel::EmitIndication, this);
}

} // namespace ns3
