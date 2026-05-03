/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-rrc-helper.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnRrcHelper");

namespace ntnrrc
{

NtnRrcHelper::NtnRrcHelper() = default;

void
NtnRrcHelper::SetPayloadMode(PayloadMode mode)
{
    m_payloadMode = mode;
}

void
NtnRrcHelper::SetReferencePosition(const Vector& earthFixedRefPosition)
{
    m_referencePos = earthFixedRefPosition;
}

Ptr<NtnTimingAdvance>
NtnRrcHelper::InstallTimingAdvance(Ptr<MobilityModel> ueMob, Ptr<MobilityModel> satMob) const
{
    Ptr<NtnTimingAdvance> ta = CreateObject<NtnTimingAdvance>();
    ta->SetUeMobility(ueMob);
    ta->SetSatelliteMobility(satMob);
    ta->SetReferencePosition(m_referencePos);
    ta->SetPayloadMode(m_payloadMode);
    return ta;
}

} // namespace ntnrrc
} // namespace ns3
