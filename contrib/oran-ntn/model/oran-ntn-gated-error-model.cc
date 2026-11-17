/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "oran-ntn-gated-error-model.h"

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(OranNtnGatedErrorModel);

TypeId
OranNtnGatedErrorModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnGatedErrorModel")
                            .SetParent<ErrorModel>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnGatedErrorModel>();
    return tid;
}

OranNtnGatedErrorModel::OranNtnGatedErrorModel() = default;

std::function<bool()>
OranNtnGatedErrorModel::MirrorOutage(Ptr<RateErrorModel> control)
{
    return [control]() { return control && control->GetRate() >= 1.0; };
}

bool
OranNtnGatedErrorModel::DoCorrupt(Ptr<Packet> p)
{
    if (m_down && m_down())
    {
        ++m_gated;
        return true;
    }
    if (m_inner && m_inner->IsCorrupt(p))
    {
        ++m_innerDrops;
        return true;
    }
    ++m_passed;
    return false;
}

void
OranNtnGatedErrorModel::DoReset()
{
    m_gated = 0;
    m_innerDrops = 0;
    m_passed = 0;
}

} // namespace ns3
