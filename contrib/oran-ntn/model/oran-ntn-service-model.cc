/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnServiceModel");

TypeId
OranNtnServiceModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModel")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn");
    return tid;
}

TypeId
OranNtnServiceModelRegistry::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModelRegistry")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnServiceModelRegistry>();
    return tid;
}

OranNtnServiceModelRegistry::OranNtnServiceModelRegistry() = default;

bool
OranNtnServiceModelRegistry::Register(Ptr<OranNtnServiceModel> sm)
{
    if (sm == nullptr)
    {
        NS_LOG_WARN("oran-ntn ServiceModelRegistry: refusing nullptr SM");
        return false;
    }
    const uint16_t id = sm->RicFunctionId();
    if (m_plugins.count(id) != 0)
    {
        NS_LOG_WARN("oran-ntn ServiceModelRegistry: RIC Function ID "
                    << id << " already registered with "
                    << m_plugins[id]->Name() << ", refusing "
                    << sm->Name());
        return false;
    }
    m_plugins[id] = sm;
    NS_LOG_INFO("oran-ntn ServiceModelRegistry: registered " << sm->Name()
                 << " (" << sm->Version() << ") at RIC Function ID " << id);
    return true;
}

Ptr<OranNtnServiceModel>
OranNtnServiceModelRegistry::Lookup(uint16_t id) const
{
    auto it = m_plugins.find(id);
    return (it == m_plugins.end()) ? nullptr : it->second;
}

std::vector<uint16_t>
OranNtnServiceModelRegistry::GetFunctionIds() const
{
    std::vector<uint16_t> ids;
    ids.reserve(m_plugins.size());
    for (const auto& kv : m_plugins)
    {
        ids.push_back(kv.first);
    }
    return ids;
}

} // namespace ns3
