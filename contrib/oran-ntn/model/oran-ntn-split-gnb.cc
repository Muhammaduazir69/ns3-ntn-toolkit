/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-split-gnb.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnSplitGnbEntity");

TypeId
OranNtnSplitGnbEntity::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnSplitGnbEntity")
                            .SetParent<Object>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnSplitGnbEntity>();
    return tid;
}

OranNtnSplitGnbEntity::OranNtnSplitGnbEntity()
    : m_e2Node(CreateObject<OranNtnE2Node>())
{
}

OranNtnSplitGnbEntity::~OranNtnSplitGnbEntity() = default;

void
OranNtnSplitGnbEntity::DoDispose()
{
    m_e2Node = nullptr;
    m_node = nullptr;
    m_controlCb = nullptr;
    Object::DoDispose();
}

void
OranNtnSplitGnbEntity::Configure(OranNodeType role,
                                   uint32_t entity_id,
                                   uint32_t gnb_id)
{
    m_role = role;
    m_entityId = entity_id;
    m_gnbId = gnb_id;
    if (m_e2Node)
    {
        m_e2Node->SetNodeId(entity_id);
    }
}

void
OranNtnSplitGnbEntity::AdvertiseRanFunction(
    const RanFunctionAdvertised& rf)
{
    for (auto& existing : m_ranFunctions)
    {
        if (existing.ric_function_id == rf.ric_function_id)
        {
            existing = rf;
            if (m_e2Node)
            {
                m_e2Node->RegisterRanFunction(rf.ric_function_id,
                                                rf.description);
            }
            return;
        }
    }
    m_ranFunctions.push_back(rf);
    if (m_e2Node)
    {
        m_e2Node->RegisterRanFunction(rf.ric_function_id,
                                        rf.description);
    }
}

bool
OranNtnSplitGnbEntity::HasRanFunction(uint16_t ric_function_id) const
{
    for (const auto& rf : m_ranFunctions)
    {
        if (rf.ric_function_id == ric_function_id)
        {
            return true;
        }
    }
    return false;
}

void
OranNtnSplitGnbEntity::AdvertiseRoleDefaults()
{
    switch (m_role)
    {
    case OranNodeType::O_CU_CP:
        AdvertiseRanFunction({/*id=*/3,
                                "E2SM-RC",
                                "v1.03",
                                "Mobility ControlAction (Style 3)"});
        break;
    case OranNodeType::O_CU_UP:
        AdvertiseRanFunction({/*id=*/147,
                                "E2SM-KPM",
                                "v3.00",
                                "CU-UP PDCP/SDAP volume metrics"});
        break;
    case OranNodeType::O_DU:
        AdvertiseRanFunction({/*id=*/147,
                                "E2SM-KPM",
                                "v3.00",
                                "DU PHY/MAC KPM"});
        break;
    case OranNodeType::O_RU:
        AdvertiseRanFunction({/*id=*/1000,
                                "E2SM-CCC",
                                "v1.00",
                                "Cell Configuration and Control"});
        AdvertiseRanFunction({/*id=*/1001,
                                "E2SM-NTN-Ephemeris",
                                "v1.00",
                                "SIB19 ephemeris publisher"});
        break;
    default:
        break;
    }
}

bool
OranNtnSplitGnbEntity::SubmitKpmIndication(const E2KpmReport& report)
{
    if (!HasRanFunction(147))
    {
        return false;
    }
    if (m_e2Node)
    {
        m_e2Node->SubmitKpmMeasurement(report);
    }
    ++m_indicationsEmitted;
    return true;
}

bool
OranNtnSplitGnbEntity::ReceiveControl(const E2RcAction& act)
{
    if (m_role != OranNodeType::O_CU_CP &&
        m_role != OranNodeType::O_CU_UP &&
        m_role != OranNodeType::O_DU &&
        m_role != OranNodeType::O_RU)
    {
        return false;
    }
    // Per WG3 spec mapping: mobility/RRC ControlActions belong on the
    // CU-CP; RF-touching CCC actions on the RU; DU handles
    // scheduler-level controls only. Reject mis-routed actions.
    switch (act.actionType)
    {
    case E2RcActionType::HANDOVER_TRIGGER:
    case E2RcActionType::HANDOVER_CANCEL:
    case E2RcActionType::DC_SETUP:
    case E2RcActionType::DC_TEARDOWN:
        if (m_role != OranNodeType::O_CU_CP)
        {
            return false;
        }
        break;
    case E2RcActionType::BEAM_SWITCH:
    case E2RcActionType::BEAM_HOP_SCHEDULE:
    case E2RcActionType::TX_POWER_CONTROL:
    case E2RcActionType::DOPPLER_COMP_UPDATE:
    case E2RcActionType::TIMING_ADVANCE_UPDATE:
    case E2RcActionType::ACTION_THZ_RIS_CONFIG:
        if (m_role != OranNodeType::O_RU)
        {
            return false;
        }
        break;
    case E2RcActionType::SLICE_PRB_ALLOCATION:
    case E2RcActionType::MCS_OVERRIDE:
    case E2RcActionType::PRB_RESERVATION:
        if (m_role != OranNodeType::O_DU)
        {
            return false;
        }
        break;
    default:
        // Other action types are role-agnostic; accept anywhere.
        break;
    }
    ++m_controlsAccepted;
    if (m_controlCb)
    {
        m_controlCb(act);
    }
    return true;
}

} // namespace ns3
