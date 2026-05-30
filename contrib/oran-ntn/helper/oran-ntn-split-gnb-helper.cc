/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-split-gnb-helper.h"

#include "ns3/log.h"

namespace ns3
{

namespace
{

Ptr<OranNtnSplitGnbEntity>
MakeEntity(OranNodeType role, uint32_t entity_id, uint32_t gnb_id,
            Ptr<Node> node)
{
    auto e = CreateObject<OranNtnSplitGnbEntity>();
    e->Configure(role, entity_id, gnb_id);
    e->SetNode(node);
    e->AdvertiseRoleDefaults();
    return e;
}

} // namespace

OranNtnSplitGnbHelper::SplitGnb
OranNtnSplitGnbHelper::Build(uint32_t gnb_id, NodeContainer& out_nodes)
{
    NodeContainer fresh;
    fresh.Create(4);
    out_nodes.Add(fresh);
    return BuildOnNodes(gnb_id,
                         fresh.Get(0),
                         fresh.Get(1),
                         fresh.Get(2),
                         fresh.Get(3));
}

OranNtnSplitGnbHelper::SplitGnb
OranNtnSplitGnbHelper::BuildOnNodes(uint32_t gnb_id,
                                      Ptr<Node> cu_cp_node,
                                      Ptr<Node> cu_up_node,
                                      Ptr<Node> du_node,
                                      Ptr<Node> ru_node)
{
    SplitGnb g;
    g.cu_cp = MakeEntity(OranNodeType::O_CU_CP,
                          gnb_id * 100 + 0,
                          gnb_id,
                          cu_cp_node);
    g.cu_up = MakeEntity(OranNodeType::O_CU_UP,
                          gnb_id * 100 + 1,
                          gnb_id,
                          cu_up_node);
    g.du = MakeEntity(OranNodeType::O_DU,
                       gnb_id * 100 + 2,
                       gnb_id,
                       du_node);
    g.ru = MakeEntity(OranNodeType::O_RU,
                       gnb_id * 100 + 3,
                       gnb_id,
                       ru_node);

    g.f1 = CreateObject<OranNtnF1Interface>();
    g.f1->Attach(g.cu_cp, g.du);
    g.f1->AttachCuUp(g.cu_up);

    g.ofh = CreateObject<OranNtnOfhInterface>();
    g.ofh->Attach(g.du, g.ru);
    return g;
}

} // namespace ns3
