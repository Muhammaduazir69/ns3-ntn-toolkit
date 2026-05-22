/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SPLIT_GNB_HELPER_H
#define ORAN_NTN_SPLIT_GNB_HELPER_H

// Composition helper for the CU/CU-UP/DU/RU split (Roadmap §4.1.9).

#include "../model/oran-ntn-f1-interface.h"
#include "../model/oran-ntn-ofh-interface.h"
#include "../model/oran-ntn-split-gnb.h"

#include <ns3/node-container.h>
#include <ns3/ptr.h>

#include <cstdint>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Build a full split-gNB (CU-CP + CU-UP + DU + RU) wired with
 * F1 + OFH transports.
 */
class OranNtnSplitGnbHelper
{
  public:
    struct SplitGnb
    {
        Ptr<OranNtnSplitGnbEntity> cu_cp;
        Ptr<OranNtnSplitGnbEntity> cu_up;
        Ptr<OranNtnSplitGnbEntity> du;
        Ptr<OranNtnSplitGnbEntity> ru;
        Ptr<OranNtnF1Interface> f1;
        Ptr<OranNtnOfhInterface> ofh;
    };

    /// Build entities on *new* ns-3 Nodes (appended to `out_nodes`).
    /// `gnb_id` is shared by all four entities; per-entity id is
    /// derived as `gnb_id * 100 + role`.
    static SplitGnb Build(uint32_t gnb_id, NodeContainer& out_nodes);

    /// Variant taking pre-built nodes (one per entity).
    static SplitGnb BuildOnNodes(uint32_t gnb_id,
                                  Ptr<Node> cu_cp_node,
                                  Ptr<Node> cu_up_node,
                                  Ptr<Node> du_node,
                                  Ptr<Node> ru_node);
};

} // namespace ns3

#endif // ORAN_NTN_SPLIT_GNB_HELPER_H
