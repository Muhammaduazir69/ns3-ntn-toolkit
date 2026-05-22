/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SPLIT_GNB_H
#define ORAN_NTN_SPLIT_GNB_H

// CU / DU / RU split per O-RAN.WG3 (Roadmap §4.1.9).
//
// A real O-RAN gNB is built out of separate logical entities:
//
//   * O-CU-CP — RRC, X2-CP / Xn-CP, F1AP-CP
//   * O-CU-UP — SDAP, PDCP-UP, F1AP-UP
//   * O-DU    — RLC, MAC, high-PHY, F1AP-DU side
//   * O-RU    — low-PHY, RF, Open Fronthaul (eCPRI / RoE) DU side
//
// Each terminates E2 independently. In WG3-spec deployments the CU-CP
// advertises mobility-related RAN functions (E2SM-RC Style 3 HO), the
// DU advertises PHY/MAC KPM (E2SM-KPM), and the RU advertises radio
// configuration (E2SM-CCC for NTN extensions like LEO pass on/off,
// Doppler retune, beam reconfig).
//
// This module models the split as four `OranNtnSplitGnbEntity`
// instances tied together by `OranNtnF1Interface` (CU↔DU) and
// `OranNtnOfhInterface` (DU↔RU). The composition helper
// `OranNtnSplitGnbHelper` builds a complete logical gNB out of them.
//
// Each entity owns:
//   * one ns-3 `Node` (so users can attach NetDevices, mobility, etc.)
//   * one `OranNtnE2Node` (independent E2 termination)
//   * a list of advertised `RanFunctionAdvertised` records — drives
//     which RIC Function IDs the entity responds to in E2 Subscribe

#include "oran-ntn-e2-interface.h"
#include "oran-ntn-types.h"

#include <ns3/node.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/// One RAN function advertised by a split-gNB entity in its E2 Setup.
struct RanFunctionAdvertised
{
    uint16_t ric_function_id; //!< RIC Function ID (3=RC, 147=KPM, 1000=CCC, 1001=NTN-Ephemeris)
    std::string sm_name;       //!< e.g. "E2SM-KPM"
    std::string sm_version;    //!< e.g. "v3.00"
    /// Free-form description (matches O-RAN E2AP RANfunctionDefinition tag).
    std::string description;
};

/**
 * \ingroup oran-ntn
 * \brief One split-gNB entity (CU-CP / CU-UP / DU / RU).
 *
 * Holds an ns-3 Node aggregate, an OranNtnE2Node, and the set of
 * advertised RAN functions for its role.
 */
class OranNtnSplitGnbEntity : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnSplitGnbEntity();
    ~OranNtnSplitGnbEntity() override;

    /// Set role + numeric entity id. Once set, role is immutable.
    void Configure(OranNodeType role,
                    uint32_t entity_id,
                    uint32_t gnb_id);

    OranNodeType GetRole() const { return m_role; }
    uint32_t GetEntityId() const { return m_entityId; }
    uint32_t GetGnbId() const { return m_gnbId; }

    void SetNode(Ptr<Node> node) { m_node = node; }
    Ptr<Node> GetNode() const { return m_node; }

    /// Each entity terminates E2 independently.
    Ptr<OranNtnE2Node> GetE2Node() const { return m_e2Node; }

    /// Advertise a supported RAN function. Idempotent: re-advertising
    /// the same RIC Function ID updates the description.
    void AdvertiseRanFunction(const RanFunctionAdvertised& rf);
    bool HasRanFunction(uint16_t ric_function_id) const;
    const std::vector<RanFunctionAdvertised>&
        GetRanFunctions() const { return m_ranFunctions; }

    /// Convenience: install role-specific defaults.
    ///   CU-CP : RC v1.03 (RIC Function ID 3)
    ///   CU-UP : KPM v3.00 for SDAP/PDCP volume metrics (147)
    ///   DU    : KPM v3.00 (RIC Function ID 147)
    ///   RU    : CCC v1.00 + NTN-Ephemeris (1000 + 1001)
    void AdvertiseRoleDefaults();

    /// Submit an indication to the local E2 termination (per role).
    /// Returns true if the entity is configured to handle the RAN
    /// function for this report.
    bool SubmitKpmIndication(const E2KpmReport& report);

    /// Counters — useful for tests.
    uint64_t IndicationsEmitted() const { return m_indicationsEmitted; }
    uint64_t ControlsAccepted() const { return m_controlsAccepted; }

    /// Hook fired whenever the entity ingests an RC action targeted at
    /// its role. Used by tests to confirm routing.
    using ControlObserver = std::function<void(const E2RcAction&)>;
    void SetControlObserver(ControlObserver cb) { m_controlCb = std::move(cb); }

    /// Force-receive a control action (the F1 / OFH layer pushes here
    /// after a delayed delivery).
    bool ReceiveControl(const E2RcAction& act);

  protected:
    void DoDispose() override;

  private:
    OranNodeType m_role{OranNodeType::O_DU};
    uint32_t m_entityId{0};
    uint32_t m_gnbId{0};
    Ptr<Node> m_node;
    Ptr<OranNtnE2Node> m_e2Node;
    std::vector<RanFunctionAdvertised> m_ranFunctions;
    uint64_t m_indicationsEmitted{0};
    uint64_t m_controlsAccepted{0};
    ControlObserver m_controlCb;
};

} // namespace ns3

#endif // ORAN_NTN_SPLIT_GNB_H
