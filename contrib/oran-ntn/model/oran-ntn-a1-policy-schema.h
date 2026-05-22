/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_A1_POLICY_SCHEMA_H
#define ORAN_NTN_A1_POLICY_SCHEMA_H

// A1 policy-type registry aligned with O-RAN-SC (Roadmap §4.1.11).
// Type IDs match the OSC policy-management catalog
//   https://docs.o-ran-sc.org/projects/o-ran-sc-it-dep/en/latest/installation-policies.html
// plus a toolkit-reserved range 20050+ for NTN-specific extensions.
//
// JSON Schema files live alongside this header at
//   contrib/oran-ntn/a1-policies/{type_id}-{slug}.json
// and are referenced by `A1PolicySchema::schema_file`. The runtime does
// not parse them (no JSON Schema validator dependency in the v2.1
// baseline); they are reference documents an xApp author can validate
// against externally before injecting policies.

#include "oran-ntn-types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace oranntn
{
namespace a1
{

/// Integer policy-type IDs aligned to O-RAN-SC. Values are the exact
/// numbers OSC uses on the A1 wire; reviewers can pin these in any
/// downstream xApp.
enum class PolicyTypeId : uint32_t
{
    OscTrafficSteering    = 20000,
    OscQualityOfExperience= 20001,
    OscHandoverControl    = 20008,
    OscMlModelSelection   = 20020,

    // ns3-ntn-toolkit reserved extensions (20050+).
    NtnBeamPriority       = 20050,
};

/// Returns the WG2 A1AP "policy type" semantic name (no integer ID).
const char* PolicyTypeIdName(PolicyTypeId id);

/// One entry in the policy-type registry.
struct A1PolicySchema
{
    PolicyTypeId type_id;
    /// Human-readable slug used in the JSON filename (e.g.
    /// "handover-control" -> a1-policies/20008-handover-control.json).
    std::string slug;
    /// Relative path under contrib/oran-ntn/a1-policies/ for the JSON
    /// Schema file. Reviewers can fetch it from the toolkit checkout.
    std::string schema_file;
    /// Whether this is an O-RAN-SC standard type or a toolkit extension.
    bool is_osc_standard;
};

/// Returns the full toolkit registry of A1 policy types. Stable order
/// matches the integer type ID ascending.
const std::vector<A1PolicySchema>& Registry();

/// Look up a registry entry by its integer ID. Returns nullptr if the
/// type is not registered.
const A1PolicySchema* Lookup(PolicyTypeId id);

/// Map the toolkit's local A1PolicyType enum to its OSC PolicyTypeId.
/// Returns 0 when no canonical mapping exists yet (will be filled in as
/// the toolkit's xApps gain OSC-aligned policy support).
uint32_t OscIdForLocalType(A1PolicyType local);

} // namespace a1
} // namespace oranntn
} // namespace ns3

#endif // ORAN_NTN_A1_POLICY_SCHEMA_H
