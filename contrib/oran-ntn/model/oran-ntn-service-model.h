/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SERVICE_MODEL_H
#define ORAN_NTN_SERVICE_MODEL_H

// Service-Model plugin ABI (Realism-Adoption-Roadmap-2026 §3 T4).
//
// Every Service Model the toolkit ships — KPM v3, RC v1.03 (with Style 3
// Connected-Mode Mobility from 4.1.3), E2SM-CCC, NTN-Ephemeris-SM, THz-RIS-SM
// — derives from OranNtnServiceModel. The abstract base exposes the wire-side
// contract: `RicFunctionId()` identifies the SM on E2AP, `EncodeIndication()`
// and `DecodeControl()` are the encode/decode pinches that T2's ASN.1-PER
// implementation will hook into.
//
// The registry (OranNtnServiceModelRegistry) is a small lookup that lets the
// Near-RT RIC resolve "RIC Function ID -> SM plugin" at subscription time.
//
// Reference: FlexRIC src/lib/sm/sm_proc_data.h + the per-SM plugin contracts
//   <https://gitlab.eurecom.fr/mosaic5g/flexric/-/tree/master/src/sm>

#include <ns3/object.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief Abstract Service-Model plugin (Roadmap §3 T4).
 *
 * Concrete plugins:
 *   - OranNtnServiceModelKpm  KPM v3.00, RIC Function ID 147
 *   - OranNtnServiceModelRc   RC v1.03 with Style 3 Connected-Mode Mobility,
 *                             RIC Function ID 3
 *   - OranNtnServiceModelCcc  E2SM-CCC (planned, 4.1.8)
 *   - OranNtnServiceModelNtnEphemeris (planned, 4.1.7)
 *   - OranNtnServiceModelThzRis (planned, 4.3.6)
 */
class OranNtnServiceModel : public Object
{
  public:
    static TypeId GetTypeId();
    ~OranNtnServiceModel() override = default;

    /// O-RAN RIC Function ID on the E2 interface. The integer values for
    /// standard SMs are fixed by O-RAN-SC / FlexRIC; toolkit extensions live
    /// in a reserved range (200+).
    virtual uint16_t RicFunctionId() const = 0;

    /// Short name for diagnostics + manifests (e.g. "KPM", "RC").
    virtual std::string Name() const = 0;

    /// SM version string in the WG3 convention (e.g. "v3.00", "v1.03").
    virtual std::string Version() const = 0;

    /// Encode an in-memory indication message body. The opaque return blob
    /// is what carries inside `ric_indication_t::ric_indication_message`.
    /// The v2.1 baseline uses a debug-friendly TLV form; T2's ASN.1-PER
    /// implementation will replace it without changing this signature.
    /// `body` is a void* pointing at the SM-specific concrete struct (e.g.
    /// flexric::kpm_v3::kpm_ind_msg_format_1_t* for the KPM SM).
    virtual std::vector<uint8_t>
        EncodeIndication(const void* body) const = 0;

    /// Decode a wire-side control-message blob into the SM-specific struct.
    /// `out` is a void* pointing at the destination struct. Returns true on
    /// success; false if the blob doesn't parse against this SM's grammar.
    virtual bool DecodeControl(const std::vector<uint8_t>& msg,
                                void* out) const = 0;
};

/**
 * \ingroup oran-ntn
 * \brief Service-Model registry — RIC Function ID -> plugin lookup.
 *
 * One instance per Near-RT RIC. xApps consult `Lookup(RicFunctionId)` at
 * subscription time to find the encoder/decoder for the SM they want to
 * speak.
 */
class OranNtnServiceModelRegistry : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnServiceModelRegistry();
    ~OranNtnServiceModelRegistry() override = default;

    /// Register a plugin. Returns false (and logs) if a plugin is already
    /// registered for the same RIC Function ID.
    bool Register(Ptr<OranNtnServiceModel> sm);

    /// Returns nullptr if no SM is registered for `id`.
    Ptr<OranNtnServiceModel> Lookup(uint16_t id) const;

    /// All registered RIC Function IDs in ascending order.
    std::vector<uint16_t> GetFunctionIds() const;

    size_t Size() const { return m_plugins.size(); }

  private:
    std::map<uint16_t, Ptr<OranNtnServiceModel>> m_plugins;
};

} // namespace ns3

#endif // ORAN_NTN_SERVICE_MODEL_H
