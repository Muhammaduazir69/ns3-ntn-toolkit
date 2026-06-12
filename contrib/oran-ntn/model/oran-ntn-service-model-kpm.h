/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SERVICE_MODEL_KPM_H
#define ORAN_NTN_SERVICE_MODEL_KPM_H

#include "oran-ntn-flexric-types.h"
#include "oran-ntn-service-model.h"

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief KPM v3.00 Service Model plugin (Roadmap §3 T4, first concrete SM).
 *
 * Carries E2SM-KPM Indication Format 1 bodies using the FlexRIC-verbatim
 * `kpm_ind_msg_format_1_t` shape from oran-ntn-flexric-types.h. The encoder
 * emits a debug-friendly TLV form in v2.1; T2 will swap in ASN.1-PER
 * without touching the call-sites.
 *
 * RIC Function ID 147 matches the canonical KPM SM ID used by FlexRIC,
 * srsRAN, and OAI's E2 agent.
 *
 * \note Currently exercised by unit tests only.
 */
class OranNtnServiceModelKpm : public OranNtnServiceModel
{
  public:
    static constexpr uint16_t kRicFunctionId = 147;

    static TypeId GetTypeId();
    OranNtnServiceModelKpm() = default;
    ~OranNtnServiceModelKpm() override = default;

    uint16_t RicFunctionId() const override { return kRicFunctionId; }
    std::string Name() const override { return "KPM"; }
    std::string Version() const override { return "v3.00"; }

    std::vector<uint8_t> EncodeIndication(const void* body) const override;
    bool DecodeControl(const std::vector<uint8_t>& msg,
                       void* out) const override;
    /// Round-trip the body for tests: decode an indication blob produced
    /// by EncodeIndication back into a kpm_ind_msg_format_1_t.
    bool DecodeIndication(const std::vector<uint8_t>& msg,
                          oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t&
                              out) const;
};

} // namespace ns3

#endif // ORAN_NTN_SERVICE_MODEL_KPM_H
