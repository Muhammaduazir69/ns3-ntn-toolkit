/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SERVICE_MODEL_RC_H
#define ORAN_NTN_SERVICE_MODEL_RC_H

#include "oran-ntn-rc-style3.h"
#include "oran-ntn-service-model.h"

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief RC v1.03 Service Model plugin (Roadmap §3 T4, second concrete SM).
 *
 * Carries E2SM-RC v1.03 Connected-Mode Mobility (Style 3) ControlMessages
 * from 4.1.3. The v2.1 encoder uses a debug-friendly TLV; T2 will swap in
 * ASN.1-PER.
 *
 * RIC Function ID 3 matches the canonical RC SM ID used by FlexRIC / OSC.
 *
 * \note Currently exercised by unit tests only.
 */
class OranNtnServiceModelRc : public OranNtnServiceModel
{
  public:
    static constexpr uint16_t kRicFunctionId = 3;

    static TypeId GetTypeId();
    OranNtnServiceModelRc() = default;
    ~OranNtnServiceModelRc() override = default;

    uint16_t RicFunctionId() const override { return kRicFunctionId; }
    std::string Name() const override { return "RC"; }
    std::string Version() const override { return "v1.03"; }

    /// RC doesn't emit Indication-Message bodies in the v2.1 baseline (RC
    /// reports are Format-2/3 RAN-Control reports out of scope here).
    /// Returns an empty blob; callers should not call this for RC.
    std::vector<uint8_t> EncodeIndication(const void* body) const override;

    /// Encode a Style 3 ControlMessage into the wire-side
    /// `ric_control_message` octet string.
    std::vector<uint8_t>
        EncodeControl(const oranntn::rc_v103::style3::ControlMessage& msg) const;

    /// Decode a wire-side `ric_control_message` blob into a
    /// `ControlMessage`. Returns false on parse error.
    bool DecodeControl(const std::vector<uint8_t>& msg,
                       void* out) const override;
};

} // namespace ns3

#endif // ORAN_NTN_SERVICE_MODEL_RC_H
