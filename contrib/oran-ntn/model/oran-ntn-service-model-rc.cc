/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-rc.h"

#include "ns3/log.h"

#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnServiceModelRc");

TypeId
OranNtnServiceModelRc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModelRc")
                            .SetParent<OranNtnServiceModel>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnServiceModelRc>();
    return tid;
}

namespace
{

// Debug-friendly TLV ControlMessage encoding for v2.1. ASN.1-PER under T2.
//
// All numerics little-endian.
//   uint8  style_id           (always 3 for Style 3)
//   uint8  action_id          (1=HO control, 2=CHO, 3=DAPS-HO)
//
//   Action 1:
//     uint16 plmn_len, plmn
//     uint64 nr_cell_identity
//     uint8  handover_type (0..3)
//     uint8  has_secondary_cell, [if 1: uint16 plmn_len, plmn, uint64 nci]
//
//   Action 2:
//     uint32 conditional_reconfiguration_id
//     uint32 num_candidates
//     for each candidate:
//       uint16 plmn_len, plmn
//       uint64 nr_cell_identity
//       uint32 trigger_len, trigger bytes
//
//   Action 3:
//     uint16 plmn_len, plmn
//     uint64 nr_cell_identity
//     uint8  has_termination, [if 1: uint8 cause]

void
WriteU8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }

void
WriteU16(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back(v & 0xFF);
    b.push_back((v >> 8) & 0xFF);
}

void
WriteU32(std::vector<uint8_t>& b, uint32_t v)
{
    b.push_back(v & 0xFF);
    b.push_back((v >> 8) & 0xFF);
    b.push_back((v >> 16) & 0xFF);
    b.push_back((v >> 24) & 0xFF);
}

void
WriteU64(std::vector<uint8_t>& b, uint64_t v)
{
    for (int i = 0; i < 8; ++i) { b.push_back((v >> (8 * i)) & 0xFF); }
}

void
WriteString(std::vector<uint8_t>& b, const std::string& s)
{
    WriteU16(b, static_cast<uint16_t>(s.size()));
    b.insert(b.end(), s.begin(), s.end());
}

void
WriteBlob(std::vector<uint8_t>& b, const std::vector<uint8_t>& v)
{
    WriteU32(b, static_cast<uint32_t>(v.size()));
    b.insert(b.end(), v.begin(), v.end());
}

bool
ReadU8(const std::vector<uint8_t>& b, size_t& i, uint8_t& v)
{
    if (i + 1 > b.size()) { return false; }
    v = b[i++];
    return true;
}

bool
ReadU16(const std::vector<uint8_t>& b, size_t& i, uint16_t& v)
{
    if (i + 2 > b.size()) { return false; }
    v = static_cast<uint16_t>(b[i] | (b[i + 1] << 8));
    i += 2;
    return true;
}

bool
ReadU32(const std::vector<uint8_t>& b, size_t& i, uint32_t& v)
{
    if (i + 4 > b.size()) { return false; }
    v = static_cast<uint32_t>(b[i]) |
        (static_cast<uint32_t>(b[i + 1]) << 8) |
        (static_cast<uint32_t>(b[i + 2]) << 16) |
        (static_cast<uint32_t>(b[i + 3]) << 24);
    i += 4;
    return true;
}

bool
ReadU64(const std::vector<uint8_t>& b, size_t& i, uint64_t& v)
{
    if (i + 8 > b.size()) { return false; }
    v = 0;
    for (int k = 0; k < 8; ++k)
    {
        v |= static_cast<uint64_t>(b[i + k]) << (8 * k);
    }
    i += 8;
    return true;
}

bool
ReadString(const std::vector<uint8_t>& b, size_t& i, std::string& s)
{
    uint16_t len;
    if (!ReadU16(b, i, len)) { return false; }
    if (i + len > b.size()) { return false; }
    s.assign(reinterpret_cast<const char*>(&b[i]), len);
    i += len;
    return true;
}

bool
ReadBlob(const std::vector<uint8_t>& b, size_t& i,
         std::vector<uint8_t>& out)
{
    uint32_t len;
    if (!ReadU32(b, i, len)) { return false; }
    if (i + len > b.size()) { return false; }
    out.assign(b.begin() + i, b.begin() + i + len);
    i += len;
    return true;
}

} // namespace

std::vector<uint8_t>
OranNtnServiceModelRc::EncodeIndication(const void* /*body*/) const
{
    NS_LOG_WARN("oran-ntn RC SM: EncodeIndication is a no-op; the v2.1 "
                "baseline does not implement Format-2/3 RC reports");
    return {};
}

std::vector<uint8_t>
OranNtnServiceModelRc::EncodeControl(
    const oranntn::rc_v103::style3::ControlMessage& msg) const
{
    using namespace oranntn::rc_v103::style3;
    std::vector<uint8_t> out;
    WriteU8(out, msg.style_id);
    const uint8_t actionId = ActionId(msg.action);
    WriteU8(out, actionId);

    if (std::holds_alternative<HandoverControl>(msg.action))
    {
        const auto& h = std::get<HandoverControl>(msg.action);
        WriteString(out, h.target_primary_cell_id.plmn_id);
        WriteU64(out, h.target_primary_cell_id.nr_cell_identity);
        WriteU8(out, static_cast<uint8_t>(h.handover_type));
        if (h.new_secondary_cell_id.has_value())
        {
            WriteU8(out, 1);
            WriteString(out, h.new_secondary_cell_id->plmn_id);
            WriteU64(out, h.new_secondary_cell_id->nr_cell_identity);
        }
        else
        {
            WriteU8(out, 0);
        }
    }
    else if (std::holds_alternative<ConditionalHandoverControl>(msg.action))
    {
        const auto& c = std::get<ConditionalHandoverControl>(msg.action);
        WriteU32(out, c.conditional_reconfiguration_id);
        WriteU32(out, static_cast<uint32_t>(c.candidate_cell_list.size()));
        for (const auto& cc : c.candidate_cell_list)
        {
            WriteString(out, cc.target_primary_cell_id.plmn_id);
            WriteU64(out, cc.target_primary_cell_id.nr_cell_identity);
            WriteBlob(out, cc.trigger_condition);
        }
    }
    else
    {
        const auto& d = std::get<DapsHandoverControl>(msg.action);
        WriteString(out, d.target_primary_cell_id.plmn_id);
        WriteU64(out, d.target_primary_cell_id.nr_cell_identity);
        if (d.daps_termination_policy.has_value())
        {
            WriteU8(out, 1);
            WriteU8(out, static_cast<uint8_t>(*d.daps_termination_policy));
        }
        else
        {
            WriteU8(out, 0);
        }
    }
    return out;
}

bool
OranNtnServiceModelRc::DecodeControl(const std::vector<uint8_t>& msg,
                                       void* outPtr) const
{
    using namespace oranntn::rc_v103::style3;
    auto* out = static_cast<ControlMessage*>(outPtr);
    size_t i = 0;
    if (!ReadU8(msg, i, out->style_id)) { return false; }
    if (out->style_id != 3) { return false; }
    uint8_t actionId;
    if (!ReadU8(msg, i, actionId)) { return false; }

    if (actionId == 1)
    {
        HandoverControl h{};
        if (!ReadString(msg, i, h.target_primary_cell_id.plmn_id))
            return false;
        if (!ReadU64(msg, i, h.target_primary_cell_id.nr_cell_identity))
            return false;
        uint8_t ht;
        if (!ReadU8(msg, i, ht)) { return false; }
        h.handover_type = static_cast<HandoverType>(ht);
        uint8_t hasSecond;
        if (!ReadU8(msg, i, hasSecond)) { return false; }
        if (hasSecond)
        {
            NrCellGlobalId sec{};
            if (!ReadString(msg, i, sec.plmn_id)) { return false; }
            if (!ReadU64(msg, i, sec.nr_cell_identity)) { return false; }
            h.new_secondary_cell_id = sec;
        }
        out->action = h;
    }
    else if (actionId == 2)
    {
        ConditionalHandoverControl c{};
        if (!ReadU32(msg, i, c.conditional_reconfiguration_id))
            return false;
        uint32_t numC;
        if (!ReadU32(msg, i, numC)) { return false; }
        c.candidate_cell_list.reserve(numC);
        for (uint32_t k = 0; k < numC; ++k)
        {
            ConditionalHandoverControl::CandidateCell cc{};
            if (!ReadString(msg, i, cc.target_primary_cell_id.plmn_id))
                return false;
            if (!ReadU64(msg, i, cc.target_primary_cell_id.nr_cell_identity))
                return false;
            if (!ReadBlob(msg, i, cc.trigger_condition)) { return false; }
            c.candidate_cell_list.push_back(cc);
        }
        out->action = c;
    }
    else if (actionId == 3)
    {
        DapsHandoverControl d{};
        if (!ReadString(msg, i, d.target_primary_cell_id.plmn_id))
            return false;
        if (!ReadU64(msg, i, d.target_primary_cell_id.nr_cell_identity))
            return false;
        uint8_t hasCause;
        if (!ReadU8(msg, i, hasCause)) { return false; }
        if (hasCause)
        {
            uint8_t c;
            if (!ReadU8(msg, i, c)) { return false; }
            d.daps_termination_policy =
                static_cast<DapsTerminationCause>(c);
        }
        out->action = d;
    }
    else
    {
        NS_LOG_WARN("oran-ntn RC SM: unknown action id " << static_cast<int>(actionId));
        return false;
    }
    return true;
}

} // namespace ns3
