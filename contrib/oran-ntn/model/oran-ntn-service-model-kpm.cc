/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-kpm.h"

#include "ns3/log.h"

#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnServiceModelKpm");

TypeId
OranNtnServiceModelKpm::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModelKpm")
                            .SetParent<OranNtnServiceModel>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnServiceModelKpm>();
    return tid;
}

namespace
{

// Debug-friendly TLV encoding for v2.1. Replaced by ASN.1-PER under T2.
// Layout (little-endian):
//   uint32 gran_period_ms
//   uint32 num_meas_info
//   for each meas_info:
//     uint16 name_len, name bytes (UTF-8)
//     uint32 num_records
//     for each record:
//       uint8  value_form     (0=integer, 1=real, 2=no_value)
//       int64  int_val        (host byte order)
//       double real_val
//     uint32 num_labels
//     for each label:
//       uint8  has_five_qi, [uint8 five_qi]
//       uint16 s_nssai_len, [bytes]
//       uint16 plmn_id_len,  [bytes]

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
WriteBytes(std::vector<uint8_t>& b, const uint8_t* p, size_t n)
{
    b.insert(b.end(), p, p + n);
}

void
WriteString(std::vector<uint8_t>& b, const std::string& s)
{
    WriteU16(b, static_cast<uint16_t>(s.size()));
    WriteBytes(b, reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

void
WriteI64(std::vector<uint8_t>& b, int64_t v)
{
    uint8_t buf[8];
    std::memcpy(buf, &v, 8);
    WriteBytes(b, buf, 8);
}

void
WriteDouble(std::vector<uint8_t>& b, double v)
{
    uint8_t buf[8];
    std::memcpy(buf, &v, 8);
    WriteBytes(b, buf, 8);
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
ReadI64(const std::vector<uint8_t>& b, size_t& i, int64_t& v)
{
    if (i + 8 > b.size()) { return false; }
    std::memcpy(&v, &b[i], 8);
    i += 8;
    return true;
}

bool
ReadDouble(const std::vector<uint8_t>& b, size_t& i, double& v)
{
    if (i + 8 > b.size()) { return false; }
    std::memcpy(&v, &b[i], 8);
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

} // namespace

std::vector<uint8_t>
OranNtnServiceModelKpm::EncodeIndication(const void* body) const
{
    const auto* in = static_cast<
        const oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t*>(body);
    std::vector<uint8_t> out;
    out.reserve(64);
    WriteU32(out, in->gran_period_ms);
    WriteU32(out, static_cast<uint32_t>(in->meas_info_lst.size()));
    for (const auto& mi : in->meas_info_lst)
    {
        WriteString(out, mi.meas_type.meas_name);
        WriteU32(out, static_cast<uint32_t>(mi.meas_record_lst.size()));
        for (const auto& rec : mi.meas_record_lst)
        {
            out.push_back(static_cast<uint8_t>(rec.form));
            WriteI64(out, rec.int_val);
            WriteDouble(out, rec.real_val);
        }
        WriteU32(out, static_cast<uint32_t>(mi.label_info_lst.size()));
        for (const auto& lbl : mi.label_info_lst)
        {
            out.push_back(lbl.five_qi.has_value() ? 1 : 0);
            out.push_back(lbl.five_qi.has_value() ? *lbl.five_qi : 0);
            WriteString(out, lbl.s_nssai.value_or(""));
            WriteString(out, lbl.plmn_id.value_or(""));
        }
    }
    return out;
}

bool
OranNtnServiceModelKpm::DecodeIndication(
    const std::vector<uint8_t>& msg,
    oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t& out) const
{
    size_t i = 0;
    uint32_t gran = 0;
    uint32_t numInfo = 0;
    if (!ReadU32(msg, i, gran)) { return false; }
    if (!ReadU32(msg, i, numInfo)) { return false; }
    out.gran_period_ms = gran;
    out.meas_info_lst.clear();
    out.meas_info_lst.reserve(numInfo);
    for (uint32_t k = 0; k < numInfo; ++k)
    {
        oranntn::flexric::kpm_v3::meas_info_format_1_lst_t mi{};
        mi.meas_type.form =
            oranntn::flexric::kpm_v3::meas_type_form_t::name;
        if (!ReadString(msg, i, mi.meas_type.meas_name)) { return false; }
        uint32_t numRec = 0;
        if (!ReadU32(msg, i, numRec)) { return false; }
        mi.meas_record_lst.reserve(numRec);
        for (uint32_t r = 0; r < numRec; ++r)
        {
            oranntn::flexric::kpm_v3::meas_record_item_t rec{};
            if (i >= msg.size()) { return false; }
            rec.form =
                static_cast<oranntn::flexric::kpm_v3::meas_value_form_t>(
                    msg[i]);
            ++i;
            if (!ReadI64(msg, i, rec.int_val)) { return false; }
            if (!ReadDouble(msg, i, rec.real_val)) { return false; }
            mi.meas_record_lst.push_back(rec);
        }
        uint32_t numLbl = 0;
        if (!ReadU32(msg, i, numLbl)) { return false; }
        mi.label_info_lst.reserve(numLbl);
        for (uint32_t l = 0; l < numLbl; ++l)
        {
            oranntn::flexric::kpm_v3::label_info_t lbl{};
            if (i + 2 > msg.size()) { return false; }
            const bool hasFiveQi = msg[i] != 0;
            ++i;
            const uint8_t fq = msg[i];
            ++i;
            if (hasFiveQi) { lbl.five_qi = fq; }
            std::string s;
            if (!ReadString(msg, i, s)) { return false; }
            if (!s.empty()) { lbl.s_nssai = s; }
            if (!ReadString(msg, i, s)) { return false; }
            if (!s.empty()) { lbl.plmn_id = s; }
            mi.label_info_lst.push_back(lbl);
        }
        out.meas_info_lst.push_back(mi);
    }
    return true;
}

bool
OranNtnServiceModelKpm::DecodeControl(const std::vector<uint8_t>& /*msg*/,
                                       void* /*out*/) const
{
    // KPM is a reporting SM; ControlRequests over KPM are not part of WG3
    // v3.00. Return false unconditionally so consumers route control
    // through the RC SM instead.
    return false;
}

} // namespace ns3
