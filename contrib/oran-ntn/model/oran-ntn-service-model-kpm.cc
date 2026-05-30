/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-kpm.h"

#include "../asn1/asn1-per-codec.h"

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

// Aligned-PER encoding for E2SM-KPM Indication Format 1 (Roadmap §3 T2).
// Layout in PER terms:
//   kpm_ind_msg_format_1_t ::= SEQUENCE {
//     gran_period_ms       INTEGER,
//     meas_info_lst        SEQUENCE OF MeasInfoFormat1
//   }
//   MeasInfoFormat1 ::= SEQUENCE {
//     meas_type            CHOICE { name [0] UTF8String, id [1] INTEGER },
//     meas_record_lst      SEQUENCE OF MeasRecord,
//     label_info_lst       SEQUENCE OF LabelInfo
//   }
//   MeasRecord ::= SEQUENCE {
//     form    INTEGER,     -- meas_value_form_t (0=int, 1=real, 2=no_value)
//     int_val INTEGER,
//     real_val INTEGER     -- real serialised as IEEE-754 raw bits
//   }
//   LabelInfo ::= SEQUENCE {
//     five_qi    [0] INTEGER OPTIONAL,
//     s_nssai    [1] UTF8String OPTIONAL,
//     plmn_id    [2] UTF8String OPTIONAL
//   }

std::vector<uint8_t>
OranNtnServiceModelKpm::EncodeIndication(const void* body) const
{
    const auto* in = static_cast<
        const oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t*>(body);
    oranntn::asn1::PerWriter w;
    w.WriteInteger(static_cast<int64_t>(in->gran_period_ms));
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(in->meas_info_lst.size()));
    for (const auto& mi : in->meas_info_lst)
    {
        // meas_type CHOICE: name (0) or id (1).
        if (mi.meas_type.form ==
            oranntn::flexric::kpm_v3::meas_type_form_t::name)
        {
            w.WriteChoiceIndex(0);
            w.WriteUtf8String(mi.meas_type.meas_name);
        }
        else
        {
            w.WriteChoiceIndex(1);
            w.WriteInteger(static_cast<int64_t>(mi.meas_type.meas_id));
        }
        w.WriteLengthDeterminant(
            static_cast<uint32_t>(mi.meas_record_lst.size()));
        for (const auto& rec : mi.meas_record_lst)
        {
            w.WriteInteger(static_cast<int64_t>(rec.form));
            w.WriteInteger(rec.int_val);
            int64_t bits;
            std::memcpy(&bits, &rec.real_val, sizeof(bits));
            w.WriteInteger(bits);
        }
        w.WriteLengthDeterminant(
            static_cast<uint32_t>(mi.label_info_lst.size()));
        for (const auto& lbl : mi.label_info_lst)
        {
            w.BeginSequencePreamble(3);
            w.SetPreambleBit(0, lbl.five_qi.has_value());
            w.SetPreambleBit(1, lbl.s_nssai.has_value());
            w.SetPreambleBit(2, lbl.plmn_id.has_value());
            w.EndSequencePreamble();
            if (lbl.five_qi.has_value())
            {
                w.WriteInteger(static_cast<int64_t>(*lbl.five_qi));
            }
            if (lbl.s_nssai.has_value())
            {
                w.WriteUtf8String(*lbl.s_nssai);
            }
            if (lbl.plmn_id.has_value())
            {
                w.WriteUtf8String(*lbl.plmn_id);
            }
        }
    }
    return w.Take();
}

bool
OranNtnServiceModelKpm::DecodeIndication(
    const std::vector<uint8_t>& msg,
    oranntn::flexric::kpm_v3::kpm_ind_msg_format_1_t& out) const
{
    try
    {
        oranntn::asn1::PerReader r(msg);
        out.gran_period_ms = static_cast<uint32_t>(r.ReadInteger());
        const uint32_t numInfo = r.ReadLengthDeterminant();
        out.meas_info_lst.clear();
        out.meas_info_lst.reserve(numInfo);
        for (uint32_t k = 0; k < numInfo; ++k)
        {
            oranntn::flexric::kpm_v3::meas_info_format_1_lst_t mi{};
            const uint8_t ch = r.ReadChoiceIndex();
            if (ch == 0)
            {
                mi.meas_type.form =
                    oranntn::flexric::kpm_v3::meas_type_form_t::name;
                mi.meas_type.meas_name = r.ReadUtf8String();
            }
            else
            {
                mi.meas_type.form =
                    oranntn::flexric::kpm_v3::meas_type_form_t::id;
                mi.meas_type.meas_id =
                    static_cast<uint32_t>(r.ReadInteger());
            }
            const uint32_t numRec = r.ReadLengthDeterminant();
            mi.meas_record_lst.reserve(numRec);
            for (uint32_t rr = 0; rr < numRec; ++rr)
            {
                oranntn::flexric::kpm_v3::meas_record_item_t rec{};
                rec.form = static_cast<
                    oranntn::flexric::kpm_v3::meas_value_form_t>(r.ReadInteger());
                rec.int_val = r.ReadInteger();
                int64_t bits = r.ReadInteger();
                std::memcpy(&rec.real_val, &bits, sizeof(rec.real_val));
                mi.meas_record_lst.push_back(rec);
            }
            const uint32_t numLbl = r.ReadLengthDeterminant();
            mi.label_info_lst.reserve(numLbl);
            for (uint32_t l = 0; l < numLbl; ++l)
            {
                oranntn::flexric::kpm_v3::label_info_t lbl{};
                const uint16_t pre = r.ReadSequencePreamble(3);
                const bool has5q = (pre >> 2) & 1;
                const bool hasS = (pre >> 1) & 1;
                const bool hasP = pre & 1;
                if (has5q)
                {
                    lbl.five_qi =
                        static_cast<uint8_t>(r.ReadInteger());
                }
                if (hasS)
                {
                    lbl.s_nssai = r.ReadUtf8String();
                }
                if (hasP)
                {
                    lbl.plmn_id = r.ReadUtf8String();
                }
                mi.label_info_lst.push_back(lbl);
            }
            out.meas_info_lst.push_back(mi);
        }
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("KPM SM: PER decode error: " << exc.what());
        return false;
    }
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
