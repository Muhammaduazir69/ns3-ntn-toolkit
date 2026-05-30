/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-rc.h"

#include "../asn1/asn1-per-codec.h"

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

// Aligned-PER encoding for E2SM-RC v1.03 Style 3 ControlMessage (Roadmap §3 T2).
// Layout in PER terms:
//   ControlMessage ::= SEQUENCE {
//     style_id    INTEGER (3),
//     action      CHOICE { ho [0] HandoverControl,
//                          cho [1] ConditionalHandoverControl,
//                          daps [2] DapsHandoverControl }
//   }
//   NrCellGlobalId ::= SEQUENCE { plmn UTF8String, nci INTEGER }
//   HandoverControl ::= SEQUENCE {
//     target NrCellGlobalId, ht INTEGER,
//     secondary [0] NrCellGlobalId OPTIONAL
//   }
//   ConditionalHandoverControl ::= SEQUENCE {
//     reconfig_id INTEGER,
//     candidates SEQUENCE OF SEQUENCE { target NrCellGlobalId,
//                                       trigger OCTET STRING }
//   }
//   DapsHandoverControl ::= SEQUENCE {
//     target NrCellGlobalId,
//     termination [0] INTEGER OPTIONAL
//   }

namespace
{

void
WriteNrCgi(oranntn::asn1::PerWriter& w,
            const oranntn::rc_v103::style3::NrCellGlobalId& cgi)
{
    w.WriteUtf8String(cgi.plmn_id);
    w.WriteInteger(static_cast<int64_t>(cgi.nr_cell_identity));
}

void
ReadNrCgi(oranntn::asn1::PerReader& r,
           oranntn::rc_v103::style3::NrCellGlobalId& cgi)
{
    cgi.plmn_id = r.ReadUtf8String();
    cgi.nr_cell_identity = static_cast<uint64_t>(r.ReadInteger());
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
    oranntn::asn1::PerWriter w;
    w.WriteInteger(static_cast<int64_t>(msg.style_id));
    const uint8_t actionId = ActionId(msg.action);
    w.WriteChoiceIndex(static_cast<uint8_t>(actionId - 1));

    if (std::holds_alternative<HandoverControl>(msg.action))
    {
        const auto& h = std::get<HandoverControl>(msg.action);
        WriteNrCgi(w, h.target_primary_cell_id);
        w.WriteInteger(static_cast<int64_t>(h.handover_type));
        w.BeginSequencePreamble(1);
        w.SetPreambleBit(0, h.new_secondary_cell_id.has_value());
        w.EndSequencePreamble();
        if (h.new_secondary_cell_id.has_value())
        {
            WriteNrCgi(w, *h.new_secondary_cell_id);
        }
    }
    else if (std::holds_alternative<ConditionalHandoverControl>(msg.action))
    {
        const auto& c = std::get<ConditionalHandoverControl>(msg.action);
        w.WriteInteger(static_cast<int64_t>(c.conditional_reconfiguration_id));
        w.WriteLengthDeterminant(
            static_cast<uint32_t>(c.candidate_cell_list.size()));
        for (const auto& cc : c.candidate_cell_list)
        {
            WriteNrCgi(w, cc.target_primary_cell_id);
            w.WriteOctetString(cc.trigger_condition);
        }
    }
    else
    {
        const auto& d = std::get<DapsHandoverControl>(msg.action);
        WriteNrCgi(w, d.target_primary_cell_id);
        w.BeginSequencePreamble(1);
        w.SetPreambleBit(0, d.daps_termination_policy.has_value());
        w.EndSequencePreamble();
        if (d.daps_termination_policy.has_value())
        {
            w.WriteInteger(
                static_cast<int64_t>(*d.daps_termination_policy));
        }
    }
    return w.Take();
}

bool
OranNtnServiceModelRc::DecodeControl(const std::vector<uint8_t>& msg,
                                       void* outPtr) const
{
    using namespace oranntn::rc_v103::style3;
    auto* out = static_cast<ControlMessage*>(outPtr);
    try
    {
        oranntn::asn1::PerReader r(msg);
        out->style_id = static_cast<uint8_t>(r.ReadInteger());
        if (out->style_id != 3)
        {
            return false;
        }
        const uint8_t ch = r.ReadChoiceIndex();
        if (ch == 0)
        {
            HandoverControl h{};
            ReadNrCgi(r, h.target_primary_cell_id);
            h.handover_type = static_cast<HandoverType>(r.ReadInteger());
            const uint16_t pre = r.ReadSequencePreamble(1);
            if (pre & 1)
            {
                NrCellGlobalId sec{};
                ReadNrCgi(r, sec);
                h.new_secondary_cell_id = sec;
            }
            out->action = h;
        }
        else if (ch == 1)
        {
            ConditionalHandoverControl c{};
            c.conditional_reconfiguration_id =
                static_cast<uint32_t>(r.ReadInteger());
            const uint32_t numC = r.ReadLengthDeterminant();
            c.candidate_cell_list.reserve(numC);
            for (uint32_t k = 0; k < numC; ++k)
            {
                ConditionalHandoverControl::CandidateCell cc{};
                ReadNrCgi(r, cc.target_primary_cell_id);
                cc.trigger_condition = r.ReadOctetString();
                c.candidate_cell_list.push_back(cc);
            }
            out->action = c;
        }
        else if (ch == 2)
        {
            DapsHandoverControl d{};
            ReadNrCgi(r, d.target_primary_cell_id);
            const uint16_t pre = r.ReadSequencePreamble(1);
            if (pre & 1)
            {
                d.daps_termination_policy =
                    static_cast<DapsTerminationCause>(r.ReadInteger());
            }
            out->action = d;
        }
        else
        {
            NS_LOG_WARN("RC SM: unknown action CHOICE index "
                        << static_cast<int>(ch));
            return false;
        }
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("RC SM: PER decode error: " << exc.what());
        return false;
    }
}

} // namespace ns3
