/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-ccc.h"

#include "../asn1/asn1-per-codec.h"

#include "ns3/log.h"

#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnServiceModelCcc");

TypeId
OranNtnServiceModelCcc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModelCcc")
                            .SetParent<OranNtnServiceModel>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnServiceModelCcc>();
    return tid;
}

namespace
{

void
WriteCellConfig(oranntn::asn1::PerWriter& w,
                 const oranntn::ccc::CellConfigRecord& c)
{
    w.BeginSequencePreamble(2);
    w.SetPreambleBit(0, c.arfcn_dl.has_value());
    w.SetPreambleBit(1, c.arfcn_ul.has_value());
    w.EndSequencePreamble();
    w.WriteInteger(static_cast<int64_t>(c.nr_cell_global_id));
    w.WriteInteger(static_cast<int64_t>(c.dtx_us_log2));
    w.WriteInteger(static_cast<int64_t>(c.drx_us_log2));
    w.WriteInteger(static_cast<int64_t>(c.output_power_dbm));
    w.WriteInteger(static_cast<int64_t>(c.prb_pool_total));
    w.WriteInteger(static_cast<int64_t>(c.prb_pool_reserved));
    w.WriteInteger(static_cast<int64_t>(c.antenna_mask));
    if (c.arfcn_dl.has_value())
    {
        w.WriteInteger(static_cast<int64_t>(*c.arfcn_dl));
    }
    if (c.arfcn_ul.has_value())
    {
        w.WriteInteger(static_cast<int64_t>(*c.arfcn_ul));
    }
}

void
ReadCellConfig(oranntn::asn1::PerReader& r,
                oranntn::ccc::CellConfigRecord& c)
{
    const uint16_t pre = r.ReadSequencePreamble(2);
    const bool hasDl = (pre >> 1) & 1;
    const bool hasUl = pre & 1;
    c.nr_cell_global_id = static_cast<uint64_t>(r.ReadInteger());
    c.dtx_us_log2 = static_cast<uint8_t>(r.ReadInteger());
    c.drx_us_log2 = static_cast<uint8_t>(r.ReadInteger());
    c.output_power_dbm = static_cast<int16_t>(r.ReadInteger());
    c.prb_pool_total = static_cast<uint16_t>(r.ReadInteger());
    c.prb_pool_reserved = static_cast<uint16_t>(r.ReadInteger());
    c.antenna_mask = static_cast<uint64_t>(r.ReadInteger());
    if (hasDl)
    {
        c.arfcn_dl = static_cast<uint32_t>(r.ReadInteger());
    }
    if (hasUl)
    {
        c.arfcn_ul = static_cast<uint32_t>(r.ReadInteger());
    }
}

void
WritePerfObjective(oranntn::asn1::PerWriter& w,
                    const oranntn::ccc::PerformanceObjective& o)
{
    w.WriteInteger(static_cast<int64_t>(o.metric));
    int64_t bits;
    double tv = o.target_value;
    std::memcpy(&bits, &tv, sizeof(bits));
    w.WriteInteger(bits);
    double tol = o.tolerance;
    std::memcpy(&bits, &tol, sizeof(bits));
    w.WriteInteger(bits);
    w.WriteInteger(static_cast<int64_t>(o.scope_nr_cgi));
    w.WriteInteger(static_cast<int64_t>(o.scope_slice_id));
}

void
ReadPerfObjective(oranntn::asn1::PerReader& r,
                   oranntn::ccc::PerformanceObjective& o)
{
    o.metric = static_cast<oranntn::ccc::PerformanceObjective::Metric>(
        r.ReadInteger());
    int64_t bits = r.ReadInteger();
    double v;
    std::memcpy(&v, &bits, sizeof(v));
    o.target_value = v;
    bits = r.ReadInteger();
    std::memcpy(&v, &bits, sizeof(v));
    o.tolerance = v;
    o.scope_nr_cgi = static_cast<uint64_t>(r.ReadInteger());
    o.scope_slice_id = static_cast<uint8_t>(r.ReadInteger());
}

} // namespace

std::vector<uint8_t>
OranNtnServiceModelCcc::EncodeIndication(const void* body) const
{
    const auto* in =
        static_cast<const oranntn::ccc::CccIndMsgFormat1*>(body);
    oranntn::asn1::PerWriter w;
    w.WriteInteger(static_cast<int64_t>(in->snapshot_seq));
    w.WriteLengthDeterminant(static_cast<uint32_t>(in->cells.size()));
    for (const auto& c : in->cells)
    {
        WriteCellConfig(w, c);
    }
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(in->perf_objectives.size()));
    for (const auto& o : in->perf_objectives)
    {
        WritePerfObjective(w, o);
    }
    return w.Take();
}

bool
OranNtnServiceModelCcc::DecodeIndication(
    const std::vector<uint8_t>& msg,
    oranntn::ccc::CccIndMsgFormat1& out) const
{
    try
    {
        oranntn::asn1::PerReader r(msg);
        out.snapshot_seq = static_cast<uint32_t>(r.ReadInteger());
        const uint32_t numCells = r.ReadLengthDeterminant();
        out.cells.clear();
        out.cells.reserve(numCells);
        for (uint32_t i = 0; i < numCells; ++i)
        {
            oranntn::ccc::CellConfigRecord c{};
            ReadCellConfig(r, c);
            out.cells.push_back(c);
        }
        const uint32_t numObj = r.ReadLengthDeterminant();
        out.perf_objectives.clear();
        out.perf_objectives.reserve(numObj);
        for (uint32_t i = 0; i < numObj; ++i)
        {
            oranntn::ccc::PerformanceObjective o{};
            ReadPerfObjective(r, o);
            out.perf_objectives.push_back(o);
        }
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("CCC SM: PER decode error: " << exc.what());
        return false;
    }
}

std::vector<uint8_t>
OranNtnServiceModelCcc::EncodeControl(
    const oranntn::ccc::CccControlAction& a) const
{
    oranntn::asn1::PerWriter w;
    w.WriteInteger(static_cast<int64_t>(a.op));
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(a.cell_updates.size()));
    for (const auto& c : a.cell_updates)
    {
        WriteCellConfig(w, c);
    }
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(a.objective_updates.size()));
    for (const auto& o : a.objective_updates)
    {
        WritePerfObjective(w, o);
    }
    return w.Take();
}

bool
OranNtnServiceModelCcc::DecodeControl(const std::vector<uint8_t>& msg,
                                       void* outPtr) const
{
    auto* out = static_cast<oranntn::ccc::CccControlAction*>(outPtr);
    try
    {
        oranntn::asn1::PerReader r(msg);
        out->op = static_cast<oranntn::ccc::CccControlAction::Op>(
            r.ReadInteger());
        const uint32_t numCells = r.ReadLengthDeterminant();
        out->cell_updates.clear();
        out->cell_updates.reserve(numCells);
        for (uint32_t i = 0; i < numCells; ++i)
        {
            oranntn::ccc::CellConfigRecord c{};
            ReadCellConfig(r, c);
            out->cell_updates.push_back(c);
        }
        const uint32_t numObj = r.ReadLengthDeterminant();
        out->objective_updates.clear();
        out->objective_updates.reserve(numObj);
        for (uint32_t i = 0; i < numObj; ++i)
        {
            oranntn::ccc::PerformanceObjective o{};
            ReadPerfObjective(r, o);
            out->objective_updates.push_back(o);
        }
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("CCC SM: PER decode error: " << exc.what());
        return false;
    }
}

} // namespace ns3
