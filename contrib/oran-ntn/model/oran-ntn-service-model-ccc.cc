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

// ---- 4.1.8 NTN-extension helpers --------------------------------------------

void
WriteDouble(oranntn::asn1::PerWriter& w, double v)
{
    int64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    w.WriteInteger(bits);
}

double
ReadDouble(oranntn::asn1::PerReader& r)
{
    int64_t bits = r.ReadInteger();
    double v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

void
WriteLeoPass(oranntn::asn1::PerWriter& w,
              const oranntn::ccc::LeoPassToggleIe& p)
{
    w.WriteInteger(static_cast<int64_t>(p.nr_cell_global_id));
    w.WriteInteger(static_cast<int64_t>(p.beam_id));
    w.WriteInteger(p.enable ? 1 : 0);
    WriteDouble(w, p.t_event_s);
}

void
ReadLeoPass(oranntn::asn1::PerReader& r,
             oranntn::ccc::LeoPassToggleIe& p)
{
    p.nr_cell_global_id = static_cast<uint64_t>(r.ReadInteger());
    p.beam_id = static_cast<uint16_t>(r.ReadInteger());
    p.enable = r.ReadInteger() != 0;
    p.t_event_s = ReadDouble(r);
}

void
WriteBeamReconfig(oranntn::asn1::PerWriter& w,
                   const oranntn::ccc::BeamReconfigIe& b)
{
    w.WriteInteger(static_cast<int64_t>(b.nr_cell_global_id));
    w.WriteInteger(static_cast<int64_t>(b.beam_id));
    WriteDouble(w, b.steering_az_deg);
    WriteDouble(w, b.steering_el_deg);
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(b.codebook_weights.size()));
    for (double v : b.codebook_weights)
    {
        WriteDouble(w, v);
    }
}

void
ReadBeamReconfig(oranntn::asn1::PerReader& r,
                  oranntn::ccc::BeamReconfigIe& b)
{
    b.nr_cell_global_id = static_cast<uint64_t>(r.ReadInteger());
    b.beam_id = static_cast<uint16_t>(r.ReadInteger());
    b.steering_az_deg = ReadDouble(r);
    b.steering_el_deg = ReadDouble(r);
    const uint32_t n = r.ReadLengthDeterminant();
    b.codebook_weights.clear();
    b.codebook_weights.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        b.codebook_weights.push_back(ReadDouble(r));
    }
}

void
WriteDopplerRetune(oranntn::asn1::PerWriter& w,
                    const oranntn::ccc::DopplerRetuneIe& d)
{
    w.WriteInteger(static_cast<int64_t>(d.nr_cell_global_id));
    w.WriteInteger(static_cast<int64_t>(d.arfcn_dl));
    w.WriteInteger(static_cast<int64_t>(d.arfcn_ul));
    WriteDouble(w, d.doppler_offset_hz);
}

void
ReadDopplerRetune(oranntn::asn1::PerReader& r,
                   oranntn::ccc::DopplerRetuneIe& d)
{
    d.nr_cell_global_id = static_cast<uint64_t>(r.ReadInteger());
    d.arfcn_dl = static_cast<uint32_t>(r.ReadInteger());
    d.arfcn_ul = static_cast<uint32_t>(r.ReadInteger());
    d.doppler_offset_hz = ReadDouble(r);
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
    // 4.1.8 — NTN extensions: three optional vectors at the tail. Older
    // toolkit consumers parsing only the 4.1.6 prefix still decode the
    // first two lists; the new tail extends the wire format compatibly.
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(a.leo_pass_updates.size()));
    for (const auto& p : a.leo_pass_updates)
    {
        WriteLeoPass(w, p);
    }
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(a.beam_reconfigs.size()));
    for (const auto& b : a.beam_reconfigs)
    {
        WriteBeamReconfig(w, b);
    }
    w.WriteLengthDeterminant(
        static_cast<uint32_t>(a.doppler_retunes.size()));
    for (const auto& d : a.doppler_retunes)
    {
        WriteDopplerRetune(w, d);
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
        // 4.1.8 — NTN extensions. Reader stays compatible with the older
        // 4.1.6 wire form: if the buffer ends here, leave the extension
        // vectors empty.
        out->leo_pass_updates.clear();
        out->beam_reconfigs.clear();
        out->doppler_retunes.clear();
        if (!r.Eof())
        {
            const uint32_t numLeo = r.ReadLengthDeterminant();
            out->leo_pass_updates.reserve(numLeo);
            for (uint32_t i = 0; i < numLeo; ++i)
            {
                oranntn::ccc::LeoPassToggleIe p{};
                ReadLeoPass(r, p);
                out->leo_pass_updates.push_back(p);
            }
            const uint32_t numBeam = r.ReadLengthDeterminant();
            out->beam_reconfigs.reserve(numBeam);
            for (uint32_t i = 0; i < numBeam; ++i)
            {
                oranntn::ccc::BeamReconfigIe b{};
                ReadBeamReconfig(r, b);
                out->beam_reconfigs.push_back(b);
            }
            const uint32_t numDop = r.ReadLengthDeterminant();
            out->doppler_retunes.reserve(numDop);
            for (uint32_t i = 0; i < numDop; ++i)
            {
                oranntn::ccc::DopplerRetuneIe d{};
                ReadDopplerRetune(r, d);
                out->doppler_retunes.push_back(d);
            }
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
