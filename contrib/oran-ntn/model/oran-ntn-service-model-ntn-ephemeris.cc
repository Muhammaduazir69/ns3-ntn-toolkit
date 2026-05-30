/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "oran-ntn-service-model-ntn-ephemeris.h"

#include "../asn1/asn1-per-codec.h"

#include "ns3/log.h"

#include <cstring>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranNtnServiceModelNtnEphemeris");

TypeId
OranNtnServiceModelNtnEphemeris::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranNtnServiceModelNtnEphemeris")
                            .SetParent<OranNtnServiceModel>()
                            .SetGroupName("OranNtn")
                            .AddConstructor<OranNtnServiceModelNtnEphemeris>();
    return tid;
}

namespace
{

/// Pack a double into a PER INTEGER (raw IEEE-754 64-bit bit pattern).
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
    const int64_t bits = r.ReadInteger();
    double v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

} // namespace

std::vector<uint8_t>
OranNtnServiceModelNtnEphemeris::EncodeIndication(const void* body) const
{
    const auto* in =
        static_cast<const oranntn::ephemeris::Sib19NtnConfig*>(body);
    oranntn::asn1::PerWriter w;

    // SEQUENCE preamble for the two OPTIONAL top-level fields:
    //   slot 0 = service_window present
    //   slot 1 = ntn_ul_sync_validity_duration_ms present
    w.BeginSequencePreamble(2);
    w.SetPreambleBit(0, in->service_window.has_value());
    w.SetPreambleBit(1,
                      in->ntn_ul_sync_validity_duration_ms.has_value());
    w.EndSequencePreamble();

    // epochTime
    w.WriteInteger(static_cast<int64_t>(in->epoch.sfn));
    w.WriteInteger(static_cast<int64_t>(in->epoch.subframe));

    // ephemeris CHOICE: 0 = orbital, 1 = positionVelocity
    if (std::holds_alternative<oranntn::ephemeris::OrbitalElementsIe>(
            in->ephemeris))
    {
        w.WriteChoiceIndex(0);
        const auto& e =
            std::get<oranntn::ephemeris::OrbitalElementsIe>(in->ephemeris);
        WriteDouble(w, e.semi_major_axis_m);
        WriteDouble(w, e.eccentricity);
        WriteDouble(w, e.inclination_rad);
        WriteDouble(w, e.raan_rad);
        WriteDouble(w, e.arg_perigee_rad);
        WriteDouble(w, e.mean_anomaly_rad);
    }
    else
    {
        w.WriteChoiceIndex(1);
        const auto& p =
            std::get<oranntn::ephemeris::PositionVelocityIe>(in->ephemeris);
        WriteDouble(w, p.pos_x_m);
        WriteDouble(w, p.pos_y_m);
        WriteDouble(w, p.pos_z_m);
        WriteDouble(w, p.vel_x_mps);
        WriteDouble(w, p.vel_y_mps);
        WriteDouble(w, p.vel_z_mps);
    }

    // ta-Info
    WriteDouble(w, in->ta.ta_common_us);
    WriteDouble(w, in->ta.ta_common_drift_us_per_s);
    WriteDouble(w, in->ta.ta_common_drift_variation_us_per_s2);

    // cellSpecificKoffset
    w.WriteInteger(static_cast<int64_t>(in->cell_specific_koffset_slots));

    // OPTIONAL service window
    if (in->service_window.has_value())
    {
        WriteDouble(w, in->service_window->t_service_start_s);
        WriteDouble(w, in->service_window->t_service_dur_s);
    }
    // OPTIONAL UL sync validity duration
    if (in->ntn_ul_sync_validity_duration_ms.has_value())
    {
        w.WriteInteger(static_cast<int64_t>(
            *in->ntn_ul_sync_validity_duration_ms));
    }
    return w.Take();
}

bool
OranNtnServiceModelNtnEphemeris::DecodeIndication(
    const std::vector<uint8_t>& msg,
    oranntn::ephemeris::Sib19NtnConfig& out) const
{
    try
    {
        oranntn::asn1::PerReader r(msg);
        const uint16_t pre = r.ReadSequencePreamble(2);
        const bool hasWin = (pre >> 1) & 1;
        const bool hasDur = pre & 1;

        out.epoch.sfn = static_cast<uint16_t>(r.ReadInteger());
        out.epoch.subframe = static_cast<uint8_t>(r.ReadInteger());

        const uint8_t ch = r.ReadChoiceIndex();
        if (ch == 0)
        {
            oranntn::ephemeris::OrbitalElementsIe e{};
            e.semi_major_axis_m = ReadDouble(r);
            e.eccentricity = ReadDouble(r);
            e.inclination_rad = ReadDouble(r);
            e.raan_rad = ReadDouble(r);
            e.arg_perigee_rad = ReadDouble(r);
            e.mean_anomaly_rad = ReadDouble(r);
            out.ephemeris = e;
        }
        else if (ch == 1)
        {
            oranntn::ephemeris::PositionVelocityIe p{};
            p.pos_x_m = ReadDouble(r);
            p.pos_y_m = ReadDouble(r);
            p.pos_z_m = ReadDouble(r);
            p.vel_x_mps = ReadDouble(r);
            p.vel_y_mps = ReadDouble(r);
            p.vel_z_mps = ReadDouble(r);
            out.ephemeris = p;
        }
        else
        {
            NS_LOG_WARN("Ephemeris SM: unknown CHOICE index "
                        << static_cast<int>(ch));
            return false;
        }

        out.ta.ta_common_us = ReadDouble(r);
        out.ta.ta_common_drift_us_per_s = ReadDouble(r);
        out.ta.ta_common_drift_variation_us_per_s2 = ReadDouble(r);
        out.cell_specific_koffset_slots =
            static_cast<int16_t>(r.ReadInteger());
        if (hasWin)
        {
            oranntn::ephemeris::ServiceWindowIe sw{};
            sw.t_service_start_s = ReadDouble(r);
            sw.t_service_dur_s = ReadDouble(r);
            out.service_window = sw;
        }
        else
        {
            out.service_window.reset();
        }
        if (hasDur)
        {
            out.ntn_ul_sync_validity_duration_ms =
                static_cast<uint32_t>(r.ReadInteger());
        }
        else
        {
            out.ntn_ul_sync_validity_duration_ms.reset();
        }
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("Ephemeris SM: PER decode error: " << exc.what());
        return false;
    }
}

} // namespace ns3
