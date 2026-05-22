/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_SERVICE_MODEL_NTN_EPHEMERIS_H
#define ORAN_NTN_SERVICE_MODEL_NTN_EPHEMERIS_H

// NTN-Ephemeris Service Model (Roadmap §4.1.7).
//
// New toolkit-defined SM with no upstream equivalent yet. Publishes the
// payload an NTN cell would put into 3GPP TS 38.331 SIB19 (§6.3.1):
//
//   ephemerisInfo  CHOICE { orbital, positionVelocity }
//   ta-Info        ta-Common + ta-CommonDrift + ta-CommonDriftVariation
//   cellSpecificKoffset
//   epochTime      (SFN, subframe-within-SFN)
//   t-Service      service-window start + duration
//
// A FlexRIC xApp subscribing to RIC Function ID 1001 receives an
// Indication every time the cell's SIB19 contents change (orbital
// update, TA-info refresh, Koffset retune). The xApp can use the
// ephemeris to plan handovers, predict service windows, or feed the
// CHO trigger table (§4.1.3 Style 3 ControlAction).
//
// On-the-wire encoding goes through the T2 Aligned-PER codec so the
// blob fits straight into a `ric_indication_t::ric_indication_message`.

#include "oran-ntn-service-model.h"

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace ns3
{

namespace oranntn
{
namespace ephemeris
{

/// SIB19 ephemerisInfo CHOICE alternative 0 — Keplerian orbital
/// elements (the form satellite operators publish via TLEs).
struct OrbitalElementsIe
{
    double semi_major_axis_m;    //!< a
    double eccentricity;          //!< e (0 = circular)
    double inclination_rad;       //!< i
    double raan_rad;              //!< Right Ascension of Ascending Node
    double arg_perigee_rad;       //!< omega
    double mean_anomaly_rad;      //!< M at epoch
};

/// SIB19 ephemerisInfo CHOICE alternative 1 — ECEF position+velocity
/// state vector at the SIB19 epoch.
struct PositionVelocityIe
{
    double pos_x_m;
    double pos_y_m;
    double pos_z_m;
    double vel_x_mps;
    double vel_y_mps;
    double vel_z_mps;
};

/// SIB19 epochTime IE — (SFN, subframe) at which the ephemeris and TA
/// values are anchored. SFN range 0..1023, subframe 0..9.
struct EpochTimeIe
{
    uint16_t sfn;
    uint8_t subframe;
};

/// SIB19 ta-Info IE — common Timing Advance and its first / second
/// derivatives across the service window.
struct TimingAdvanceIe
{
    double ta_common_us;                 //!< TA at epoch
    double ta_common_drift_us_per_s;     //!< first derivative
    double ta_common_drift_variation_us_per_s2; //!< second derivative
};

/// SIB19 t-Service IE — service window.
struct ServiceWindowIe
{
    double t_service_start_s; //!< absolute start time (wall-clock seconds)
    double t_service_dur_s;   //!< duration
};

/// Whole-SIB19 NTN configuration record.
struct Sib19NtnConfig
{
    EpochTimeIe epoch;
    /// 0 = orbital, 1 = position-velocity. (The std::variant index also
    /// signals this, but the explicit form lets non-variant consumers
    /// route the data.)
    std::variant<OrbitalElementsIe, PositionVelocityIe> ephemeris;
    TimingAdvanceIe ta;
    int16_t cell_specific_koffset_slots; //!< per TS 38.331
    std::optional<ServiceWindowIe> service_window;
    /// Optional integer in milliseconds — UE TA-validity duration.
    std::optional<uint32_t> ntn_ul_sync_validity_duration_ms;
};

} // namespace ephemeris
} // namespace oranntn

/**
 * \ingroup oran-ntn
 * \brief E2SM-NTN-Ephemeris v1.00 plugin (Roadmap §4.1.7).
 *
 * RIC Function ID 1001 (toolkit-reserved 1000+ range). Encodes
 * Sib19NtnConfig as an E2 Indication so xApps can subscribe to NTN
 * ephemeris updates and plan ahead.
 */
class OranNtnServiceModelNtnEphemeris : public OranNtnServiceModel
{
  public:
    static constexpr uint16_t kRicFunctionId = 1001;

    static TypeId GetTypeId();
    OranNtnServiceModelNtnEphemeris() = default;
    ~OranNtnServiceModelNtnEphemeris() override = default;

    uint16_t RicFunctionId() const override { return kRicFunctionId; }
    std::string Name() const override { return "NTN-Ephemeris"; }
    std::string Version() const override { return "v1.00"; }

    /// `body` must point at an oranntn::ephemeris::Sib19NtnConfig.
    std::vector<uint8_t> EncodeIndication(const void* body) const override;

    /// Ephemeris SM is publish-only — no control direction.
    bool DecodeControl(const std::vector<uint8_t>& /*msg*/,
                       void* /*out*/) const override
    {
        return false;
    }

    /// Inverse of EncodeIndication for tests.
    bool DecodeIndication(const std::vector<uint8_t>& msg,
                          oranntn::ephemeris::Sib19NtnConfig& out) const;
};

} // namespace ns3

#endif // ORAN_NTN_SERVICE_MODEL_NTN_EPHEMERIS_H
