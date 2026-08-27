/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// NtnSatLinkErrorModel — a per-packet C/N0 -> BLER error model for satellite
// GSL/ISL hops, replacing the binary geometry "contact gate" used by the
// constellation/ISL routed examples.
//
// The routed transport (ntn-constellation-real-routed, *-isl-routed, sagin
// multihop) forwards real IP packets through satellite nodes but decided
// per-hop loss with a binary in-contact/out-of-contact gate. This ErrorModel
// instead computes, for every packet, the received Es/No from the LIVE slant
// range between the two endpoint mobility models (FSPL + configured EIRP / G-T /
// bandwidth) and maps it through a DVB-S2 MODCOD waterfall to a block-error
// probability, so a marginal pass degrades gracefully and a closed link drops
// hard — a measured link quality, not an on/off switch.
//
// The waterfall is the analytic DVB-S2 form (erfc around the MODCOD Es/No
// threshold); it can be swapped for the satellite module's exact
// SatLinkResultsDvbS2 BLER tables where the SNS3 data path is configured.

#ifndef NTN_SAT_LINK_ERROR_MODEL_H
#define NTN_SAT_LINK_ERROR_MODEL_H

#include "ns3/error-model.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"
#include "ns3/ptr.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{
namespace ntncon
{

class NtnSatLinkErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId();
    NtnSatLinkErrorModel();

    /// The two link endpoints whose live positions set the slant range.
    void SetEndpoints(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx);

    /// Received Es/No (dB) for the current endpoint geometry.
    double CurrentEsNoDb() const;
    /// BLER for the current geometry (0..1).
    double CurrentBler() const;
    /// Slant range (m) between the endpoints right now.
    double CurrentSlantRangeM() const;

    /// Link elevation (deg) at the lower endpoint for the current geometry.
    ///
    /// CON-2: the model had no elevation input at all, only the slant range
    /// that geometry implies. Every excess-loss term in an Earth-space budget
    /// is elevation-dependent, so without this the atmosphere cannot be
    /// modelled at all. Returns 90 for an inter-satellite link, where there is
    /// no atmospheric path.
    double CurrentElevationDeg() const;

    /// Total atmospheric excess loss (dB) applied to the current geometry:
    /// gaseous plus rain plus scintillation. Zero when all three are disabled.
    double CurrentExcessLossDb() const;

    /// Telemetry from the last DoCorrupt() call.
    double LastEsNoDb() const { return m_lastEsNoDb; }
    double LastBler() const { return m_lastBler; }
    /// CON-2: the excess-loss breakdown behind the last Es/No, so a result can
    /// show what the atmosphere cost rather than only the total.
    double LastGaseousDb() const { return m_lastGaseousDb; }
    double LastRainDb() const { return m_lastRainDb; }
    double LastScintillationDb() const { return m_lastScintDb; }

  private:
    bool DoCorrupt(Ptr<Packet> p) override;
    void DoReset() override;

    double SlantRangeM() const;
    double EsNoDbFor(double slantM) const;
    double BlerFor(double esNoDb) const;
    /// CON-2: elevation at the lower endpoint, from the two ECEF positions.
    double ElevationDegFor(const Vector& lower, const Vector& upper) const;
    /// CON-2: gaseous + rain + scintillation for an elevation, in dB.
    double ExcessLossDbFor(double elevationDeg) const;

    Ptr<MobilityModel> m_tx;
    Ptr<MobilityModel> m_rx;
    Ptr<UniformRandomVariable> m_rng;

    double m_eirpDbw{20.0};      //!< satellite/Tx EIRP (dBW)
    double m_rxGtDbK{1.0};       //!< receiver G/T (dB/K)
    double m_bandwidthHz{30e6};  //!< noise bandwidth (Hz)
    double m_carrierHz{20e9};    //!< carrier frequency (Hz)
    double m_thresholdEsNoDb{1.0}; //!< MODCOD Es/No threshold (dB), e.g. QPSK 1/2
    double m_waterfallDb{1.0};   //!< waterfall steepness (dB)

    // ---- CON-2: atmospheric excess loss ------------------------------------
    //
    // The budget was EIRP - FSPL + G/T - k - 10log10(B) and nothing else, at a
    // 20 GHz Ka default. At Ka, rain alone runs from single digits to well over
    // 15 dB at 0.01 percent availability, and gaseous absorption plus
    // scintillation add several dB at low elevation. The omission grows as
    // elevation falls, which is exactly where handover decisions are made, so
    // the C/N0 and BLER series this model wrote to sim_health.csv under
    // provenance "bler-errormodel" were optimistic in a systematic,
    // geometry-correlated way.
    //
    // All three terms reuse the ITU models the toolkit already ships in
    // thz-ntn rather than introducing a second implementation of the same
    // recommendations.
    bool m_enableGaseous{true};    //!< ITU-R P.676-13 gaseous absorption
    bool m_enableRain{true};       //!< ITU-R P.618-13 / P.838-3 rain
    bool m_enableScintillation{true}; //!< ITU-R P.618-13 tropospheric scintillation
    double m_rainRateMmH{0.0};     //!< R_0.01 point rate; 0 disables rain
    double m_groundAltKm{0.05};    //!< ground-station altitude (km MSL)
    double m_stationLatDeg{45.0};  //!< station latitude, for the P.618 factors
    double m_antennaDiameterM{0.6};//!< rx aperture, for scintillation averaging

    // ---- CON-2: MODCOD curve ----
    //
    // BlerFor was 0.5*erfc((esno - threshold)/waterfall), described in a
    // comment as "DVB-S2-style" with no MODCOD table behind it: a generic
    // sigmoid with two tunable numbers. The vendored SNS3 satellite module
    // already ships the real DVB-S2 forward-link look-up tables
    // (data/additional-input/linkresults/s2_*.txt), so the honest curve was
    // one dependency away.
    //
    // Off by default: switching every existing scenario onto a different BLER
    // curve is a results change, not a bug fix, so a scenario opts in.
    bool m_useDvbS2LinkResults{false};
    /// MODCOD index into SatEnums::SatModcod_t. Defaults to QPSK 1/2.
    uint32_t m_modcod{0};
    mutable Ptr<Object> m_linkResults; //!< SatLinkResultsDvbS2, built lazily

    mutable Ptr<Object> m_gas;    //!< itu::Itu676AbsorptionModel, built lazily
    mutable Ptr<Object> m_rain;   //!< itu::Itu618LossModel, built lazily
    mutable Ptr<Object> m_scint;  //!< ThzNtnScintillation, built lazily

    mutable double m_lastEsNoDb{0.0};
    mutable double m_lastBler{0.0};
    mutable double m_lastGaseousDb{0.0};
    mutable double m_lastRainDb{0.0};
    mutable double m_lastScintDb{0.0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_SAT_LINK_ERROR_MODEL_H
