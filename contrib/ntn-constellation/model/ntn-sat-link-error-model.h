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

    /// Telemetry from the last DoCorrupt() call.
    double LastEsNoDb() const { return m_lastEsNoDb; }
    double LastBler() const { return m_lastBler; }

  private:
    bool DoCorrupt(Ptr<Packet> p) override;
    void DoReset() override;

    double SlantRangeM() const;
    double EsNoDbFor(double slantM) const;
    double BlerFor(double esNoDb) const;

    Ptr<MobilityModel> m_tx;
    Ptr<MobilityModel> m_rx;
    Ptr<UniformRandomVariable> m_rng;

    double m_eirpDbw{20.0};      //!< satellite/Tx EIRP (dBW)
    double m_rxGtDbK{1.0};       //!< receiver G/T (dB/K)
    double m_bandwidthHz{30e6};  //!< noise bandwidth (Hz)
    double m_carrierHz{20e9};    //!< carrier frequency (Hz)
    double m_thresholdEsNoDb{1.0}; //!< MODCOD Es/No threshold (dB), e.g. QPSK 1/2
    double m_waterfallDb{1.0};   //!< waterfall steepness (dB)

    mutable double m_lastEsNoDb{0.0};
    mutable double m_lastBler{0.0};
};

} // namespace ntncon
} // namespace ns3

#endif // NTN_SAT_LINK_ERROR_MODEL_H
