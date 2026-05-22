/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SAGIN_HST_MOBILITY_MODEL_H
#define NTN_SAGIN_HST_MOBILITY_MODEL_H

// 3GPP TR 38.901 §7.5 HST mobility model (Roadmap §4.4.3).
//
// Linear motion along a fixed rail corridor at a configured speed. The
// gNB perpendicular offset Dmin and the cell-spacing Ds are exposed for
// scenario builders; they don't affect the train's own motion but matter
// for path-loss / Doppler / handover scheduling logic that consumes this
// mobility.
//
// Position is computed analytically from sim time — no internal scheduled
// events, so the model is cheap to install on many UEs.

#include "hst-trace.h"

#include "ns3/mobility-model.h"

namespace ns3
{
namespace sagin
{

class HstMobilityModel : public MobilityModel
{
  public:
    static TypeId GetTypeId();
    HstMobilityModel();
    ~HstMobilityModel() override = default;

    /// Install a trace. The model retains a copy of the trace's metadata
    /// (speed, dmin, cellSpacing) and uses sample list for interpolation.
    void SetTrace(const HstTrace& trace);

    /// Set the carrier frequency used by `GetDopplerHz`. Default 30 GHz.
    void SetCarrierFrequencyHz(double freqHz);

    /// Carrier Doppler shift in Hz at the current sim time given a static
    /// gNB position. Positive when train approaches the gNB.
    double GetDopplerHz(double gnb_x_m) const;

    /// Train metadata (read-only).
    double GetSpeedKmh() const { return m_trace.speed_kmh; }
    double GetDminM() const { return m_trace.dmin_m; }
    double GetCellSpacingM() const { return m_trace.cellSpacing_m; }

  private:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

    HstTrace m_trace;
    double m_carrierHz{30e9};
};

} // namespace sagin
} // namespace ns3

#endif // NTN_SAGIN_HST_MOBILITY_MODEL_H
