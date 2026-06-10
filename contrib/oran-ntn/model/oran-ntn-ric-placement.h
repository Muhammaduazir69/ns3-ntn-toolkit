/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// OranNtnRicPlacement — RIC placement as an EXPERIMENT VARIABLE (Deng 2026
// Sec. IV-B; adoption plan WS3). Where the (Near-RT) RIC runs determines the
// E2 indication/control latency, computed from REAL geometry:
//
//   OnBoardSatellite : processing only — the RIC rides the regenerative
//                      payload next to the O-DU (paper "Space RIC").
//   Haps             : sat->HAP slant / c + processing.
//   GroundGateway    : sat->ground slant / c + processing (paper's measured
//                      NTN fronthaul band: 2.1–18 ms one-way at LEO).
//   GroundCloud      : slant / c + terrestrial backhaul to the regional cloud.
//
// BindToGeometry() keeps an OranNtnE2Node's feeder-link delay tracking the
// LIVE slant range of the serving satellite's REAL mobility model, so a
// control decision made on stale KPM is visibly worse in the measured KPIs —
// exactly the experiment the paper calls for.

#ifndef ORAN_NTN_RIC_PLACEMENT_H
#define ORAN_NTN_RIC_PLACEMENT_H

#include "oran-ntn-e2-interface.h"

#include <ns3/mobility-model.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/ptr.h>

#include <cstdint>

namespace ns3
{

class OranNtnRicPlacement : public Object
{
  public:
    enum class Site : uint8_t
    {
        OnBoardSatellite = 0,
        Haps = 1,
        GroundGateway = 2,
        GroundCloud = 3,
    };

    static TypeId GetTypeId();
    OranNtnRicPlacement() = default;

    void SetSite(Site s) { m_site = s; }
    Site GetSite() const { return m_site; }
    void SetProcessingDelay(Time t) { m_processing = t; }
    void SetCloudBackhaulDelay(Time t) { m_cloudBackhaul = t; }

    /// One-way E2 delay for the CURRENT geometry (slant range in metres,
    /// from the real mobility models of the DU and the RIC site).
    Time ComputeE2Delay(double slantRangeM) const;

    /**
     * \brief Keep \p e2's feeder-link delay tracking the live DU->RIC-site
     *        slant range every \p period. Both mobility models must be the
     *        REAL ones (SGP4 / TR 38.811); the delay is slant/c + site terms.
     */
    void BindToGeometry(Ptr<MobilityModel> duMobility,
                        Ptr<MobilityModel> ricSiteMobility,
                        Ptr<OranNtnE2Node> e2,
                        Time period);

    /// Most recent delay applied by the geometry binder.
    Time GetLastAppliedDelay() const { return m_lastApplied; }

  private:
    void GeometryTick();

    Site m_site{Site::GroundGateway};
    Time m_processing{MicroSeconds(500)};
    Time m_cloudBackhaul{MilliSeconds(20)};

    Ptr<MobilityModel> m_du;
    Ptr<MobilityModel> m_ricSite;
    Ptr<OranNtnE2Node> m_e2;
    Time m_period{Seconds(1.0)};
    Time m_lastApplied{Seconds(0)};
};

} // namespace ns3

#endif // ORAN_NTN_RIC_PLACEMENT_H
