/*
 * Copyright (c) 2026 Muhammad Uzair
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Geographic context for the ns3-ai Wi-Fi examples (rate-control, multi-bss): the Wi-Fi
 * scenario is read as a satellite-backhauled hotspot at a real ground site. The Wi-Fi
 * nodes keep their local Cartesian positions, which the scene treats as east-north-up
 * offsets (m) about the site, so every distance the Wi-Fi channel sees is unchanged.
 * One device-less node carries the backhaul satellite: the member of a Walker-delta
 * shell (Vallado SGP4 through Sgp4MobilityModel) with the highest elevation over the
 * site at t = 0, projected into the site's ENU frame by NtnEnuProjectionMobilityModel.
 * Its ReferenceLatitude/Longitude/Altitude attributes make the globe scene of
 * StartSceneFromNodeList an ENU scene about the site. The node has no device and no
 * Internet stack, so the network and its measurements are not affected.
 */

#ifndef NTN_WIFI_SITE_CONTEXT_H
#define NTN_WIFI_SITE_CONTEXT_H

#include "ns3/node.h"
#include "ns3/ntn-geocentric-mobility-adapter.h"
#include "ns3/ntn-tr38811-mobility-model.h"
#include "ns3/sgp4-mobility-model.h"
#include "ns3/walker-constellation.h"

#include <cmath>
#include <string>

namespace ntnwifisite
{

using namespace ns3;

/// Ground site of the hotspot and the backhaul shell (CLI-exposed by the examples).
struct SiteConfig
{
    double latDeg{35.2971}; //!< Skardu (PK), a typical satellite-backhaul site
    double lonDeg{75.6333};
    double altM{2228.0};
    double shellAltKm{550.0};
    double shellInclinationDeg{53.0};
    uint32_t shellPlanes{72};
    uint32_t shellSatsPerPlane{22};
};

/// Summary of the added context satellite.
struct SiteContext
{
    Ptr<Node> satNode;       //!< device-less backhaul satellite node (null if none visible)
    double elevationDeg{-90.0}; //!< its elevation over the site at t = 0
    double slantRangeKm{0.0};   //!< its slant range at t = 0
    std::string propagator;     //!< "sgp4-vallado" or "kepler-j2"
};

/**
 * Add the backhaul satellite context node. Call after every scenario node exists, so the
 * scenario's node ids do not shift.
 *
 * @param cfg site and shell
 * @return the added node and its geometry at t = 0
 */
inline SiteContext
AddBackhaulSatelliteContext(const SiteConfig& cfg)
{
    SiteContext out;
    ntncon::WalkerConfig w;
    w.num_planes = cfg.shellPlanes;
    w.total_sats = cfg.shellPlanes * cfg.shellSatsPerPlane;
    w.altitude_km = cfg.shellAltKm;
    w.inclination_deg = cfg.shellInclinationDeg;
    w.epoch_unix_s = 1735689600.0; // 2025-01-01T00:00:00Z
    const Vector site = ntncon::NtnGeocentricMobilityAdapter::GeodeticToEcef(
        Vector(cfg.latDeg, cfg.lonDeg, cfg.altM));
    Ptr<ntncon::Sgp4MobilityModel> best;
    for (const auto& el : ntncon::WalkerConstellation::BuildDelta(w))
    {
        auto m = CreateObject<ntncon::Sgp4MobilityModel>();
        m->SetElementsSgp4(el);
        const double e = m->GetElevationDeg(cfg.latDeg, cfg.lonDeg);
        if (e > out.elevationDeg)
        {
            out.elevationDeg = e;
            best = m;
        }
    }
    if (!best || out.elevationDeg < 0.0)
    {
        return out;
    }
    out.slantRangeKm = CalculateDistance(best->GetPosition(), site) / 1e3;
    out.propagator = best->GetPropagatorName();
    Ptr<NtnEnuProjectionMobilityModel> enu = CreateObject<NtnEnuProjectionMobilityModel>();
    enu->SetSource(best);
    enu->SetReference(cfg.latDeg, cfg.lonDeg, cfg.altM);
    out.satNode = CreateObject<Node>();
    out.satNode->AggregateObject(enu);
    return out;
}

} // namespace ntnwifisite

#endif // NTN_WIFI_SITE_CONTEXT_H
