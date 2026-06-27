/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W7)
 *
 * V2V relay via LEO.
 *
 * Vehicles that cannot reach a LEO satellite directly (foliage / canyon /
 * NLOS) fall back to relaying through a peer vehicle that has line of
 * sight. We pick the relay greedily — the peer with the best LEO uplink
 * SNR within `MaxV2vRangeM` metres.
 *
 * The model is deliberately simple — relay assignment is recomputed per
 * tick — but the API matches what a real L2/L3 NR sidelink Mode 1
 * (gNB-scheduled) / Mode 2 (autonomous) scheduler (3GPP TS 38.300 Cl.16,
 * Rel-16/17) would expose. Slot-level sidelink scheduling is not in scope
 * for W7. This is an abstracted system-level relay; it does NOT model
 * PSCCH/PSSCH/PSFCH or sidelink HARQ (TS 38.211/212/213/214) and is
 * therefore non-PHY-conformant.
 */
#ifndef NTN_V2X_LEO_RELAY_H
#define NTN_V2X_LEO_RELAY_H

#include "ns3/mobility-model.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "v2x-leo-direct.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnv2x
{

struct RelayDecision
{
    std::string vehId;
    bool directToLeo{false};
    std::string relayPeerId{}; ///< empty if direct
    double directSnrDb{0.0};
    double viaRelaySnrDb{0.0};
    double v2vRangeM{0.0};     ///< distance to chosen relay
};

class V2xLeoRelay : public Object
{
  public:
    static TypeId GetTypeId();
    V2xLeoRelay();

    void SetMaxV2vRangeM(double m);
    void SetMinDirectSnrDb(double snrDb);

    void SetSatellite(Ptr<MobilityModel> sat);
    Ptr<MobilityModel> GetSatellite() const;

    void RegisterVehicle(const std::string& id, Ptr<MobilityModel> mob);

    /// Per-vehicle NLOS blockage (canyon / foliage / superstructure) subtracted
    /// from that vehicle's direct LEO SNR — same 3GPP-style shadowing term the
    /// measured-radio recipe (ntn-v2x-real-stack) applies, so a shadowed vehicle
    /// genuinely needs to relay. 0 dB (default) = clear line of sight.
    void SetVehicleBlockageDb(const std::string& id, double blockageDb);

    /// One-shot evaluation: returns relay decisions for all vehicles.
    std::vector<RelayDecision> EvaluateAll() const;

    std::size_t VehicleCount() const;

  private:
    Ptr<V2xLeoDirect> m_direct;
    Ptr<MobilityModel> m_sat;
    std::map<std::string, Ptr<MobilityModel>> m_vehicles;
    std::map<std::string, double> m_blockageDb; ///< per-vehicle NLOS shadowing
    double m_maxV2vRangeM{1500.0};
    double m_minDirectSnrDb{6.0};
};

} // namespace ntnv2x
} // namespace ns3

#endif // NTN_V2X_LEO_RELAY_H
