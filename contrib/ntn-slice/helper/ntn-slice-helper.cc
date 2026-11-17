/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#include "ntn-slice-helper.h"

#include "ns3/ntn-real-stack-helper.h"

namespace ns3
{
namespace ntnslice
{

NtnSliceStack
NtnSliceHelper::ThreeSliceDefault()
{
    NtnSliceStack stack;
    stack.embb = DefaultEmbb();
    stack.urllc = DefaultUrllc();
    stack.mmtc = DefaultMmtc();

    stack.selector = CreateObject<NtnSliceSelector>();
    stack.selector->SetDefault(stack.embb.snssai);
    // Industry-typical mappings. URLLC tends to use DSCP EF (46) per RFC 4594;
    // mMTC traffic typically lands on a non-elevated DSCP via well-known IoT ports.
    stack.selector->AddDscpRule(46, stack.urllc.snssai);
    stack.selector->AddPortRangeRule(5683, 5684, stack.mmtc.snssai); // CoAP
    stack.selector->AddAppLabelRule("urllc", stack.urllc.snssai);
    stack.selector->AddAppLabelRule("mmtc", stack.mmtc.snssai);
    stack.selector->AddAppLabelRule("embb", stack.embb.snssai);

    stack.orchestrator = CreateObject<SliceOrchestratorXapp>();
    stack.orchestrator->RegisterSlice(stack.embb);
    stack.orchestrator->RegisterSlice(stack.urllc);
    stack.orchestrator->RegisterSlice(stack.mmtc);

    stack.monitor = CreateObject<SliceIsolationMonitor>();
    stack.monitor->RegisterSlice(stack.embb);
    stack.monitor->RegisterSlice(stack.urllc);
    stack.monitor->RegisterSlice(stack.mmtc);

    return stack;
}

bool
NtnSliceHelper::ConfigureCellLinkBudget(NtnRealStackHelper& rs, double satPowerDbm, bool mmwave)
{
    if (satPowerDbm >= 0.0)
    {
        rs.SetSatConductedPowerDbm(satPowerDbm);
        return false;
    }
    if (mmwave)
    {
        rs.SetSatConductedPowerDbm(55.0);
        return false;
    }
    rs.ApplyTr38821Set1Defaults();
    return true;
}

Ptr<MobilityModel>
NtnSliceHelper::ConfigureFeeder(NtnRealStackHelper& rs,
                                double subLatDeg,
                                double subLonDeg,
                                double gwDistanceKm)
{
    if (gwDistanceKm <= 0.0)
    {
        return nullptr;
    }
    // 111.195 km per degree of latitude (mean Earth radius 6371 km).
    return rs.SetFeederGatewayGeodetic(subLatDeg + gwDistanceKm / 111.195, subLonDeg, 0.0);
}

double
NtnSliceHelper::OneWayFloorMs(double altitudeKm, bool liveFeeder, double backhaulMs)
{
    const double legMs = altitudeKm * 1e3 / 299792458.0 * 1e3;
    return legMs + (liveFeeder ? legMs : backhaulMs);
}

} // namespace ntnslice
} // namespace ns3
