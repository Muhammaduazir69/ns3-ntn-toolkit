/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-rrc-helper.h"

#include <ns3/log.h>
#include <ns3/mobility-model.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnRrcHelper");

namespace ntnrrc
{

NtnRrcHelper::NtnRrcHelper() = default;

void
NtnRrcHelper::SetPayloadMode(PayloadMode mode)
{
    m_payloadMode = mode;
}

void
NtnRrcHelper::SetReferencePosition(const Vector& earthFixedRefPosition)
{
    m_referencePos = earthFixedRefPosition;
}

Ptr<NtnTimingAdvance>
NtnRrcHelper::InstallTimingAdvance(Ptr<MobilityModel> ueMob, Ptr<MobilityModel> satMob) const
{
    Ptr<NtnTimingAdvance> ta = CreateObject<NtnTimingAdvance>();
    ta->SetUeMobility(ueMob);
    ta->SetSatelliteMobility(satMob);
    ta->SetReferencePosition(m_referencePos);
    ta->SetPayloadMode(m_payloadMode);
    if (m_gatewayMob)
    {
        ta->SetGatewayMobility(m_gatewayMob);
    }
    return ta;
}

Ptr<NtnSib19Broadcaster>
NtnRrcHelper::InstallSib19Broadcaster(Ptr<MobilityModel> satMob,
                                      uint16_t cellId,
                                      Ptr<NtnTimingAdvance> ta,
                                      Time period) const
{
    Ptr<NtnSib19Broadcaster> bc = CreateObject<NtnSib19Broadcaster>();
    bc->SetSatelliteMobility(satMob);
    if (ta)
    {
        bc->SetTimingAdvance(ta);
    }
    bc->SetReferencePosition(m_referencePos);
    bc->SetCellId(cellId);
    bc->SetPayloadMode(m_payloadMode);
    bc->SetPeriod(period);
    // P0-6: reference point by payload (TS 38.213 4.2).
    const UlSyncReference ref = GetEffectiveUlSyncReference();
    bc->SetUlSyncReference(ref);
    if (m_gatewayMob)
    {
        // Gateway reference: the common delay is the feeder leg. Satellite
        // reference on a transparent payload: kMac needs the feeder geometry.
        bc->SetGatewayPosition(m_gatewayMob->GetPosition());
    }
    return bc;
}

UlSyncReference
NtnRrcHelper::GetEffectiveUlSyncReference() const
{
    if (m_ulRefSet)
    {
        return m_ulRef;
    }
    if (m_payloadMode == PayloadMode::Transparent && m_gatewayMob)
    {
        return UlSyncReference::Gateway;
    }
    return UlSyncReference::Satellite;
}

Ptr<NtnUeLocationReporter>
NtnRrcHelper::InstallUeLocationReporter(Ptr<MobilityModel> ueMob,
                                        LocationReportMode mode,
                                        Time period,
                                        double accuracyMetres) const
{
    Ptr<NtnUeLocationReporter> rep = CreateObject<NtnUeLocationReporter>();
    rep->SetUeMobility(ueMob);
    rep->SetReportingMode(mode);
    rep->SetReportingPeriod(period);
    rep->SetAccuracyMetres(accuracyMetres);
    return rep;
}

Ptr<NtnDrxStateMachine>
NtnRrcHelper::InstallDrx(const NtnDrxConfig& cfg) const
{
    Ptr<NtnDrxStateMachine> drx = CreateObject<NtnDrxStateMachine>();
    drx->SetConfig(cfg);
    return drx;
}

} // namespace ntnrrc
} // namespace ns3
