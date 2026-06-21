/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026 Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "ntn-scene-helper.h"

#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSceneHelper");

namespace ntnobs
{

void
NtnSceneHelper::SetEnuFrame(double lat0Deg, double lon0Deg, double alt0M)
{
    m_enu = true;
    m_lat0 = lat0Deg;
    m_lon0 = lon0Deg;
    m_alt0 = alt0M;
}

Ptr<NtnSceneRecorder>
NtnSceneHelper::Build(NodeContainer sats, NodeContainer ues, NodeContainer gateways)
{
    if (!Enabled())
    {
        return nullptr;
    }

    Ptr<NtnSceneRecorder> rec = CreateObject<NtnSceneRecorder>();
    if (m_enu)
    {
        rec->SetFrame(NtnSceneRecorder::LocalEnu);
        rec->SetEnuReference(m_lat0, m_lon0, m_alt0);
    }
    else
    {
        rec->SetFrame(NtnSceneRecorder::EcefGlobal);
    }
    rec->SetSampleInterval(m_dt);

    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        rec->TrackNode(sats.Get(i), NtnSceneRecorder::Sat, "sat-" + std::to_string(i));
    }
    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        rec->TrackNode(ues.Get(i), NtnSceneRecorder::Ue, "ue-" + std::to_string(i));
    }
    for (uint32_t i = 0; i < gateways.GetN(); ++i)
    {
        rec->TrackNode(gateways.Get(i), NtnSceneRecorder::Gateway, "gw-" + std::to_string(i));
    }

    if (!m_netSim.empty())
    {
        rec->EnableNetSimulyzer(m_netSim, m_sceneScale);
    }
    if (!m_czml.empty())
    {
        rec->EnableCzml(m_czml);
    }
    if (m_live)
    {
        rec->EnableLiveStdout(true);
    }
    rec->Start();
    return rec;
}

} // namespace ntnobs
} // namespace ns3
