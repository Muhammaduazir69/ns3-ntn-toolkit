/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-cir-propagation-loss-model.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaCirPropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(SionnaCirPropagationLossModel);

TypeId
SionnaCirPropagationLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaCirPropagationLossModel")
                            .SetParent<PropagationLossModel>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaCirPropagationLossModel>();
    return tid;
}

SionnaCirPropagationLossModel::SionnaCirPropagationLossModel() = default;
SionnaCirPropagationLossModel::~SionnaCirPropagationLossModel() = default;

void
SionnaCirPropagationLossModel::SetSnapshot(const CirSnapshot& snap)
{
    m_snap = snap;
    // Reference narrowband combine at the snapshot time (dt = 0): the coherent
    // sum of the raw tap amplitudes. The fading deviation is measured relative
    // to this so the time-0 charge is exactly 0 dB.
    std::complex<double> h{0.0, 0.0};
    for (const auto& t : m_snap.taps)
    {
        h += t.amplitude;
    }
    m_refGain = std::max(std::norm(h), 1e-12);
}

double
SionnaCirPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                             Ptr<MobilityModel> /*a*/,
                                             Ptr<MobilityModel> /*b*/) const
{
    if (m_snap.taps.empty())
    {
        m_lastFadingDb = 0.0;
        return txPowerDbm;
    }

    const double dt = Simulator::Now().GetSeconds() - m_snap.t_ref_s;
    // Evolve every tap by its Doppler phase (Sionna paths.apply_doppler()).
    const CirEvolution ev =
        CirDopplerSynthesizer::Synthesize(m_snap, m_txVel, m_rxVel, dt);

    // Narrowband coherent combine of the FULL multipath CIR at time t.
    std::complex<double> h{0.0, 0.0};
    for (const auto& a : ev.amplitudes)
    {
        h += a;
    }
    const double gain = std::max(std::norm(h), 1e-12);

    // Charge the fading deviation about the snapshot reference (chained on the
    // base Friis path loss; +ve dB = a fade, -ve = constructive peak).
    m_lastFadingDb = -10.0 * std::log10(gain / m_refGain);
    return txPowerDbm - m_lastFadingDb;
}

int64_t
SionnaCirPropagationLossModel::DoAssignStreams(int64_t /*stream*/)
{
    return 0;
}

} // namespace ns3
