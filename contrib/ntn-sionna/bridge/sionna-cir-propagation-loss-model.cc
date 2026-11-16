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
#include <utility>

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
    // SIONNA-07: normalize to the MEAN power of the CIR, sum_p |a_p|^2, so the
    // charged term is unit-mean-power small-scale fading on top of the Friis
    // head (the same normalization the TR 38.811 Section 6.7.2 Rician term
    // uses). The reference used to be the coherent sum |sum_p a_p|^2 at t=0,
    // which for an in-phase profile is a constructive PEAK: every later sample
    // was then charged against that peak, a positive mean bias of
    // 10 log10(|sum a|^2 / sum |a|^2), 3.5 dB for the shipped 4-tap profile,
    // added to every packet as if it were path loss.
    double p = 0.0;
    for (const auto& t : m_snap.taps)
    {
        p += std::norm(t.amplitude);
    }
    m_refGain = std::max(p, 1e-12);
}

double
SionnaCirPropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                             Ptr<MobilityModel> a,
                                             Ptr<MobilityModel> b) const
{
    if (m_snap.taps.empty())
    {
        m_lastFadingDb = 0.0;
        return txPowerDbm;
    }

    // SIONNA-06. Both mobility models used to be commented out of the signature
    // and the tap Doppler was computed from m_txVel / m_rxVel, velocities set
    // once through SetTxVelocity / SetRxVelocity and never updated. On a LEO
    // link that is the wrong quantity twice over: the endpoints are moving at
    // 7.5 km/s and their relative velocity turns over completely during a pass,
    // and a scenario that never called the setters got a Doppler of exactly
    // zero on a satellite link while the model reported it as evolving the CIR.
    //
    // Prefer the live mobility this method is handed. Fall back to the stored
    // velocities so a caller driving the model without mobility (a replay
    // harness, or a unit test feeding a fixed geometry) behaves as before.
    Vector txVel = m_txVel;
    Vector rxVel = m_rxVel;
    if (a)
    {
        txVel = a->GetVelocity();
    }
    if (b)
    {
        rxVel = b->GetVelocity();
    }
    // SIONNA-07: the snapshot describes the satellite -> ground direction
    // (tx_dir leaves the satellite, rx_dir is the angle of arrival at the
    // ground end). The same model sits on the uplink chain too, where the
    // spectrum channel calls it with a = UE and b = satellite; projecting the
    // UE velocity on the satellite departure directions there gave the uplink a
    // different, unphysical fading process. The channel is reciprocal, so put
    // each endpoint's velocity back on its own side of the snapshot. Ground is
    // the lower-z end in a local ENU frame and the smaller-norm end in ECEF.
    if (a && b)
    {
        const Vector pa = a->GetPosition();
        const Vector pb = b->GetPosition();
        const double na = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
        const double nb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
        const bool ecef = std::max(na, nb) > 0.9 * 6371000.0;
        const bool aIsGround = ecef ? (na < nb) : (pa.z < pb.z);
        if (aIsGround)
        {
            std::swap(txVel, rxVel);
        }
    }

    const double dt = Simulator::Now().GetSeconds() - m_snap.t_ref_s;
    // Evolve every tap by its Doppler phase (Sionna paths.apply_doppler()).
    const CirEvolution ev = CirDopplerSynthesizer::Synthesize(m_snap, txVel, rxVel, dt);

    // Narrowband coherent combine of the FULL multipath CIR at time t.
    std::complex<double> h{0.0, 0.0};
    for (const auto& a : ev.amplitudes)
    {
        h += a;
    }
    const double gain = std::max(std::norm(h), 1e-12);

    // Charge the fading deviation about the CIR mean power (chained on the
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
