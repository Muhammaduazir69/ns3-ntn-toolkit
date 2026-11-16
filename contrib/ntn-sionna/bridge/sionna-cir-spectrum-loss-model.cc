/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-cir-spectrum-loss-model.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaCirSpectrumLossModel");
NS_OBJECT_ENSURE_REGISTERED(SionnaCirSpectrumLossModel);

TypeId
SionnaCirSpectrumLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaCirSpectrumLossModel")
                            .SetParent<SpectrumPropagationLossModel>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaCirSpectrumLossModel>();
    return tid;
}

SionnaCirSpectrumLossModel::SionnaCirSpectrumLossModel() = default;
SionnaCirSpectrumLossModel::~SionnaCirSpectrumLossModel() = default;

void
SionnaCirSpectrumLossModel::SetSnapshot(const CirSnapshot& snap)
{
    m_snap = snap;
    double g = 0.0;
    for (const auto& t : m_snap.taps)
    {
        g += std::norm(t.amplitude);
    }
    m_refGain = std::max(g, 1e-30);
}

double
SionnaCirSpectrumLossModel::GetLastMeanGainDb() const
{
    if (m_lastGainDb.empty())
    {
        return 0.0;
    }
    return std::accumulate(m_lastGainDb.begin(), m_lastGainDb.end(), 0.0) / m_lastGainDb.size();
}

double
SionnaCirSpectrumLossModel::GetLastSpreadDb() const
{
    if (m_lastGainDb.empty())
    {
        return 0.0;
    }
    return *std::max_element(m_lastGainDb.begin(), m_lastGainDb.end()) -
           *std::min_element(m_lastGainDb.begin(), m_lastGainDb.end());
}

std::vector<double>
SionnaCirSpectrumLossModel::GainsAt(const std::vector<std::complex<double>>& amps,
                                    const std::vector<double>& dfHz) const
{
    std::vector<double> g(dfHz.size(), 1.0);
    const size_t np = std::min(amps.size(), m_snap.taps.size());
    for (size_t k = 0; k < dfHz.size(); ++k)
    {
        std::complex<double> h{0.0, 0.0};
        for (size_t p = 0; p < np; ++p)
        {
            const double ph = -2.0 * M_PI * dfHz[k] * m_snap.taps[p].delay_s;
            h += amps[p] * std::complex<double>(std::cos(ph), std::sin(ph));
        }
        g[k] = std::max(std::norm(h) / m_refGain, 1e-12);
    }
    return g;
}

Ptr<SpectrumValue>
SionnaCirSpectrumLossModel::DoCalcRxPowerSpectralDensity(Ptr<const SpectrumSignalParameters> params,
                                                         Ptr<const MobilityModel> a,
                                                         Ptr<const MobilityModel> b) const
{
    Ptr<SpectrumValue> psd = params->psd->Copy();
    if (m_snap.taps.empty())
    {
        return psd;
    }
    Vector txVel(0, 0, 0);
    Vector rxVel(0, 0, 0);
    if (a && b)
    {
        txVel = a->GetVelocity();
        rxVel = b->GetVelocity();
        // Keep the snapshot's satellite (tx) and ground (rx) sides on the
        // uplink too (reciprocity), as SionnaCirPropagationLossModel does.
        const Vector pa = a->GetPosition();
        const Vector pb = b->GetPosition();
        const double na = std::sqrt(pa.x * pa.x + pa.y * pa.y + pa.z * pa.z);
        const double nb = std::sqrt(pb.x * pb.x + pb.y * pb.y + pb.z * pb.z);
        const bool ecef = std::max(na, nb) > 0.9 * 6371000.0;
        if (ecef ? (na < nb) : (pa.z < pb.z))
        {
            std::swap(txVel, rxVel);
        }
    }
    const double dt = Simulator::Now().GetSeconds() - m_snap.t_ref_s;
    const CirEvolution ev = CirDopplerSynthesizer::Synthesize(m_snap, txVel, rxVel, dt);
    const double fc = 299792458.0 / std::max(m_snap.wavelength_m, 1e-12);
    std::vector<double> df;
    df.reserve(psd->GetValuesN());
    for (auto it = psd->ConstBandsBegin(); it != psd->ConstBandsEnd(); ++it)
    {
        df.push_back(it->fc - fc);
    }
    const auto g = GainsAt(ev.amplitudes, df);
    m_lastGainDb.resize(g.size());
    size_t i = 0;
    for (auto v = psd->ValuesBegin(); v != psd->ValuesEnd() && i < g.size(); ++v, ++i)
    {
        *v *= g[i];
        m_lastGainDb[i] = 10.0 * std::log10(g[i]);
    }
    return psd;
}

} // namespace ns3
