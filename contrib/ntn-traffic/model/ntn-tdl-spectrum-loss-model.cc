/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-tdl-spectrum-loss-model.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnTdlSpectrumLossModel");
NS_OBJECT_ENSURE_REGISTERED(NtnTdlSpectrumLossModel);

TypeId
NtnTdlSpectrumLossModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NtnTdlSpectrumLossModel")
                            .SetParent<Object>()
                            .SetGroupName("NtnTraffic")
                            .AddConstructor<NtnTdlSpectrumLossModel>();
    return tid;
}

NtnTdlSpectrumLossModel::NtnTdlSpectrumLossModel()
{
    m_reRng = CreateObject<NormalRandomVariable>();
    m_reRng->SetAttribute("Mean", DoubleValue(0.0));
    m_reRng->SetAttribute("Variance", DoubleValue(0.5));
    m_imRng = CreateObject<NormalRandomVariable>();
    m_imRng->SetAttribute("Mean", DoubleValue(0.0));
    m_imRng->SetAttribute("Variance", DoubleValue(0.5));

    // Built-in STAND-IN profile: a LOS tap plus two scattered taps at 100 ns
    // and 400 ns, giving an RMS delay spread of order 100 ns and therefore a
    // coherence bandwidth of a few MHz - selective across a 20 MHz NTN channel,
    // which is the behaviour the toolkit had none of.
    //
    // This is NOT TR 38.811 Table 6.9.2. See the header: those values could not
    // be verified here, and attributing invented taps to a 3GPP table is the
    // defect this audit found elsewhere. Call SetTaps() with the real table.
    SetTaps({{0.0, 0.0, true}, {100e-9, -6.0, false}, {400e-9, -12.0, false}});
    m_provenance = "stand-in profile (NOT TR 38.811 Table 6.9.2); "
                   "call SetTaps() with the standard's tap table";

    // TR 38.811 section 6.7.2 S-band LOS Rician K mean (dB) against elevation.
    m_kTable = {{10.0, 4.1}, {20.0, 6.6}, {30.0, 8.7}, {40.0, 10.4},
                {50.0, 11.6}, {60.0, 12.6}, {70.0, 13.5}, {80.0, 14.2},
                {90.0, 14.6}};
}

void
NtnTdlSpectrumLossModel::SetTaps(std::vector<NtnTdlTap> taps)
{
    m_taps = std::move(taps);
    // Normalise to unit total power: a fading model must not add or remove
    // average energy from the link, only redistribute it across frequency.
    double total = 0.0;
    for (const auto& t : m_taps)
    {
        total += std::pow(10.0, t.powerDb / 10.0);
    }
    if (total > 0.0)
    {
        const double corrDb = -10.0 * std::log10(total);
        for (auto& t : m_taps)
        {
            t.powerDb += corrDb;
        }
    }
}

double
NtnTdlSpectrumLossModel::GetRmsDelaySpreadS() const
{
    double p = 0.0;
    double pt = 0.0;
    double pt2 = 0.0;
    for (const auto& t : m_taps)
    {
        const double lin = std::pow(10.0, t.powerDb / 10.0);
        p += lin;
        pt += lin * t.delayS;
        pt2 += lin * t.delayS * t.delayS;
    }
    if (p <= 0.0)
    {
        return 0.0;
    }
    const double mean = pt / p;
    return std::sqrt(std::max(0.0, pt2 / p - mean * mean));
}

void
NtnTdlSpectrumLossModel::SetRicianKTable(std::vector<std::pair<double, double>> t)
{
    m_kTable = std::move(t);
    std::sort(m_kTable.begin(), m_kTable.end());
}

double
NtnTdlSpectrumLossModel::RicianKdB(double elevDeg) const
{
    if (m_kTable.empty())
    {
        return 0.0;
    }
    if (elevDeg <= m_kTable.front().first)
    {
        return m_kTable.front().second;
    }
    if (elevDeg >= m_kTable.back().first)
    {
        return m_kTable.back().second;
    }
    for (std::size_t i = 1; i < m_kTable.size(); ++i)
    {
        if (elevDeg <= m_kTable[i].first)
        {
            const auto& lo = m_kTable[i - 1];
            const auto& hi = m_kTable[i];
            const double f = (elevDeg - lo.first) / (hi.first - lo.first);
            return lo.second + f * (hi.second - lo.second);
        }
    }
    return m_kTable.back().second;
}

std::vector<double>
NtnTdlSpectrumLossModel::ComputeGains(const std::vector<double>& freqsHz, double elevDeg) const
{
    std::vector<double> gains(freqsHz.size(), 1.0);
    if (m_taps.empty() || freqsHz.empty())
    {
        return gains;
    }

    // Draw one complex coefficient per tap, held across the band: the taps are
    // what vary in DELAY, and it is the delay that makes the transfer function
    // frequency-selective. Redrawing per subcarrier would give white noise
    // across frequency instead of a channel with a coherence bandwidth.
    const double kLin = std::pow(10.0, RicianKdB(elevDeg) / 10.0);
    std::vector<std::complex<double>> coeff(m_taps.size());
    for (std::size_t i = 0; i < m_taps.size(); ++i)
    {
        const double amp = std::pow(10.0, m_taps[i].powerDb / 20.0);
        std::complex<double> c(m_reRng->GetValue(), m_imRng->GetValue());
        if (m_taps[i].isLos)
        {
            // Rician: a specular part carrying K/(K+1) of the tap power and a
            // scattered part carrying 1/(K+1).
            const double specular = std::sqrt(kLin / (kLin + 1.0));
            const double scatter = std::sqrt(1.0 / (kLin + 1.0));
            c = std::complex<double>(specular, 0.0) + scatter * c;
        }
        coeff[i] = amp * c;
    }

    for (std::size_t f = 0; f < freqsHz.size(); ++f)
    {
        std::complex<double> h(0.0, 0.0);
        for (std::size_t i = 0; i < m_taps.size(); ++i)
        {
            const double phase = -2.0 * M_PI * freqsHz[f] * m_taps[i].delayS;
            h += coeff[i] * std::complex<double>(std::cos(phase), std::sin(phase));
        }
        gains[f] = std::norm(h);
    }
    return gains;
}

void
NtnTdlSpectrumLossModel::ApplyTo(Ptr<SpectrumValue> psd, const std::vector<double>& freqsHz,
                                 double elevDeg) const
{
    if (!psd)
    {
        return;
    }
    const auto gains = ComputeGains(freqsHz, elevDeg);
    std::size_t i = 0;
    for (auto it = psd->ValuesBegin(); it != psd->ValuesEnd() && i < gains.size(); ++it, ++i)
    {
        *it *= gains[i];
    }
}

} // namespace ns3
