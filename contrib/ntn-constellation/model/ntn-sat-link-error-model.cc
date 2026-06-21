/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "ntn-sat-link-error-model.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <cmath>

namespace ns3
{
namespace ntncon
{

NS_LOG_COMPONENT_DEFINE("NtnSatLinkErrorModel");
NS_OBJECT_ENSURE_REGISTERED(NtnSatLinkErrorModel);

namespace
{
constexpr double kC = 299792458.0;
constexpr double kBoltzDbwHzK = -228.6; // 10*log10(k), k = 1.380649e-23 J/K
} // namespace

TypeId
NtnSatLinkErrorModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ntncon::NtnSatLinkErrorModel")
            .SetParent<ErrorModel>()
            .SetGroupName("NtnConstellation")
            .AddConstructor<NtnSatLinkErrorModel>()
            .AddAttribute("EirpDbw", "Transmit EIRP (dBW).", DoubleValue(20.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_eirpDbw),
                          MakeDoubleChecker<double>())
            .AddAttribute("RxGtDbK", "Receiver figure of merit G/T (dB/K).", DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_rxGtDbK),
                          MakeDoubleChecker<double>())
            .AddAttribute("BandwidthHz", "Noise bandwidth (Hz).", DoubleValue(30e6),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_bandwidthHz),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("CarrierHz", "Carrier frequency (Hz).", DoubleValue(20e9),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_carrierHz),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("ThresholdEsNoDb",
                          "DVB-S2 MODCOD Es/No quasi-error-free threshold (dB).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_thresholdEsNoDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("WaterfallDb", "BLER waterfall steepness (dB).", DoubleValue(1.0),
                          MakeDoubleAccessor(&NtnSatLinkErrorModel::m_waterfallDb),
                          MakeDoubleChecker<double>(0.05));
    return tid;
}

NtnSatLinkErrorModel::NtnSatLinkErrorModel()
{
    m_rng = CreateObject<UniformRandomVariable>();
}

void
NtnSatLinkErrorModel::SetEndpoints(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx)
{
    m_tx = tx;
    m_rx = rx;
}

double
NtnSatLinkErrorModel::SlantRangeM() const
{
    if (!m_tx || !m_rx)
    {
        return -1.0;
    }
    const Vector a = m_tx->GetPosition();
    const Vector b = m_rx->GetPosition();
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

double
NtnSatLinkErrorModel::EsNoDbFor(double slantM) const
{
    if (slantM <= 0.0)
    {
        return -1e9;
    }
    // Free-space path loss (dB).
    const double fspl = 20.0 * std::log10(4.0 * M_PI * slantM * m_carrierHz / kC);
    // Es/No = EIRP - FSPL + G/T - k(dB) - 10log10(B).
    return m_eirpDbw - fspl + m_rxGtDbK - kBoltzDbwHzK - 10.0 * std::log10(m_bandwidthHz);
}

double
NtnSatLinkErrorModel::BlerFor(double esNoDb) const
{
    // DVB-S2-style waterfall: BLER ~ 1 well below threshold, ~0 above, with an
    // erfc transition of width m_waterfallDb around the QEF threshold.
    const double x = (esNoDb - m_thresholdEsNoDb) / m_waterfallDb;
    return 0.5 * std::erfc(x);
}

double
NtnSatLinkErrorModel::CurrentSlantRangeM() const
{
    return SlantRangeM();
}

double
NtnSatLinkErrorModel::CurrentEsNoDb() const
{
    return EsNoDbFor(SlantRangeM());
}

double
NtnSatLinkErrorModel::CurrentBler() const
{
    return BlerFor(EsNoDbFor(SlantRangeM()));
}

bool
NtnSatLinkErrorModel::DoCorrupt(Ptr<Packet> p)
{
    if (!IsEnabled())
    {
        return false;
    }
    const double slant = SlantRangeM();
    if (slant < 0.0)
    {
        return false; // no geometry configured -> pass
    }
    m_lastEsNoDb = EsNoDbFor(slant);
    m_lastBler = BlerFor(m_lastEsNoDb);
    return m_rng->GetValue(0.0, 1.0) < m_lastBler;
}

void
NtnSatLinkErrorModel::DoReset()
{
    m_lastEsNoDb = 0.0;
    m_lastBler = 0.0;
}

} // namespace ntncon
} // namespace ns3
