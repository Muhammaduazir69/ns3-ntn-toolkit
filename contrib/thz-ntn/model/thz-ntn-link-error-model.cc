/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-link-error-model.h"

#include "ns3/satellite-link-results.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{
namespace thzntn
{

NS_LOG_COMPONENT_DEFINE("ThzNtnLinkErrorModel");
NS_OBJECT_ENSURE_REGISTERED(ThzNtnLinkErrorModel);

TypeId
ThzNtnLinkErrorModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::thzntn::ThzNtnLinkErrorModel")
                            .SetParent<ErrorModel>()
                            .SetGroupName("ThzNtn")
                            .AddConstructor<ThzNtnLinkErrorModel>();
    return tid;
}

ThzNtnLinkErrorModel::ThzNtnLinkErrorModel()
    : m_antTempK(std::numeric_limits<double>::quiet_NaN())
{
    m_uv = CreateObject<UniformRandomVariable>();
}

int64_t
ThzNtnLinkErrorModel::AssignStreams(int64_t stream)
{
    m_uv->SetStream(stream);
    return 1;
}

namespace
{
Ptr<SatLinkResultsDvbS2>
DvbS2Table()
{
    static Ptr<SatLinkResultsDvbS2> lr;
    if (!lr)
    {
        lr = CreateObject<SatLinkResultsDvbS2>();
        lr->Initialize();
    }
    return lr;
}
} // namespace

double
ThzNtnLinkErrorModel::DvbS2Bler(SatEnums::SatModcod_t m, double esNoDb)
{
    const double b = DvbS2Table()->GetBler(m, SatEnums::NORMAL_FRAME, esNoDb);
    return std::min(1.0, std::max(0.0, b));
}

double
ThzNtnLinkErrorModel::DvbS2RequiredEsNoDb(SatEnums::SatModcod_t m, double blerTarget)
{
    return DvbS2Table()->GetEsNoDb(m, SatEnums::NORMAL_FRAME, blerTarget);
}

void
ThzNtnLinkErrorModel::SetEndpoints(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx)
{
    m_tx = tx;
    m_rx = rx;
}

void
ThzNtnLinkErrorModel::SetPropagationChain(Ptr<PropagationLossModel> chain)
{
    m_chain = chain;
}

void
ThzNtnLinkErrorModel::SetAntennaGainsDb(double txDbi, double rxDbi)
{
    m_txGainDbi = txDbi;
    m_rxGainDbi = rxDbi;
    m_cacheTime = Seconds(-1);
}

double
ThzNtnLinkErrorModel::NoiseFloorDbm() const
{
    if (std::isnan(m_antTempK))
    {
        return -174.0 + 10.0 * std::log10(std::max(1.0, m_bandwidthHz)) + m_noiseFigureDb;
    }
    const double t0 = 290.0;
    const double tSys = m_antTempK + t0 * (std::pow(10.0, m_noiseFigureDb / 10.0) - 1.0);
    const double kB = 1.380649e-23;
    return 10.0 * std::log10(kB * tSys * std::max(1.0, m_bandwidthHz)) + 30.0;
}

double
ThzNtnLinkErrorModel::CurrentRxPowerDbm() const
{
    if (!m_chain || !m_tx || !m_rx)
    {
        return -std::numeric_limits<double>::infinity();
    }
    // The chain takes the EIRP: transmit power plus the transmit array gain.
    // The receive gain is added afterwards, as it is not part of propagation.
    const double eirpDbm = m_txPowerDbm + m_txGainDbi;
    return m_chain->CalcRxPower(eirpDbm, m_tx, m_rx) + m_rxGainDbi;
}

double
ThzNtnLinkErrorModel::CurrentSnrDb() const
{
    const double rx = CurrentRxPowerDbm();
    if (!std::isfinite(rx))
    {
        return -999.0;
    }
    m_lastSnrDb = rx - NoiseFloorDbm();
    return m_lastSnrDb;
}

bool
ThzNtnLinkErrorModel::DoCorrupt(Ptr<Packet> p)
{
    if (!IsEnabled())
    {
        return false;
    }
    ++m_checked;
    // The geometry is re-evaluated per packet, so a moving satellite changes
    // delivery over the pass rather than the run being decided once at setup.
    const double snr = PacketSnrDb();
    bool corrupt;
    if (m_rule == DVBS2_WATERFALL)
    {
        const double bler = DvbS2Bler(m_modcod, snr);
        m_sumBler += bler;
        corrupt = m_uv->GetValue() < bler;
    }
    else
    {
        corrupt = (snr < m_decodeSnrDb);
    }
    if (corrupt)
    {
        ++m_corrupted;
    }
    return corrupt;
}

double
ThzNtnLinkErrorModel::PacketSnrDb()
{
    const Time now = Simulator::Now();
    if (m_cacheWindow.IsStrictlyPositive() && m_cacheTime >= Seconds(0) &&
        now - m_cacheTime < m_cacheWindow)
    {
        return m_lastSnrDb;
    }
    m_cacheTime = now;
    return CurrentSnrDb();
}

void
ThzNtnLinkErrorModel::DoReset()
{
    m_checked = 0;
    m_corrupted = 0;
    m_sumBler = 0.0;
}

} // namespace thzntn
} // namespace ns3
