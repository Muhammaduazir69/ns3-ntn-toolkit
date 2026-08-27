/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "thz-ntn-link-error-model.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <cmath>

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

ThzNtnLinkErrorModel::ThzNtnLinkErrorModel() = default;

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
}

double
ThzNtnLinkErrorModel::NoiseFloorDbm() const
{
    return -174.0 + 10.0 * std::log10(std::max(1.0, m_bandwidthHz)) + m_noiseFigureDb;
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
    const double snr = CurrentSnrDb();
    const bool corrupt = (snr < m_decodeSnrDb);
    if (corrupt)
    {
        ++m_corrupted;
    }
    return corrupt;
}

void
ThzNtnLinkErrorModel::DoReset()
{
    m_checked = 0;
    m_corrupted = 0;
}

} // namespace thzntn
} // namespace ns3
