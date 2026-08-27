/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9 / Roadmap §4.2.1)
 */
#include "ns3-sionna-channel.h"

#include <sstream>

#include "sionna-udp-transport.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NtnSionnaChannel");
NS_OBJECT_ENSURE_REGISTERED(NtnSionnaChannel);

TypeId
NtnSionnaChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NtnSionnaChannel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("NtnSionna")
            .AddConstructor<NtnSionnaChannel>()
            .AddAttribute("FrequencyHz",
                          "Carrier frequency the server should ray-trace at",
                          DoubleValue(2.0e9),
                          MakeDoubleAccessor(&NtnSionnaChannel::m_freqHz),
                          MakeDoubleChecker<double>(1e6, 1e12));
    return tid;
}

NtnSionnaChannel::NtnSionnaChannel()
    : m_freqHz(2.0e9),
      m_seq(0),
      m_fallbacks(0)
{
}

NtnSionnaChannel::~NtnSionnaChannel() = default;

void
NtnSionnaChannel::SetTransport(Ptr<SionnaTransport> transport)
{
    m_transport = transport;
}

Ptr<SionnaUdpTransport>
NtnSionnaChannel::EnsureUdpTransport()
{
    Ptr<SionnaUdpTransport> udp =
        DynamicCast<SionnaUdpTransport>(m_transport);
    if (udp == nullptr)
    {
        udp = CreateObject<SionnaUdpTransport>();
        m_transport = udp;
    }
    return udp;
}

void
NtnSionnaChannel::SetServer(const std::string& host, uint16_t port)
{
    Ptr<SionnaUdpTransport> udp = EnsureUdpTransport();
    udp->SetServer(host, port);
}

void
NtnSionnaChannel::SetFrequencyHz(double freqHz)
{
    m_freqHz = freqHz;
}

void
NtnSionnaChannel::SetTimeoutMs(uint32_t timeoutMs)
{
    Ptr<SionnaUdpTransport> udp = EnsureUdpTransport();
    udp->SetTimeoutMs(timeoutMs);
}

void
NtnSionnaChannel::SetTxArray(const MimoArrayConfig& arr)
{
    m_defaultTxArray = arr;
}

void
NtnSionnaChannel::SetRxArray(const MimoArrayConfig& arr)
{
    m_defaultRxArray = arr;
}

void
NtnSionnaChannel::ClearArrays()
{
    m_defaultTxArray.reset();
    m_defaultRxArray.reset();
}

void
NtnSionnaChannel::SetRis(const RisConfig& ris)
{
    m_defaultRis = ris;
}

void
NtnSionnaChannel::ClearRis()
{
    m_defaultRis.reset();
}

uint64_t
NtnSionnaChannel::GetQueriesSent() const
{
    return m_transport == nullptr ? 0 : m_transport->GetQueriesSent();
}

std::string
NtnSionnaChannel::ProvenanceLine() const
{
    // WF-12: say plainly what produced these numbers.
    // WF-12: report EVALUATIONS, not transport sends. GetQueriesSent() is zero
    // when there is no transport at all, so differencing it against the
    // fallbacks would report "queries=0, fallback=5", which reads as though
    // nothing happened when in fact five path losses were answered by free
    // space.
    const uint64_t traced = m_rayTraced.load();
    const uint64_t fb = m_fallbacks.load();
    const uint64_t q = traced + fb;
    std::ostringstream os;
    os << "[sionna/provenance] evaluations=" << q << " ray-traced=" << traced
       << " free-space-fallback=" << fb;
    if (q == 0)
    {
        os << "  -> NO path-loss query was made; nothing here is ray traced";
    }
    else if (fb == 0)
    {
        os << "  -> all ray traced";
    }
    else if (fb == q)
    {
        os << "  -> EVERY query fell back: these results are closed-form free space, "
              "not ray tracing. Start the Sionna server, or set RequireLiveTransport "
              "to make this abort instead of substituting.";
    }
    else
    {
        os << "  -> MIXED: " << (100.0 * static_cast<double>(fb) / static_cast<double>(q))
           << "% of queries are closed-form free space, not ray traced";
    }
    return os.str();
}

uint64_t
NtnSionnaChannel::GetTimeouts() const
{
    return m_transport == nullptr ? 0 : m_transport->GetTimeouts();
}

double
NtnSionnaChannel::GetLastRttMs() const
{
    return m_transport == nullptr ? 0.0 : m_transport->GetLastRttMs();
}

double
NtnSionnaChannel::FreeSpacePathLossDb(double distM, double freqHz)
{
    double d = std::max(distM, 1e-3);
    double fGhz = freqHz / 1e9;
    return 20.0 * std::log10(d) + 20.0 * std::log10(fGhz) + 32.45;
}

double
NtnSionnaChannel::DoCalcRxPower(double txPowerDbm,
                                 Ptr<MobilityModel> a,
                                 Ptr<MobilityModel> b) const
{
    Vector pa = a->GetPosition();
    Vector pb = b->GetPosition();

    SionnaTransport::Request req{pa.x, pa.y, pa.z, pb.x, pb.y, pb.z,
                                  m_freqHz, ++m_seq,
                                  m_losOnly,
                                  m_defaultTxArray,
                                  m_defaultRxArray,
                                  m_defaultRis};
    double pl = std::numeric_limits<double>::infinity();
    if (m_transport != nullptr)
    {
        SionnaTransport::Response rsp = m_transport->Query(req);
        if (rsp.ok && std::isfinite(rsp.path_loss_db))
        {
            pl = rsp.path_loss_db;
            ++m_rayTraced; // WF-12: an evaluation the tracer actually answered
        }
    }
    if (!std::isfinite(pl))
    {
        // SIONNA-03: the fallback is real and sometimes reasonable, but it must
        // never be invisible. Before this it incremented a counter nobody read.
        ++m_fallbacks;
        NS_ABORT_MSG_IF(m_requireLiveTransport,
                        "NtnSionnaChannel: RequireLiveTransport is set but the ray-traced "
                        "query did not return a finite path loss (absent transport, socket "
                        "failure, timeout, malformed reply, or a missing Sionna import). "
                        "Refusing to substitute closed-form free-space path loss for a "
                        "result the scenario presents as ray traced.");
        if (!m_warnedFallback)
        {
            m_warnedFallback = true;
            NS_LOG_WARN("NtnSionnaChannel: falling back to closed-form free-space path loss; "
                        "this run's channel is NOT ray traced. Set RequireLiveTransport to "
                        "make this fatal, and read GetFallbacks() for the count.");
        }
        double d = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) +
                             (pa.y - pb.y) * (pa.y - pb.y) +
                             (pa.z - pb.z) * (pa.z - pb.z));
        pl = FreeSpacePathLossDb(d, m_freqHz);
    }
    return txPowerDbm - pl;
}

int64_t
NtnSionnaChannel::DoAssignStreams(int64_t /*stream*/)
{
    // No internal RNG — Sionna RT is the only randomness source and it
    // lives in the transport process.
    return 0;
}

} // namespace ns3
