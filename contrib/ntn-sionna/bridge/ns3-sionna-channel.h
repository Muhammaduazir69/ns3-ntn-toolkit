/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9 / Roadmap §4.2.1)
 *
 * ns3-sionna-channel — propagation-loss model that delegates queries to a
 * SionnaTransport (UDP / pybind11 / none). Falls back to free-space path
 * loss when the transport returns +inf so a simulation always makes
 * progress, with `m_fallbacks` recording how often that happens.
 *
 *   FSPL_dB = 20*log10(d_m) + 20*log10(f_GHz) + 32.45
 *
 * is the same closed form Sionna RT converges to in an empty scene, so
 * the fall-back keeps a simulation moving forward without distorting
 * calibrated comparisons. Tests use the FSPL identity to validate the
 * transport stack end-to-end.
 */
#ifndef NTN_SIONNA_CHANNEL_H
#define NTN_SIONNA_CHANNEL_H

#include "sionna-transport.h"

#include "ns3/propagation-loss-model.h"

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>

namespace ns3
{

class SionnaUdpTransport;

class NtnSionnaChannel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();

    NtnSionnaChannel();
    ~NtnSionnaChannel() override;

    /// Install a concrete transport. Pass nullptr to fall back to the
    /// always-fail SionnaNoneTransport (every query becomes FSPL).
    void SetTransport(Ptr<SionnaTransport> transport);
    Ptr<SionnaTransport> GetTransport() const { return m_transport; }

    /// Convenience wrappers for the UDP transport — preserved so existing
    /// example code still compiles. They route to the underlying UDP
    /// transport object; if the current transport is not UDP the call
    /// installs a fresh SionnaUdpTransport.
    void SetServer(const std::string& host, uint16_t port);
    void SetFrequencyHz(double freqHz);
    void SetTimeoutMs(uint32_t timeoutMs);

    /// Default MIMO array descriptors (Roadmap §4.2.2). When set, every
    /// outgoing query includes the descriptor; clear to revert to SISO.
    void SetTxArray(const MimoArrayConfig& arr);
    void SetRxArray(const MimoArrayConfig& arr);
    void ClearArrays();

    /// Default RIS descriptor (Roadmap §4.2.3). When set, every outgoing
    /// query includes the RIS so the server adds an `rt.RIS` to the scene
    /// before running the path solver. Clear to drop the surface.
    void SetRis(const RisConfig& ris);
    void ClearRis();

    /// Counters used by tests/examples. Mirror the transport's view, plus
    /// the channel-local fallback counter.
    uint64_t GetQueriesSent() const;
    uint64_t GetTimeouts() const;
    uint64_t GetFallbacks() const { return m_fallbacks.load(); }
    double GetLastRttMs() const;

    /// Path loss for the matched-scenario FSPL fall-back. Public so the
    /// tests / examples can reproduce the reference curve without a
    /// server running.
    static double FreeSpacePathLossDb(double distM, double freqHz);

  protected:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    int64_t DoAssignStreams(int64_t stream) override;

  private:
    Ptr<SionnaUdpTransport> EnsureUdpTransport();

    double m_freqHz;
    Ptr<SionnaTransport> m_transport;
    mutable std::atomic<uint64_t> m_seq;
    mutable std::atomic<uint64_t> m_fallbacks;

    std::optional<MimoArrayConfig> m_defaultTxArray;
    std::optional<MimoArrayConfig> m_defaultRxArray;
    std::optional<RisConfig> m_defaultRis;
};

} // namespace ns3

#endif // NTN_SIONNA_CHANNEL_H
