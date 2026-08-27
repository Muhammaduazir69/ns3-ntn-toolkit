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

    /**
     * \brief WF-12: a one-line provenance summary for a scenario to print.
     *
     * `GetFallbacks()` existed and NONE of the twelve Sionna examples read it,
     * printed it or gated on it. On a host without Sionna, or with the UDP
     * server not started, every one of those examples produced pure free-space
     * results while still printing Sionna framing, and nothing in the output
     * said so. The counter is only useful if a run reports it.
     *
     * Returns a line naming how many path-loss queries were ray traced and how
     * many fell back to closed-form free space.
     */
    std::string ProvenanceLine() const;
    /// True when every query was answered by the ray tracer.
    bool AllQueriesRayTraced() const
    {
        return m_fallbacks.load() == 0 && m_rayTraced.load() > 0;
    }
    /// Path-loss evaluations answered by the ray tracer (WF-12).
    uint64_t GetRayTraced() const { return m_rayTraced.load(); }

    /// SIONNA-01: ask the server to trace beyond the direct path.
    ///
    /// Defaults to true (LOS only), which is what the bridge has always
    /// actually done, because the transport never sent the key and the server
    /// defaulted it. Set false to enable reflection, diffraction and
    /// scattering. Named on the channel so a scenario claiming a ray-traced
    /// result has to say so explicitly.
    void SetLosOnly(bool losOnly) { m_losOnly = losOnly; }
    bool GetLosOnly() const { return m_losOnly; }

    /// SIONNA-03: abort rather than silently substituting free-space path loss
    /// when the transport is absent or fails.
    ///
    /// The channel falls back to a closed-form FSPL on a missing transport, a
    /// socket failure, a receive timeout, a malformed reply or a missing Sionna
    /// import, and used to do so with no log, no trace and no way for a
    /// scenario to discover that its "ray-traced" channel was free space for
    /// the whole run. Set true where the ray-traced result is load-bearing.
    void SetRequireLiveTransport(bool require) { m_requireLiveTransport = require; }
    bool GetRequireLiveTransport() const { return m_requireLiveTransport; }
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
    /// WF-12: path-loss evaluations answered by the ray tracer.
    ///
    /// GetQueriesSent() counts transport SENDS, which is zero when there is no
    /// transport at all, so it cannot be differenced against the fallback count
    /// to say how many evaluations were traced. This counts the answers.
    mutable std::atomic<uint64_t> m_rayTraced{0};
    bool m_losOnly{true};                ///< SIONNA-01: wire value for los_only
    bool m_requireLiveTransport{false};  ///< SIONNA-03: abort instead of falling back
    mutable bool m_warnedFallback{false}; ///< SIONNA-03: warn once, not per query

    std::optional<MimoArrayConfig> m_defaultTxArray;
    std::optional<MimoArrayConfig> m_defaultRxArray;
    std::optional<RisConfig> m_defaultRis;
};

} // namespace ns3

#endif // NTN_SIONNA_CHANNEL_H
