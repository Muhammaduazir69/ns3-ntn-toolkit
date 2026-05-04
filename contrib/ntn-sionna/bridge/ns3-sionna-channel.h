/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W9)
 *
 * ns3-sionna-channel — UDP client to a NVIDIA Sionna RT path-loss server.
 *
 * Wire protocol matches contrib/ntn-sionna/bridge/sionna-server.py:
 *   request : {"tx":[x,y,z],"rx":[x,y,z],"freq_hz":<f>,"id":<n>}
 *   response: {"id":<n>,"path_loss_db":<d>,"n_paths":<k>,"compute_ms":<m>}
 *
 * On UDP timeout we fall back to free-space path loss
 *   PL_dB = 20*log10(d_m) + 20*log10(f_GHz) + 32.45
 * — the same closed form Sionna RT converges to in an empty scene, so the
 * fall-back keeps a simulation moving forward without distorting calibrated
 * comparisons. Per-call timeout drives the gate at <50 ms RTT (see W9
 * validation in contrib/ntn-sionna/README.md).
 */
#ifndef NTN_SIONNA_CHANNEL_H
#define NTN_SIONNA_CHANNEL_H

#include "ns3/propagation-loss-model.h"

#include <atomic>
#include <cstdint>
#include <string>

namespace ns3
{

class NtnSionnaChannel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();

    NtnSionnaChannel();
    ~NtnSionnaChannel() override;

    void SetServer(const std::string& host, uint16_t port);
    void SetFrequencyHz(double freqHz);
    void SetTimeoutMs(uint32_t timeoutMs);

    /// Counts since last reset; used by tests / examples to assert gate health.
    uint64_t GetQueriesSent() const { return m_queriesSent.load(); }
    uint64_t GetTimeouts() const { return m_timeouts.load(); }
    uint64_t GetFallbacks() const { return m_fallbacks.load(); }
    double GetLastRttMs() const { return m_lastRttMs.load(); }

    /// Path loss for the matched-scenario FSPL fall-back. Public so the
    /// example can reproduce the reference curve without a server running.
    static double FreeSpacePathLossDb(double distM, double freqHz);

  protected:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    int64_t DoAssignStreams(int64_t stream) override;

  private:
    /// Returns +inf on failure; caller is expected to fall back.
    double QuerySionnaDb(double txX, double txY, double txZ,
                         double rxX, double rxY, double rxZ) const;

    bool EnsureSocket() const;

    std::string m_host;
    uint16_t m_port;
    double m_freqHz;
    uint32_t m_timeoutMs;

    // Mutable members are touched from the const DoCalcRxPower override.
    mutable int m_sock;
    mutable std::atomic<uint64_t> m_seq;
    mutable std::atomic<uint64_t> m_queriesSent;
    mutable std::atomic<uint64_t> m_timeouts;
    mutable std::atomic<uint64_t> m_fallbacks;
    mutable std::atomic<double> m_lastRttMs;
};

} // namespace ns3

#endif // NTN_SIONNA_CHANNEL_H
