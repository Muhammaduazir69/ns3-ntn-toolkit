/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Roadmap §4.2.7)
 *
 * ntn-sionna-cascade-channel — composes an `NtnSionnaChannel` (geometric
 * multipath from Sionna RT, with FSPL fall-back) with an
 * `NtnAtmosphericLossChain` (P.676 gaseous + P.618/P.838 rain + P.681 LMS).
 *
 *   Rx_dBm = NtnSionnaChannel::DoCalcRxPower(Tx_dBm, a, b)
 *          - Chain::ComputeAttenuationDb(a, b)
 *          - Chain::StepLmsDb()
 *
 * Both sub-models can be reconfigured at runtime via their attribute
 * surfaces; the cascade itself only mediates the call and accumulates the
 * resulting attenuation for observability.
 */
#ifndef NTN_SIONNA_CASCADE_CHANNEL_H
#define NTN_SIONNA_CASCADE_CHANNEL_H

#include "ns3-sionna-channel.h"
#include "ntn-atmospheric-loss-chain.h"

#include "ns3/propagation-loss-model.h"

#include <atomic>

namespace ns3
{

class SionnaTransport;

/**
 * \ingroup ntn-sionna
 *
 * \brief Sionna RT base channel + ITU-R atmospheric cascade.
 *
 * Drops in as a `PropagationLossModel` replacement for `NtnSionnaChannel`
 * in any spectrum/PHY stack. The cascade always advances the LMS Markov
 * chain (if enabled) once per Rx-power query so subsequent queries see a
 * temporally correlated shadowing trace.
 */
class NtnSionnaCascadeChannel : public PropagationLossModel
{
  public:
    static TypeId GetTypeId();

    NtnSionnaCascadeChannel();
    ~NtnSionnaCascadeChannel() override;

    /// Use this base channel for the geometric (multipath) prediction.
    /// If null at construction time, a fresh `NtnSionnaChannel` is created.
    void SetSionnaChannel(Ptr<NtnSionnaChannel> base);
    Ptr<NtnSionnaChannel> GetSionnaChannel() const { return m_base; }

    /// Use this loss chain. If null at construction time, a fresh
    /// `NtnAtmosphericLossChain` with default-configured P.676 + disabled
    /// rain/LMS is created.
    void SetAtmosphericChain(Ptr<NtnAtmosphericLossChain> chain);
    Ptr<NtnAtmosphericLossChain> GetAtmosphericChain() const { return m_chain; }

    /// Convenience: install the transport on the inner base channel.
    void SetTransport(Ptr<SionnaTransport> transport);
    Ptr<SionnaTransport> GetTransport() const;

    /// Convenience: set carrier frequency on BOTH the base Sionna channel
    /// (drives the ray-traced query) and the loss chain (drives P.676 +
    /// P.838 coefficients). Keeping these in lockstep is a common slip,
    /// hence the single entry point.
    void SetFrequencyHz(double freqHz);
    double GetFrequencyHz() const;

    /// Counters for tests/observability.
    uint64_t GetCascadeQueries() const { return m_cascadeQueries.load(); }
    double GetLastTotalAttenuationDb() const { return m_lastTotalDb; }
    NtnAtmosphericLossChain::Components GetLastComponents() const;

  protected:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;
    int64_t DoAssignStreams(int64_t stream) override;

  private:
    void EnsureSubModels();

    Ptr<NtnSionnaChannel> m_base;
    Ptr<NtnAtmosphericLossChain> m_chain;
    mutable std::atomic<uint64_t> m_cascadeQueries;
    mutable double m_lastTotalDb;
};

} // namespace ns3

#endif // NTN_SIONNA_CASCADE_CHANNEL_H
