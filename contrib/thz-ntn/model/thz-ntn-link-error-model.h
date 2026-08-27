/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only
//
// thz-ntn-link-error-model - carry packets at true THz, above the 3GPP cap.
//
// Why this exists (audit THZ-07). Every example in this module that carries
// packets forces freqGHz = 100.0, and not by preference: the in-tree 3GPP
// spectrum model asserts 500 MHz <= f <= 100 GHz
// (three-gpp-propagation-loss-model.cc:358, and the same bound in
// three-gpp-channel-model.cc), so the NR/mmwave bridge cannot be driven above
// it. The seven examples that DO run at 140-300 GHz contain no
// InternetStackHelper, NetDeviceContainer, PointToPointHelper, FlowMonitor,
// OnOffHelper or PacketSink at all - they compute link budgets into CSV. So the
// module's headline 200-400 GHz band had no example in which a packet was ever
// transmitted, and every measured-plane THz result in the repo was a W-band
// result.
//
// This closes that by not using the 3GPP model. ThzNtnPropagationLossModel is
// already a real ns3::PropagationLossModel with no frequency cap, carrying this
// module's own gaseous, weather, scintillation and pointing terms; chaining it
// behind FriisPropagationLossModel gives a complete THz path loss. Verified at
// 300 GHz over a 550 km slant: 175.8 dB from a 30 dBm transmitter, with no
// assertion tripped.
//
// This error model puts that chain in the PACKET path: for every packet it asks
// the chain what the received power is for the CURRENT geometry, forms an SNR
// against a thermal-noise floor, and corrupts below a decode threshold. It is
// not a SpectrumPhy - there is no per-subcarrier processing, no MAC and no HARQ
// - and it does not claim to be. What it is, is a data plane at 300 GHz whose
// delivery depends on real THz physics rather than on a hardcoded 15 ms wire.

#ifndef THZ_NTN_LINK_ERROR_MODEL_H
#define THZ_NTN_LINK_ERROR_MODEL_H

#include "ns3/error-model.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{
namespace thzntn
{

/**
 * \ingroup thz-ntn
 * \brief Per-packet corruption from a live THz link budget.
 */
class ThzNtnLinkErrorModel : public ErrorModel
{
  public:
    static TypeId GetTypeId();
    ThzNtnLinkErrorModel();

    /// The two link endpoints whose live positions set the geometry.
    void SetEndpoints(Ptr<MobilityModel> tx, Ptr<MobilityModel> rx);
    /// The propagation chain. Chain Friis -> ThzNtnPropagationLossModel for a
    /// complete THz path; anything without a frequency cap works.
    void SetPropagationChain(Ptr<PropagationLossModel> chain);

    void SetTxPowerDbm(double dbm) { m_txPowerDbm = dbm; }
    void SetAntennaGainsDb(double txDbi, double rxDbi);
    void SetBandwidthHz(double hz) { m_bandwidthHz = hz; }
    void SetNoiseFigureDb(double db) { m_noiseFigureDb = db; }
    /// Minimum SNR for a decode (dB).
    void SetDecodeSnrDb(double db) { m_decodeSnrDb = db; }

    /// Received power (dBm) for the current geometry, gains included.
    double CurrentRxPowerDbm() const;
    /// Thermal noise plus noise figure over the bandwidth (dBm).
    double NoiseFloorDbm() const;
    /// SNR (dB) for the current geometry.
    double CurrentSnrDb() const;
    /// Whether the link currently closes.
    bool LinkCloses() const { return CurrentSnrDb() >= m_decodeSnrDb; }

    uint64_t GetPacketsChecked() const { return m_checked; }
    uint64_t GetPacketsCorrupted() const { return m_corrupted; }
    double GetLastSnrDb() const { return m_lastSnrDb; }

  private:
    bool DoCorrupt(Ptr<Packet> p) override;
    void DoReset() override;

    Ptr<MobilityModel> m_tx;
    Ptr<MobilityModel> m_rx;
    Ptr<PropagationLossModel> m_chain;
    double m_txPowerDbm{30.0};
    double m_txGainDbi{40.0};
    double m_rxGainDbi{45.0};
    double m_bandwidthHz{1.0e9};
    double m_noiseFigureDb{8.0};
    double m_decodeSnrDb{0.0};

    mutable double m_lastSnrDb{0.0};
    uint64_t m_checked{0};
    uint64_t m_corrupted{0};
};

} // namespace thzntn
} // namespace ns3

#endif // THZ_NTN_LINK_ERROR_MODEL_H
