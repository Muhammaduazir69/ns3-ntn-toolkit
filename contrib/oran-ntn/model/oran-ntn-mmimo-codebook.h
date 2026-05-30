/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef ORAN_NTN_MMIMO_CODEBOOK_H
#define ORAN_NTN_MMIMO_CODEBOOK_H

// On-RU analog-beamformer codebook for the 4.1.12 two-stage AI-RAN
// xApp.
//
// In the NU × NVIDIA × AmpliTech open-source mMIMO AI-RAN demo the
// O-RU side beamformer is a 64-element codebook (one codeword per
// RF chain × analog beam direction). The xApp picks one codeword
// per RF chain to form `W_RF` ∈ ℂ^(N_tx × N_RF). The downstream
// digital baseband precoder `W_BB` ∈ ℂ^(N_RF × N_layers) is the
// output of the AI-Aerial-style NN.
//
// Combined hybrid beamformer:
//
//     W = W_RF · W_BB   (shape N_tx × N_layers)
//
// This module stores N entries of length `num_tx_antennas` each, in
// flattened (re_0, im_0, re_1, im_1, …) form so we can hand them
// straight to the T7 wire format if needed.

#include <ns3/object.h>

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup oran-ntn
 * \brief O-RU codebook of N complex steering vectors (Roadmap §4.1.12).
 */
class OranNtnMmimoCodebook : public Object
{
  public:
    static TypeId GetTypeId();
    OranNtnMmimoCodebook();
    ~OranNtnMmimoCodebook() override;

    /// Manually add one entry. Length must equal `num_tx * 2`
    /// (interleaved re/im float pairs).
    void AddEntry(const std::vector<float>& re_im);

    /// Reserve and configure dimensions in advance (clears existing
    /// entries).
    void Configure(uint32_t num_tx, uint32_t num_entries_hint = 64);

    /// Build the default 64-element DFT codebook covering azimuth
    /// directions uniformly in [-π/3, +π/3]. Half-wavelength uniform
    /// linear array assumption; the resulting codebook size equals
    /// `num_entries`.
    void PopulateDftAzimuthSweep(uint32_t num_tx,
                                   uint32_t num_entries,
                                   double az_min_rad = -1.047197551,
                                   double az_max_rad = +1.047197551);

    uint32_t NumTx() const { return m_numTx; }
    uint32_t Size() const { return static_cast<uint32_t>(m_entries.size()); }
    const std::vector<float>& GetEntry(uint32_t idx) const
    {
        return m_entries.at(idx);
    }

    /// Return the index of the codeword that best matches the given
    /// target complex steering vector `target_re_im`. "Best" = argmax
    /// of the squared magnitude of the inner product (conjugate match).
    uint32_t BestMatch(const std::vector<float>& target_re_im) const;

    /// Score every codeword against `target_re_im` and return the
    /// scores in original codebook order. Useful for tests that want
    /// to verify ranking, not just argmax.
    std::vector<float>
        ScoreAll(const std::vector<float>& target_re_im) const;

  protected:
    void DoDispose() override;

  private:
    uint32_t m_numTx{0};
    std::vector<std::vector<float>> m_entries;
};

} // namespace ns3

#endif // ORAN_NTN_MMIMO_CODEBOOK_H
