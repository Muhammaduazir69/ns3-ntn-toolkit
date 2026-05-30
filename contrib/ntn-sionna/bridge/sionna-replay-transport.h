/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_REPLAY_TRANSPORT_H
#define NTN_SIONNA_REPLAY_TRANSPORT_H

// Precompute/replay backend (Roadmap §4.2.6).
//
// Lets users record an offline Sionna-RT pass once, then re-use the
// resulting path-loss table across many ns-3 simulations without ever
// launching a live Sionna server. Records are stored in a toolkit-
// native portable binary file:
//
//   Header   :  "NTNRPLAY\1"         (9 bytes, magic + version)
//                uint64 record_count
//                uint32 freq_hz_default
//   Record[] :  double t_s
//                double sat_pos[3]   (ECEF or local m, caller-defined)
//                double ue_pos[3]
//                double freq_hz
//                double path_loss_db
//                uint32 n_paths
//                double compute_ms
//
// `Writer` builds the file from a stream of `Record`s; `Reader` mmaps
// (or fully loads) it at construction. `SionnaReplayTransport` then
// implements `SionnaTransport::Query()` by picking the nearest record
// — closest in (Δt, Δr_tx, Δr_rx) using a simple lex-rank or a
// configurable weighted distance.
//
// HDF5 support is intentionally *not* a build dependency: users who
// want HDF5 / pyAerial-style tables can run the toolkit-side converter
// (`tools/ntn-replay-hdf5.py` — out of scope of this header).

#include "sionna-transport.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

struct ReplayRecord
{
    double t_s{0.0};
    double sat_pos[3]{0.0, 0.0, 0.0};
    double ue_pos[3]{0.0, 0.0, 0.0};
    double freq_hz{2.0e9};
    double path_loss_db{0.0};
    uint32_t n_paths{1};
    double compute_ms{0.0};
};

class SionnaReplayWriter
{
  public:
    /// Open `path` for writing. Returns false on filesystem error.
    bool Open(const std::string& path, uint64_t freq_hz_default);

    /// Append one record. Records may be appended in arbitrary order
    /// — the reader sorts at load time.
    bool Write(const ReplayRecord& r);

    /// Flush and close. Idempotent.
    bool Close();

    uint64_t RecordsWritten() const { return m_count; }

  private:
    std::string m_path;
    void* m_fp{nullptr};
    uint64_t m_count{0};
};

class SionnaReplayReader
{
  public:
    /// Load `path` fully into RAM. Returns false on filesystem or
    /// magic/version mismatch.
    bool Load(const std::string& path);

    size_t Size() const { return m_records.size(); }
    const ReplayRecord& At(size_t i) const { return m_records.at(i); }
    uint64_t DefaultFreqHz() const { return m_freqDefault; }

    /// Find the record whose (t, tx, rx) is closest to the query.
    /// Returns the index, or SIZE_MAX if empty / not loaded.
    size_t NearestIndex(double t_s,
                          const double sat_pos[3],
                          const double ue_pos[3]) const;

    /// Weighting used by NearestIndex(). Defaults: time 1.0, space 1.0
    /// per metre. Callers can re-balance if their scenario uses
    /// different units.
    void SetDistanceWeights(double w_time_s, double w_space_m);

  private:
    std::vector<ReplayRecord> m_records;
    uint64_t m_freqDefault{0};
    double m_wTime{1.0};
    double m_wSpace{1.0};
};

/**
 * \ingroup ntn-sionna
 * \brief Replay-only SionnaTransport (Roadmap §4.2.6).
 *
 * Looks up the nearest precomputed `ReplayRecord` for each Query.
 * Doppler / phase rotation between snapshots can be layered on top via
 * §4.2.11 `CirDopplerSynthesizer`.
 */
class SionnaReplayTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId();
    SionnaReplayTransport();
    ~SionnaReplayTransport() override = default;

    /// Load a precomputed file. Must be called before Query().
    bool LoadFile(const std::string& path);

    Response Query(const Request& req) const override;

    std::string Name() const override { return "sionna-replay"; }
    bool IsAvailable() const override { return m_reader.Size() > 0; }

    const SionnaReplayReader& Reader() const { return m_reader; }

    /// Optional override for the current "wall-clock" used as t_s in
    /// the lookup. By default queries use Simulator::Now().GetSeconds().
    void SetClockOverride(double t_s);
    void ClearClockOverride();

  private:
    SionnaReplayReader m_reader;
    bool m_useClockOverride{false};
    double m_clockOverride{0.0};
};

} // namespace ns3

#endif // NTN_SIONNA_REPLAY_TRANSPORT_H
