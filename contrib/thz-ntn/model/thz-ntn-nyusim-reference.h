/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

// ntn-ntn-nyusim-reference — loader for the NYUSIM-140 close-in (CI) path-loss
// reference dataset (Roadmap §4.3.4). Reads `data/nyusim-140-reference.csv`
// shipped with the toolkit and exposes the loaded entries to calibration
// code in C++ tests/examples. CSV rows after `#` comments take the form
//
//   scenario,freq_ghz,dist_m,pl_db,std_db,environment
//
// with `scenario ∈ {los, nlos}` and `environment ∈ {urban, suburban, indoor}`.
//
// Reference: Rappaport et al., IEEE Access 7 (2019), and Sun et al.,
// IEEE Comm. Mag. 2017 (NYUSIM). The dataset uses the deterministic CI
// predictor (chi_sigma = 0) so calibration consumers can compare model
// predictions directly without integrating out the log-normal shadowing.

#ifndef THZ_NTN_NYUSIM_REFERENCE_H
#define THZ_NTN_NYUSIM_REFERENCE_H

#include <ns3/object.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup thz-ntn
 *
 * \brief Reader + container for the NYUSIM-140 reference dataset.
 *
 * Construct, point at a CSV path (or accept the default location under the
 * shipped `data/` directory), call Load(). The entries are then iterable
 * via `GetEntries()`; selection helpers narrow by environment + scenario.
 */
class ThzNtnNyusimReference : public Object
{
  public:
    /// Single row from the reference CSV.
    struct Entry
    {
        bool los;                  //!< true for LOS scenario, false for NLOS
        double freq_ghz;            //!< carrier frequency in GHz
        double dist_m;              //!< 3-D Tx–Rx distance in meters
        double pl_db;               //!< mean path loss in dB (chi_sigma = 0)
        double std_db;              //!< NYUSIM-reported shadow std deviation (dB)
        std::string environment;    //!< "urban", "suburban", "indoor"
    };

    static TypeId GetTypeId();

    ThzNtnNyusimReference();
    ~ThzNtnNyusimReference() override;

    /// Override the CSV path. Default: the toolkit's bundled file under
    /// `contrib/thz-ntn/data/nyusim-140-reference.csv` resolved against
    /// `NS_TEST_SOURCEDIR` (when running tests) or the contrib path.
    void SetCsvPath(const std::string& path) { m_csvPath = path; }
    const std::string& GetCsvPath() const { return m_csvPath; }

    /// Read the CSV. Returns the number of entries loaded. Returns 0 on
    /// I/O failure; check `LastError()` for a description.
    std::size_t Load();

    /// All loaded entries in insertion order.
    const std::vector<Entry>& GetEntries() const { return m_entries; }

    /// Filter: matching environment + scenario.
    std::vector<Entry> Select(const std::string& environment,
                              bool los) const;

    /// Reverse-lookup the deterministic CI-model value at the given
    /// (env, los, dist) using the same (n, FSPL_1m) coefficients NYUSIM
    /// uses. Caller can compare against a tested model's prediction.
    static double ReferenceCiPathLossDb(const std::string& environment,
                                         bool los,
                                         double dist_m,
                                         double freq_hz);

    /// Last error string (empty on success).
    const std::string& LastError() const { return m_lastError; }

  private:
    bool ParseLine(const std::string& line, Entry& out, std::string& err);

    std::string m_csvPath;
    std::vector<Entry> m_entries;
    std::string m_lastError;
};

} // namespace ns3

#endif // THZ_NTN_NYUSIM_REFERENCE_H
