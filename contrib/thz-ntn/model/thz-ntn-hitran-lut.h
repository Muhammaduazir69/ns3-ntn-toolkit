/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef THZ_NTN_HITRAN_LUT_H
#define THZ_NTN_HITRAN_LUT_H

// HITRAN-2024 specific attenuation LUT (Roadmap §4.3.1).
//
// Look-up table of specific attenuation in dB/km, gridded on (frequency,
// altitude). The toolkit ships a small pre-computed CSV under
//   contrib/thz-ntn/data/hitran2024-lut-subthz.csv
// covering the canonical sub-THz windows used by NTN candidates
// (220/280/340/410 GHz, plus shoulders) for altitudes 0..30 km.
//
// The companion tool
//   contrib/thz-ntn/tools/hitran2024-lut-gen.py
// regenerates the LUT from the toolkit's Van Vleck–Weisskopf model, or
// (when a HITRAN-2024 `.par` file is supplied) ingests the line-by-line
// release directly. Reviewers can re-run the tool and diff the LUT.
//
// On-disk format (CSV):
//   header: "freq_ghz,alt_km,attenuation_db_per_km"
//   body:   one row per (f, h) pair
//
// LoadCsv() returns false on I/O / parse error so call-sites can fall
// back to the in-process Van Vleck–Weisskopf path.

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{
namespace thzntn
{

/// HITRAN release this build tracks. Reviewers pin this in citations + the
/// reproducibility manifest.
inline constexpr const char* kHitranRelease = "HITRAN-2024";

/// Canonical sub-THz windows the v2.1 LUT covers, in GHz. Each window's
/// neighbourhood ±20 GHz is sampled in the bundled LUT so the bilinear
/// interpolator stays accurate across the trough.
inline constexpr double kSubThzWindowsGHz[] = {220.0, 280.0, 340.0, 410.0};

class HitranLut
{
  public:
    HitranLut() = default;

    /// Load a HITRAN-2024 CSV LUT from disk.
    /// Returns false on I/O / parse error; on failure IsLoaded() == false
    /// and `Get` returns NaN so call-sites can route to the Van Vleck
    /// fallback.
    bool LoadCsv(const std::string& path);

    /// True iff the LUT has been successfully loaded.
    bool IsLoaded() const { return !m_freqGhz.empty() && !m_altKm.empty(); }

    /// Bilinear-interpolate specific attenuation in dB/km at the given
    /// frequency (Hz) and altitude (km). Out-of-range coordinates are
    /// clamped to the LUT edges; if the LUT is not loaded, returns NaN.
    double Get(double freqHz, double altKm) const;

    /// LUT grid metadata (used by tests + manifest emitters).
    const std::vector<double>& FrequencyGridGhz() const { return m_freqGhz; }
    const std::vector<double>& AltitudeGridKm() const { return m_altKm; }
    size_t Size() const { return m_attDbKm.size(); }

    /// Release tag this LUT was generated against ("HITRAN-2024" or the
    /// header line of a third-party LUT). Empty when not yet loaded.
    const std::string& ReleaseTag() const { return m_releaseTag; }

  private:
    /// Find the lower-bound index of `x` in a sorted vector. Returns
    /// (size - 1) when x is past the end.
    size_t LowerIndex(const std::vector<double>& v, double x) const;

    std::string m_releaseTag;
    std::vector<double> m_freqGhz;        //!< ascending grid
    std::vector<double> m_altKm;          //!< ascending grid
    /// Row-major flatten: m_attDbKm[freqIdx * altSize + altIdx].
    std::vector<double> m_attDbKm;
};

} // namespace thzntn
} // namespace ns3

#endif // THZ_NTN_HITRAN_LUT_H
