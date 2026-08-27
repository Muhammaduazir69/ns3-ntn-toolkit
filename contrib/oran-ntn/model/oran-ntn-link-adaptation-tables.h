/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * ORAN-06: the real link-adaptation tables the PHY-KPM extractor claims to use.
 *
 * The extractor published a CQI that was always 0, an MCS that was always 0, an
 * unbounded Shannon spectral efficiency, and a "DVB-S2X ModCod index" computed
 * as min(27, SE * 5.4), which is a straight line with no table behind it. The
 * struct fields carried the standards names regardless, so a consumer reading
 * E2KpmReport::modCod got a number that looked like an ETSI index and was not.
 *
 * These are the published tables:
 *   - 3GPP TS 38.214 Table 5.2.2.1-3, CQI table 2 (up to 256QAM)
 *   - 3GPP TS 38.214 Table 5.1.3.1-2, MCS index table 2 for PDSCH (256QAM)
 *   - ETSI EN 302 307-1 Table 13, DVB-S2 MODCOD required Es/N0 on AWGN
 *
 * The 3GPP tables are selected by achievable spectral efficiency; the DVB-S2
 * table is selected by required Es/N0 against the measured SINR, which is how a
 * real ACM loop picks a ModCod.
 */
#ifndef ORAN_NTN_LINK_ADAPTATION_TABLES_H
#define ORAN_NTN_LINK_ADAPTATION_TABLES_H

#include <cstdint>

namespace ns3
{
namespace oran
{

/// One DVB-S2 MODCOD: its signalled index, required Es/N0 and spectral efficiency.
struct DvbS2ModCod
{
    uint8_t index;        //!< MODCOD value signalled in the PLS code
    const char* name;     //!< e.g. "8PSK 3/4"
    double reqEsN0Db;     //!< ideal Es/N0 for QEF at FECFRAME 64800, AWGN
    double spectralEff;   //!< bit/s per Hz
};

/**
 * \brief Link-adaptation lookups against the published tables.
 *
 * All static: these are tables, not state. Every entry is quoted from the
 * standard rather than fitted, so a wrong value is a transcription error that a
 * test can catch by checking a known row.
 */
class OranNtnLinkAdaptationTables
{
  public:
    /// Highest spectral efficiency 5G NR can carry: TS 38.214 Table 5.1.3.1-2
    /// MCS 27 (256QAM, R = 948/1024). Shannon is not bounded by this; NR is.
    static constexpr double kMaxNrSpectralEfficiency = 7.4063;

    /**
     * \brief Shannon capacity for an SINR, capped at what NR can actually carry.
     *
     * The extractor reported log2(1+SINR) uncapped, so a 40 dB link published a
     * spectral efficiency of 13.3 bit/s/Hz that no NR modulation and coding
     * scheme can deliver. The cap is the standard's own ceiling.
     */
    static double SpectralEfficiencyFromSinrDb(double sinrDb);

    /**
     * \brief CQI index for an achievable spectral efficiency.
     *
     * TS 38.214 Table 5.2.2.1-3 (CQI table 2, up to 256QAM). Returns the
     * highest index whose tabulated efficiency the link can support, or 0 for
     * "out of range" when it cannot support even CQI 1, which is the meaning
     * the table gives index 0.
     */
    static uint8_t CqiFromSpectralEfficiency(double spectralEff);

    /// Tabulated efficiency of a CQI index (0 for out-of-range).
    static double SpectralEfficiencyOfCqi(uint8_t cqi);

    /**
     * \brief PDSCH MCS index for an achievable spectral efficiency.
     *
     * TS 38.214 Table 5.1.3.1-2 (MCS index table 2, 256QAM). Returns the
     * highest MCS whose efficiency fits, clamped to 0 at the bottom: MCS 0 is a
     * real scheme, unlike CQI 0.
     */
    static uint8_t McsFromSpectralEfficiency(double spectralEff);

    /// Tabulated efficiency and modulation order of an MCS index.
    static double SpectralEfficiencyOfMcs(uint8_t mcs);
    static uint8_t ModulationOrderOfMcs(uint8_t mcs);

    /**
     * \brief DVB-S2 MODCOD for a measured Es/N0, with an optional ACM margin.
     *
     * ETSI EN 302 307-1 Table 13. Returns the most efficient MODCOD whose
     * required Es/N0 is at or below (esN0Db - marginDb). Below the weakest
     * MODCOD the link does not close and the lowest entry is returned with
     * `closes` false, because "QPSK 1/4 anyway" is not what an ACM loop does.
     */
    static DvbS2ModCod ModCodFromEsN0Db(double esN0Db, double marginDb, bool& closes);

    /// Table access for tests and for callers that want to walk it.
    static uint32_t GetDvbS2ModCodCount();
    static DvbS2ModCod GetDvbS2ModCod(uint32_t i);
};

} // namespace oran
} // namespace ns3

#endif // ORAN_NTN_LINK_ADAPTATION_TABLES_H
