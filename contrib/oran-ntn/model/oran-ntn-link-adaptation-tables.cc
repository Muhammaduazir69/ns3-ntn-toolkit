/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2026  Muhammad Uzair
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "oran-ntn-link-adaptation-tables.h"

#include <algorithm>
#include <cmath>

namespace ns3
{
namespace oran
{

namespace
{

/// 3GPP TS 38.214 Table 5.2.2.1-3: CQI table 2 (up to 256QAM).
/// Index 0 is "out of range" and has no efficiency.
const double kCqiEfficiency[16] = {
    0.0,    // 0: out of range
    0.1523, // 1: QPSK,   R = 78/1024
    0.3770, // 2: QPSK,   R = 193/1024
    0.8770, // 3: QPSK,   R = 449/1024
    1.4766, // 4: 16QAM,  R = 378/1024
    1.9141, // 5: 16QAM,  R = 490/1024
    2.4063, // 6: 16QAM,  R = 616/1024
    2.7305, // 7: 64QAM,  R = 466/1024
    3.3223, // 8: 64QAM,  R = 567/1024
    3.9023, // 9: 64QAM,  R = 666/1024
    4.5234, // 10: 64QAM, R = 772/1024
    5.1152, // 11: 64QAM, R = 873/1024
    5.5547, // 12: 256QAM, R = 711/1024
    6.2266, // 13: 256QAM, R = 797/1024
    6.9141, // 14: 256QAM, R = 885/1024
    7.4063, // 15: 256QAM, R = 948/1024
};

/// 3GPP TS 38.214 Table 5.1.3.1-2: MCS index table 2 for PDSCH (256QAM).
/// Qm is the modulation order; efficiency is Qm * R.
const uint8_t kMcsQm[28] = {
    2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 4, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 8, 8, 8, 8, 8, 8, 8, 8,
};
const double kMcsEfficiency[28] = {
    0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953,
    1.9141, 2.1602, 2.4063, 2.5703, 2.7305, 3.0293, 3.3223,
    3.6094, 3.9023, 4.2129, 4.5234, 4.8164, 5.1152, 5.3320,
    5.5547, 5.8906, 6.2266, 6.5703, 6.9141, 7.1602, 7.4063,
};

/// ETSI EN 302 307-1 Table 13: DVB-S2 MODCOD, ideal Es/N0 for quasi-error-free
/// operation at FECFRAME length 64800 on AWGN, with the spectral efficiency the
/// same table gives. Ordered by increasing efficiency, which is NOT the same as
/// increasing required Es/N0 (8PSK 3/5 needs more Es/N0 than QPSK 9/10 while
/// carrying slightly less), so selection must scan rather than binary-search.
const DvbS2ModCod kDvbS2[] = {
    {4,  "QPSK 1/4",    -2.35, 0.490243},
    {5,  "QPSK 1/3",    -1.24, 0.656448},
    {6,  "QPSK 2/5",    -0.30, 0.789412},
    {7,  "QPSK 1/2",     1.00, 0.988858},
    {8,  "QPSK 3/5",     2.23, 1.188304},
    {9,  "QPSK 2/3",     3.10, 1.322253},
    {10, "QPSK 3/4",     4.03, 1.487473},
    {11, "QPSK 4/5",     4.68, 1.587196},
    {12, "QPSK 5/6",     5.18, 1.654663},
    {13, "QPSK 8/9",     6.20, 1.766451},
    {14, "QPSK 9/10",    6.42, 1.788612},
    {15, "8PSK 3/5",     5.50, 1.779991},
    {16, "8PSK 2/3",     6.62, 1.980636},
    {17, "8PSK 3/4",     7.91, 2.228124},
    {18, "8PSK 5/6",     9.35, 2.478562},
    {19, "8PSK 8/9",    10.69, 2.646012},
    {20, "8PSK 9/10",   10.98, 2.679207},
    {21, "16APSK 2/3",   8.97, 2.637201},
    {22, "16APSK 3/4",  10.21, 2.966728},
    {23, "16APSK 4/5",  11.03, 3.165623},
    {24, "16APSK 5/6",  11.61, 3.300184},
    {25, "16APSK 8/9",  12.89, 3.523143},
    {26, "16APSK 9/10", 13.13, 3.567342},
    {27, "32APSK 3/4",  12.73, 3.703295},
    {28, "32APSK 4/5",  13.64, 3.951571},
    {29, "32APSK 5/6",  14.28, 4.119540},
    {30, "32APSK 8/9",  15.69, 4.397854},
    {31, "32APSK 9/10", 16.05, 4.453027},
};
const uint32_t kDvbS2Count = sizeof(kDvbS2) / sizeof(kDvbS2[0]);

} // namespace

double
OranNtnLinkAdaptationTables::SpectralEfficiencyFromSinrDb(double sinrDb)
{
    if (!std::isfinite(sinrDb))
    {
        return 0.0;
    }
    const double lin = std::pow(10.0, sinrDb / 10.0);
    const double shannon = std::log2(1.0 + lin);
    // NR cannot exceed its own highest MCS however good the link is.
    return std::min(shannon, kMaxNrSpectralEfficiency);
}

uint8_t
OranNtnLinkAdaptationTables::CqiFromSpectralEfficiency(double spectralEff)
{
    if (!std::isfinite(spectralEff) || spectralEff < kCqiEfficiency[1])
    {
        return 0; // the table's own "out of range"
    }
    uint8_t best = 1;
    for (uint8_t i = 1; i <= 15; ++i)
    {
        if (kCqiEfficiency[i] <= spectralEff)
        {
            best = i;
        }
    }
    return best;
}

double
OranNtnLinkAdaptationTables::SpectralEfficiencyOfCqi(uint8_t cqi)
{
    return (cqi <= 15) ? kCqiEfficiency[cqi] : 0.0;
}

uint8_t
OranNtnLinkAdaptationTables::McsFromSpectralEfficiency(double spectralEff)
{
    if (!std::isfinite(spectralEff) || spectralEff <= kMcsEfficiency[0])
    {
        return 0; // MCS 0 is a real scheme; there is no out-of-range MCS
    }
    uint8_t best = 0;
    for (uint8_t i = 0; i < 28; ++i)
    {
        if (kMcsEfficiency[i] <= spectralEff)
        {
            best = i;
        }
    }
    return best;
}

double
OranNtnLinkAdaptationTables::SpectralEfficiencyOfMcs(uint8_t mcs)
{
    return (mcs < 28) ? kMcsEfficiency[mcs] : 0.0;
}

uint8_t
OranNtnLinkAdaptationTables::ModulationOrderOfMcs(uint8_t mcs)
{
    return (mcs < 28) ? kMcsQm[mcs] : 0;
}

DvbS2ModCod
OranNtnLinkAdaptationTables::ModCodFromEsN0Db(double esN0Db, double marginDb, bool& closes)
{
    const double usable = esN0Db - marginDb;
    closes = false;
    DvbS2ModCod best = kDvbS2[0];
    for (uint32_t i = 0; i < kDvbS2Count; ++i)
    {
        if (kDvbS2[i].reqEsN0Db <= usable)
        {
            // Most EFFICIENT among those that close, not merely the last one
            // that fits: the table is ordered by efficiency but its required
            // Es/N0 is not monotone, so "last that fits" would pick wrong.
            if (!closes || kDvbS2[i].spectralEff > best.spectralEff)
            {
                best = kDvbS2[i];
                closes = true;
            }
        }
    }
    if (!closes)
    {
        best = kDvbS2[0]; // report the weakest, and say it does not close
    }
    return best;
}

uint32_t
OranNtnLinkAdaptationTables::GetDvbS2ModCodCount()
{
    return kDvbS2Count;
}

DvbS2ModCod
OranNtnLinkAdaptationTables::GetDvbS2ModCod(uint32_t i)
{
    return (i < kDvbS2Count) ? kDvbS2[i] : kDvbS2[0];
}

} // namespace oran
} // namespace ns3
