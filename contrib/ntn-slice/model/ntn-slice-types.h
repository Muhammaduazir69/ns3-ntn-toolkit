/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 *
 * Network-slice identifiers per 3GPP TS 23.501 §5.15.2 and Annex A:
 *
 *   S-NSSAI = (SST, [SD])
 *
 *   SST = Slice/Service Type (1 byte):
 *     1 → eMBB        (Enhanced Mobile Broadband)
 *     2 → URLLC       (Ultra-Reliable Low-Latency Communications)
 *     3 → mIoT/mMTC   (Massive IoT)
 *     4 → V2X         (reserved for W7)
 *
 *   SD  = Slice Differentiator (3 bytes), optional. 0xFFFFFF = unset.
 *
 * Default `SliceProfile`s for the three canonical slices follow TS 22.261
 * Table 7.1-1 (eMBB / URLLC / mMTC reference KPIs).
 */
#ifndef NTN_SLICE_TYPES_H
#define NTN_SLICE_TYPES_H

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>

namespace ns3
{
namespace ntnslice
{

/// SST values per 3GPP TS 23.501 Annex A.
enum class SliceSst : uint8_t
{
    Embb = 1,    ///< throughput-heavy, latency-tolerant
    Urllc = 2,   ///< latency-critical, may sacrifice throughput
    Mmtc = 3,    ///< many low-rate IoT devices
    V2x = 4,     ///< vehicular (W7)
    Unknown = 0,
};

/// 3-byte SD (Slice Differentiator). 0xFFFFFF = "unset".
constexpr uint32_t kSdUnset = 0xFFFFFFu;

/// S-NSSAI value-type: (SST, SD).
struct Snssai
{
    SliceSst sst{SliceSst::Unknown};
    uint32_t sd{kSdUnset};   ///< only the low 24 bits are valid

    constexpr bool operator==(const Snssai& other) const
    {
        return sst == other.sst && sd == other.sd;
    }
    constexpr bool operator<(const Snssai& other) const
    {
        if (sst != other.sst)
            return static_cast<uint8_t>(sst) < static_cast<uint8_t>(other.sst);
        return sd < other.sd;
    }
};

/// 32-bit packed encoding (SST<<24 | SD) used on the wire and as map key.
constexpr uint32_t
PackSnssai(const Snssai& s)
{
    return (static_cast<uint32_t>(s.sst) << 24) | (s.sd & 0xFFFFFFu);
}

constexpr Snssai
UnpackSnssai(uint32_t packed)
{
    return Snssai{static_cast<SliceSst>((packed >> 24) & 0xFF),
                  packed & 0xFFFFFFu};
}

inline std::ostream&
operator<<(std::ostream& os, const Snssai& s)
{
    os << "(sst=" << static_cast<int>(s.sst) << ",sd=0x" << std::hex << s.sd << std::dec << ")";
    return os;
}

/// Human-readable SST name. Matches the snake_case convention used in InfluxDB tags.
inline std::string_view
SstName(SliceSst sst)
{
    switch (sst)
    {
    case SliceSst::Embb: return "embb";
    case SliceSst::Urllc: return "urllc";
    case SliceSst::Mmtc: return "mmtc";
    case SliceSst::V2x: return "v2x";
    case SliceSst::Unknown: return "unknown";
    }
    return "unknown";
}

/**
 * @brief Service-class profile attached to a slice.
 *
 * Defaults derived from 3GPP TS 22.261 Table 7.1-1 (the IMT-2020 KPI baseline)
 * and TR 38.913 §6.1. These are starting points — applications can override
 * any field via `SliceProfile{...}`.
 */
struct SliceProfile
{
    Snssai snssai{};
    /// E2E one-way latency budget (ms). Below → packet violates SLA.
    double latencyBudgetMs{50.0};
    /// Min reserved throughput in Mbps. The orchestrator must guarantee this
    /// even under congestion (the "isolation" promise).
    double minThroughputMbps{0.0};
    /// Max throughput cap in Mbps. 0 = uncapped.
    double maxThroughputMbps{0.0};
    /// Reliability (1 − loss probability) the slice expects.
    double reliability{0.99};
    /// Priority — higher values served first under contention.
    uint8_t priority{0};
    /// Allow GEO satellite as a possible serving cell (URLLC = false).
    bool allowGeo{true};
    std::string label{};
};

/// Default eMBB profile per TS 22.261 (mobile broadband).
inline SliceProfile
DefaultEmbb(uint32_t sd = kSdUnset)
{
    return SliceProfile{
        .snssai = {SliceSst::Embb, sd},
        .latencyBudgetMs = 50.0,
        .minThroughputMbps = 50.0,
        .maxThroughputMbps = 1000.0,
        .reliability = 0.99,
        .priority = 5,
        .allowGeo = true,
        .label = "eMBB",
    };
}

/// Default URLLC profile per TS 22.261 (1 ms latency, 1e-5 BLER).
/// `allowGeo = false` enforces "GEO mode-skip" — URLLC bypasses GEO satellites.
inline SliceProfile
DefaultUrllc(uint32_t sd = kSdUnset)
{
    return SliceProfile{
        .snssai = {SliceSst::Urllc, sd},
        .latencyBudgetMs = 5.0,
        .minThroughputMbps = 5.0,
        .maxThroughputMbps = 100.0,
        .reliability = 0.99999,
        .priority = 9,
        .allowGeo = false,
        .label = "URLLC",
    };
}

/// Default mMTC profile per TS 22.261 (many devices, low rate).
inline SliceProfile
DefaultMmtc(uint32_t sd = kSdUnset)
{
    return SliceProfile{
        .snssai = {SliceSst::Mmtc, sd},
        .latencyBudgetMs = 1000.0,
        .minThroughputMbps = 1.0,
        .maxThroughputMbps = 10.0,
        .reliability = 0.95,
        .priority = 1,
        .allowGeo = true,
        .label = "mMTC",
    };
}

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_TYPES_H
