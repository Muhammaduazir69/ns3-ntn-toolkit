/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 */
#ifndef NTN_SLICE_SELECTOR_H
#define NTN_SLICE_SELECTOR_H

#include "ntn-slice-types.h"

#include "ns3/object.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnslice
{

/// Lightweight 5-tuple-ish flow descriptor for selector input.
struct FlowDescriptor
{
    uint16_t srcPort{0};
    uint16_t dstPort{0};
    uint8_t dscp{0};       ///< IP DSCP code-point
    std::string appLabel{}; ///< optional application identifier
};

/**
 * @brief Maps incoming flows to S-NSSAIs by simple prioritised rules.
 *
 * Rules are evaluated in insertion order — first match wins. The matcher
 * supports three rule types so realistic deployments (which typically rely
 * on DSCP marking, well-known port ranges, or explicit app tags) can all be
 * expressed without registry-style boilerplate.
 *
 * If no rule matches, @c Match returns the @c default Snssai (set via
 * @c SetDefault), so callers always get a valid slice — falling back to
 * eMBB by convention.
 */
class NtnSliceSelector : public Object
{
  public:
    static TypeId GetTypeId();
    NtnSliceSelector();

    enum class RuleType
    {
        ByDscp,
        ByDstPortRange,
        ByAppLabel,
    };

    struct Rule
    {
        RuleType type;
        uint8_t dscp{};
        uint16_t portLo{};
        uint16_t portHi{};
        std::string appLabel{};
        Snssai snssai{};
    };

    void SetDefault(const Snssai& s);
    Snssai GetDefault() const;

    void AddRule(const Rule& rule);
    std::size_t RuleCount() const;

    /// Convenience helpers.
    void AddDscpRule(uint8_t dscp, const Snssai& s);
    void AddPortRangeRule(uint16_t lo, uint16_t hi, const Snssai& s);
    void AddAppLabelRule(const std::string& label, const Snssai& s);

    Snssai Match(const FlowDescriptor& f) const;

  private:
    Snssai m_default{SliceSst::Embb, kSdUnset};
    std::vector<Rule> m_rules;
};

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_SELECTOR_H
