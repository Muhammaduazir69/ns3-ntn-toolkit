/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2026 Muhammad Uzair (ns3-ntn-toolkit, Workstream W6)
 *
 * Slice isolation monitor.
 *
 * Streams per-packet latency observations into per-slice rolling windows,
 * computes the p99 tail latency and packet-drop rate, and raises a
 * `BreachEvent` whenever a slice exceeds its `latencyBudgetMs` budget or
 * fails its `reliability` reliability target.
 *
 * The point of the monitor is to detect *isolation* breaches — i.e. one
 * slice's congestion affecting another. A simple co-existence guarantee:
 *
 *   For every observation tick, each slice's tail latency must remain
 *   within its own profile budget regardless of any other slice's load.
 */
#ifndef NTN_SLICE_ISOLATION_MONITOR_H
#define NTN_SLICE_ISOLATION_MONITOR_H

#include "ntn-slice-types.h"

#include "ns3/object.h"
#include "ns3/traced-callback.h"

#include <deque>
#include <map>
#include <vector>

namespace ns3
{
namespace ntnslice
{

struct BreachEvent
{
    Snssai snssai;
    double observedLatencyP99Ms{0.0};
    double latencyBudgetMs{0.0};
    double observedLossRate{0.0};
    double reliabilityTarget{0.0};
    bool latencyBreach{false};
    bool reliabilityBreach{false};
    uint64_t windowSamples{0};
};

class SliceIsolationMonitor : public Object
{
  public:
    static TypeId GetTypeId();
    SliceIsolationMonitor();

    /// Register a slice with the monitor.
    void RegisterSlice(const SliceProfile& profile);

    /// Window size (number of samples per slice).
    void SetWindow(uint32_t window);

    /// Record one observation. `delivered = false` counts as loss.
    void RecordPacket(const Snssai& s, double latencyMs, bool delivered);

    /// Check the current window for each slice; emit a BreachEvent per slice.
    std::vector<BreachEvent> EvaluateAll() const;

    /// Convenience: count of slices currently breaching.
    std::size_t BreachCount() const;

    typedef TracedCallback<const BreachEvent&> BreachTrace;

  private:
    struct PerSlice
    {
        SliceProfile profile;
        std::deque<double> latencyMsWindow;
        std::deque<bool> deliveredWindow;
    };

    BreachEvent EvaluateSlice(const PerSlice& ps) const;

    std::map<Snssai, PerSlice> m_slices;
    uint32_t m_window{200};
    BreachTrace m_traceBreach;
};

} // namespace ntnslice
} // namespace ns3

#endif // NTN_SLICE_ISOLATION_MONITOR_H
