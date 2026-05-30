/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_TRANSPORT_H
#define NTN_SIONNA_TRANSPORT_H

// Abstract transport for the ntn-sionna bridge (Roadmap §4.2.1).
//
// SionnaTransport hides whether path-loss queries are sent to a separate
// process over UDP / ZMQ or executed in-process via pybind11. Call sites in
// NtnSionnaChannel never depend on the transport choice; the concrete
// implementation is selected at runtime via the `TransportKind` attribute.
//
// Concrete impls:
//   SionnaUdpTransport     UDP + JSON, matches sionna-server.py wire
//                          protocol — works even when Sionna RT is unavailable
//                          locally (server can run on a GPU box).
//   SionnaPybindTransport  in-process embed of CPython + Sionna RT (compiled
//                          only when ENABLE_SIONNA_PYBIND11 is set).
//   SionnaNoneTransport    sentinel — every query "fails", so the caller
//                          falls back to FSPL. Used in tests + when Sionna is
//                          not desired but the channel object must still
//                          register with the propagation stack.

#include <ns3/object.h>

#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace ns3
{

/// PlanarArray descriptor matching Sionna RT 2.0.1 `rt.PlanarArray` arguments
/// (Roadmap §4.2.2). When supplied on a Request the server reconfigures
/// `scene.tx_array` / `scene.rx_array` per query.
struct MimoArrayConfig
{
    uint8_t rows{1};
    uint8_t cols{1};
    /// Antenna spacing in wavelengths. 0.5 (default) matches the Sionna
    /// `vertical_spacing = horizontal_spacing = 0.5` convention.
    double spacing_lambda{0.5};
    /// Sionna RT 2.0.1 antenna pattern: "iso", "tr38901", "dipole".
    std::string pattern{"iso"};
    /// "V", "H", or "VH" (Sionna's cross-polarisation convention).
    std::string polarization{"V"};
};

/// RIS (Reconfigurable Intelligent Surface) descriptor matching Sionna RT 2.0
/// `rt.RIS` arguments (Roadmap §4.2.3). When supplied on a Request the server
/// instantiates one `rt.RIS` in the scene with the supplied geometry + phase
/// profile, then runs the path solver with `los_only=False` so reflections off
/// the surface contribute to the returned `path_loss_db`.
///
/// `phase_profile`:
///   "focus" — focuses on `focal_xyz` (steered constructive overlay)
///   "flat"  — zero-phase profile (mirror-like reflector)
///   "random" — uniformly randomised phases (worst case for coherent boost)
struct RisConfig
{
    /// Centre of the surface in metres (same frame as tx_x/y/z).
    double pos_x{0.0};
    double pos_y{0.0};
    double pos_z{0.0};
    /// Outward normal vector (unit length on the server side after parse).
    double normal_x{0.0};
    double normal_y{0.0};
    double normal_z{1.0};
    /// Number of reconfigurable elements along each axis of the surface.
    uint16_t rows{32};
    uint16_t cols{32};
    /// Element spacing in wavelengths along each axis.
    double spacing_lambda{0.5};
    /// Phase profile mode: "focus", "flat", "random".
    std::string phase_profile{"focus"};
    /// Optional focal point used when phase_profile == "focus".
    double focal_x{0.0};
    double focal_y{0.0};
    double focal_z{0.0};
};

/**
 * \ingroup ntn-sionna
 * \brief Abstract Sionna RT transport.
 *
 * Threading: `Query` is called from the ns-3 main thread synchronously.
 * Concrete impls may use blocking I/O internally, but must respect their
 * own timeout so the simulator can't stall indefinitely.
 */
class SionnaTransport : public Object
{
  public:
    struct Request
    {
        double tx_x;
        double tx_y;
        double tx_z;
        double rx_x;
        double rx_y;
        double rx_z;
        double freq_hz;
        uint64_t request_id;
        /// Optional MIMO array config (Roadmap §4.2.2). When present, the
        /// transport forwards the (rows, cols, spacing, pattern, pol) per
        /// side; when absent the server falls back to its SISO defaults.
        std::optional<MimoArrayConfig> tx_array;
        std::optional<MimoArrayConfig> rx_array;
        /// Optional RIS surface (Roadmap §4.2.3). When present, the server
        /// adds an `rt.RIS` to the scene at the supplied (pos, normal) with
        /// rows × cols reconfigurable elements and the given phase profile,
        /// then runs the exact-paths solver so reflections off the surface
        /// contribute to `path_loss_db`.
        std::optional<RisConfig> ris;
    };

    struct Response
    {
        double path_loss_db; //!< +inf on failure; caller falls back to FSPL
        uint32_t n_paths;
        double compute_ms;   //!< server-side compute, not RTT
        bool ok;
        /// Number of antenna ports reported by the server (1 when SISO).
        /// (Roadmap §4.2.2.)
        uint16_t tx_ports{1};
        uint16_t rx_ports{1};
    };

    static TypeId GetTypeId();
    ~SionnaTransport() override = default;

    /// Synchronous query. Concrete impls own their own timeout policy. On
    /// failure return `Response{path_loss_db = +inf, ok = false}`.
    virtual Response Query(const Request& req) const = 0;

    /// Short name for diagnostics + reproducibility manifest.
    virtual std::string Name() const = 0;

    /// Quick liveness probe — true when Query() is likely to succeed.
    /// Tests / examples use this to choose between live transport and FSPL
    /// fallback up front.
    virtual bool IsAvailable() const = 0;

    /// Per-instance counters. Used by tests + examples to assert health
    /// without coupling them to internal state.
    uint64_t GetQueriesSent() const { return m_queriesSent.load(); }
    uint64_t GetTimeouts() const { return m_timeouts.load(); }
    uint64_t GetFailures() const { return m_failures.load(); }
    double GetLastRttMs() const { return m_lastRttMs.load(); }

    /// Resets the counters. Useful between test phases.
    void ResetCounters()
    {
        m_queriesSent.store(0);
        m_timeouts.store(0);
        m_failures.store(0);
        m_lastRttMs.store(0.0);
    }

  protected:
    SionnaTransport() = default;

    mutable std::atomic<uint64_t> m_queriesSent{0};
    mutable std::atomic<uint64_t> m_timeouts{0};
    mutable std::atomic<uint64_t> m_failures{0};
    mutable std::atomic<double> m_lastRttMs{0.0};
};

/// Always-fails transport. Useful when the user wants the FSPL closed-form
/// behaviour with no server overhead, or when wiring up code paths in
/// tests that should never hit a real Sionna instance.
class SionnaNoneTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId();
    SionnaNoneTransport() = default;
    ~SionnaNoneTransport() override = default;

    Response Query(const Request& /*req*/) const override
    {
        ++m_queriesSent;
        ++m_failures;
        return {std::numeric_limits<double>::infinity(), 0, 0.0, false};
    }

    std::string Name() const override { return "none"; }
    bool IsAvailable() const override { return false; }
};

} // namespace ns3

#endif // NTN_SIONNA_TRANSPORT_H
