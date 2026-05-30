/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_SIONNA_PYBIND_TRANSPORT_H
#define NTN_SIONNA_PYBIND_TRANSPORT_H

// In-process pybind11 transport stub (Roadmap §4.2.1).
//
// The full implementation embeds a CPython interpreter and calls into the
// Sionna RT Python API directly, removing the UDP round-trip cost for
// scenarios where ns-3 and Sionna run on the same host.
//
// The v2.1 baseline declares the class so call-sites can target it; the
// embedded-interpreter body is gated on ENABLE_SIONNA_PYBIND11 (a follow-up
// CMake flag). With the flag off, IsAvailable() returns false and Query()
// returns +inf so the caller falls back to FSPL or the UDP transport.
//
// Why this is a stub today: pybind11 embed pulls libpython3 into every
// ns-3 binary, which is a non-trivial dependency for users who only want
// the UDP path. Gating it lets the toolkit ship without the dependency
// and turn it on per-build.

#include "sionna-transport.h"

#include <string>

namespace ns3
{

class SionnaPybindTransport : public SionnaTransport
{
  public:
    static TypeId GetTypeId();
    SionnaPybindTransport();
    ~SionnaPybindTransport() override;

    /// Path to the Python module exposing `query_path_loss(tx, rx, freq)`.
    /// Honoured only when the toolkit is built with ENABLE_SIONNA_PYBIND11.
    void SetModule(const std::string& moduleName);
    /// Sionna scene XML path.
    void SetScene(const std::string& sceneXml);

    Response Query(const Request& req) const override;
    std::string Name() const override { return "pybind11"; }
    bool IsAvailable() const override;

  private:
    std::string m_moduleName{"sionna_bridge"};
    std::string m_sceneXml; //!< empty => use module default
    mutable bool m_initialised{false};
};

} // namespace ns3

#endif // NTN_SIONNA_PYBIND_TRANSPORT_H
