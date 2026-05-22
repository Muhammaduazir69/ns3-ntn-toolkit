/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#include "sionna-pybind-transport.h"

#include "ns3/log.h"

#include <chrono>
#include <cmath>
#include <limits>

#ifdef ENABLE_SIONNA_PYBIND11
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
#endif

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SionnaPybindTransport");

TypeId
SionnaPybindTransport::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SionnaPybindTransport")
                            .SetParent<SionnaTransport>()
                            .SetGroupName("NtnSionna")
                            .AddConstructor<SionnaPybindTransport>();
    return tid;
}

SionnaPybindTransport::SionnaPybindTransport() = default;
SionnaPybindTransport::~SionnaPybindTransport() = default;

void
SionnaPybindTransport::SetModule(const std::string& moduleName)
{
    m_moduleName = moduleName;
    m_initialised = false;
}

void
SionnaPybindTransport::SetScene(const std::string& sceneXml)
{
    m_sceneXml = sceneXml;
    m_initialised = false;
}

bool
SionnaPybindTransport::IsAvailable() const
{
#ifdef ENABLE_SIONNA_PYBIND11
    // Try a one-shot import; sets the cached `m_initialised` flag.
    if (m_initialised)
    {
        return true;
    }
    try
    {
        // The interpreter is owned by whoever embedded it (toolkit-level
        // singleton in a follow-up); IsAvailable() is best-effort here.
        py::module::import(m_moduleName.c_str());
        m_initialised = true;
        return true;
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("SionnaPybindTransport: cannot import "
                    << m_moduleName << ": " << exc.what());
        return false;
    }
#else
    return false;
#endif
}

SionnaTransport::Response
SionnaPybindTransport::Query(const Request& req) const
{
    Response rsp{std::numeric_limits<double>::infinity(), 0, 0.0, false};

#ifdef ENABLE_SIONNA_PYBIND11
    if (!IsAvailable())
    {
        ++m_failures;
        return rsp;
    }
    auto t0 = std::chrono::steady_clock::now();
    try
    {
        py::module mod = py::module::import(m_moduleName.c_str());
        py::object fn = mod.attr("query_path_loss");
        py::list tx;
        tx.append(req.tx_x);
        tx.append(req.tx_y);
        tx.append(req.tx_z);
        py::list rx;
        rx.append(req.rx_x);
        rx.append(req.rx_y);
        rx.append(req.rx_z);
        py::object result = fn(tx, rx, req.freq_hz);
        rsp.path_loss_db = result.attr("__getitem__")(0).cast<double>();
        rsp.n_paths =
            static_cast<uint32_t>(result.attr("__getitem__")(1).cast<int>());
        ++m_queriesSent;
        rsp.ok = std::isfinite(rsp.path_loss_db);
    }
    catch (const std::exception& exc)
    {
        NS_LOG_WARN("SionnaPybindTransport: Query failed: " << exc.what());
        ++m_failures;
    }
    auto t1 = std::chrono::steady_clock::now();
    rsp.compute_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    m_lastRttMs.store(rsp.compute_ms);
#else
    (void)req;
    ++m_failures;
#endif
    return rsp;
}

} // namespace ns3
