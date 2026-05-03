/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Copyright (c) 2026 Muhammad Uzair
// SPDX-License-Identifier: GPL-2.0-only

#ifndef NTN_NETSIMULYZER_EXPORTER_H
#define NTN_NETSIMULYZER_EXPORTER_H

#include <ns3/object.h>
#include <ns3/vector.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace ns3
{
namespace ntnobs
{

/**
 * \ingroup ntn-observability
 *
 * Streams a NIST NetSimulyzer-compatible JSON trace.
 *
 * NetSimulyzer (https://github.com/usnistgov/NetSimulyzer) is a 3D viewer
 * that consumes a JSON trace produced by ns-3 modules. We produce the same
 * file format here without depending on NetSimulyzer's source — researchers
 * who clone NetSimulyzer can open the resulting trace directly.
 *
 * Schema covered (subset of NetSimulyzer's 1.0 schema sufficient for NTN):
 *   - configuration (one document-level object)
 *   - nodes[]              — id, model, default position
 *   - events[]             — node-move, log-message, decoration
 *   - series[]             — value-vs-time scalar streams (XY plot)
 *
 * Output is incremental: opening Start() appends `{` and an opening events
 * array; every Move/Log/Sample writes one JSON object; Stop() closes the
 * file. The NetSimulyzer JSON reader accepts this streaming layout.
 */
class NtnNetSimulyzerExporter : public Object
{
  public:
    static TypeId GetTypeId();

    NtnNetSimulyzerExporter();
    ~NtnNetSimulyzerExporter() override;

    void SetOutputPath(const std::string& path);
    void SetUseSimulationTime(bool yes);

    /// Register a node (must be done before Start()).
    void AddNode(uint32_t id,
                 const std::string& model,
                 const Vector& initialPosition,
                 double scale = 1.0);

    /// Register a scalar value series (e.g. RSRP over time for one UE).
    /// Returns a series index used in subsequent SampleSeries() calls.
    uint32_t AddSeries(const std::string& name,
                       const std::string& xLabel,
                       const std::string& yLabel,
                       const std::string& unit = "");

    void Start();

    /// Update node position. Must be called between Start() and Stop().
    void NodeMove(uint32_t nodeId, double timeSec, const Vector& position);

    /// Log a textual event attached to a node (will appear on the timeline).
    void LogMessage(uint32_t nodeId, double timeSec, const std::string& message);

    /// Append (timeSec, value) to a series.
    void SampleSeries(uint32_t seriesIdx, double timeSec, double value);

    void Stop();

    uint64_t GetEventCount() const;

  protected:
    void DoDispose() override;

  private:
    struct NodeDecl
    {
        uint32_t id;
        std::string model;
        Vector initialPosition;
        double scale;
    };
    struct SeriesDecl
    {
        uint32_t idx;
        std::string name;
        std::string xLabel;
        std::string yLabel;
        std::string unit;
    };

    void WriteHeader();
    void WriteFooter();
    void WriteEvent(const std::string& json);

    std::string m_path;
    std::ofstream m_out;
    bool m_useSimulationTime{true};
    bool m_running{false};
    bool m_firstEvent{true};
    std::vector<NodeDecl> m_nodes;
    std::vector<SeriesDecl> m_series;
    uint64_t m_eventCount{0};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_NETSIMULYZER_EXPORTER_H
