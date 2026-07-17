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
 * Streams the toolkit's OWN lightweight scene JSON (schema
 * `ntn-observability-native-1.0`) — NOT the official usnistgov NetSimulyzer
 * 1.0 schema. The NetSimulyzer desktop app cannot open this file: its reader
 * expects kebab-case `node-position` / `xy-series-append` events and a
 * different document layout. This exporter is a compact, self-describing trace
 * for the toolkit's own tooling (control-center, tests, quick inspection).
 *
 * For a schema-exact file that opens directly in the NetSimulyzer app, drive
 * the vendored contrib/netsimulyzer Orchestrator — see the
 * ntn-netsimulyzer-official-demo example (Path B).
 *
 * Schema emitted (native):
 *   - configuration (one document-level object, carries the schema tag)
 *   - nodes[]              — id, model, default position
 *   - series[]             — value-vs-time scalar streams (XY plot)
 *   - events[]             — NodeMove, LogMessage, SeriesSample
 *
 * Output is incremental: Start() opens `{` and the events array; every
 * Move/Log/Sample writes one JSON object; Stop() closes the file.
 */
class NtnNetSimulyzerExporter : public Object
{
  public:
    static TypeId GetTypeId();

    NtnNetSimulyzerExporter();
    ~NtnNetSimulyzerExporter() override;

    void SetOutputPath(const std::string& path);

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
    bool m_running{false};
    bool m_firstEvent{true};
    std::vector<NodeDecl> m_nodes;
    std::vector<SeriesDecl> m_series;
    uint64_t m_eventCount{0};
};

} // namespace ntnobs
} // namespace ns3

#endif // NTN_NETSIMULYZER_EXPORTER_H
