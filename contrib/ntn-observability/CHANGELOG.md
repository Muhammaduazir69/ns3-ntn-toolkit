# Changelog

All notable changes to this module are documented here. The format is
based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to Semantic Versioning.

## [Unreleased]

### Added

- New `ntn-repro-manifest` model (`NtnReproManifest`) — a reproducibility
  manifest recording git SHA, ns-3 version, scenario, TLE epoch,
  constellation, Sionna / HITRAN / ITU versions, service models, CLI argv and
  extras, with `WriteJson()` / `LoadJson()` round-tripping.
- New `ntn-observability-traffic` example with a real ns-3 data plane (LEO
  downlink over point-to-point + IP + UDP apps + FlowMonitor) whose InfluxDB
  KPIs come from a real `PacketSink` byte counter, not synthetic values.
- Five repro-manifest unit tests (JSON round-trip, schema-header presence,
  unknown-key tolerance, malformed-JSON rejection, escape round-trip),
  bringing the suite to 10 cases.

## [1.0.0]

### Added

- Initial release of `ntn-observability` — observability sinks plus
  reproducibility manifests for 6G NTN simulation.
- **`ntn-metric-schema.h`** — header-only canonical metric schema
  (`measurement::` / `tag::` / `field::` string tables), pinned by
  `MetricSchemaStableTest`.
- **`NtnInfluxSink`** — InfluxDB v2 line-protocol encoder with key escaping,
  buffered file or UDP transport, and run-id tagging.
- **`NtnNetSimulyzerExporter`** — streaming JSON writer for the NIST
  NetSimulyzer 1.0 schema.
- **`NtnObservabilityHelper`** — one-call setup of both sinks with consistent
  run-id tagging.
- Four pre-built Grafana dashboards (Overview, Handover, Radio, ISL) and a
  Docker stack (InfluxDB 2.7 + Grafana 10.4 + Telegraf sidecar).
- Example program `ntn-observability-demo` and a unit-test suite
  (`test/ntn-observability-test-suite.cc`, suite name `ntn-observability`).
