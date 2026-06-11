..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

ntn-traffic Module
==================

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

``ntn-traffic`` is the traffic and measurement backbone of the
ns3-ntn-toolkit. It provides three layers:

* **ORAN-NTN application suite** (June 2026 update). Every packet of an
  ``NtnOranApplication`` carries an ``NtnOranPayloadHeader`` — a 24-byte
  in-band wire header (version, payload type, sequence number, TX
  timestamp in ns, 5QI, S-NSSAI SST/SD, QFI, srcId/dstId) serialized as
  real bytes, so QoS/slice identity and measurement primitives survive
  GTP re-encapsulation through the EPC. ``NtnOranSink`` measures, per
  QoS flow, one-way delay (in-band timestamps), RFC 3550 jitter,
  sequence-gap loss and throughput from the received bytes themselves.
  ``NtnCommandAndControlApp`` adds platform command-and-control
  telemetry (real mobility samples + battery model, 5QI 69).

* **AI-native flow monitoring.** ``NtnOranAiFlowMonitor`` builds on the
  real ns-3 FlowMonitor: ``NtnOranFlowClassifier`` keys flows by the
  ORAN identity in packet bytes (srcId + dstId + 5QI + S-NSSAI) and
  ``NtnOranFlowProbe`` reports each packet at the application
  measurement points. The monitor publishes per-flow KPM series under
  3GPP TS 28.552 / O-RAN E2SM-KPM names (``DRB.UEThpDl``,
  ``DRB.RlcSduDelayDl``, ``DRB.PacketLossRateDl``,
  ``DRB.PdcpSduVolumeDl``, ``L1M.RS-SINR``), sliding-window AI feature
  vectors (mean/slope), an EWMA z-score anomaly detector, and
  XML / CSV / InfluxDB-line-protocol / E2-indication exporters.

* **Real NTN radio helper.** ``NtnRealStackHelper`` installs a real
  mmwave NR air interface (SpectrumPhy, MAC, RLC/PDCP, RRC, EPC)
  between satellite gNB and ground UE nodes with their own mobility
  models (SGP4, TR 38.811), NTN-ized for LEO geometry. It offers
  pre-canned traffic profiles, explicit per-5QI flows
  (``InstallOranFlow``), the AI-native monitor
  (``EnableOranFlowMonitor``), a satellite payload-option delay model
  (Transparent / RegenerativeRu / RegenerativeRuDu / FullGnb) driven
  live from feeder geometry (``SetFeederGeometry``), and measured-KPI
  accessors (PHY-trace SINR/TBLER, in-band app KPIs).

Classic NTN-calibrated traffic generators are retained: CBR, NRTV over
TCP/UDP, 3GPP HTTP adapted for satellite RTTs, a lightweight per-packet
time-tag, and the legacy point-to-point ``NtnRealisticTrafficHelper``
data plane.

Public classes:

* ``NtnOranPayloadHeader``, ``NtnOranApplication``, ``NtnOranSink``,
  ``NtnCommandAndControlApp``, ``NtnCncTelemetry``
* ``NtnOranAiFlowMonitor``, ``NtnOranFlowClassifier``,
  ``NtnOranFlowProbe``
* ``NtnRealStackHelper``, ``NtnStaticExtraLossModel``
* ``NtnRealisticTrafficHelper``
* ``CbrApplication``
* ``NrtvTcpClient``, ``NrtvTcpServer``, ``NrtvUdpServer``,
  ``NrtvVariables``, ``NrtvHeader``, ``NrtvVideoWorker``
* ``ThreeGppHttpSatelliteClient``, ``ThreeGppHttpVariables``
* ``TrafficTimeTag``

Examples
--------

* ``ntn-oran-qos-flows`` — four 3GPP QoS flows (5QI 1/2/82/9) plus C&C
  telemetry on one real LEO cell; per-flow measured KPIs and KPM
  exports.
* ``ntn-tr38821-calibration`` — measured SINR calibrated against the
  TR 38.821 Set-1 LEO-600 S-band link budget over a live SGP4 pass.
* ``ntn-real-stack-smoke`` — minimal ``NtnRealStackHelper`` validation.
* ``nrtv-p2p-example``, ``nrtv-variables-plot`` — classic NRTV traffic
  model demos.

Tests
-----

Run from the ns-3 root with ``./test.py -s <suite>``:
``ntn-oran-application``, ``ntn-oran-ai-flow-monitor``, ``cbr-test``,
``nrtv``.

See Paper 1 (SoftwareX) for the per-module inventory table.
