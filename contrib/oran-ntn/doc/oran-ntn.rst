..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

oran-ntn Module
===============

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

The ``oran-ntn`` module brings an O-RAN control plane to ns-3 NTN
simulations: a multi-tier RIC (a satellite-hosted real-time tier, a
ground Near-RT RIC, and per-satellite Space RICs with an autonomous
mode under feeder outage), an A1 policy layer, an xApp framework with
thirteen reference xApps plus an ONNX-Runtime inference xApp, a
conflict manager, cross-domain SMO analytics (NWDAF / transport
controller / SMO loop), and measurable models for satellite payload
architectures, fronthaul splits, platform classes, and functional
role switching. The module links against ``ntn-traffic``, whose
``NtnRealStackHelper`` provides a real mmwave NR NTN cell; all KPIs
consumed by the RIC tiers are measured in-band on that stack.

Model description
-----------------

The source code lives in ``contrib/oran-ntn/model``,
``contrib/oran-ntn/helper``, ``contrib/oran-ntn/asn1``, and
``contrib/oran-ntn/flexric-bridge``.

Design
~~~~~~

The module exposes the following public classes:

* ``OranNtnNearRtRic`` — ground Near-RT RIC: E2 node registry,
  subscriptions, indication routing, action pipeline, KPM logging.
* ``OranNtnRtRic`` — real-time RIC tier co-located with the O-DU on
  the satellite; registered RT actions run per UE per loop on
  measured PHY KPIs (SINR/TBLER), and a loop period of 10 ms or more
  refuses to start (the O-RAN real-time bound).
* ``OranNtnRicPlacement`` — RIC siting as an experiment variable
  (``OnBoardSatellite`` / ``Haps`` / ``GroundGateway`` /
  ``GroundCloud``); the one-way E2 delay is computed from the live
  slant range plus per-site terms, and ``BindToGeometry()`` keeps an
  E2 node's feeder delay tracking the real mobility models.
* ``OranNtnXappBase`` — xApp base class (decision interval,
  confidence, latency tracking). Thirteen concrete xApps inherit it:
  ``OranNtnXappHoPredict``, ``OranNtnXappBeamHop``,
  ``OranNtnXappSliceManager``, ``OranNtnXappDopplerComp``,
  ``OranNtnXappTnNtnSteering``, ``OranNtnXappEnergyHarvest``,
  ``OranNtnXappInterferenceMgmt``, ``OranNtnXappMultiConn``,
  ``OranNtnXappPredictiveAlloc``, ``OranNtnXappIsac``,
  ``OranNtnXappThzSpectrum``, ``OranNtnXappThzBeamMgmt``,
  ``OranNtnXappThzRis``.
* ``OranNtnOnnxXapp`` — AI-native inference xApp: loads ``.onnx``
  models (exported from the toolkit gym environments) through ONNX
  Runtime and infers on measured feature vectors. ONNX Runtime is an
  optional dependency auto-detected at configure time; without it the
  xApp falls back to a caller-registered heuristic policy.
* ``OranNtnNwdaf`` / ``OranNtnTpnController`` /
  ``OranNtnCrossDomainSmo`` — cross-domain SMO trio: per-slice
  analytics with an SLA-risk score from measured KPM, transport-path
  ranking/switching by live latency, and a closed coordination loop
  across RAN quota, transport path, and edge compute.
* ``NtnFhSplitModel`` — fronthaul split options Opt 2 / 7.2a / 7.2b /
  Opt 8 with one-way latency bounds and fronthaul-rate multipliers;
  ``IsFeasible()`` / ``ChooseBestSplit()`` evaluate splits against the
  measured feeder delay and capacity.
* ``NtnPlatformSpec`` — per-platform-class constraints (UAV
  untethered/tethered, HAP, LEO, MEO, GEO): payload mass, power,
  endurance (enforceable via ``ScheduleEnduranceEnd()``), latency
  class, network role.
* ``OranNtnRoleSwitch`` — functional role switching (RU → RU+DU →
  full gNB) on measured triggers (battery fraction, measured
  fronthaul latency, injected failures) with a configurable
  service-interruption window.
* ``OranNtnA1PolicyManager`` / ``OranNtnA1Adapter`` — 11 A1 policy
  types with versioning and acknowledgement, plus an OSC-aligned
  policy schema registry.
* Service models behind a plugin ABI: E2SM-KPM, E2SM-RC v1.03
  (29 action types, Style 3 connected-mode mobility), E2SM-CCC, and
  an NTN-ephemeris extension. An ASN.1 Aligned-PER codec and an
  SCTP/TCP E2 listener support external (FlexRIC-style) RIC bridging.
* ``OranNtnConflictManager`` — five resolution strategies
  (PRIORITY, TEMPORAL, MERGE, A1_GUIDED, ML_BASED) over
  resource-key contracts.
* ``OranNtnSpaceRic`` — satellite-hosted RIC with autonomous mode
  and ISL messaging (8 message types).
* ``OranNtnSplitGnbEntity`` / ``OranNtnF1Interface`` /
  ``OranNtnOfhInterface`` — CU/DU/RU split with per-entity E2
  termination, built via ``OranNtnSplitGnbHelper``.

Scope and limitations
~~~~~~~~~~~~~~~~~~~~~~~

* The federated-learning path ships four aggregators but an
  end-to-end FL training campaign is not part of the released
  scenarios.
* A dedicated ``E2SM-HO-PRED`` codec class is specified in the
  companion paper but not yet shipped; the HO-prediction xApp
  consumes the measurement tuple through E2SM-KPM/RC.
* ``OranNtnOnnxXapp`` expects a single float input tensor ``[1,N]``
  and a single float output tensor; without ONNX Runtime installed it
  runs the registered heuristic only.

Examples
~~~~~~~~

* ``oran-ntn-full-scenario`` — 66-satellite Walker constellation on
  real SGP4 mobility (per-UE serving selection by live max-elevation);
  the first ``--numRealCells`` satellites carry a real measured mmwave
  cell (provenance ``phy-trace``) while scale-out UEs use a
  TR 38.821-style budget over the same live geometry (provenance
  ``geometry-budget``); 5 live xApps, feeder-outage windows with
  autonomous Space-RIC decisions; writes ``kpm_feed.csv`` (per-row
  provenance), ``action_log.csv``, ``xapp_metrics.csv``,
  ``space_ric_metrics.csv``, ``conflict_log.csv``,
  ``kpm_dataset.csv``, ``kpm_canonical.csv``, ``ric_metrics.txt``,
  ``sim_health.csv``.
* ``oran-ntn-real-stack-scenario`` — the Near-RT RIC driven by KPM
  built from the measured per-UE SINR/TBLER of a real mmwave NR NTN
  cell on SGP4 orbits with TR 38.811 UE mobility.
* ``oran-ntn-ric-controlled-traffic`` — closed RIC loop: E2-KPM →
  mMIMO precoder xApp → E2SM-RC ``BEAM_SWITCH`` via
  ``ReceiveRcAction()`` (one feeder delay per direction, indications
  aligned to the 100 ms RIC tick) → beam gain → delivered goodput on
  a real measured mmwave flow.
* ``ntn-e2e-full-stack`` — one shared real cell simultaneously feeds
  the Near-RT RIC, the ``ntn-slice`` SLA monitor, and a measured-KPI
  observability CSV.
* ``oran-ntn-ric-placement-ab`` — measured control-loop reaction
  time per RIC placement (on-board vs gateway vs cloud), with the E2
  latency derived from the live slant range.
* ``oran-ntn-cross-domain-slice`` — NWDAF + SMO + TPN + ONNX xApp
  over two real SGP4 transport paths; the transport controller
  switches the route at the geometric latency crossover.
* ``oran-ntn-payload-options-ab`` — transparent vs regenerative
  (O-RU / O-RU+O-DU / full gNB) payloads measured as in-band one-way
  delay on one real cell, with FH-split feasibility checks.
* ``ntn-platform-latency-validation`` — per-platform (UAV/HAP/LEO/
  MEO/GEO) round-trip time validated against its latency band with
  real packets.
* ``oran-ntn-emergency-communication`` — post-disaster scenario:
  gateway failure, role switch to a full on-board gNB with a real
  service-interruption window, and a dedicated emergency slice
  (SST=5) protected by the SMO.

Testing
-------

Three unit-test suites cover RIC bookkeeping, service-model
round-trips, A1 policies, conflict strategies, Space-RIC behaviour,
the multi-tier RIC additions, and the payload/platform models:

.. sourcecode:: bash

   ./test.py -s oran-ntn                  # 58 cases: core RIC / SM / A1 / Space-RIC
   ./test.py -s oran-ntn-multi-tier-ric   # RT-RIC timescale bound, placement geometry,
                                          # cross-domain loop, ONNX heuristic fallback
   ./test.py -s oran-ntn-ws4              # payload delay ladder, FH-split feasibility,
                                          # UAV endurance enforcement, role-switch E2E

References
~~~~~~~~~~

* O-RAN Alliance WG3, Near-RT RIC architecture & E2AP.
* O-RAN Alliance WG4, fronthaul split options (with 3GPP TR 38.801).
* O-RAN Alliance, NTN deployment white paper (2025).
* 3GPP TR 38.811 / TR 38.821, NTN study items.
* 3GPP TS 28.552, 5G performance measurements (KPM naming).
