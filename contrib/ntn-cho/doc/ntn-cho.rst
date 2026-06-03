..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

ntn-cho Module
==============

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

The ``ntn-cho`` module provides a 3GPP Release-17 Conditional Handover
(CHO) framework for Non-Terrestrial Networks (NTN) in ns-3.  It adds an
orbit-informed **Time-to-Exit (TTE)** trigger to the standard Rel-17
CHO state machine and ships the classical A3 / location / time-based
selectors as baselines for comparison.

Orbit propagation and the Walker constellation geometry are
**consumed** from the SNS3 ``satellite`` module
(``SatSGP4MobilityModel`` + antenna-gain patterns); ``ntn-cho`` does not
implement orbit propagation or a constellation generator of its own.

Model description
-----------------

The source code lives in ``contrib/ntn-cho/model`` and
``contrib/ntn-cho/helper``.

Design
~~~~~~

The module exposes the following public classes:

* ``NtnChoAlgorithm`` — Rel-17 CHO state machine
  (``CHO_IDLE → CHO_PREPARED → CHO_CONDITION_MONITORING →
  CHO_EXECUTING → CHO_COMPLETED``) with selectable a3 / location / time
  / tte-aware triggers.
* ``NtnTteEstimator`` — Time-to-Exit estimator: forward propagation
  plus a binary search for the beam-exit instant, cached per candidate.
* ``NtnOrbitPredictor`` — wraps the ``satellite`` module's
  ``SatSGP4MobilityModel`` and antenna-gain-pattern container to predict
  satellite/beam positions and the best beam per UE.
* ``NtnMeasurementModel`` — RSRP / SINR / elevation / Doppler from
  satellite beams using 3GPP TR 38.811 NTN channel scenarios.
* ``NtnAiInterface`` — optional ns3-ai bridge (observation / action) for
  learning-based HO policies; requires the ``ns3-ai`` module.

Helpers:

* ``NtnChoHelper`` — wires up a CHO scenario and reports KPIs.
* ``NtnRealisticMobilityHelper`` (+ ``NtnMobilityScenarios``) — per-class
  ground-UE motion for the seven TR 38.811 §6.1.1.1 UE classes.

Scope and limitations
~~~~~~~~~~~~~~~~~~~~~~~

* The module targets LEO NTN scenarios; GEO/MEO geometries are not
  validated.
* The CHO procedure follows TS 38.331 Release 17 conditional handover.
  Multi-connectivity (split-CHO) is not implemented.
* Orbit propagation and constellation geometry are delegated to the
  ``satellite`` module; ``ntn-cho`` does not own that code.

Attributes
~~~~~~~~~~

The registered, commonly-tuned attributes are:

* ``NtnTteEstimator::PredictionStep`` / ``MaxPredictionWindow`` /
  ``BinarySearchTolerance`` / ``MaxBinarySearchIterations``.
* ``NtnOrbitPredictor::MinGainThreshold`` — beam-edge gain threshold.
* ``NtnChoAlgorithm::EnableMultiBandCho`` /
  ``ThzBeamTrackingThreshold`` / ``ThzSnrThreshold`` / ``ThzBeamwidth``.

The remaining trigger parameters (D1 distance, SINR quality threshold,
minimum TTE, A3 offset / time-to-trigger) are set per run via the
example command-line arguments.

Output
~~~~~~

``ntn-cho-full-constellation`` writes to ``--outputDir``:
``handover_events.csv``, ``measurements.csv``, ``tte_computations.csv``,
``satellite_tracks.csv``, ``ue_tracks.csv``, ``kpi_timeseries.csv``,
the matching GeoJSON layers, and ``kpi_summary.txt``.

Examples
~~~~~~~~

* ``ntn-cho-leo-basic.cc`` — event-driven smoke test with real UDP
  traffic.
* ``ntn-cho-full-constellation.cc`` — multi-satellite Walker
  constellation; ``--algorithm=a3|location|time|tte-aware``.
* ``ntn-cho-handover-traffic.cc`` — a UDP flow that follows the UE
  across a satellite handover (PointToPoint links + FlowMonitor).
* ``ntn-realistic-mobility-demo.cc`` — seven-class UE mobility demo.

Testing
-------

The ``ntn-cho`` unit-test suite (``test/ntn-cho-test-suite.cc``)
validates the CHO algorithm, the CHO state machine, and the NTN
measurement model:

.. sourcecode:: bash

   ./test.py --suite=ntn-cho

References
~~~~~~~~~~

* 3GPP TS 38.331 v17, Radio Resource Control (RRC); Protocol
  specification.
* 3GPP TR 38.821 v16, Solutions for NR to support NTN.
* 3GPP TR 38.811 v15, Study on NR to support NTN.
