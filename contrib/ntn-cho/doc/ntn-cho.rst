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
CHO state machine and implements the **five standardized 3GPP NTN
handover trigger classes** — measurement-based (event A3),
location-based (CondEventD1), time-based (CondEventT1, ephemeris
window), elevation-based, and timing-advance-based — plus two
forward-looking mechanisms: Rel-19 conditional LTM (L1-filtered
measurements with a MAC-CE-style fast switch) and trajectory-predictive
CHO (PCHO), with optional RACH-less execution from ephemeris/GNSS
timing-advance pre-compensation.

Orbit propagation and the Walker constellation geometry are **consumed**,
not implemented here. ``NtnOrbitPredictor`` reads any ns-3
``MobilityModel`` registered with ``SetKinematicsSource()``; the
constellation-scale campaign (``ntn-cho-tte-campaign``) registers
``ntn-constellation``'s ``Sgp4MobilityModel``, which for Walker elements
propagates Kepler plus the secular J2 drift of node and perigee (not
Vallado SGP4). The SNS3 ``satellite`` module's ``SatSGP4MobilityModel`` and
antenna-gain-pattern grids are used only by scenarios that do not register a
kinematics source and do not select the analytic beam.
The real-stack examples additionally use the sibling toolkit modules
``ntn-traffic`` (``NtnRealStackHelper`` — a real mmwave NR NTN cell with
measured traffic) and ``ntn-constellation`` (``Sgp4MobilityModel`` +
``WalkerConstellation``).

Model description
-----------------

The source code lives in ``contrib/ntn-cho/model`` and
``contrib/ntn-cho/helper``.

Design
~~~~~~

The module exposes the following public classes:

* ``NtnChoAlgorithm`` — Rel-17 CHO state machine
  (``CHO_IDLE → CHO_PREPARED → CHO_CONDITION_MONITORING →
  CHO_EXECUTING → CHO_COMPLETED``).  Trigger types:
  ``TRIGGER_EVENT_A3``, ``TRIGGER_LOCATION_D1``, ``TRIGGER_TIME_BASED``,
  ``TRIGGER_TTE_AWARE``, ``TRIGGER_THZ_BEAM_QUALITY``,
  ``TRIGGER_LTM_CONDITIONAL``, ``TRIGGER_TRAJECTORY_PREDICTIVE``, and
  the standardized NTN classes ``TRIGGER_TIME_T1``,
  ``TRIGGER_ELEVATION``, ``TRIGGER_TIMING_ADVANCE``.  ``ChoConfig``
  carries the per-class parameters (``t1WindowDuration``,
  ``elevationMinDeg`` / ``elevationHystDeg``, ``orbitAltitudeKm``,
  ``taServingMax`` / ``taAdvantage``, the LTM/PCHO knobs, and
  ``rachLess``).  For the standardized classes the admitted set already
  encodes the trigger condition, so ``SelectBestCandidate()`` returns
  the strongest measured candidate (maximum SINR); the TTE-aware path
  selects the maximum TTE with a SINR tie-break.
* ``NtnTteEstimator``: Time-to-Exit estimator, coarse forward search
  over the predicted orbit plus a bisection for the exit instant. With a
  line-of-sight probability table (``SetLosProbabilityTable``) it also
  returns the expected usable time to exit, the integral of P_LOS along the
  predicted pass. ``ComputeTteSigma`` gives the first-order sigma of the
  exit time by central differences over the injected RTN ephemeris error.
  The estimator itself does not cache; ``NtnChoAlgorithm`` caches candidate
  predictions when ``ChoConfig::tteCacheRefresh`` is positive (recomputed
  after that interval or 100 m of terminal motion, aged by the elapsed time
  in between; the serving cell is always recomputed).
* ``NtnOrbitPredictor``: the network's copy of each satellite's state:
  the registered mobility model's state plus an optional fixed RTN
  ephemeris error per satellite (the along-track axis follows the inertial
  velocity), propagated two-body along the orbit plane and rotated with the
  Earth. Beams are the SNS3 pattern grid or an analytic aperture or steered
  (cos^n scan-loss) beam; ``SetSteeredBeamElevationIndexed`` indexes the
  scan loss by the topocentric (WGS-84) elevation so that a threshold from
  ``GainThresholdForMinElevationDb(e)`` is exactly the contour of elevation e.
* ``NtnMeasurementModel`` — RSRP / SINR / elevation / signed Doppler
  from satellite beams using 3GPP TR 38.811 NTN channel scenarios.
* ``NtnTr38811MobilityModel`` — the 3GPP TR 38.811 §6.1.1.1 NTN UE
  classes as a real ns-3 ``MobilityModel`` (ECEF, the same frame as the
  satellite SGP4 models), so per-class UE motion drives the radio
  geometry, Doppler, and the TTE estimator.  Includes the ``ntngeo``
  geometry utilities and ``NtnTr38811MobilityHelper``.
* ``NtnAiInterface`` — optional ns3-ai bridge (observation / action) for
  learning-based HO policies; requires the ``ns3-ai`` module.

Exit-triggered time-to-exit CHO (TTE-CHO) and its extensions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With ``tteExitTriggered`` the rule stays while the serving cell's predicted
exit exceeds ``tteExitMargin`` plus ``tteMarginZ`` times its exit-time sigma,
and then takes the admissible candidate (quality at least
``qualityThreshold_dB``, predicted exit at least ``tteMinimum`` and later than
the serving cell's) with the best score. A measurement safety net leaves at
once when the serving quality has been below ``tteServingFloor_dB`` for
``tteServingFloorTtt``; it then accepts any candidate above the quality floor,
whatever its predicted exit. The score is the predicted exit, or, with
``rankByUsableTime``, the expected usable time to exit. ``escapeHoldoff``
keeps the cell left through the safety net out of the target set for that
long unless it is the only one. ``UpdateCandidateElevation`` feeds the
topocentric elevation that the elevation trigger reads.

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
  ``satellite`` / ``ntn-constellation`` modules; ``ntn-cho`` does not
  own that code.
* The trajectory-predictive (PCHO) forecaster is a documented
  linear-trend stand-in for the GRU predictor of the referenced work.

Attributes
~~~~~~~~~~

The registered, commonly-tuned attributes are:

* ``NtnTteEstimator::PredictionStep`` / ``MaxPredictionWindow`` /
  ``BinarySearchTolerance`` / ``MaxBinarySearchIterations``.
* ``NtnOrbitPredictor::MinGainThreshold`` — beam-edge gain threshold.
* ``NtnChoAlgorithm::EnableMultiBandCho`` /
  ``ThzBeamTrackingThreshold`` / ``ThzSnrThreshold`` / ``ThzBeamwidth``.

The remaining trigger parameters (D1 distance, SINR quality threshold,
minimum TTE, A3 offset / time-to-trigger, T1 window, elevation floor and
hysteresis, TA limits, LTM/PCHO knobs, RACH-less execution) are fields
of ``NtnChoAlgorithm::ChoConfig``, set per run via the example
command-line arguments.

Output
~~~~~~

``ntn-cho-full-constellation`` writes to ``--outputDir``:
``handover_events.csv``, ``measurements.csv``, ``tte_computations.csv``,
``satellite_tracks.csv``, ``ue_tracks.csv``, ``kpi_timeseries.csv``,
the matching GeoJSON layers, and ``kpi_summary.txt``.  The real-stack
examples write a ``sim_health.csv`` (via
``NtnRealStackHelper::WriteHealthReport()``) whose SINR/TBLER carry
mmwave PHY-trace provenance, and print measured-KPI summaries to stdout.

Examples
~~~~~~~~

* ``ntn-cho-leo-basic.cc`` — smoke test on a real mmwave NR NTN cell:
  SGP4 serving satellite, TR 38.811 UEs, CHO fed the measured SINR.
* ``ntn-cho-full-constellation.cc`` — multi-satellite Walker
  constellation; ``--algorithm=a3|location|time|tte-aware``.
* ``ntn-cho-handover-traffic.cc`` — real UDP downlink across a CHO
  handover between two SGP4 satellites on a real mmwave cell;
  ``--trigger=tte-aware|ltm|pcho|a3|d1|t1|elevation|ta`` selects the
  mechanism (the last five are the standardized 3GPP NTN classes) and
  ``--rachLess=1`` enables RACH-less execution.
* ``ntn-cho-real-stack.cc`` — real-stack flagship; TTE-aware / LTM /
  PCHO decided on the measured mmwave SINR over real orbital dynamics.
* ``ntn-realistic-mobility-demo.cc`` — seven-class UE mobility demo.

Testing
-------

Two suites ship with the module:

* ``ntn-cho`` (``test/ntn-cho-test-suite.cc``) — unit tests for the CHO
  algorithm, the CHO state machine, and the NTN measurement model.
* ``ntn-standards-validation``
  (``test/ntn-standards-validation-test.cc``) — validates the mobility
  architecture against official orbital theory rather than against
  itself: SGP4 propagation vs Keplerian theory (radius ``Re + h``,
  speed ``sqrt(mu/a)``, quarter-period arc, 550 km / 53°), the
  zenith-pass Doppler envelope (Doppler null at culmination, S-band
  shift inside the published LEO envelope), ENU pass geometry
  (culmination at zenith, monotonic elevation decay), and the
  spherical-Earth elevation/slant relation used by the elevation
  trigger, cross-checked against the TR 38.821 LEO-600 reference
  (~1932 km slant at 10° elevation).

.. sourcecode:: bash

   ./test.py -s ntn-cho
   ./test.py -s ntn-standards-validation

References
~~~~~~~~~~

* 3GPP TS 38.331 v17, Radio Resource Control (RRC); Protocol
  specification.
* 3GPP TR 38.821 v16, Solutions for NR to support NTN.
* 3GPP TR 38.811 v15, Study on NR to support NTN.
