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
simulations: a ground Near-RT RIC, per-satellite Space RICs with an
autonomous mode under feeder outage, an A1 policy layer, an xApp
framework with thirteen reference xApps, and a conflict manager.

Model description
-----------------

The source code lives in ``contrib/oran-ntn/model`` and
``contrib/oran-ntn/helper``.

Design
~~~~~~

The module exposes the following public classes:

* ``OranNtnNearRtRic`` — ground Near-RT RIC: E2 node registry,
  subscriptions, indication routing, action pipeline, KPM logging.
* ``OranNtnXappBase`` — xApp base class (decision interval,
  confidence, latency tracking). Thirteen concrete xApps inherit it:
  ``OranNtnXappHoPredict``, ``OranNtnXappBeamHop``,
  ``OranNtnXappSliceManager``, ``OranNtnXappDopplerComp``,
  ``OranNtnXappTnNtnSteering``, ``OranNtnXappEnergyHarvest``,
  ``OranNtnXappInterferenceMgmt``, ``OranNtnXappMultiConn``,
  ``OranNtnXappPredictiveAlloc``, ``OranNtnXappIsac``,
  ``OranNtnXappThzSpectrum``, ``OranNtnXappThzBeamMgmt``,
  ``OranNtnXappThzRis``.
* ``OranNtnA1PolicyManager`` / ``OranNtnA1Adapter`` — 11 A1 policy
  types with versioning and acknowledgement.
* Service models: E2SM-KPM, E2SM-RC v1.03 (29 action types),
  E2SM-CCC, and an NTN-ephemeris extension.
* ``OranNtnConflictManager`` — five resolution strategies
  (PRIORITY, TEMPORAL, MERGE, A1_GUIDED, ML_BASED) over
  resource-key contracts.
* ``OranNtnSpaceRic`` — satellite-hosted RIC with autonomous mode
  and ISL messaging (8 message types).

Scope and limitations
~~~~~~~~~~~~~~~~~~~~~~~

* The federated-learning path ships four aggregators but an
  end-to-end FL training campaign is not part of the released
  scenarios.
* A dedicated ``E2SM-HO-PRED`` codec class is specified in the
  companion paper but not yet shipped; the HO-prediction xApp
  consumes the measurement tuple through E2SM-KPM/RC.

Examples
~~~~~~~~

* ``oran-ntn-full-scenario`` — 66-satellite Walker constellation,
  5 live xApps, real UDP traffic plane, feeder-outage windows with
  autonomous Space-RIC decisions; writes ``action_log.csv``,
  ``xapp_metrics.csv``, ``space_ric_metrics.csv``,
  ``conflict_log.csv``, ``kpm_dataset.csv``, ``ric_metrics.txt``,
  ``sim_health.csv``.
* ``oran-ntn-ric-controlled-traffic`` — RIC control actions steering
  a real UDP flow.

Testing
-------

The ``oran-ntn`` unit-test suite registers 58 test cases covering
RIC bookkeeping, service-model round-trips, A1 policies, conflict
strategies, Space-RIC behaviour, and failure modes:

.. sourcecode:: bash

   ./test.py --suite=oran-ntn

References
~~~~~~~~~~

* O-RAN Alliance WG3, Near-RT RIC architecture & E2AP.
* O-RAN Alliance, NTN deployment white paper (2025).
* 3GPP TR 38.811 / TR 38.821, NTN study items.
