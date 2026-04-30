..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

oran-ntn Module
===============

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

The ``oran-ntn`` module provides an O-RAN Near-RT RIC and Space RIC
implementation for Non-Terrestrial Networks in ns-3, together with 13
xApps, an A1 policy engine, and an E2SM-RC action runtime.

Model description
-----------------

Key classes:

* ``OranRic`` — Near-RT RIC core with E2AP session management,
  subscription handling, and a conflict-resolver hook.
* ``Xapp`` — abstract xApp base class.  Concrete xApps:
  ``TrafficSteeringXapp``, ``HandoverControlXapp``,
  ``QosOptimiserXapp``, ``BeamManagementXapp``, ``LoadBalancerXapp``,
  ``AnomalyDetectorXapp``, ``EnergySaverXapp``, ``SlicingXapp``,
  ``RlmXapp``, ``CellOnOffXapp``, ``AdmissionControlXapp``,
  ``InterferenceCoordinatorXapp``, ``SpaceRicXapp``.
* ``A1PolicyEngine`` — 11 policy types; see Paper 3 Table IV.
* ``E2smRcRuntime`` — 28 actions; see Paper 3 Table V.
* ``ConflictResolver`` — pair-wise matrix for co-located xApps.
* ``SpaceRic`` — autonomous-mode metrics for stressed feeder-link
  scenarios.

References
~~~~~~~~~~

* O-RAN Alliance WG1, O-RAN Architecture Description v10.0, 2024.
* O-RAN Alliance WG3, E2SM-KPM v03.00, E2SM-RC v01.03.
* Lacava A. et al., *An Open RAN Framework for Non-Terrestrial
  Networks*, IEEE TNSM, 2025.
* Shen P. et al., *Federated A1 Transfer Accounting for Space RICs*,
  IEEE TNSM, 2025.

Usage
-----

See ``examples/oran-ntn-scenario-b-full-xapps.cc`` for a 60-second
full-xApp Monte-Carlo-free scenario.  Per-xApp decision intervals are
configurable attributes.

Output
~~~~~~

Each run writes: ``action_log.csv``, ``conflict_log.csv``,
``kpm_dataset.csv``, ``xapp_metrics.csv``, ``space_ric_metrics.csv``.

Validation
----------

The 60-s full-xApp run reproduced in Paper 3 produced 7369 actions
with 0 conflicts; the dataset is included under
``papers/sim_runs/oran-ntn/run1/``.
