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
(CHO) framework for Non-Terrestrial Networks (NTN) in ns-3.  The
module integrates four CHO algorithms, an SGP4 orbit propagator, a
Walker-Star constellation generator, and an mmWave+satellite channel
adaptor built on top of the public ``mmwave`` contributed module.

Model description
-----------------

The source code for the new module lives in
``src/ntn-cho/model`` and ``src/ntn-cho/helper``.

Design
~~~~~~

The module exposes the following public classes:

* ``NtnTteEstimator`` — cached ``O(log n)`` estimator of the residual
  beam-dwell time.
* ``NtnCondHandoverAlgorithm`` — abstract CHO base class.  Concrete
  subclasses: ``NtnTteCondHandoverAlgorithm``,
  ``NtnA3CondHandoverAlgorithm``,
  ``NtnTimeBasedCondHandoverAlgorithm``,
  ``NtnLocationBasedCondHandoverAlgorithm``.
* ``NtnSgp4Mobility`` — SGP4 orbit propagator following the public
  Vallado reference implementation.
* ``NtnConstellationHelper`` — Walker-Star ``(T,P,F)`` constellation
  generator.
* ``MmWaveSatChannel``, ``MmWaveSatPropagationLossModel``,
  ``MmWaveSatSpectrumPropagationLossModel`` — mmWave + satellite
  channel bridge.

Scope and limitations
~~~~~~~~~~~~~~~~~~~~~

* The module targets LEO NTN scenarios; GEO/MEO are supported but not
  yet validated.
* The RRC connection procedure follows TS 38.331 Release 17
  conditional handover.  Handling of multi-connectivity (split-CHO) is
  not yet implemented.
* The channel is frequency-flat over the assumed 400 MHz sub-band.

References
~~~~~~~~~~

* 3GPP TS 38.331 v17, Radio Resource Control (RRC); Protocol
  specification.
* 3GPP TR 38.821 v16, Solutions for NR to support NTN.
* Vallado D. A., *Fundamentals of Astrodynamics and Applications*,
  4th ed., Microcosm Press, 2013.

Usage
-----

Helpers
~~~~~~~

``NtnConstellationHelper`` sets up the full Walker-Star constellation
and attaches SGP4 mobility to every satellite node.  Helpers for
channel, EPC, and mobility are also provided.

Attributes
~~~~~~~~~~

The most commonly-tuned attributes are:

* ``NtnTteCondHandoverAlgorithm::TteThresholdS`` — trigger threshold
  on residual dwell time, seconds.
* ``NtnTteCondHandoverAlgorithm::HysteresisDb`` — hysteresis margin on
  the A3 offset, dB.
* ``NtnConstellationHelper::TotalSats`` / ``Planes`` / ``Phasing``
  / ``AltitudeKm`` — Walker-Star parameters.

A full attribute table is given in Paper 2, Table V
(``papers/paper2_taes_tte_cho/main.tex``).

Output
~~~~~~

Each simulation writes six CSVs: ``handovers.csv``,
``kpi_summary.csv``, ``tte_trace.csv``, ``beam_dwell.csv``,
``track.csv``, ``sinr.csv``.

Examples
~~~~~~~~

Representative examples are provided:

* ``ntn-cho-scenario-a.cc`` — baseline single-cell handover.
* ``ntn-cho-scenario-b-mc.cc`` — Monte-Carlo entry point.
* ``ntn-cho-walker-star.cc`` — full Walker-Star constellation sweep.

Validation
----------

The module ships a Monte-Carlo harness (5 seeds × 4 CHO algorithms)
that reproduces the results reported in Paper 2 (IEEE TAES).  Run:

.. sourcecode:: bash

   cd papers/sim_runs
   ./run_mc_sweep.sh
   python3 build_figures.py
