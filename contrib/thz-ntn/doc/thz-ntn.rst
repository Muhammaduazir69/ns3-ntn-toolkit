..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

thz-ntn Module
==============

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

The ``thz-ntn`` module provides sub-THz / THz propagation,
atmospheric absorption, UM-MIMO array models, reconfigurable
intelligent surfaces (RIS), and integrated sensing and communication
(ISAC) primitives for ns-3 NTN scenarios.

Model description
-----------------

Key classes:

* ``ThzAtmosphericAbsorptionModel`` — HITRAN-2020 line-by-line model
  with band-averaged shortcut tables (140–460 GHz).
* ``ThzIslChannel`` — inter-satellite D-band / 300 GHz link channel
  (100–1000 km validated).
* ``ThzUmMimoArray`` — up to 16384 antenna elements, analog-hybrid
  beamforming.
* ``ThzRisSurface`` — quantised-phase RIS with 1–8 bit sweep.
* ``ThzIsacSensor`` — ISAC CRB estimator for three debris classes.
* ``ThzBeamTracker`` — 60-s LEO-pass beam-tracking helper.

Validation
----------

Cross-checked against ITU-R P.676-13 (max deviation 0.61 dB at 10°
elevation) and the *am* atmospheric model (max deviation 0.33 dB).
Per-band validation plots are in
``papers/figures/fig_thz_*``; raw CSVs are under
``papers/sim_runs/thz-ntn/``.

References
~~~~~~~~~~

* Gordon, I. E., et al., *The HITRAN2020 molecular spectroscopic
  database*, J. Quant. Spectrosc. Radiat. Transf., 277, 2022.
* ITU-R P.676-13, Attenuation by atmospheric gases and related
  effects, 2022.
* Akyildiz, I. F., et al., *Terahertz Band: Next Frontier for
  Wireless Communications*, Physical Communication, 2014.
