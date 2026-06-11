..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

thz-ntn Module
==============

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

The ``thz-ntn`` module provides sub-THz / THz propagation, molecular
absorption, antenna arrays and beamforming, reconfigurable intelligent
surfaces (RIS), inter-satellite links (ISL), and integrated sensing and
communication (ISAC) primitives for ns-3 NTN scenarios. It targets the
100 GHz - 1 THz band (D-band and sub-THz).

Design
------

The composite channel (``ThzNtnChannelModel``, a ``PropagationLossModel``)
cascades free-space loss, molecular absorption, weather attenuation,
scintillation, pointing error, and a hardware-impairment stage. The
hardware-impairment stage is a registered placeholder: ``ThzNtnHardwareImpairments``
is a registered ns-3 object and the channel exposes an ``EnableHardwareImpairments``
attribute, but the per-link SNR degradation is not yet applied (the call site in
``thz-ntn-channel-model.cc`` is commented out pending the model implementation).

Key classes (from ``model/*.h``):

* ``ThzNtnMolecularAbsorption`` - HITRAN line-by-line gaseous absorption over
  an ITU-R P.835 stratified atmosphere.
* ``HitranLut`` (namespace ``ns3::thzntn``) - bundled HITRAN-2024 specific
  attenuation lookup table; the build pins ``kHitranRelease = "HITRAN-2024"``
  and ships ``data/hitran2024-lut-subthz.csv``.
* ``ThzNtnFreeSpaceLoss`` - free-space path loss; extends ``SatFreeSpaceLoss``.
* ``ThzNtnChannelModel`` - composite cascade propagation loss model.
* ``ThzNtnPropagationLossModel`` - the molecular-absorption and weather
  calculators re-homed as a real ``PropagationLossModel`` (atmospheric excess
  loss only), so they can be chained onto a live spectrum channel (e.g. via
  ``NtnRealStackHelper::AddExtraPropagationLoss`` in ``contrib/ntn-traffic``)
  and attenuate actual packets.
* ``Itu838RainModel``, ``Itu618LossModel``, ``Itu676AbsorptionModel``,
  ``Itu681LmsModel`` (``thz-ntn-itu-recommendations.h``) - per-recommendation
  reference implementations.
* ``ThzNtnAntennaArray`` - UPA / UCA / Cassegrain arrays, gain and beamwidth.
* ``ThzNtnBeamforming`` - DFT codebook generation and beam-squint analysis
  (``ComputeBeamSquintLoss_dB``).
* ``ThzNtnBeamTracking`` - beam tracking with a 4-state EKF
  (state ``[theta, phi, dTheta/dt, dPhi/dt]``) driven by the satellite
  ``MobilityModel``.
* ``ThzNtnIslChannel`` - inter-satellite link channel (D-band / 300 GHz).
* ``ThzNtnRis`` - reconfigurable intelligent surface with N^2 array gain and
  phase-quantisation loss.
* ``ThzNtnIsac`` - integrated sensing and communication for space-debris
  ranging.
* ``ThzNtnNyusimReference``, ``ThzNtnNyusimCalibrator`` - NYUSIM-140 GHz
  reference loader and calibrator.

Scope
-----

The module models the PHY-layer link physics (channel, antenna/beamforming,
ISL, RIS, ISAC) plus link-budget and waveform utilities. The traffic examples
apply this physics to a real mmwave NR NTN data plane (via
``ThzNtnPropagationLossModel`` and ``NtnRealStackHelper``), with SGP4 satellite
mobility and TR 38.811 ground terminals. The hardware-impairment stage is
registered but not active (see Design). The ISAC scheduler is under redesign;
the legacy ``thz-ntn-isac.cc`` example is excluded from the build.

Usage
-----

Build and run an example:

.. code-block:: bash

   ./ns3 configure --enable-examples --enable-tests
   ./ns3 build thz-ntn
   ./ns3 run thz-ntn-leo-ground

See ``doc/EXAMPLES.md`` for the full list of built examples.

Testing
-------

The test suite ``test/thz-ntn-test-suite.cc`` registers 38 test cases covering
molecular absorption, FSPL, weather, pointing error, hardware-impairment
arithmetic, atmospheric windows, link budget, antenna gain, beamforming
codebooks, ISL vacuum propagation, RIS scaling, and ISAC resolution. Run with:

.. code-block:: bash

   ./test.py --suite=thz-ntn

References
~~~~~~~~~~

* Gordon, I. E., et al., *The HITRAN2020 molecular spectroscopic database*,
  J. Quant. Spectrosc. Radiat. Transf., 277, 2022. (The module's LUT tracks the
  HITRAN-2024 release.)
* ITU-R P.676, Attenuation by atmospheric gases and related effects.
* ITU-R P.618, Propagation data and prediction methods for Earth-space
  telecommunication systems.
* Akyildiz, I. F., et al., *Terahertz Band: Next Frontier for Wireless
  Communications*, Physical Communication, 2014.
