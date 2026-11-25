.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Simulation-wide settings
************************

During the adaptations to support RRC, some settings previously defined in lower layers are moved to NrHelper,
due to their wide effects on the simulation. Some of these settings are simulator specific, and will be kept there
(e.g. ``RbOverhead``, moved from ``NrGnbPhy`` to ``NrHelper``).
Other settings are actually configurable according to the standard, and will be moved out once support is finished
(e.g. ``NumRbPerRbg``, moved from ``NrGnbMac`` to ``NrHelper``). More details about these in the :doc:`PHY layer <phy-layer>` and :doc:`MAC layer <mac-layer>` sections.
