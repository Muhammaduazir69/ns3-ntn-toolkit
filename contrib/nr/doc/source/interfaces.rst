.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Interfaces
**********
This chapter documents the EPC / core-network interfaces of the 'NR' module: the gNB-to-core S1
interface, the SGW-to-PGW S5 and MME-to-SGW S11 interfaces, and the inter-gNB X2 interface. As an
NR non-standalone (NSA) deployment over a ported LTE EPC, these reuse the LTE interfaces; the O-RAN
E2 interface will be added here in the future.

:numref:`fig-interfaces-overview` gives the full picture: a single end-to-end lifecycle (attach, user
data, X2 handover and release) with every message labelled by the interface it uses (S1-MME, S1-U, S5,
S11, X2-C, X2-U). The per-interface sections below zoom into each one.

.. _fig-interfaces-overview:

.. figure:: figures/epc/interfaces-overview.*
   :align: center

   Combined view of the S1, S5, S11 and X2 interfaces over a full connection lifecycle.

.. toctree::
   :maxdepth: 2

   S1 <s1-interface>
   S5 <s5-interface>
   S11 <s11-interface>
   X2 <x2-interface>
