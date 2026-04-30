..
   SPDX-License-Identifier: GPL-2.0-only
   Copyright (c) 2026 Muhammad Uzair and contributors

ntn-traffic Module
==================

.. include:: replace.txt
.. highlight:: cpp

Overview
--------

``ntn-traffic`` provides traffic generators calibrated for NTN
scenarios: CBR, NRTV over TCP/UDP, 3GPP HTTP adapted for satellite
RTTs, and a lightweight per-packet time-tag for latency budget
accounting.

Public classes:

* ``CbrApplication``
* ``NrtvTcpClient``, ``NrtvTcpServer``, ``NrtvUdpServer``,
  ``NrtvVariables``, ``NrtvHeader``, ``NrtvVideoWorker``
* ``ThreeGppHttpSatelliteClient``, ``ThreeGppHttpVariables``
* ``TrafficTimeTag``

See Paper 1 (SoftwareX) for the per-module inventory table.
