.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

Extensions
**********
This chapter collects extensions of the 'NR' module to additional 3GPP work items. They are
documented here as they mature.

NR-U
====
NR-U extends the 'NR' module to operation in **unlicensed spectrum**: it adds energy detection,
multiple channel-access managers (including duty-cycling) and Listen-Before-Talk (LBT) based
procedures (ETSI-compliant Category 2/3/4 LBT), as well as experimental Wi-Fi coexistence and
directional-LBT support. It is distributed as a **separate module** at
https://gitlab.com/cttc-lena/nr-u, with documentation at https://cttc-lena.gitlab.io/nr-u/ and
installation instructions in its ``README``. The current NR-U code is compatible with 5G-LENA v1.2
and ns-3.35; it is not under active development, although it may still be updated.

NR V2X
======
The sidelink and NR-V2X extension adds **direct device-to-device (vehicular) communication**. It
implements the NR sidelink following 3GPP TR 38.885: broadcast (Mode 4-like) and out-of-coverage V2V
operation, PSCCH/PSSCH multiplexing, Mode-2 UE-selected resource allocation (sensing-based and random,
with semi-persistent scheduling), and the sidelink control information (SCI). It is under active
development in the ``nr-v2x-dev`` branch of the NR repository
(https://gitlab.com/cttc-lena/nr/-/tree/nr-v2x-dev); the design is described in Section 2.16 of the
extension documentation (https://5g-lena.cttc.es/static/archive/NR_V2X_V0.1_doc.pdf). The latest
release is compatible with 5G-LENA v3.1 and ns-3.42.
