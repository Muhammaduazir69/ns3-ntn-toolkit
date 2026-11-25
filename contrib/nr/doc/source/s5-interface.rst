.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

S5 interface
************
The S5 interface connects the Serving Gateway (SGW) to the Packet Data Network Gateway (PGW) inside
the EPC, following the EPS architecture of [TS23401]_. As in LTE, it has both a user plane and a
control plane:

* **S5-U** carries user data between the SGW and the PGW over GTP-U (GTPv1-U, [TS29281]_);
* **S5-C** carries control signalling between the SGW and the PGW over GTP-C (GTPv2-C, [TS29274]_).

The endpoints are ``NrEpcSgwApplication`` (which also terminates S1-U and the S11 interface) and
``NrEpcPgwApplication`` (which also terminates the SGi/Gi interface toward the external network
through a ``VirtualNetDevice``).

With home-routed roaming between two core networks of a simulation (see the :doc:`EPC <epc>`
section), the SGW of the visited network and the PGW of the home network of a UE communicate as on
S5, over an S8 link between the two networks: the SGW sends the messages and the uplink packets of
each session to the PGW selected by the MME, and the PGW answers the SGW that created the session.

User plane (S5-U)
=================
S5-U encapsulates the end-to-end IP packets in GTP-U/UDP/IP (UDP port 2152) using the ``NrGtpuHeader``,
without extension header (the PDU Session Container is only carried on S1-U). The data path uses a
single TEID end-to-end: the SGW does not translate between an S1-U and an S5-U TEID in the data
plane. As on S1-U, EPS-bearer QoS is **not** enforced on the link.

Downlink classification -- deciding which QoS flow (and therefore which GTP-U tunnel) a packet belongs
to -- is performed at the PGW: ``NrEpcPgwApplication`` runs an ``NrQosRuleClassifier`` in the
downlink direction to obtain a QFI, then maps the QFI to the corresponding TEID via
``m_teidByFlowIdMap``. This replaces the LTE arrangement, where the classifier was keyed directly by
TEID.

Control plane (S5-C)
====================
S5-C uses GTPv2-C [TS29274]_, encoded with ``NrGtpcHeader``. The message set is inherited from LTE
with the bearer vocabulary renamed to QoS flows (the numeric message-type values are unchanged):

* ``CreateSessionRequest`` / ``CreateSessionResponse``;
* ``ModifyFlowRequest`` / ``ModifyFlowResponse`` (the LTE ``ModifyBearer*``);
* ``DeleteFlowRequest`` / ``DeleteFlowCommand`` / ``DeleteFlowResponse`` (the LTE ``DeleteBearer*``).

The flow-context information elements carry the QFI, the ``NrQosFlow`` and the ``NrQosRule``; the
serialized QoS-rule IE additionally carries the rule precedence and its QFI.

TEID assignment is the key mechanic to note here (the broader statement that the S1-U, S5-U SGW and
S5-U PGW TEIDs are mutually independent is made in the :doc:`EPC <epc>` section). The SGW allocates
the per-flow S5-U data TEID from a counter (``++m_teidCount``) and uses the UE's IMSI as the S5-C
control TEID; the **PGW does not allocate new TEIDs** -- it echoes the SGW's S5-U TEID back when
binding the flow. The Create Session exchange that establishes these tunnels is driven over
:doc:`S11 <s11-interface>` during the NAS attach.

:numref:`fig-s5` shows the GTP-C message exchange on S5-C; it mirrors S11 (same ``NrGtpcHeader``
encoding) but runs between the SGW and the PGW.

.. _fig-s5:

.. figure:: figures/epc/s5.*
   :align: center

   GTP-C message exchange on S5-C (SGW to PGW).
