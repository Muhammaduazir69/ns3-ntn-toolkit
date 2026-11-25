.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

S11 interface
*************
The S11 interface connects the Mobility Management Entity (MME) to the Serving Gateway (SGW) and is a
**control-plane-only** interface, following the EPS architecture of [TS23401]_. It carries GTPv2-C
(GTP-C, [TS29274]_) signalling over UDP and is used to set up, modify and tear down the user-plane
GTP-U tunnels of the :doc:`S1 <s1-interface>` and :doc:`S5 <s5-interface>` interfaces.

S11 service interface
=====================
The S11 service interface is ``NrEpcS11Sap``, split into an MME side (``NrEpcS11SapMme``) and an SGW
side (``NrEpcS11SapSgw``). There is no separate C++ SAP object instantiated for the wire protocol
itself: the MME and SGW exchange encoded GTP-C messages over a UDP socket (``m_s11Socket``) and track
the peers through Fully-Qualified TEID (F-TEID) maps. The MME's S11 control F-TEID uses the
``S11_MME_GTPC`` interface type.

Messages
========
The GTP-C messages exchanged on S11 mirror those of :doc:`S5 <s5-interface>` (same ``NrGtpcHeader``
encoding, [TS29274]_):

* ``CreateSessionRequest`` / ``CreateSessionResponse``;
* ``ModifyFlowRequest`` / ``ModifyFlowResponse``;
* ``DeleteFlowRequest`` / ``DeleteFlowCommand`` / ``DeleteFlowResponse``.

The flow-context elements (``FlowContextToBeCreated``, ``FlowContextCreated``) carry the QFI, the
``NrQosFlow`` and the ``NrQosRule`` -- the NR replacements for the LTE bearer/TFT types.

Procedures
==========
* **Attach.** When a UE attaches (driven by the :doc:`NAS <nas-layer>` layer), the MME issues a
  Create Session Request over S11; the SGW forwards it to the PGW over :doc:`S5 <s5-interface>`, and
  the responses establish the S1-U and S5 user-plane bearers.
* **Handover (path switch).** When the MME receives an S1-AP Path Switch Request from the target gNB
  (see the :doc:`S1 <s1-interface>` section), it issues Modify Flow Request messages over S11 so that
  the SGW switches the S1-U tunnels from the source gNB to the target gNB.

The QFI numbering used in these procedures (QFI 1 default, QFI 2 reserved for sidelink, 3+ dedicated,
assigned by ``NrEpcMmeApplication::AddFlow``) and the E-RAB-ID/DRBID/LCID relationships are described
in the :doc:`EPC <epc>` section.

:numref:`fig-s11` shows the GTP-C message exchange on S11 across the attach, handover (path-switch)
and release procedures.

.. _fig-s11:

.. figure:: figures/epc/s11.*
   :align: center

   GTP-C message exchange on S11 (MME to SGW).
