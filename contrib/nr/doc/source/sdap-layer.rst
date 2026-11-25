.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

SDAP layer
**********
The Service Data Adaptation Protocol (SDAP), specified in [TS37324]_, is the NR-specific layer that
sits above PDCP on the user plane. Within the 5G QoS framework of [TS38300]_, its job is to map 5G
**QoS flows** onto **data radio bearers (DRBs)** -- a single DRB may carry several QoS flows -- and,
when configured, to add an SDAP header carrying the QoS Flow Identifier (QFI) so the receiver can
perform reflective QoS.

SDAP is a distinct layer from :doc:`PDCP <pdcp-layer>`: PDCP provides sequence numbering and the
transfer of data/SN status per radio bearer, whereas SDAP provides the QoS-flow-to-DRB mapping above
it. They are documented separately for that reason.

Status in the 'NR' module
=========================
**SDAP is not implemented as a layer in the 'NR' module.** There is no SDAP entity, no SDAP header,
and no QoS-flow-to-DRB multiplexing. Instead, the SDAP function is approximated by a fixed,
deterministic 1:1 mapping together with IP-5-tuple classification spread across the helper, RRC, NAS
and EPC. The remainder of this section documents what exists today in place of SDAP, so that the gap
relative to a standard SDAP entity is explicit.

QoS flow to DRB mapping (the SDAP substitute)
=============================================
Each QoS flow maps to exactly one DRB, and each DRB to exactly one logical channel, through a fixed
arithmetic relation defined in ``nr-common`` (namespace ``nr``):

.. code-block:: text

   DRBID == LCID                 (identity: Lcid2Drbid / Drbid2Lcid)
   QFI   == DRBID - 2  == LCID - 2   (Drbid2Qfi / Qfi2Drbid, Lcid2Qfi / Qfi2Lcid)

so ``DRBID = LCID = QFI + 2``. There is therefore no many-flows-to-one-DRB multiplexing such as a
real SDAP entity would perform. The mapping is applied when a bearer is set up at the gNB RRC
(``NrGnbRrc``), which allocates the DRB identity, derives the logical-channel identity and the QoS
flow identity from it, and records them on the bearer; the UE RRC (``NrUeRrc``) mirrors the inverse
mapping (``Qfi2Drbid``).

QoS flow selection (classification)
===================================
The decision of which QoS flow a packet belongs to -- normally an SDAP/NAS responsibility -- is made
by IP 5-tuple classifiers built from QoS rules (``NrQosRule``, the replacement for the LTE Traffic
Flow Template):

* **Uplink** classification is done in the UE NAS: ``NrEpcUeNas::Send`` classifies each packet with
  ``m_qosRuleClassifier`` to obtain a QFI and sends it down the access stratum (see the
  :doc:`NAS layer <nas-layer>` section).
* **Downlink** classification is done in the core network at the PGW
  (``NrEpcPgwApplication``), which classifies each packet to a QFI and maps it to the corresponding
  GTP-U tunnel.

QoS flow identifier on the data path
====================================
Because there is no SDAP header, the QFI is carried internally by a packet tag, ``NrQosFlowTag``,
rather than by an over-the-air header field. At the gNB the tag is attached when an uplink SDU is
delivered up the stack (for logical channels above the signalling range), and on the downlink the
QFI read from the tag is converted back to the logical channel and DRB identities using the same
``Qfi2Lcid`` / ``Qfi2Drbid`` relations.

Related QoS machinery
=====================
The QoS-flow abstractions that stand in for SDAP are: ``NrQosFlow`` (which carries the 5QI and its
standardized characteristics), ``NrQosRule`` and ``NrQosRuleClassifier`` (the packet filters), and
the BWP steering performed by ``BwpManagerAlgorithm::GetBwpForQosFlow``, which selects the bandwidth
part for a flow based on its 5QI. QoS flows are activated through the helper API
(``NrHelper::ActivateDedicatedQosFlow``).

Not modelled
============
Relative to a standard SDAP entity, the following are absent: QoS-flow-to-DRB multiplexing (more than
one flow per DRB), insertion/removal of the SDAP header, end-marker handling on flow remapping, and
reflective QoS.
