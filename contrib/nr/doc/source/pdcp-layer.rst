.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

PDCP layer
**********
The Packet Data Convergence Protocol (PDCP) layer was ported from the |ns3| 'LTE' module
(``LtePdcp`` became ``NrPdcp``) and is, functionally, a verbatim port: the NR entity implements the
same minimal feature set as the LTE one. That feature set, originally specified for LTE in
[TS36323]_, also covers the basics of the NR PDCP ([TS38323]_) as modelled here. Rather than linking
to the LTE documentation, this section describes the entity as it stands in the 'NR' module.

Modelled and non-modelled features
==================================
The PDCP entity (``NrPdcp``, one per radio bearer) supports exactly three functions:

* transfer of data (user plane or control plane);
* maintenance of PDCP sequence numbers (SNs);
* transfer of the SN status, for use upon handover.

The following PDCP functions are intentionally **not** modelled:

* header compression/decompression of IP flows using ROHC;
* in-sequence delivery of upper-layer PDUs at re-establishment of lower layers;
* duplicate elimination of lower-layer SDUs at re-establishment for radio bearers mapped on RLC AM;
* ciphering and deciphering of user-plane and control-plane data;
* integrity protection and verification of control-plane data;
* duplicate discarding.

Consequently the entity is intentionally lightweight: it carries a 2-byte header with a 12-bit
sequence number and **no** hyperframe number, ciphering, integrity protection, or PDCP control PDUs.
On transmit it stamps each SDU with a timestamp tag (for the per-PDU delay trace), assigns the next
sequence number, adds the header and hands the PDU down to RLC. On receive it removes the header,
records the transfer delay, and delivers the SDU upward **in arrival order** -- there is no PDCP
reordering or duplicate detection, since in-order delivery and reordering are an RLC responsibility.
Figure :ref:`fig-pdcpDataFlow` shows the data path and the SAP boundaries above (RRC/EPC) and below
(RLC).

.. _fig-pdcpDataFlow:

.. figure:: figures/pdcp/pdcp_data_flow.*
   :align: center

   PDCP transmit and receive data flow.

Header and sequence number
==========================
The PDCP header (``NrPdcpHeader``) is 2 bytes long and carries a 1-bit Data/Control (D/C) flag and a
12-bit sequence number (``MAX_PDCP_SN = 4096``; the SN is masked with ``0x0FFF`` and wraps at 4095).
NR-specific longer SN formats (15- or 18-bit) and the optional SDAP/QoS byte are not present, in
keeping with the minimal feature set above.

Service interface
=================
``NrPdcp`` exposes the PDCP service interface to the upper layer (RRC), divided into two parts:

* the ``NrPdcpSapProvider`` part is provided by the PDCP layer and used by the upper RRC layer;
* the ``NrPdcpSapUser`` part is provided by the upper RRC layer and used by the PDCP layer.

There are two service primitives:

* ``NrPdcpSapProvider::TransmitPdcpSdu`` -- RRC uses it to send an RRC PDU down to the PDCP entity at
  the transmitting peer;
* ``NrPdcpSapUser::ReceivePdcpSdu`` -- the PDCP entity uses it to deliver an RRC PDU up to the RRC
  entity at the receiving peer.

Below, ``NrPdcp`` uses the RLC service interface (``NrRlcSapProvider`` / ``NrRlcSapUser``) described
in the :doc:`RLC layer <rlc-layer>` section. Transmit and receive PDUs are reported through the
``TxPDU`` (RNTI, LCID, size) and ``RxPDU`` (RNTI, LCID, size, delay) trace sources.

PDCP discard
============
Although the PDCP entity itself performs no timer-based discard, the model includes a PDCP-discard
mechanism (carried over from the LTE port, where it lives in the RLC code) that is evaluated when an
SDU is handed to RLC (in ``NrRlcUm``): if the head-of-line delay exceeds the discard timer
(``DiscardTimerMs`` or, if zero, the QoS flow's packet delay budget), the packet is dropped and
accounted via the TX-drop trace.

Handover: sequence-number continuity
====================================
During handover, PDCP sequence-number continuity is preserved **only for AM data radio bearers and
only on the network side**: the source gNB reads each AM bearer's PDCP status (``GetStatus()``) and
sends it over X2 (SN STATUS TRANSFER), and the target gNB restores it with ``SetStatus()``. In-flight
downlink data is forwarded over X2-U while the source is in ``HANDOVER_LEAVING`` and buffered at the
target while it is in ``HANDOVER_JOINING``; a forwarded packet whose tunnel is gone is dropped and
traced. The UE side simply recreates its PDCP entities, resetting their sequence numbers to zero.
UM bearers receive no SN transfer. Figure :ref:`fig-pdcpHandover` summarizes this.

.. _fig-pdcpHandover:

.. figure:: figures/pdcp/pdcp_handover.*
   :align: center

   PDCP re-establishment and X2-U data forwarding during handover.
