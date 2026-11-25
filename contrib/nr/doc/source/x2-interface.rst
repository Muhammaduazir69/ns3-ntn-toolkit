.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

X2 interface
************
The X2 interface logically interconnects two gNBs and is used to run X2-based handover. It follows
the LTE X2 general principles of [TS36420]_ and the X2 Application Protocol of [TS36423]_, and
implements the Mobility-Management elementary procedures of the X2-based handover described in
[TS36300]_ (Handover Request, Handover Request Acknowledge, SN Status Transfer, UE Context Release).
Only **seamless** handover is supported; **lossless** handover is not. The interface is modelled as a
point-to-point link with a dedicated device in each gNB, implemented by the ``NrEpcX2`` object
installed inside every gNB.

Like S1, X2 has a control plane and a user plane, carried over two UDP sockets per peer cell:

* **X2-C** carries the X2-AP PDUs. A real stack runs X2-AP over SCTP; since SCTP is not modelled,
  **UDP** (port 4444) is used instead.
* **X2-U** carries forwarded bearer data during handover, encapsulated over GTP-U/UDP/IP
  ([TS29281]_, port 2152). EPS-bearer QoS is not enforced on it.

An X2 interface between two cells is created with
``NrEpcX2::AddX2Interface(cell1, addr1, cell2, addr2)``. Unlike the LTE model (which allowed several
remote cells per interface through cell-id vectors), the NR model uses scalar cell ids, so each X2
interface connects exactly one remote cell.

X2 service interface
====================
The X2 entity provides services to the gNB RRC through the X2 SAP, split into
``NrEpcX2SapProvider`` (provided by the X2 entity, used by RRC) and ``NrEpcX2SapUser`` (provided by
RRC, used by the X2 entity). RRC messages are carried as a transparent container inside the X2
messages. The provider primitives (each mirrored by a ``Recv*`` callback on the user side) are:

* handover execution -- ``SendHandoverRequest``, ``SendHandoverRequestAck``,
  ``SendHandoverPreparationFailure``, ``SendSnStatusTransfer``, ``SendUeContextRelease``,
  ``SendHandoverCancel``;
* user-data forwarding -- ``SendUeData`` (X2-U);
* Self-Organizing Network (SON) support -- ``SendLoadInformation``, ``SendResourceStatusUpdate``.

The corresponding X2-AP messages are encoded by the ``NrEpcX2*Header`` classes (e.g.
``NrEpcX2HandoverRequestHeader``, ``NrEpcX2SnStatusTransferHeader``,
``NrEpcX2UeContextReleaseHeader``, ``NrEpcX2HandoverCancelHeader``). The SON primitives
(``LoadInformation``, ``ResourceStatusUpdate``) are not used by the current RRC; they are provided so
that SON algorithms can be developed on top of them.

:numref:`fig-x2ap` shows the X2-AP message exchange between the source and target gNBs during an
X2-based handover (the SON primitives are included for completeness). The end-to-end, EPC-level view
of the same handover -- including the S1 path switch -- is in the :doc:`EPC <epc>` section.

.. _fig-x2ap:

.. figure:: figures/epc/x2ap.*
   :align: center

   X2-AP message exchange between two gNBs.

Handover and data forwarding
============================
During an X2 handover the source gNB transfers each AM bearer's PDCP sequence-number state to the
target with SN Status Transfer (see the :doc:`PDCP <pdcp-layer>` section) and forwards in-flight
downlink data to the target over X2-U: ``NrEpcX2::DoSendUeData`` adds a ``NrGtpuHeader`` with the
per-bearer forwarding TEID and sends it on the X2-U socket. The handover leaving/joining windows are
bounded by the ``HandoverLeavingTimeoutDuration`` and ``HandoverJoiningTimeoutDuration`` attributes
of ``NrGnbRrc``; there is no dedicated handover-failure recovery. The end-to-end handover procedure
(including inter-numerology handover and the BWP switch) is described in the
:doc:`RRC <rrc-layer>` section.

The NR-specific change on X2 is, as elsewhere, the QoS vocabulary: the bearer-setup item carries an
``NrQosFlow`` (with its 5QI) instead of an ``EpsBearer`` (with its QCI); the on-wire layout is
unchanged.
