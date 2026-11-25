.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

S1 interface
************
The S1 interface connects the gNB to the Evolved Packet Core (EPS architecture, [TS23401]_). Because
the 'NR' module implements an NR non-standalone (NSA) deployment over a ported LTE EPC, S1 is reused
as in LTE (a true 5G SA core would replace it with NG/NGAP and the N-interfaces; see the
:doc:`EPC <epc>` section). It has two planes:

* the **S1-MME** control plane between the gNB and the MME, carrying S1-AP signalling;
* the **S1-U** user plane between the gNB and the Serving Gateway (SGW), carrying user data over
  GTP-U.

The classes involved are ``NrEpcGnbApplication`` (the gNB-side S1-U bridge and S1-AP endpoint) and
``NrEpcMmeApplication`` (the MME).

Control plane (S1-AP)
=====================
The control plane is modelled by exchanging encoded S1-AP messages between the gNB and the MME (the
S1 Application Protocol, [TS36413]_). The service interface is ``NrEpcS1apSap``, split into a gNB side
(``NrEpcS1apSapGnb``) and an MME side (``NrEpcS1apSapMme``), with the following primitives:

* MME side -- ``InitialUeMessage``, ``ErabReleaseIndication``, ``InitialContextSetupResponse``,
  ``PathSwitchRequest``, ``UeContextReleaseComplete``;
* gNB side -- ``InitialContextSetupRequest``, ``PathSwitchRequestAcknowledge``,
  ``UeContextReleaseCommand``.

The MME sends a UE Context Release Command when another core network serves the UE (Cancel
Location, see the :doc:`EPC <epc>` section): the gNB application asks the RRC to release the UE
(``NrEpcGnbS1SapUser::UeContextReleaseCommand()``), which releases its RRC connection if the RNTI
still identifies that UE, and answers with a UE Context Release Complete in any case.

Internally the gNB RRC talks to the gNB application through a second SAP,
``NrEpcGnbS1SapProvider`` / ``NrEpcGnbS1SapUser``. The NR adaptation replaces the LTE bearer
vocabulary with QoS flows here: the release indication carries a QFI
(``DoReleaseIndication(imsi, rnti, qfi)``), the handover path-switch item is ``FlowToBeSwitched``
(``{qfi, teid}``, formerly ``ErabToBeSwitched``), and the data-radio-bearer setup request carries a
``NrQosFlow`` plus its QFI. The E-RAB-level QoS parameters exchanged over S1-AP are an ``NrQosFlow``
rather than an ``EpsBearer``.

The QFI/E-RAB-ID numbering performed by the MME (``NrEpcMmeApplication::AddFlow``) is documented in
the :doc:`EPC <epc>` section and is not repeated here.

:numref:`fig-s1ap` summarizes the S1-AP message exchange across the attach, handover-completion and
release procedures. These primitives are direct SAP calls between the gNB and MME applications and are
not serialized on the wire.

.. _fig-s1ap:

.. figure:: figures/epc/s1ap.*
   :align: center

   S1-AP (S1-MME) message exchange between the gNB and the MME.

User plane (S1-U)
=================
S1-U encapsulates the user's end-to-end IP packets in GTP-U (GTPv1-U, [TS29281]_) over UDP/IP (UDP
port 2152), exactly as in a real EPC. The GTP-U header is ``NrGtpuHeader``, which carries the PDU
Session Container extension header (TS 38.415) with the QFI of the packet in both directions, as on the
N3 interface of a 5G core network: 16 bytes in all. The SGW adds it on the downlink, and the gNB on the
uplink; the gNB maps the downlink packets to their QoS flow with it. The QoS of an EPS bearer is
**not** enforced on the S1-U link; link over-provisioning is assumed.

The main NR-specific change is on the data path between the gNB application and the radio NetDevice.
Where LTE tagged packets with an ``EpsBearerTag`` (carrying a bearer id), NR uses ``NrQosFlowTag``
carrying the **QFI** (``NrEpcGnbApplication`` adds it on the downlink toward the NetDevice and reads
it back on the uplink). The TEID lookup maps are correspondingly re-keyed by QFI:
``m_rqfiTeidMap`` (RNTI -> QFI -> TEID) on transmit and ``m_teidRqfiMap`` (TEID -> {RNTI, QFI}) on
receive.

The S1-U TEID is **not** allocated by the gNB: it is received in the GTP-C Create Session Response
(over S11/S5) and merely stored by ``NrEpcGnbApplication``. How the SGW and PGW assign and exchange
the various TEIDs -- and the fact that they are independent of the QFI/DRBID/LCID values -- is covered
in the :doc:`EPC <epc>` and :doc:`S5 <s5-interface>` / :doc:`S11 <s11-interface>` sections.
