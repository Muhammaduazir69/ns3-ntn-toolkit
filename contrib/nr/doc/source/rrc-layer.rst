.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

RRC layer
*********

We are still in the process of porting all documentation, but we have already
made significant changes to the ``NrRrcSap::MasterInformationBlock`` (MIB) to include the cell numerology.
The ``NrRrcSap::SystemInformationBlockType1`` (SIB1) message has also been
updated to include the 5G-NR ``ServingCellConfigCommon`` field, which contains:

* the numerology of the downlink BWP
* the numerology of the uplink carrier
* the ARFCN of the uplink carrier (the equivalent of ``frequencyInfoUL`` in [TS38331]_): for a TDD
  carrier it points at the carrier itself, while for an FDD cell it points at the dedicated
  UL-only carrier
* the number of symbols per slot
* the number of downlink control symbols
* the number of uplink control symbols
* the TDD pattern
* the RBG size (technically a flag, with the actual size inferred from the BWP bandwidth)

This information allows the simulator to configure the initial DL BWP and perform initial cell selection automatically.
As a result, UEs can now be placed directly into the simulation and will connect automatically to a nearby cell.
In case of radio link failure, they will also search for a new cell to reconnect to.
For FDD cells, the advertised uplink carrier also lets the UE derive its primary UL BWP (and the
corresponding bandwidth-part routing) at cell selection, so no manual ``PrimaryUlIndex`` or
``SetOutputLink()`` configuration is needed; ``NrHelper::AttachToGnb()`` performs the equivalent
derivation from the gNB configuration when the attachment bypasses system information decoding.

Handover, inter-numerology and bandwidth parts
==============================================

Figure :ref:`fig-x2-handover` shows the end-to-end X2-based handover procedure as modeled,
across the UE, the source gNB, the target gNB and the MME. The target cell's broadcast PHY
configuration (``ServingCellConfigCommon``) is carried inside the handover command, which is
what lets the UE re-tune to a target cell that uses a *different* numerology.

.. _fig-x2-handover:

.. figure:: figures/rrc/x2-handover.*
   :align: center

   Sequence diagram of the X2-based handover procedure

Handover support has been added, following the same architecture as LTE.
A UE is able to hand over between cells on the same carrier frequency (intra-frequency)
as well as between cells on different carrier frequencies (inter-frequency), including
cells configured with different numerologies and BWPs.
For inter-frequency handover, the gNB must be told which neighbour frequencies to
measure: ``NrGnbRrc::AddNeighbourMeasFrequency()`` registers a neighbour ARFCN (and its
bandwidth) before the cell is configured, which creates an inter-frequency measurement
object so that a connected UE measures and can be handed over to cells on that frequency.
The UE must also have a bandwidth part configured on each candidate carrier frequency,
as it re-tunes the corresponding BWP to the target cell upon handover. The NR PHY measures
all configured BWPs simultaneously, so no measurement gaps are required.
The measurement-driven inter-frequency handover is verified in the
``nr-inter-freq-handover`` test, which also asserts end-to-end downlink data continuity
both before and (for a sustained period) after the inter-BWP re-tune. All four
combinations are exercised and keep the user plane flowing across the re-tune:
same-numerology and inter-numerology, each under both the ideal and the real RRC protocol.
The UE PHY slot machine is re-stamped onto the target BWP timeline during the re-tune
(clamping stale slot/var-TTI boundaries and restarting the loop when the numerology
changes) so that the UE resumes scheduling cleanly on the target cell, and forwarded
X2-U user-plane packets that arrive outside the handover data-forwarding window are
dropped rather than aborting the simulation.

Inter-numerology handover requires the UE to re-tune the target BWP to the *target*
cell's numerology, TDD pattern and control-symbol layout. Two pieces make this work.
First, the target cell's broadcast PHY configuration (``ServingCellConfigCommon``) is
carried in the handover command (``RrcConnectionReconfiguration`` mobility control info),
so the UE configures the target BWP from the target cell rather than from the source
cell's last-decoded SIB1. The handover command also carries the target cell's UL carrier
(frequency, bandwidth and numerology) and its RACH configuration: the UE re-targets its
primary UL BWP to the carrier the target cell receives on, re-synchronizes that BWP's PHY
to the target cell, and applies the target RACH configuration to the corresponding MAC.
This supports handover between FDD cells whose UL/DL carrier roles are inverted with
respect to each other, exercised (with a different numerology on each of the four
carriers) by the ``nr-handover-rach-config`` test suite. On the target gNB side, the
non-contention preamble is reserved on the MAC of the carrier where the random access
actually arrives, which for FDD is the UL-only carrier rather than the primary one. Second, while connected (or mid-handover) the UE keeps every
candidate BWP tuned and can overhear a neighbour cell's periodic MIB on another carrier.
A MIB is a per-cell broadcast, so it is *routed to the BWP that is actually tuned to the
originating cell's carrier* rather than applied to whichever BWP happens to be primary.
The UE maintains a cell-to-carrier map (populated as cells are measured/synchronized) and
resolves the matching BWP via its ARFCN. Only the *serving* cell's MIB updates the serving
numerology/bandwidth; a neighbour's MIB configures its own measurement BWP so its SSB stays
decodable for RSRP measurement, and can never reconfigure the serving BWP. This routing is
correct by construction and replaces an earlier band-aid that simply discarded any
non-serving MIB while connected. Without it, the re-tuned BWP would keep (or be repeatedly
reset to) the source numerology, so DL data reception - and hence the DL-CQI feedback the
target gNB needs to schedule downlink - would never recover.

Single-serving-cell invariant and dual connectivity
###################################################

The connected UE has exactly **one** serving cell, reached through the primary DL/UL BWP.
The other tuned BWPs exist only to receive SSB and measure RSRP on neighbour carriers (and,
in the future, to act as secondary BWPs of the *same* serving cell). They never carry a
second data connection. Handover re-tunes that single primary BWP to the target cell using
the ``ServingCellConfigCommon`` carried in the handover command; it does not add a second
serving cell.

A connected UE can also switch its primary BWP between BWPs of the *same* serving cell at
runtime. The policy, ``NrUeRrc::EvaluateSameCellBwpSwitch``, runs on each UE measurement update:
it compares the per-carrier RSRP of the serving cell's carriers and, when another same-cell
carrier is stronger than the serving one by more than the ``BwpSwitchHysteresis`` margin
(default 3 dB), it triggers the switch and notifies the gNB. The mechanism,
``NrUeRrc::SwitchPrimaryBwpSameCell``, re-points the primary DL/UL index and re-binds the RNTI;
it is guarded so it can only move between BWPs tuned to the current serving cell (moving to a
different cell would be a handover, or - if kept simultaneously - dual connectivity). The gNB
mirrors the change (``NrUeManager::SetPrimaryBwp`` -> ``BwpManagerGnb``) so downlink scheduling
follows the UE to the new BWP. Re-applying the dedicated radio configuration and bearer mapping
to the new BWP is still left as a TODO.

Figures :ref:`fig-bwpSwitchSeq` and :ref:`fig-bwpSwitchFsm` show the switch sequence and the
decision logic. The end-to-end behaviour - that the cell does not change (it is an intra-cell
switch, not a handover) and that the gNB tracks the new primary BWP - is verified by the
``nr-same-cell-bwp-switch`` system test.

.. _fig-bwpSwitchSeq:

.. figure:: figures/rrc/bwp-same-cell-switch.*
   :align: center

   Same-cell primary bandwidth-part switch.

.. _fig-bwpSwitchFsm:

.. figure:: figures/rrc/bwp-same-cell-switch-fsm.*
   :align: center

   Same-cell bandwidth-part switch decision logic.

**Dual connectivity (DC) is out of scope and is future work.** DC - dual MAC/RLC stacks,
split bearers, a master node (MN) and secondary node (SN), and two simultaneous connections
to different cells - is intentionally not implemented. Even though the multi-BWP tuning and
per-cell MIB routing described above could provide some of the plumbing, adding DC would
violate the single-serving-cell invariant and require substantial additional machinery
(SN addition/release signalling, split-bearer PDCP, per-leg flow control); it is therefore
left as future work rather than built opportunistically.

The precoding matrix effect on interference is not accounted for when the transmitter
and receiver have different numerologies. CSI feedback currently requires the antenna ports,
maximum rank, and RI/PMI algorithm to be the same across gNBs.
Beamforming is not currently configured, using quasi-omni beam by default.
Tests were performed with TDD.

RSRP measurement range was adjusted to the 5G-NR range, as per 3GPP [TS38133]_.

This section describes in detail the different models supported and developed at the RRC layer.
The RRC model in the simulator provides the following functionality:

 - generation (at the gNB) and interpretation (at the UE) of System Information (in particular the Master Information Block and,
   only System Information Block Type 1 and 2)
 - RRC connection establishment procedure
 - RRC reconfiguration procedure, supporting the following use cases:
   + reconfiguration of the PHYs
   + reconfiguration of UE measurements
   + data radio bearer setup
 - RRC measurements
 - idle cell selection
 - handover
 - radio link failure


The RRC model is divided into the following components:

 - the RRC entities `NrUeRrc` and `NrGnbRrc`, which implement the state machines of the RRC entities respectively at the UE and the gNB;
 - the RRC SAPs `NrUeRrcSapProvider`, `NrUeRrcSapUser`, `NrGnbRrcSapProvider`, `NrGnbRrcSapUser`, which allow the RRC
   entities to send and receive RRC messages and information elements;
 - the RRC protocol classes `NrUeRrcProtocolIdeal`, `NrGnbRrcProtocolIdeal`, `NrUeRrcProtocolReal`, `NrGnbRrcProtocolReal`,
   which implement two different models for the transmission of RRC messages.

Additionally, the RRC components use various other SAPs in order to
interact with the rest of the protocol stack.

UE RRC State Machine
====================

In Figure :ref:`fig-nr-ue-rrc-states` we represent the state machine
as implemented in the RRC UE entity.

.. _fig-nr-ue-rrc-states:

.. figure:: figures/rrc/nr-ue-rrc-states.*
   :scale: 60 %
   :align: center

   UE RRC State Machine

Radio link failure (RLF) detection and the subsequent re-establishment or cell
reselection are modeled; see the radio-link-failure description above and the
``nr-rlf-*`` test suites.

gNB RRC State Machine
=====================

The gNB RRC maintains the state for each UE that is attached to the cell. From an implementation point of view, the state of each UE is
contained in an instance of the UeManager class. The state machine is represented in Figure :ref:`fig-nr-gnb-rrc-states`.

.. _fig-nr-gnb-rrc-states:

.. figure:: figures/rrc/nr-gnb-rrc-states.*
   :scale: 70 %
   :align: center

   gNB RRC State Machine for each UE


Broadcast of System Information
===============================

System information blocks are broadcasted by gNB to UEs at predefined time intervals, adapted from Section 5.2.1.2 of [TS36331]_. The supported system
information blocks are:

 - Master Information Block (MIB)
      Contains parameters related to the PHY layer, generated during cell
      configuration and broadcasted once per radio frame (every 10 ms, at the
      beginning of the frame) as a control message.

 - System Information Block Type 1 (SIB1)
      Contains information regarding network access, broadcasted once per radio
      frame at the middle of the frame (subframe 5) as a control message. Not
      used in manual attachment method. UE must have decoded MIB before it can
      receive SIB1.

 - System Information Block Type 2 (SIB2)
      Contains UL- and RACH-related settings, scheduled to transmit via RRC
      protocol 16 ms after cell configuration, and then repeats every 80 ms
      (configurable through the `NrGnbRrc::SystemInformationPeriodicity`
      attribute). UE must be camped to a cell in order to be able to receive
      its SIB2.

Reception of system information is fundamental for UE to advance in its lifecycle. MIB enables the UE to increase the initial DL bandwidth to
the actual operating bandwidth of the network. SIB2 is required before the UE is allowed to switch to CONNECTED state.


Radio Admission Control
=======================

Radio Admission Control is supported by having the gNB RRC reply to an RRC CONNECTION REQUEST message sent by the UE with either
an RRC CONNECTION SETUP message or an RRC CONNECTION REJECT message, depending on whether the new UE is to be admitted or not. In the
current implementation, the behavior is determined by the boolean attribute ``ns3::NrGnbRrc::AdmitRrcConnectionRequest``.
There is currently no Radio Admission Control algorithm that dynamically decides whether a new connection shall be admitted or not.

Radio Bearer Configuration
==========================

Some implementation choices have been made in the RRC regarding the setup of radio bearers:

 - three Logical Channel Groups (out of four available) are configured for uplink buffer status report purposes, according to the following policy:

   + LCG 0 is for signaling radio bearers
   + LCG 1 is for GBR data radio bearers
   + LCG 2 is for Non-GBR data radio bearers


RRC sequence diagrams
=====================

In this section we provide some sequence diagrams that explain the
most important RRC procedures being modeled.

.. _sec-rrc-connection-establishment:

RRC connection establishment
############################

Figure :ref:`fig-rrc-connection-establishment` shows how the RRC Connection Establishment procedure is modeled, highlighting the role
of the RRC layer at both the UE and the gNB, as well as the interaction with the other layers.

.. _fig-rrc-connection-establishment:

.. figure:: figures/rrc/rrc-connection-establishment.*
   :align: center

   Sequence diagram of the RRC Connection Establishment procedure

There are several timeouts related to this procedure, which are listed in the
following Table :ref:`tab-rrc-connection_establishment_timer`. If any of these
timers expired, the RRC connection establishment procedure is terminated in
failure.

At the UE side, as per TS 38.331, if T300 timer is expired consecutively
*connEstFailCount* times on the same cell, it should perform an initial cell selection
again.

.. _tab-rrc-connection_establishment_timer:

.. table:: Timers in RRC connection establishment procedure

   +------------+----------+------------+-------------+----------+------------+
   | Name       | Location | Timer      | Timer       | Default  | When timer |
   |            |          | starts     | stops       | duration | expired    |
   +============+==========+============+=============+==========+============+
   | Connection | gNB      | New UE     | Receive RRC | 15 ms    | Remove UE  |
   | request    | RRC      | context    | CONNECTION  | (Max)    | context    |
   | timeout    |          | added      | REQUEST     |          |            |
   +------------+----------+------------+-------------+----------+------------+
   | Connection | UE RRC   | Send RRC   | Receive RRC | 100 ms   | Reset UE   |
   | timeout    |          | CONNECTION | CONNECTION  |          | MAC        |
   | (T300      |          | REQUEST    | SETUP or    |          |            |
   | timer)     |          |            | REJECT      |          |            |
   +------------+----------+------------+-------------+----------+------------+
   | Connection | gNB      | Send RRC   | Receive RRC | 150 ms   | Remove UE  |
   | setup      | RRC      | CONNECTION | CONNECTION  |          | context    |
   | timeout    |          | SETUP      | SETUP       |          |            |
   |            |          |            | COMPLETE    |          |            |
   +------------+----------+------------+-------------+----------+------------+
   | Connection | gNB      | Send RRC   | Never       | 30 ms    | Remove UE  |
   | rejected   | RRC      | CONNECTION |             |          | context    |
   | timeout    |          | REJECT     |             |          |            |
   +------------+----------+------------+-------------+----------+------------+


.. _tab-rrc-connection_establishment_counter:

.. table:: Counters in RRC connection establishment procedure

   +------------------+----------+------------------+-----------+---------+----------------------+------------------------+
   | Name             | Location | Msg              | Monitored | Default | Limit not            | Limit reached          |
   |                  |          |                  | by        | value   | reached              |                        |
   +==================+==========+==================+===========+=========+======================+========================+
   |ConnEstFailCount  | gNB MAC  | RachConfigCommon | UE RRC    | 1       | Counter incremented  | *See the beginning     |
   |                  |          | in SIB2, HO REQ  |           |         | (kept, not reset).   | of this section for a  |
   |                  |          | and HO Ack       |           |         | Invalidate the prev  | detailed explanation.* |
   |                  |          |                  |           |         | SIB2 msg and try     |                        |
   |                  |          |                  |           |         | random access        |                        |
   |                  |          |                  |           |         | with the same cell.  |                        |
   +------------------+----------+------------------+-----------+---------+----------------------+------------------------+


.. _sec-rrc-connection-reconfiguration:

RRC connection reconfiguration
##############################

Figure :ref:`fig-rrc-connection-reconfiguration` shows how the RRC
Connection Reconfiguration procedure is modeled for the case where
MobilityControlInfo is not provided, i.e., handover is not
performed.


.. _fig-rrc-connection-reconfiguration:

.. figure:: figures/rrc/rrc-connection-reconfiguration.*
   :align: center

   Sequence diagram of the RRC Connection Reconfiguration procedure


RRC protocol models
===================

As previously anticipated, we provide two different models  for the
transmission and reception of RRC messages: *Ideal*
and *Real*. Each of them is described in one of the following
subsections.

Ideal RRC protocol model
########################

According to this model, implemented in the classes and `NrUeRrcProtocolIdeal` and
`NrGnbRrcProtocolIdeal`, all RRC messages and information elements are transmitted between the gNB and the UE in an ideal fashion,
without consuming radio resources and without errors. From an implementation point of view, this is achieved by passing the RRC data
structure directly between the UE and gNB RRC entities, without involving the lower layers (PDCP, RLC, MAC, scheduler).

Real RRC protocol model
#######################

This model is implemented in the classes `NrUeRrcProtocolReal` and `NrGnbRrcProtocolReal` and aims at modeling the transmission of RRC
PDUs as commonly performed in real LTE and NR systems. In particular:

 - for every RRC message being sent, a real RRC PDUs is created
   following the ASN.1 encoding of RRC PDUs and information elements (IEs)
   specified in [TS36331]_. Some simplifications are made with respect
   to the IEs included in the PDU, i.e., only those IEs that are
   useful for simulation purposes are included. For a detailed list,
   please see the IEs defined in `nr-rrc-sap.h` and compare with
   [TS36331]_.
 - the encoded RRC PDUs are sent on Signaling Radio Bearers and are
   subject to the same transmission modeling used for data
   communications, thus including scheduling, radio resource
   consumption, channel errors, delays, retransmissions, etc.


Signaling Radio Bearer model
############################

We now describe the NR Signaling Radio Bearer model that is used for the
*Real* RRC protocol model.

 * **SRB0** messages (over CCCH):

   - **RrcConnectionRequest**: in real NR systems, this is an RLC TM SDU sent over resources specified in the UL Grant in the RAR (not
     in UL DCIs); the reason is that C-RNTI is not known yet at this stage. In the simulator, this is modeled as a real RLC TM RLC PDU
     whose UL resources are allocated by the scheduler upon call to SCHED_DL_RACH_INFO_REQ.

   - **RrcConnectionSetup**: in the simulator this is implemented as in real NR systems, i.e., with an RLC TM SDU sent over resources
     indicated by a regular UL DCI, allocated with SCHED_DL_RLC_BUFFER_REQ triggered by the RLC TM instance that is
     mapped to LCID 0 (the CCCH).

 * **SRB1** messages (over DCCH):

   - All the SRB1 messages modeled in the simulator (e.g., **RrcConnectionSetupCompleted**) are implemented as in real NR systems,
     i.e., with a real RLC SDU sent over RLC AM using DL resources allocated via Buffer Status Reports. See the RLC model
     documentation for details.

 * **SRB2** messages (over DCCH):

   - According to [TS38331]_, "*SRB1 is for RRC messages (which may include a piggybacked NAS message) as well as for NAS messages
     prior to the establishment of SRB2, all using DCCH logical channel*", whereas "*SRB2 is for NAS messages, using DCCH
     logical channel*" and "*SRB2 has a lower-priority than SRB1 and is always configured by E-UTRAN after security
     activation*". Modeling security-related aspects is not a requirement of the NR simulation model, hence we always use
     SRB1 and never activate SRB2.

Additionally, according to [TS38331]_, SRB3 is used for specific NR RRC messages when UE is in E-UTRA NR Dual Connectivity.
Since NR dual connectivity is not supported yet, SRB3 is also not implemented.



ASN.1 encoding of RRC IE's
==========================

The messages defined in RRC SAP, common to all Ue/gNB SAP Users/Providers, are transported in a transparent container to/from a Ue/gNB. The encoding format for the different Information Elements are specified in [TS36331]_, using ASN.1 rules in the unaligned variant. The implementation in nr has been divided in the following classes:

  * NrRrcAsn1Header : Inherits Asn1Header which is implemented in ns-3 LTE module and contains the encoding / decoding of basic ASN types. NrRrcAsn1Header contains the encoding / decoding of common IE's defined in [TS36331]_ and should be extended to support also commong IE's defined in [TS38331]_.

  * Rrc specific messages/IEs classes : A class for each of the messages defined in RRC SAP header


NrRrcAsn1Header : Common IEs
============================

As some Information Elements are being used for several RRC messages, this class implements the following common IE's:

  * SrbToAddModList

  * DrbToAddModList

  * LogicalChannelConfig

  * RadioResourceConfigDedicated

  * PhysicalConfigDedicated

  * SystemInformationBlockType1

  * SystemInformationBlockType2

  * RadioResourceConfigCommonSIB


Rrc specific messages/IEs classes
=================================

The following RRC SAP have been implemented:

  * RrcConnectionRequest

  * RrcConnectionSetup

  * RrcConnectionSetupCompleted

  * RrcConnectionReconfiguration

  * RrcConnectionReconfigurationCompleted

  * HandoverPreparationInfo

  * RrcConnectionReestablishmentRequest

  * RrcConnectionReestablishment

  * RrcConnectionReestablishmentComplete

  * RrcConnectionReestablishmentReject

  * RrcConnectionRelease
