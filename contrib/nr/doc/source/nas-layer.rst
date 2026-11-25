.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

NAS layer
*********
The Non-Access Stratum (NAS) layer was ported from the |ns3| 'LTE' module (``EpcUeNas`` became
``NrEpcUeNas``) and then adapted to NR. Rather than pointing to the LTE documentation, this section
describes the model as it stands in the 'NR' module and highlights what changed during the port.

The model implements only the **UE side** of the NAS, in the class ``NrEpcUeNas``. The NAS protocol
is specified for EPS in [TS24301]_ and for the 5G System in [TS24501]_; the model follows the
simplified EPS NAS inherited from LTE, adapted to the 5G QoS-flow model (see below). As in LTE, the
focus is the *NAS Active* state (equivalent to *EMM-Registered, ECM-Connected, RRC-Connected*): the
EMM and ECM sublayers are not modelled explicitly, and the UE NAS entity interacts with the core
network through the Access Stratum to reach a state grossly equivalent to *connected*. The following
real-world procedures are therefore intentionally **not** modelled: PLMN/CSG selection, the
idle-mode location-update and paging procedures, and authentication/security. The NAS multiplexes
uplink user packets onto the appropriate radio/EPS resource using a packet classifier (see below),
and activates the default and any dedicated resources as part of the attach procedure.

State machine
=============
``NrEpcUeNas`` keeps the same five-state enumeration ported from LTE (the values are unchanged):

.. code-block:: text

   OFF                 -- detached / disconnected
   ATTACHING           -- defined, reserved for the attach handshake
   IDLE_REGISTERED     -- defined, reserved for the registered-idle state
   CONNECTING_TO_EPC   -- defined, reserved for the connection setup
   ACTIVE              -- attached and connected (NAS Active)

In practice the implementation only ever transitions between ``OFF`` and ``ACTIVE``; the three
intermediate states are defined for completeness but are not entered by the current code. Every
transition is reported through the ``StateTransition`` trace source (old state, new state). The
class exposes no TypeId attributes.

:numref:`fig-nr-nas-states` shows the state machine. Only ``OFF`` and ``ACTIVE`` are ever entered by
the current code; the three intermediate states are drawn dashed to indicate that they are declared
but never reached.

.. _fig-nr-nas-states:

.. figure:: figures/nas/nr-nas-states.*
   :align: center

   ``NrEpcUeNas`` state machine (only OFF and ACTIVE are exercised).

Connection management
=====================
``Connect()`` starts the UE attach. In NR this no longer asks the Access Stratum to connect to a
specific cell directly; instead it calls the Access-Stratum primitive ``StartCellSelection()`` with
no arguments, which scans for a suitable cell **across all configured bandwidth parts (BWPs)** -- a
NR-specific change reflecting the multi-BWP UE. The two-argument ``Connect(cellId, arfcn)`` overload
is preserved for the case where the camped cell is forced. On a successful access-stratum
connection the NAS moves to ``ACTIVE``; on failure it retries immediately.

QoS-flow activation
===================
This is the central NR adaptation. The LTE NAS activated **EPS bearers** described by a
``TftClassifier`` (Traffic Flow Template); the NR NAS activates **5G QoS flows** described by QoS
rules:

* ``ActivateEpsBearer(EpsBearer, EpcTft)`` is replaced by
  ``ActivateQosFlow(NrQosFlow flow, Ptr<NrQosRule> rule)``. A QoS Flow Identifier (QFI) must be set
  on the rule before activation (the model aborts otherwise).
* The uplink classifier ``EpcTftClassifier m_tftClassifier`` is replaced by
  ``NrQosRuleClassifier m_qosRuleClassifier``, keyed by QFI.
* The pending-activation bookkeeping struct ``BearerToBeActivated{bearer, tft}`` becomes
  ``QosFlowToBeActivated{flow, rule}``, held in ``m_qosFlowsToBeActivatedList``.

The LTE-era ``m_bidCounter`` and its hard limit of *eleven* EPS bearers per UE are removed: the QFI
is taken directly from the rule, so the number of flows is bounded by the QFI space rather than by a
bearer-id counter.

:numref:`fig-nas-qos-flow` shows how a QoS flow is activated and used: activation requests are queued
before/at attach and installed into the uplink classifier once the NAS reaches ``ACTIVE``; uplink
packets are then classified to a QFI and passed down the Access Stratum, and a flow can later be torn
down through ``DeactivateQosFlow``.

.. _fig-nas-qos-flow:

.. figure:: figures/nas/qos-flow-activation.*
   :align: center

   QoS-flow activation, uplink classification and per-flow deactivation in ``NrEpcUeNas``.

Uplink data path
================
Upper-layer (IP) packets enter through ``Send(packet, protocolNumber)``. The NAS asks
``m_qosRuleClassifier`` to classify the packet, which returns an optional QFI; if no rule matches the
packet is dropped, otherwise it is forwarded with ``SendData(packet, qfi)`` down the Access Stratum.
Downlink packets delivered up by the Access Stratum are handed to the IP stack through the
``m_forwardUpCallback``.

NAS handling of unstructured PDU sessions
=========================================
Besides IP, the NAS can carry an **unstructured** PDU session ([TS23501]_, Section 5.6.1): a session
whose payload is a single non-IP network-layer protocol that the network does not parse. The IP path
above is unchanged, and the unstructured path runs beside it.

Activation reuses ``ActivateQosFlow``, but with a rule built by
``NrQosRule::Unstructured(protocolNumber)`` that holds no packet filter. Rather than being added to
the classifier, the flow is recorded in two maps, ``m_unstructuredQfiByProtocol`` and
``m_unstructuredProtocolByQfi``, keyed by the protocol and by the QFI.

On the **uplink**, ``Send(packet, protocolNumber)`` dispatches by protocol. An IPv4 or IPv6 packet
goes through the classifier as before, any other protocol is looked up in
``m_unstructuredQfiByProtocol`` and sent on the matching QFI, and an unknown protocol is dropped with
a warning rather than aborting.

On the **downlink** the payload is not IP, so the protocol cannot be read from the packet. The NAS
recovers it from the session the packet arrived on: ``NrAsSapUser::RecvData`` now carries the QFI of
that flow, and ``m_unstructuredProtocolByQfi`` maps it back to the protocol number handed up to the
node. An IP session reports a QFI absent from the map, so its protocol keeps being read from the
packet, exactly as before.

Both maps are cleared alongside the classifier on per-flow deactivation (``DoDeactivateQosFlow``) and
on connection release (``DoNotifyConnectionReleased``).

QoS-flow deactivation
=====================
NR adds an explicit teardown path that LTE did not have: the Access Stratum can ask the NAS to drop a
single flow through ``NrAsSapUser::DeactivateQosFlow(qfi)``, handled by ``DoDeactivateQosFlow(qfi)``,
which removes the matching rule from the classifier.

Behaviour on radio link failure / connection release
=====================================================
When RRC reports that the connection was released (for example after a radio link failure; see the
RLF discussion in the :doc:`PHY layer <phy-layer>` section), the access-stratum callback
``DoNotifyConnectionReleased`` runs. The NR implementation clears the whole QoS-rule classifier in a
single ``Clear()`` call, restores the flow list to be re-activated on the next connection from
``m_qosFlowsToBeActivatedListForReconnection``, and switches the NAS state to ``OFF`` by calling
``Disconnect()``. This preserves the LTE reconnection design (restore-then-reattach), adapted to the
QoS-flow model.

Service interfaces
==================
``NrEpcUeNas`` sits between the IP stack above and RRC below:

* Towards RRC it uses the **Access-Stratum SAP**: it holds a ``NrAsSapProvider*`` (calls *into* RRC:
  ``StartCellSelection``, ``Connect``, ``SendData``, ...) and exports a ``NrAsSapUser`` (callbacks
  *from* RRC: ``NotifyConnectionSuccessful``, ``NotifyConnectionFailed``,
  ``NotifyConnectionReleased``, ``RecvData`` (carrying the flow QFI in NR) and, new in NR,
  ``DeactivateQosFlow``).
* Towards the IP stack there is no templated SAP; coupling is the public API (``Send``,
  ``ActivateQosFlow``, ``Connect``, ``Disconnect``) plus the ``m_forwardUpCallback`` used to deliver
  received packets upward.
