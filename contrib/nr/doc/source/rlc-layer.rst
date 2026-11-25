.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

RLC layer
*********
The Radio Link Control (RLC) layer was ported from the |ns3| 'LTE' module (``LteRlc`` became
``NrRlc``) and then adapted to NR. The model follows the RLC specification originally given for LTE
in [TS36322]_ (the NR counterpart is [TS38322]_); the data-transfer procedures modelled here are
common to both. Rather than pointing to the LTE documentation, this section describes the layer as it
stands in the 'NR' module and notes what changed during the port.

The model provides four RLC entity types, sitting between PDCP/RRC (above) and the MAC (below):

* **TM** -- Transparent Mode: no RLC header is added and SDUs are passed through whole. Used by RRC
  to carry control-plane signalling (e.g. broadcast/common channels), not by PDCP.
* **UM** -- Unacknowledged Mode: segmentation/concatenation, reordering and reassembly, but **no**
  retransmissions and no STATUS PDUs.
* **AM** -- Acknowledged Mode: adds an ARQ protocol (ACK/NACK, STATUS PDUs, retransmission) on top of
  the UM-style segmentation and reassembly, providing reliable, in-order delivery.
* **SM** -- Saturation Mode (``NrRlcSm``, a non-standard model): accepts **no** SDUs from upper
  layers and instead generates RLC PDUs on demand whenever the MAC signals a transmission
  opportunity, simulating an always-full buffer. It is used for radio-only scenarios without an
  EPC/IP stack -- useful for exercising non-real-time QoS schedulers, but not delay-critical ones,
  since it provides no meaningful head-of-line delay information.

Architecture and service interfaces
===================================
Each RLC entity provides the RLC service interface to the upper layer and uses the MAC service
interface below.

The **RLC service interface** is divided into two parts, and both UM and AM provide the same one:

* the ``NrRlcSapProvider`` part is provided by the RLC layer and used by the upper PDCP layer;
* the ``NrRlcSapUser`` part is provided by the upper PDCP layer and used by the RLC layer.

with primitives:

* ``NrRlcSapProvider::TransmitPdcpPdu`` -- PDCP sends a PDCP PDU down to the RLC entity;
* ``NrRlcSapUser::ReceivePdcpPdu`` -- the RLC entity delivers a PDCP PDU up to the PDCP entity.

The **MAC service interface** is likewise split:

* the ``NrMacSapProvider`` part is provided by the MAC layer and used by the upper RLC layer;
* the ``NrMacSapUser`` part is provided by the upper RLC layer and used by the MAC layer.

with primitives:

* ``NrMacSapProvider::TransmitPdu`` -- the RLC entity sends an RLC PDU down to the MAC;
* ``NrMacSapProvider::BufferStatusReport`` -- the RLC entity reports the size of its pending buffers
  to the MAC (in LTE this primitive was named ``ReportBufferStatus``; see
  :ref:`sec-rlc-nr-differences`);
* ``NrMacSapUser::NotifyTxOpportunity`` -- the MAC notifies the RLC entity of a transmission
  opportunity;
* ``NrMacSapUser::ReceivePdu`` -- the MAC delivers an RLC PDU up to the receiving RLC entity.

RLC UM
======
An RLC PDU is constructed either from complete RLC SDUs (one or more SDUs concatenated within the same PDU)
or from segmented RLC SDUs (a single SDU divided into multiple parts, with each part carried in a separate RLC PDU).

.. _fig-rlcUmTx-conSeg:

.. figure:: figures/rlc/rlc_um_pdu_tx_conSeg.*
   :align: center
   :scale: 28%

   Example of concatenation and segmentation in RLC UM layer.

During transmission, an RLC header is added to form an RLC PDU. The RLC UM header contains the following fields:

.. code-block:: text

    SN = Sequence Number
    FI = Framing Info, indicates whether the PDU starts and/or ends with an RLC SDU:
        00 = starts and ends with SDU
        01 = starts with SDU but does not end
        10 = does not start but ends with SDU
        11 = neither starts nor ends with SDU
    E = Extension Bit (0: no more LIs; 1: at least one more LI follows)
    LI = Length Indicator (length in bytes of an SDU or SDU segment within the PDU)

.. _fig-rlcUmTx:

.. figure:: figures/rlc/rlc_um_pdu_tx.*
   :align: center
   :scale: 26%

   Example of RLC PDUs creation.

Sequence Number (SN) space is circular (10-bit SN): 0, 1, …, 1023, 0, 1, 2, …., 1022, 1023, 0, 1, ....

.. _fig-rlcUmSn:

.. figure:: figures/rlc/rlc_um_sn.*
   :align: center
   :scale: 28%

   10-bit Sequence Number.


At the receiver, when an RLC PDU arrives at ``DoReceivePdu``, the first step is to check whether the packet is valid.
If it is valid, it is placed in the reception buffer ``m_rxBuffer``; otherwise, it is discarded.
Next, the code verifies whether the received SN falls within the current reception window. This window defines the range
of SNs that the receiver currently considers valid, allowing it to distinguish between old and new SNs. The maximum size
of the reception window is 512, which is half of the total 1024 possible values for a 10-bit SN (2^{10} = 1024).
The reception window is defined by the interval [VR(UR),VR(UH)), i.e. VR(UR) <= SN < VR(UH):

     * VR(UR): The oldest RLC SN that has not yet been received (lower edge of the window).
     * VR(UH): The newest RLC SN + 1 received within the current window.

If VR(UR) ≠ VR(UH), there are missing PDUs, then t-reordering timer is activated, and VR(UX) is updated, which represents
the upper edge of the reordering window. VR(UX) takes the value of VR(UH) at the moment reordering timer starts.

   * PDUs with VR(UR) ≤ SNs ≤ VR(UX) are included in the current reordering window and will be delivered together
     once t-Reordering expires.
   * PDUs with SN > VR(UX) are buffered for future reordering operations.

.. _fig-rlcUmRxWindow:

.. figure:: figures/rlc/rlc_um_rx_window.*
   :align: center
   :scale: 28%

   RLC UM reception window.

If the RLC PDU is not discarded, the first step is to check whether the current SN is within the reception window.
If it is not, VR(UH) is updated as: VR(UH) = SN + 1. Next, the code checks if there are any PDUs within the window that
have become old due to the window sliding. Any such old PDUs must be reassembled and delivered to the higher layers using
``ReassembleAndDeliver``. Finally, it verifies whether VR(UR) is still within the reception window. If it is not, VR(UR)
must be adjusted (temporary VR(UR)). If the PDU corresponding to the new temporary VR(UR) is already present in  ``m_rxBuffer``,
``ReassembleAndDeliver`` is called again. After this, VR(UR) is updated to the first SN that is not present in the buffer.

When ``ReassembleAndDeliver`` is called, the received FI is checked to determine which part of the RLC SDU is going to be
delivered to PDCP. There are two states for this: WAITING_SO_FULL (waiting for the start of a SDU) and
WAITING_SI_SF (waiting for a non-start of a SDU).

.. code-block:: text

   Example: We receive an RLC PDU with FI = 00, so we remain in the WAITING_SO_FULL state. If there is no packet loss,
   the next PDU will have FI = 00 or FI = 01.
      * If FI = 00, the state remains WAITING_SO_FULL.
      * If FI = 01, the state transitions to WAITING_SI_SF.
      * If FI = 10 or FI = 11, this transition is invalid, and an assert will be triggered.

.. _fig-rlcUmRx:

.. figure:: figures/rlc/rlc_um_rx_state_machine.*
   :align: center
   :scale: 30 %

   RLC UM reception state machine diagram.

After checking the FI, the RLC PDU is delivered to PDCP for processing.

The 'NR' UM entity adds an optional **out-of-order delivery** mode (the ``OutOfOrderDelivery``
attribute), the ``rlc-OutOfOrderDelivery`` of TS 38.322: the complete SDUs of a received PDU are
delivered at once, whatever its SN, and the segments of an SDU split across PDUs are held by SN
until all of them are received, in any order, when the SDU is reassembled and delivered. The
reordering window then only tracks which SNs were received: segments that can no longer be
completed, because a PDU carrying another segment of their SDU fell behind VR(UR) without being
received, are discarded. The port also hardens t-Reordering
handling so that a partially reassembled SDU that is stranded mid-reassembly when the timer expires
(or after a 10-bit SN wrap) is discarded rather than corrupted.

RLC AM
======
RLC Acknowledged Mode (``NrRlcAm``) adds an Automatic Repeat reQuest (ARQ) protocol on top of the
same segmentation/concatenation and reassembly used by UM, providing reliable, in-order delivery.
The transmitter keeps a sliding window ``VT(A) .. VT(MS)`` (window size 512), assigns ``SN = VT(S)``
to each new PDU, and stores a copy in ``m_txedBuffer`` (indexed by SN) until it is acknowledged.

Status reporting drives the ARQ loop. When the receiver detects a gap (its t-Reordering timer
expires) it requests a STATUS PDU; the transmitter, unless its t-StatusProhibit timer is running,
builds one that ACKs up to ``ACK_SN`` and NACKs every SN that is missing or incomplete in the
``VR(R) .. VR(MS)`` range. Polling is requested periodically (after ``PollPdu`` PDUs or ``PollByte``
bytes, or when the window stalls): it sets the poll bit and (re)starts the t-PollRetransmit timer;
if that timer expires with no STATUS PDU, the unacknowledged PDUs are moved back for retransmission.

On receiving a STATUS PDU, an ACK frees the PDU and advances ``VT(A)``, while a NACK moves the PDU
from ``m_txedBuffer`` to ``m_retxBuffer`` so it is retransmitted at the next transmission
opportunity, incrementing its retransmission counter. Figure :ref:`fig-rlcAmArq` summarizes this
per-PDU lifecycle.

.. _fig-rlcAmArq:

.. figure:: figures/rlc/rlc_am_arq.*
   :align: center

   RLC AM transmit/ARQ state machine for a single PDU.

When a PDU's retransmission counter reaches ``maxRetxThreshold`` (default 5), the entity raises the
``SetMaxRetxReachedCallback`` indication once. With the (default-off) ``NrUeRrc::RlcMaxRetxTriggersRlf``
attribute enabled, ``NrUeRrc`` then declares a radio link failure, as reaching the RLC maximum number
of retransmissions is an RLF trigger per [TS38331]_ (5.3.10.3).

RLC AM uses four timers: t-PollRetransmit (default 20 ms), t-Reordering (10 ms), t-StatusProhibit
(10 ms) and the buffer-status-report timer (20 ms). Reassembly at the receiver follows the same
WAITING_S0_FULL / WAITING_SI_SF state machine as UM (see :ref:`fig-rlcUmRx`). The transmitter,
status-report encoding and the max-retx indication are exercised by the ``nr-rlc-am-transmitter``,
``nr-rlc-am-e2e`` and ``nr-rlc-header`` test suites.

RLC TM
======
The Transparent Mode entity (``NrRlcTm``) adds no RLC header: an SDU handed down through
``TransmitPdcpPdu`` is buffered, its size is reported to the MAC, and on a transmission opportunity
large enough to carry the head-of-line SDU it is dequeued and passed to the MAC as a single RLC PDU
unchanged. The buffer size reported to the MAC is simply the sum of the queued SDU sizes (no header
overhead). TM is used to carry RRC signalling on common/broadcast channels rather than user data.

RLC SM (saturation mode)
========================
The Saturation Mode entity (``NrRlcSm``) is a non-standard model that does not accept SDUs from upper
layers. Instead it always reports a non-empty buffer to the MAC and, on every transmission
opportunity, fabricates an RLC PDU of exactly the granted size. This emulates a user with an
infinite backlog, which is convenient for scheduler and PHY-layer studies that do not need an
EPC/IP stack. Because the generated traffic has no real head-of-line delay, SM is suitable for
throughput/non-real-time scheduler evaluation but not for delay-critical schedulers. ``NrRlcSm`` is
declared together with the base class in ``nr-rlc.h``.

.. _sec-rlc-nr-differences:

Differences from the LTE RLC
============================
Beyond the mechanical ``Lte`` -> ``Nr`` renaming, the NR port introduced the following substantive
changes:

* **Max-retransmission / RLF hook (new).** ``NrRlc`` gains ``SetMaxRetxReachedCallback``; the AM
  entity invokes it once when a PDU reaches ``maxRetxThreshold``, which ``NrUeRrc`` can turn into a
  radio link failure (see RLC AM above). This implements a function the LTE model listed as
  unsupported.
* **UM out-of-order delivery (new).** The ``OutOfOrderDelivery`` attribute, which reassembles the
  SDUs from the segments held by SN in any arrival order, plus t-Reordering stale-SDU discard
  hardening (see RLC UM above).
* **MAC-SAP primitive renames.** LTE's ``ReportBufferStatus`` / ``ReportBufferStatusParameters`` are
  named ``BufferStatusReport`` / ``BufferStatusReportParameters`` in NR, the AM attribute
  ``ReportBufferStatusTimer`` is ``BufferStatusReportTimer``, and a periodic-BSR flag
  (``expBsrTimer``) is propagated from RLC to the MAC.
* **Graceful handling of an undersized transmission opportunity (AM).** Where LTE asserted and
  halted, NR logs an error and skips the opportunity.
* **Sequence-number namespacing.** ``SequenceNumber10`` now lives in ``namespace nr``
  (``nr::SequenceNumber10``); the bit width is unchanged (still 10-bit in both UM and AM).

The RLC service interface (``TransmitPdcpPdu`` / ``ReceivePdcpPdu``), the trace sources (``TxPDU``,
``RxPDU``, ``TxDrop``) and the per-mode timer defaults are otherwise unchanged from the LTE port. The
PDCP-discard / packet-delay-budget machinery used by :doc:`PDCP <pdcp-layer>` is also inherited
unchanged from LTE.
