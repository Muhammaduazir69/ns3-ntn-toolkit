.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

MAC layer
*********
This section describes the different models supported and developed at MAC layer.

Resource grouping
=================

5G-LENA previously required setting the number of RBs per RBG manually for each MAC. Until the RRC update
is finalized, that number will be set according to the ``NrHelper::NumRbPerRbg`` attribute.
It is planned to eventually compute and assign that based on the BWP bandwidth, according to 3GPP TS 38.214
nominal RBG size P for RA Type 0. The computation itself is already performed by the function
``int nr::NumRbsPerRbg(int numRbs);``.


Resource allocation model: OFDMA and TDMA
=========================================
The 'NR' module supports variable TTI DL-TDMA and DL-OFDMA with a single-beam capability. In the UL direction, we support TDMA with variable TTI only. The single-beam capability for DL-OFDMA implies that only a single receive or transmit beam can be used at any given time instant. The variable TTI means that the number of allocated symbols to one user is variable, based on the scheduler allocation, and not fixed as was happening in LTE. Of course, LTE-like behaviors can be emulated through a scheduler that always assigns all the available symbols.

In OFDMA, under the single-beam capability constraint, UEs that are served by different beams cannot be scheduled at the same time. But we do not have any limitations for what regards UEs that are served by the same beam, meaning that the simulator can schedule these UEs at the same time in the frequency domain. The implementation, as it is, is compatible with radio-frequency architectures based on single-beam capability, which is one of the main requirements for operation in bands with a high center carrier frequency (mmWave bands). Secondly, it allows meeting the occupied channel bandwidth constraint in the unlicensed spectrum. Such restriction, for example, is required at the 5 GHz and 60 GHz bands. The scheduler meets the requirements by grouping UEs per beam and, within a TTI, only UEs that are served by the same gNB beam would be allowed to be scheduled for DL transmission in different RBGs.

For decoding any transmission, the UE relies on a bitmask (that is an output of the scheduler) sent through the DCI. The bitmask is of length equal to the number of RBGs, to indicate (with 1's) the RBGs assigned to the UE. This bitmask is translated into a vector of assigned RB indices at PHY. In NR, an RBG may encompass a group of 2, 4, 8, or 16 RBs [TS38214]_ Table 5.1.2.2.1-1, depending on the SCS and the operational band. a TDMA transmission will have this bitmask all set to 1, while OFDMA transmissions will have enabled only the RBG where the UE has to listen.

An implementation detail that differentiates the 'NR' module from the 'mmWave' module, among the others, is that the scheduler has to know the beam assigned by the physical layer to each UE. Two parameters, azimuth and elevation, characterize the beam in case of CellScanBeamforming. This is only valid for the beam search beamforming method (i.e., for each UE, the transmission/reception beams are selected from a set of beams or codebook).


Scheduler
=========
In the 'NR' module, we have introduced schedulers for OFDMA and TDMA-based access
with variable TTI under single-beam capability. The main output of
a scheduler functionality is a list of DCIs for a specific slot,
each of which specifies four parameters: the transmission starting
symbol, the duration (in number of symbols) and an RBG bitmask,
in which a value of 1 in the position x represents a transmission
in the RBG number x.
The current implementation of schedulers API follows the FemtoForum specification
for LTE MAC Scheduler Interface [ff-api]_ , but
can be easily extended to be compliant with different industrial interfaces.

The core class of the NR module schedulers design is ``NrMacSchedulerNs3``.
This class defines the core scheduling process and
splits the scheduling logic into the logical blocks. Additionally, it implements
the MAC schedulers API, and thus it decouples a
scheduling logic from any specific MAC API specification. These two features
facilitate and accelerate the introduction of the new
schedulers specializations, i.e., the new schedulers only need to implement a
minimum set of specific scheduling functionalities
without having to follow any specific industrial API.

The scheduling process assigns the resources for active DL and UL
flows and notifies the MAC of the scheduling decision
for the corresponding slot. Currently, since in the uplink the TDMA is used, the
scheduling for UL flows
is designed to support only TDMA scheduling. On the
other hand, the
scheduling for DL flows is designed to allow both, TDMA and OFDMA,
modes for the downlink. The scheduling functions are
delegated to subclasses to perform the allocation of symbols among beams
(if any), allocation of RBGs in time/frequency-domain
among active UEs by using specific scheduling algorithm (e.g.,
round robin, proportional fair, etc.), and finally, the construction
of corresponding DCIs/UCIs. For example, TDMA scheduling can be easily
implemented by skipping the first step of allocating symbols
among beams and by fixing the minimum number of assignable RBGs to the total
number of RBGs. To obtain true TDMA-based access with
variable TTI, it is then necessary to group allocations for the same UE in
one single DCI/UCI which is the last step.

All schedulers in the NR module (OFDMA and TDMA) derive from ``NrMacSchedulerNs3``:
``NrMacSchedulerTdma`` extends it, and ``NrMacSchedulerOfdma`` in turn extends
``NrMacSchedulerTdma``. The HARQ retransmissions for the DL and the UL are handled
by the companion class ``NrMacSchedulerHarqRr``, invoked by ``NrMacSchedulerNs3``.
Currently, the NR module offers the scheduling of the HARQ retransmissions
in a round robin manner.

An overview of the different phases that the OFDMA schedulers follow are:

1) BSR and CQI messages processing. The MCS is computed by the AMC model
for each user based on the CQIs for the DL or SINR measurements
for the UL data channel. The MCS and BSR of each user are stored in a
structure that will be later read to determine UE capabilities and needs.
The procedure for estimating the MCS and determining the minimum number of
RBs is common to all the OFDMA-based schedulers that we may derive.

2) Upon being triggered by the MAC layer, the scheduler prepares a slot
indication. As a first step, the total number of active flows is calculated
for both UL and DL. Then, the UL is processed, and then the DL. This
requirement comes from the fact that UL and DL have, in most cases,
different delays. This delay is defined as the number of the slots that have
to pass between the moment in which the decision is taken, and the moment that
such decision is traveling in the air. The default delay parameters are 2 slots
for DL and 4 slots for UL: therefore, UL data can be penalized by the higher delay,
and hence has to be prioritized in some way when preparing the slot. For this reason,
the scheduler is also taking UL and DL decision for the same slot in different moments.

3) The UL decisions are not considered for the slot indicated by the MAC layer,
but for a slot in the future. These involve firstly any HARQ retransmission that
should be performed, for instance when the previous transmission has been NACKed.
The requirement for retransmitting any piece of data is to have enough space (indicated
by the number of RBG). This is because, while the retransmission does not need to
start at the same symbol and RB index as the previous transmission of the same TB,
it does need the same number of RBGs and MCS, since an adaptive HARQ scheme (where
the re-transmission can be scheduled with a different MCS) is not implemented.
Retransmissions that fit side by side in frequency share their symbols. With the OFDMA
schedulers, UL data is first scheduled on the RBGs that the retransmissions leave free
over their symbols. If all the symbols are used by the UL retransmissions, the
scheduling procedure ends here. Otherwise, UL data is scheduled, by assigning the
remaining resources (or less) to the UEs that have data to transmit. See
:ref:`HarqRetxScheduling` for the details. The total number of symbols reserved for UL data is
then stored internally along with the slot number to which these allocations are
referred, and the procedure for UL ends here.

4) The procedure for DL allocations is started, relative to the slot indicated by
the MAC layer. The number of symbols previously given for UL data in the current
slot has to be considered during the DL phase. Before evaluating what data can
be scheduled, that number is extracted from the internal storage, and the DL phase
can continue only if there are available symbols not used by the UL phase. If it
is the case, then, the symbols can be distributed by giving priority to the HARQ
retransmissions, and then to the new data, according to different metrics. As in the
UL, the OFDMA schedulers first place new DL data on the RBGs left free by the
retransmissions, over their symbols.

The base class for OFDMA schedulers is ``NrMacSchedulerOfdma``.
In the downlink, such class and its subclasses perform
OFDMA scheduling, while in the uplink they leverage some of the subclasses of
``NrMacSchedulerTdma`` class that implements TDMA scheduling.

The OFDMA scheduling in the downlink is composed of the two scheduling levels:
(1) the scheduling of the symbols per beam (time-domain level), where scheduler
selects a number of consecutive OFDM symbols in a slot to assign to a specific
beam, and (2) the scheduling of RBGs per UE in a beam, where the scheduler
determines the allocation of RBGs for the OFDM symbols of the corresponding
beam (frequency-domain level).

The time-domain scheduling (1) of the symbols per beam can be performed with different policies:

* Load-Based (attribute ``NrMacSchedulerOfdma::SymPerBeamType`` set to ``LOAD_BASED``)

    * The calculation of load is based on the BSRs and the assignment of symbols per beam
      is proportional to the load.

* Round-Robin (attribute ``NrMacSchedulerOfdma::SymPerBeamType`` set to ``ROUND_ROBIN``)

    * Symbols are assigned one at a time, to the active beam in the front of a circular queue,
      which is then moved to the end of the queue.

* Proportional-Fair (attribute ``NrMacSchedulerOfdma::SymPerBeamType`` set to ``PROPORTIONAL_FAIR``)

    * Symbols are assigned one at a time, by applying the proportional-fair policy to the average TBS
      of UEs in a given beam. Mean TBS bytes added by each additional symbol are then removed from total byte
      load of a beam, possibly reducing the number of remaining users in the beam for the next symbols
      in a given slot.


The frequency-domain scheduling (2) of RBGs can be performed using different scheduling algorithms
(round robin, proportional fair, max rate, QoS, RL-based, random). Which decide how RBGs are allocated among
different UEs associated to the same beam. Multiple fairness checks can be ensured in between
each level of scheduling - the time domain and the frequency domain. For instance, a UE that
already has its needs covered by a portion of the assigned resources can free these
resources for others to use.

The NR module currently offers several specializations of the OFDMA schedulers.
These specializations perform the downlink scheduling in a round robin (RR), proportional fair
(PF), max rate (MR), QoS, AI RL-based, or random manner, respectively, as explained in the following:

* RR: the available RBGs are divided evenly among UEs associated to that beam.
* PF: the available RBGs are distributed among the UEs according to a PF metric that considers the actual rate (based on the CQI) elevated to :math:`\alpha` and the average rate that has been provided in the previous slots to the different UEs. Changing the α parameter changes the PF metric. For :math:`\alpha=0`, the scheduler selects the UE with the lowest average rate. For :math:`\alpha=1`, the scheduler selects the UE with the largest ratio between actual rate and average rate.
* MR: the total available RBGs are distributed among the UEs according to a maximum rate (MR) metric that considers the actual rate (based on the CQI) of the different UEs.
* QoS: the available RBGs are distributed among the UEs based on their traffic requirements by considering QoS profile of each QoS flow (the 5QI information, such as the resource type, the priority level and the Packet Delay Budget (PDB), along with real-time measurements as provided at the MAC layer, i.e., the head-of-line delay (HOL) and the PF metric.
* AI RL-based: the available RBGs are distributed based on the RL model whose objective is to meet latency requirements. AI RL-based scheduler considers QoS profile of each QoS flow together with HOL delay.
* Random: the available RBGs are divided among UEs in a random manner to ensure that all UEs get assigned, with no clear preference to a particular UE. The generated interference is random in the power/time/frequency/spatial domains because of the random selection of UEs.

Each of these OFDMA schedulers is performing a load-based scheduling of
symbols per beam in time-domain for the downlink. In the uplink,
the scheduling is done by the TDMA schedulers.

The base class for TDMA schedulers is ``NrMacSchedulerTdma``.
This scheduler performs TDMA scheduling for both, the UL and the DL traffic.
The TDMA schedulers perform the scheduling only in the time-domain, i.e.,
by distributing OFDM symbols among the active UEs. 'NR' module offers several
specializations of TDMA schedulers: RR, PF, MR, QoS, AI and the random where
the scheduling criteria is the same as in the corresponding OFDMA
schedulers, while the scheduling is performed in time-domain instead of
the frequency-domain, and thus the resources being allocated are symbols instead of RBGs.

After the UEs receive the DCIs containing their allocated resources,
they inform the RLC layer of the available transmission opportunities.
The RLC then selects which data to transmit next. Previously, the available bytes were distributed
evenly across logical channels. Now, the bytes are allocated using a shortest-job-first policy,
where logical channels with fewer bytes to transmit are served first. This approach reduces
the likelihood of data traffic being prioritized over control messaging.

Sub-band scheduling
===================

Since nr-4.0, sub-band CQI information can be used by the schedulers to avoid allocating interfered RBGs.
This can be achieved by changing the ``NrMacSchedulerNs3::McsCsiSource``.
The default value of this attribute is set to ``WIDEBAND_MCS``, which uses the wide-band CQI,
and the MCS derived from it, to schedule each UE RBG with the same priority.
To estimate the MCS of allocated RBGs based on the sub-band CQI information, set the attribute
to one of the following: ``AVG_MCS``, ``AVG_SPEC_EFF`` and ``AVG_SINR``.

* ``AVG_MCS``: averages the approximated MCS for a given sub-band CQI (note that MCS is wideband,
  and this is a rought estimate).
* ``AVG_SPEC_EFF``: averages the approximated spectral efficiency of allocated RBGs, then
  transforms it back to an MCS estimate.
* ``AVG_SINR``: averages the SINR of allocated RBGs, then compute the resulting MCS straight
  directly from the error models. It is the most accurate estimate.

Note that when sub-band CQI is used, the RBG allocated to the UE is the one that produces
the highest MCS.

.. caution::

    Also note that after a user is selected, according to the scheduler policy,
    the list of available RBGs is sorted by the user's sub-band CQIs.
    If the first RBG with the highest sub-band CQI is different than 0, it is then assigned to the user.
    However, if its sub-band CQI is 0, no resources are assigned to the user in this slot,
    and scheduling proceeds to the next user as defined by the scheduler policy.

    This can impact simulations that rely exclusively on PDSCH-based feedback.
    Such users will never be scheduled due to CQI 0, and thus cannot provide up-to-date
    feedback since channel and interference measurements require data transmission.
    **Recommendation**: Use CSI-RS and CSI-IM instead.

    If using PDSCH-only feedback, set ``NrMacSchedulerNs3::McsCsiSource=WIDEBAND_MCS``
    for wideband CQI-based scheduling, which continues allocating RBGs even with CQI 0.

Also note that from the nr-4.0 to nr-4.1 releases, an eviction factor and a guard rail were introduced
to prevent early termination of resource allocation when there was sufficient bandwidth available,
even at lower sub-band CQI levels.

The eviction factor reverts allocations that reduce the transport block size (TBS) by more than 1%.
The guard rail prevents scheduling resource block groups (RBGs) with sub-band CQI values equal to the
maximum sub-band CQI minus 4 (e.g., 15->11, or 4->0). This follows the 3GPP expectation for sub-band
CQI ranges relative to wideband CQI, controlled by ``ns3::NrPmSearch::SubbandCqiClamping``.

Without these adjustments, bad scheduling may occur, especially with large channels where a few sub-bands
have high CQI and many have low CQI. Low CQI sub-bands could be ignored, even if scheduling the entire
bandwidth would result in a larger TBS.

To better visualize this limitation, imagine the following sub-band CQI table:

+-----+---------------------------------------+
|     |              Sub-bands                |
+-----+---+---+---+---+---+---+---+---+---+---+
| CQI | 8 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
+-----+---+---+---+---+---+---+---+---+---+---+

The scheduler could allocate only sub-band 0 with CQI 8, achieving a TBS of 28 bits with MCS 14,
or allocate all RBGs to get a TBS of 30 bits with MCS 1.

In releases nr-4.0 and 4.1, only the single high-CQI sub-band with TBS 28 and MCS 14 would be allocated.
In nr-4.2, all RBGs are allocated with a TBS of 30 and MCS 1.

This example assumes ``ns3::NrPmSearch::SubbandCqiClamping`` is set to false and the range of sub-band CQI values
exceeds the wideband CQI by [-2, +1], which is non-standard, but adequate for demonstration purposes.

.. important::

   Evolution after scheduling each additional sub-band (since nr-4.2)

   **TBS:** 28->21->15->16->15->18->22->23->26->30

   **MCS:** 14->07->04->03->02->02->02->01->01->01

This behavior is enabled by an exhaustive solution estimating the maximum achievable TBS for each new allocation.
The estimate assumes all remaining free resources will be allocated to the current user alongside already
scheduled resources.

If this estimate is lower than a previously achieved TBS, the allocation reverts to the known maximum
and stops scheduling the user.

The test case ``NrSchedOfdmaMcsTestCase`` confirms this behavior works as expected.


.. _HarqRetxScheduling:

HARQ retransmission scheduling
==============================
HARQ retransmissions are scheduled before new data, by ``NrMacSchedulerHarqRr``. Since
adaptive HARQ is not implemented, a retransmission keeps the RBGs, the number of symbols,
the MCS and the TBS of its first transmission. Its RBGs and symbols are kept too, unless
they do not fit in the slot, in which case it is reshaped into the fewest symbols left that
hold its resources.
What the scheduler decides is where in the slot the retransmission goes, and what is done
with the resources that it leaves free. The following rules apply:

* A UL retransmission is placed on the symbols of a UL retransmission already scheduled
  in the slot when both have the same number of symbols, both UEs share a beam, and their
  RBGs do not overlap. Otherwise, it gets symbols of its own. When its symbols do not fit
  in the ones left, it is reshaped into the fewest of them that hold its resources, over
  the UL band, keeping its MCS and TBS. Only when even that does not fit is the HARQ
  feedback kept for the next slot.
* DL retransmissions of a beam are placed on the same symbols, each with its RBGs. A DL
  retransmission whose RBGs are taken by another one of its beam, or whose symbols do not
  fit in the ones left, is reshaped in the same way, into symbols of its own after the
  ones of every beam. With ``NrMacSchedulerHarqRr::ConsolidateHarqRetx``, every DL
  retransmission is reshaped into symbols of its own.
* With the OFDMA schedulers, the RBGs that the retransmissions of a beam leave free over
  their symbols are given to new data of the other UEs in that beam, for both UL and DL.
  UEs in another beam cannot use them, since a symbol holds a single beam. The TDMA
  schedulers give each symbol to a single UE, so they do not do this.
* A UE never gets new data on the symbols of its own retransmission, as it cannot
  receive (or transmit) two PDSCHs (or PUSCHs) that overlap in time on a serving cell
  (TS 38.214, clauses 5.1 and 6.1).
* A UE gets a single DCI per slot, so a UE placed on the free RBGs of a retransmission
  cannot use the symbols left after the retransmissions. The retransmission spans of a
  beam are filled together. Each UE, smallest buffer first, goes to the span in which it
  gets the most resources, assuming the free RBGs of a span are split evenly among its
  UEs, as long as that share carries its data and the data of the UEs already there. A UE
  whose data fits in no span keeps the remaining symbols, unless the UEs left behind
  already have enough data to fill them, or no symbols remain.

The figures in this section show the slot allocated by the scheduler for the scenarios of
the ``nr-test-sched-harq`` test suite, plus a few more, before and after these rules were
introduced. They were drawn from the ``ResourceAssignmentMatrix`` built from the DCIs of the
slot, as done by ``ResourceAssignmentMatrix::CheckResourceMatrixFromVarTtiAllocInfo()``.
Each cell is an RBG during an OFDM symbol, labeled with the RNTI of the UE it is assigned
to. Hatched cells hold a HARQ retransmission, and dark cells hold the DL or UL CTRL
symbol. The title of each panel gives the RBG x symbols used for data, out of the ones
that are not used by CTRL nor notched. All scenarios use a 10 or 12 RBG bandwidth,
the round robin schedulers, and no SRS.

Retransmissions sharing symbols, or reshaped
############################################
In :ref:`fig-harq-ul-retx-sharing`, UEs 1 and 2 share a beam and must retransmit an
8 symbol UL transport block, on RBGs 0 to 4 and 5 to 9, respectively. Both
retransmissions were frequency multiplexed in their first transmission, but were
serialized in time when retransmitted. With 13 UL data symbols, only one fitted, and the
retransmission of UE 2 was deferred to a later slot. Both retransmissions now share
symbols 5 to 12, as in their first transmission.

.. _fig-harq-ul-retx-sharing:

.. figure:: figures/harq-scheduling/harq-ul-retx-sharing.*
   :align: center
   :scale: 60 %

   UL retransmissions of two UEs in a beam, on disjoint RBGs

A retransmission does not share the symbols of another when their RBGs overlap, or when
the UEs are in different beams. In both cases, shown in :ref:`fig-harq-ul-retx-reshape`,
the second retransmission does not fit in the 5 symbols left with its 8 symbols, and was
deferred to a later slot. It is now reshaped into the fewest of those symbols that hold
its RBGs times symbols over the whole band (5 symbols for the 6 RBGs of the overlapping
case, 4 symbols for the 5 RBGs of the other), keeping its MCS and TBS.

.. _fig-harq-ul-retx-reshape:

.. figure:: figures/harq-scheduling/harq-ul-retx-reshape.*
   :align: center
   :scale: 60 %

   UL retransmissions that cannot share symbols: overlapping RBGs 0 to 5 (top), and UEs
   in different beams (bottom). The second retransmission is reshaped into the symbols
   left.

The DL behaves in the same way, as shown in :ref:`fig-harq-dl-retx-reshape`, where UEs 1
and 2 retransmit an 8 symbol PDSCH on RBGs 0 to 5. With both UEs in a beam, the RBGs of
the retransmission scheduled second are taken; with UEs in different beams, its 8 symbols do not fit
in the 5 left after the first beam. It used to be deferred to a later slot, and is now
reshaped into those 5 symbols.

.. _fig-harq-dl-retx-reshape:

.. figure:: figures/harq-scheduling/harq-dl-retx-reshape.*
   :align: center
   :scale: 60 %

   DL retransmissions that cannot keep their shape: overlapping RBGs in a beam (top), and
   UEs in different beams (bottom). The second retransmission is reshaped into the
   symbols left.

New data on the RBGs left free by a retransmission
##################################################
In :ref:`fig-harq-ul-fill`, UE 1 retransmits an 8 symbol UL transport block on RBGs 0
to 4, while UE 2, in the same beam, reported 30 bytes of UL data. The rest of the band
used to stay empty during the retransmission, and UE 2 was given symbols of its own
before it. UE 2 is now scheduled on RBGs 5 and 6, over the symbols of the
retransmission, leaving symbols 0 to 4 free.

.. _fig-harq-ul-fill:

.. figure:: figures/harq-scheduling/harq-ul-fill.*
   :align: center
   :scale: 60 %

   UL data of UE 2 on the RBGs left free by the UL retransmission of UE 1 (OFDMA)

:ref:`fig-harq-dl-fill` shows the same scenario in the DL, where both UEs have
100 bytes of new data. UE 2 is now scheduled on the free RBGs over symbols 1 to 8,
instead of on symbols 9 to 13. UE 1 only gets its retransmission, even though it has
new data, since it cannot receive two PDSCHs that overlap in time.

.. _fig-harq-dl-fill:

.. figure:: figures/harq-scheduling/harq-dl-fill.*
   :align: center
   :scale: 60 %

   DL data of UE 2 on the RBGs left free by the DL retransmission of UE 1 (OFDMA)

With UEs in different beams, or with a TDMA scheduler, UE 2 still gets symbols of its
own, as shown in :ref:`fig-harq-ul-fill-unchanged` and :ref:`fig-harq-dl-fill-unchanged`.

.. _fig-harq-ul-fill-unchanged:

.. figure:: figures/harq-scheduling/harq-ul-fill-unchanged.*
   :align: center
   :scale: 60 %

   UL data that cannot use the free RBGs of the retransmission: UE 2 in another beam
   (left), and a TDMA scheduler (right). The allocation is unchanged.

.. _fig-harq-dl-fill-unchanged:

.. figure:: figures/harq-scheduling/harq-dl-fill-unchanged.*
   :align: center
   :scale: 60 %

   DL data that cannot use the free RBGs of the retransmission: UE 2 in another beam
   (left), and a TDMA scheduler (right). The allocation is unchanged.

Keeping the rest of the slot in use
###################################
Since a UE gets a single DCI per slot, moving a UE to the free RBGs of a retransmission
must not leave the symbols after it unused. In :ref:`fig-harq-dl-fill-utilization` and
:ref:`fig-harq-ul-fill-utilization`, UE 1 retransmits on 2 symbols and RBGs 0 and 1, and
the other UEs of the beam have new data:

* Two UEs with 100000 bytes each (top): one UE is moved to the 8 free RBGs of the
  retransmission, and the other UE takes the rest of the slot. Every resource is used,
  where 16 used to stay empty.
* One UE with 100000 bytes (middle): moving it would leave the 11 or 12 remaining
  symbols empty, so it keeps them, and the free RBGs of the retransmission stay unused,
  as before.
* A UE with 150 bytes and a UE with 100000 bytes (bottom): the UE with less data is
  moved to the free RBGs of the retransmission, and the other UE takes the rest of the
  slot.

.. _fig-harq-dl-fill-utilization:

.. figure:: figures/harq-scheduling/harq-dl-fill-utilization.*
   :align: center
   :scale: 60 %

   DL utilization with a 2 symbol retransmission and new data of other UEs

.. _fig-harq-ul-fill-utilization:

.. figure:: figures/harq-scheduling/harq-ul-fill-utilization.*
   :align: center
   :scale: 60 %

   UL utilization with a 2 symbol retransmission and new data of other UEs

Further examples
################
The following scenarios combine the rules above.

In :ref:`fig-harq-ul-shared-retx-fill`, UEs 1 and 2 retransmit 6 symbol UL transport
blocks on RBGs 0 to 3 and 4 to 7, and UE 3 reported 40 bytes. The two retransmissions
now share symbols 7 to 12, and UE 3 is placed next to them, on RBGs 8 to 10, instead of
taking a symbol of its own over the whole band. Seven symbols of the slot are left free.

.. _fig-harq-ul-shared-retx-fill:

.. figure:: figures/harq-scheduling/harq-ul-shared-retx-fill.*
   :align: center
   :scale: 60 %

   Two UL retransmissions sharing their symbols, and UL data on the RBGs they leave free

In :ref:`fig-harq-ul-mixed-retx-lengths`, the UL retransmissions of UEs 1 and 2 have 4 and
6 symbols, so they cannot share symbols even though their RBGs are disjoint. UEs 3 and 4
reported 20 bytes each, and used to take symbols 0 to 2. Each one now fits on the free
RBGs of one retransmission, leaving symbols 0 to 2 free.

.. _fig-harq-ul-mixed-retx-lengths:

.. figure:: figures/harq-scheduling/harq-ul-mixed-retx-lengths.*
   :align: center
   :scale: 60 %

   UL retransmissions with different numbers of symbols, each with its own symbols, and
   UL data on the RBGs they leave free

In :ref:`fig-harq-ul-spread-spans`, UE 5 is added to the previous scenario. UEs 3 and 4
still have 20 bytes each. With 40 bytes, UE 5 shares the free RBGs of the 6 symbol
retransmission with UE 4, while UE 3 takes the ones of the 4 symbol retransmission, and
every UE gets a transport block that carries its data, where the three UEs used to take
symbols 0 to 2. With 60 bytes, UE 5 would not get enough resources in either span to
carry its reported buffer, so it keeps symbols 0 to 2. Both cases are checked by the
``nr-test-sched-harq`` test suite.

.. _fig-harq-ul-spread-spans:

.. figure:: figures/harq-scheduling/harq-ul-spread-spans.*
   :align: center
   :scale: 60 %

   UL data spread over two retransmission spans of a beam, with 40 bytes (top) and
   60 bytes (bottom) for UE 5

In :ref:`fig-harq-dl-fill-two-beams`, UE 1 retransmits a 5 symbol DL transport block on
RBGs 0 to 5 of beam 0, UE 2 has 200 bytes in beam 0, and UE 3 has 2000 bytes in beam 1.
UE 2 used to take symbol 6 over the whole band. It is now placed on RBGs 6 to 11 of the
retransmission symbols, leaving one more symbol to beam 1, and every resource of the
slot is used.

.. _fig-harq-dl-fill-two-beams:

.. figure:: figures/harq-scheduling/harq-dl-fill-two-beams.*
   :align: center
   :scale: 60 %

   DL retransmission in beam 0, with new data in beams 0 and 1

In :ref:`fig-harq-dl-fill-small-ues`, UE 1 retransmits a 6 symbol DL transport block on
RBGs 0 to 2, UEs 2, 3 and 4 have 40 bytes each, and UE 5 has 5000 bytes, all in the same
beam. UE 4 used to be left without resources, while 9 RBGs stayed empty during the 6
symbols of the retransmission. The three small UEs now share the free RBGs of the
retransmission (RBGs 9 to 11 remain free, as their data already fits), and UE 5 takes
the rest of the slot.

.. _fig-harq-dl-fill-small-ues:

.. figure:: figures/harq-scheduling/harq-dl-fill-small-ues.*
   :align: center
   :scale: 60 %

   Several UEs with little data on the RBGs left free by a DL retransmission

In :ref:`fig-harq-ul-fill-notched`, RBGs 4 to 6 are notched (see :ref:`Notching`). UE 1
retransmits on RBGs 0 to 2, UE 2 reported 40 bytes, and UE 3 reported 3000 bytes. UE 2 is
moved to the free RBGs of the retransmission, skipping the notched ones, and UE 3 takes
the symbols before it over the whole allowed band.

.. _fig-harq-ul-fill-notched:

.. figure:: figures/harq-scheduling/harq-ul-fill-notched.*
   :align: center
   :scale: 60 %

   UL data on the RBGs left free by a UL retransmission, with notched RBGs


Scheduler operation
===================
In an NR system, the UL decisions for a slot are taken in a different moment than the DL decision for the same slot.
In particular, since the UE must have the time to prepare the data to send, the gNB takes the UL scheduler decision
in advance and then sends the UL grant taking into account these timings. Consider that the DL-DCIs are usually
prepared two slots in advance with respect to when the MAC PDU is actually over the air. For the UL case, to permit
two slots to the UE for preparing the data, the UL grant must be prepared four slots before the actual time
in which the UE transmission is over the air. In two slots, the UL grant will be sent to the UE,
and after two more slots, the gNB is expected to receive the UL data.

At PHY layer, the gNB stores all the relevant information to properly schedule reception/transmission of data in a
vector of slot allocations. The vector is guaranteed to be sorted by the starting symbol, to maintain the timing
order between allocations. Each allocation contains the DCI created by the MAC, as well as other useful information.


.. _QosSchedulers:

QoS Schedulers
==============
The 'NR' module includes QoS MAC schedulers that perform the allocation of the available
resources (i.e., symbols and Physical Resource Blocks (PRBs)) based on the different
traffic requirements of the active users (users with data in their buffers). For this,
it considers the QoS profile characteristics of each QoS flow and in particular the
5QI information, such as the resource type, the priority level and the Packet Delay
Budget (PDB), along with real-time measurements as provided at the MAC layer, i.e.,
the head-of-line delay (HOL) and the PF metric.
The user classification is based on the selection of weights that reflect the users
(or the traffic flow) to be prioritized. The calculation of the scheduling weight for
a single flow is:

.. math::
   :nowrap:

    \[
    w = \begin{cases}
    (100 - P) \dfrac{r^{\gamma}}{R(\tau)}, & \text{for non-GBR} \\
    \\
    (100 - P) \dfrac{r^{\gamma}}{R(\tau)} D, & \text{for GBR}
    \end{cases}
    \]

where :math:`P` is the default Priority Level of the QoS flow mapped to
the DRB (lower :math:`P` indicates higher priority for scheduling),
:math:`r` is the instantaneous achievable data rate calculated by
the spectrum efficiency and the channel bandwidth, :math:`R(\tau)` is the
past average data rate updated within the updated window size :math:`\tau`,
and :math:`\gamma` is a configurable parameter. Moreover, we include
the newly introduced delay budget factor :math:`D`, that is the delay-aware
weight related to the HOL packet delay and the PDB, and is calculated as:

:math:`D = \frac{\text{PDB}}{\text{PDB}-\text{HOL}}`

Notice that when :math:`\gamma=1`, and assuming the same priority for all users and
ignoring :math:`D`, the scheduler corresponds to a typical PF scheduler.
The past average data rate is calculated as:

:math:`R(\tau) = (1-\alpha) R(\tau-1) + \alpha A(\tau)`

In this equation, :math:`A(\tau)` is the current data rate over the updated window size
:math:`\tau` computed as the ratio of all successfully delivered bits (including those
bits still in retransmission) in the past updated window size, and :math:`\alpha` balances
between the current data rate (:math:`A(\tau)`) and the past average data rate in the
previous window (:math:`R(\tau-1)`).

The active users are then classified in descending order in each TTI based on the sum of
the calculated scheduling weights for all their active flows:

:math:`W = \sum_{n=1}^{N} w`

where :math:`N` is the number of active logical channels for a given user.
This classification results in scheduling first the users that have
higher :math:`W`.

.. _LcAssignment:

QoS LC Assignment
=================
The default implementation of the NR module assigns bytes to the LCs of each user
in a RR fashion. However, we also offer the option of assigning bytes to the LCs
of a user considering the load of each LC and its QoS requirements.
Notice that in the past, the implementation was located as a method in the
``NrMacSchedulerNs3`` class, thus it was not permitting any additional designs to
be considered. For this reason, the NR module in its current status includes the
base class ``NrMacSchedulerLcAlgorithm`` and two child classes, the
``NrMacSchedulerLcRR`` and the ``NrMacSchedulerLcQos``. The former assigns bytes
to the active LCs as in the initial implementation in RR fashion, while the latter
performs the assignment by taking into account the resource type and the
``e_rabGuaranteedBitRate`` of a flow. More details with respect to the algorithm
considered for the QoS LC Assignment can found in [WNS3-QosSchedulers]_.

.. _RLScheduler:

RL-based Scheduler
===================
The 'NR' module includes Reinforcement Learning (RL)-based schedulers, namely
the ``NrMacSchedulerTdmaAi`` and the ``NrMacSchedulerOfdmaAi``, that allocate
available resources (i.e., symbols and Physical Resource Blocks (PRBs)) based on the RL model.
The RL model is implemented using Python scripts and receives data from the RL-based scheduler
to determine the actions for the current state. To communicate with the RL model, the RL-based
scheduler uses the ``OpenGymEnv`` class, which sends the data to the RL model through the ``OpenGymInterface`` class.
The data sent to the RL model includes the following fields:

* observation
* reward
* isGameOver
* extraInfo

The fields are following the data format defined in the ns3-gym module.

The observation for a UE includes information for each of its active flows. Each flow's information
is contained in an LcObservation structure, which contains the following fields:

* ``rnti``
* ``lcId``
* ``fiveQi``
* ``priority``
* ``holDelay``

This structure represents the observation of the Logical Channel (LC).

The RL-based scheduler sends data to the OpenGymEnv class for each resource unit through a callback,
collecting observations for all UEs and calculating the reward based on the outcomes of the previous actions.
Additionally, the RL-based scheduler passes a function to retrieve the selected actions for the current state.

Next, the RL model selects the actions for the current state and sends them back to the RL-based scheduler.
The actions represent the weights for all active LC flows of all active UEs. After receiving the actions
through the provided function, the RL-based scheduler sorts the UEs by the sum of the weights of their active LC flows.
The scheduler then allocates resources to the UE with the highest sum of weights, and the process repeats for each resource unit.

The goal of the RL-based scheduler is to allocate resources in a way that minimizes the total delay of the UEs
considering the priority of the LCs. Additionally, the RL-based scheduler can be used to allocate resources in a way
that maximizes the throughput of the UEs. To achieve this, the reward of a UE is calculated as:

.. math::
   :nowrap:

   \[
   \text{reward} = \sum_{i=1}^{N} \frac{r^{\gamma}}{R(\tau) \times P_i \times {HOL}_i}
   \]

where :math:`N` is the number of active LCs of the UE, :math:`P_i` is the priority of the LC,
:math:`HOL_i` is the HOL delay of the LC. The total reward of the scheduler is the sum of
the rewards of all active UEs, :math:`r` is the instantaneous achievable data rate calculated by
the spectrum efficiency and the channel bandwidth, :math:`\gamma` is a configurable parameter, and :math:`R(\tau)`
is the past average data rate of the UE.

Timing relations
================
The 'NR' module supports flexible scheduling and DL HARQ Feedback timings in the
communication between the gNB and the UE as specified in [TS38213]_, [TS38214]_.
In particular, the following scheduling timings are defined:

* K0 → Delay in slots between DL DCI and corresponding DL Data reception
* K1 → Delay in slots between DL Data (PDSCH) and corresponding ACK/NACK transmission on UL
* K2 → Delay in slots between UL DCI reception in DL and UL Data (PUSCH) transmission

The values of the scheduling timings are calculated at the gNB side and result
from the processing timings that are defined in the 'NR' module as:

* N0 → minimum processing delay (in slots) needed to decode DL DCI and decode DL data (UE side)
* N1 → minimum processing delay (in slots) from the end of DL Data reception to the earliest possible start of the corresponding ACK/NACK transmission (UE side)
* N2 → minimum processing delay (in slots) needed to decode UL DCI and prepare UL data (UE side)

The values of the processing delays depend on the UE capability (1 or 2) and the configured numerology.
Typical values for N1 are 1 and 2 slots, while N2 can range from 1 to 3 slots based on the numerology and the UE capability. The processing times are defined in Table 5.3-1/2 for N1 and Table 6.4-1/2 for N2 of [TS38214]_. Although in the standard they are defined in multiples of the OFDM symbol, in the simulator we define them in multiples of slots, because then they are used to compute dynamic K values that are measured in slots. Also note that N0 is not defined in the specs, but so is K0, and so we have included both in the 'NR' module.
The values of the processing delays in the 'NR' simulator can be configured by the user through the attributes ``N0Delay``, ``N1Delay``, and ``N2Delay``, and default to 0 slots, 2 slots, and 2 slots, respectively.
The allowed ranges are bounded by the attribute checkers: N0 can be 0 or 1 slot, while N1 and N2 can range from 0 to 4 slots.

For the scheduling timings let us note that each K cannot take a value smaller than
the corresponding N value (e.g., K2 cannot be less than N2).
The procedure followed for the calculation of the scheduling and DL HARQ Feedback
timings at the gNB side is briefly described below.

For K0, the gNB calculates (based on the TDD pattern) which is the next DL (or F)
slot that follows after (minimum) N0 slots. In the current implementation we use
N0=0, as such in this case DL Data are scheduled in the same slot with the DL DCI.

For K1/K2, the gNB calculates (based on the TDD pattern) which is the next UL (or F)
slot that follows after (minimum) N1/N2 slots and calculates K1/K2 based on the
resulted slot and the current slot.

Then, the gNB communicates the scheduling timings to the UE through the DCI. In
particular, K0 and K1 are passed to the UE through the DL DCI in the time domain
resource assignment field and PDSCH-to-HARQ_feedback timing indicator, respectively,
while K2 is passed through the UL DCI in the time domain resource assignment field.
Upon reception of the DL/UL DCI, the UE extracts the values of K0/K1/K2:

* For the case of K0, UE extracts from the DL DCI its value and calculates the corresponding slot for the reception of the DL Data.
* For the case of K2, UE extracts from the UL DCI its value and calculates the corresponding slot for the transmission of its UL Data.

      For example, if UL DCI is received in slot n and K2 = 2, UE will transmit UL Data in slot (n + K2)

* For the case of K1, UE extracts from the DL DCI its value and stores it in a map based on the HARQ Process Id. This way, when the UE is going to schedule the DL HARQ feedback, it can automatically find out in which slot it will have to schedule it.


BWP manager
===========
Our implementation has a layer that acts as a 'router' of messages. Initially, it was depicted as a middle layer between the RLC and the MAC, but with time it got more functionalities. The purpose of this layer, called the bandwidth part manager, is twofold. On the first hand, as we have already seen, it is used to route the control messages to realize the FDD bandwidth part pairing. On the other hand, it is used to split or route traffic over different spectrum parts.

For the FDD pairing functionality, the pairing configuration is derived automatically for cells with a single DL/UL carrier pair: the gNB routes the outgoing messages of its UL-only bandwidth parts through its first DL-capable one, and the UE RRC keeps the routing aligned with its primary DL/UL pair (derived from SIB1, or updated on handover). The NetDevice then asks the manager for the input/output bandwidth part to which each message should be routed. Mixed TDD/FDD carrier-aggregation setups whose pairing does not involve the primary carrier still configure the mapping manually through ``SetOutputLink()``, which now overwrites any previous mapping for the same source bandwidth part. It is important to note that this feature virtually connects different physical layers.

For the flows routing among different spectrum, the layer intercepts the BSR from the RLC queues, and route them to the correct stack (MAC and PHY) that is attached to a particular spectrum region. The algorithmic part of the split is separated from the Bandwidth Part Manager. In other words, the algorithm is modularized to let the user write, change, and test different ways of performing the split. The only requirement is that such routing is done based on the QCI of the flow.


Adaptive modulation and coding model
====================================
MCS selection in NR is an implementation specific procedure.
The 'NR' module supports 1) fixing the MCS to a predefined value, both for
downlink and uplink
transmissions, separately, and 2) two different AMC models for link adaptation:

* Error model-based: the MCS index is selected to meet a target transport BLER (e.g., of at most 0.1)
* Shannon-based: chooses the highest MCS that gives a spectral efficiency lower than the one provided by the Shannon rate

In the Error model-based AMC, the PHY abstraction model described in PHY layer
section is used for link adaptation, i.e.,
to determine an MCS that satisfies the target transport BLER based
on the actual channel conditions. In particular, for a given set of SINR values,
a target transport BLER, an MCS table, and considering a transport block
composed of the group of RBs in the band (termed the CSI reference resource [TS38214]_),
the highest MCS index that meets the target transport BLER constraint is selected
at the UE. Such value is then reported through the associated CQI index to the gNB.

In the Shannon-based AMC, to compute the Shannon rate we use a coefficient
of :math:`{-}\ln(5{\times} Ber)/1.5` to account for the difference in
between the theoretical bound and real performance.

The AMC model can be configured by the user through the attribute ``AmcModel``. In case
the Error model-based AMC is selected, the attribute ``ErrorModelType`` defines
the type of the Error Model that is used when AmcModel is set to ErrorModel, which takes
the same error model type as the one configured for error modeling. In case the
Shannon-based AMC is selected, the value :math:`Ber` sets the requested bit error rate
in assigning the MCS.

In the 'NR' module, link adaptation is done at the UE side, which selects the MCS index (quantized
by 5 bits), and such index is then communicated to the gNB through a CQI index (quantized by 4 bits).

Note also that, in case of adaptive MCS, in the simulator, the gNBs DL data transmissions start with MCS0. Such MCS is used at the start and until there is a UE CQI feedback.


Transport block model
=====================
The model of the MAC Transport Blocks (TBs) provided by the simulator is simplified with respect to the 3GPP specifications. In particular, a simulator-specific class (PacketBurst) is used to aggregate MAC SDUs to achieve the simulator’s equivalent of a TB, without the corresponding implementation complexity. The multiplexing of different logical channels to and from the RLC layer is performed using a dedicated packet tag (``NrRadioBearerTag``), which produces a functionality which is partially equivalent to that of the MAC headers specified by 3GPP. The incorporation of real MAC headers has recently started, so it is expected that in the next releases such tag will be removed. At the moment, we introduced the concept of MAC header to include the Buffer Status Report as a MAC Control Element, as it is defined by the standard (with some differences, to adapt it to the ancient LTE scheduler interface).

**Transport block size determination**: Transport block size determination in NR is described in [TS38214]_, and it is used to determine the TB size of downlink and uplink shared channels, for a given MCS table, MCS index and resource allocation (in terms of OFDM symbols and RBs). The procedure included in the 'NR' module for TB size determination follows TS 38.214 Section 5.1.3.2 (DL) and 6.1.4.2 (UL) but without including quantizations and and limits. That is, including Steps 1 and 2, but skipping Steps 3 and 4, of the NR standard procedure. This is done in this way to allow the simulator to operate in larger bandwidths that the ones permitted by the NR specification. In particular, the TB size is computed in the simulator as follows:

:math:`N_{info}= R \times Q \times n_s \times n_{rb} \times (12- n_{refSc})`,

where :math:`R` is the ECR of the selected MCS, :math:`Q` is the modulation order of the selected MCS, :math:`n_s` is the number of allocated OFDM symbols, :math:`n_{rb}` is the number of allocated RBs, and :math:`n_{refSc}` is the number of reference subcarriers carrying DMRS per RB.

After this computation, we subtract the CRC attachment to the TB (24 bits), and if code block segmentation occurs, also the code block CRC attachments are subtracted, to get the final TB size.

.. _Notching:

UFA aka Notching
================
UL Frequency Avoidance (UFA) (known also as spectrum notching) is a
technique that restricts the usage of certain Resource Block Groups (RBGs)
for UL transmissions referred to as "notched" RBGs. For more details
please see [notching1]_, [notching2]_.

We opted for a flexible solution that is independent of the size, location and
continuity/discontinuity of the notched RBGs, as well as for the transmission
direction, i.e. DL/UL. The UFA feature is implemented on top of the TDMA/OFDMA
scheduler classes (``NrMacSchedulerTdma`` and ``NrMacSchedulerOfdma``).

In particular, the parent class ``NrMacSchedulerNs3`` can get as input a DL and/or
an UL notched mask through the ``SetDlNotchedRbgMask`` and ``SetUlNotchedRbgMask``
methods. The masks will define the resources per gNB that can and cannot be assigned
for its DL and/or UL transmissions. These masks are actually comprised by 1s (normal
RBGs) and 0s (notched RBGs), while the index of the position of each bit inside
the mask corresponds to each RBG index. Therefore, the size of the mask must
change in accordance to the selected bandwidth (BW).

An example of the notched mask is: 1 1 1 1 1 1 0 1 1 1 1 1 0 0 0 0 1 1 1 1 0 1 1 1 1

Based on this mask, the scheduler before performing the assignment of RBGs to the
active UEs, calculates the number of available RBGs to be distributed among the
UEs (by RR, PF, etc). Then, during the scheduling process, it is responsible for
not assigning the RBGs that conflict with the notched RBGs defined in the mask.
The result of this process is a mask for each UE that contains the RBGs assigned
to that UE. Since the scheduling processes for both DL and UL directions are included
in the gNB functionality, this mask is communicated to the UE through the DL/UL DCI.

Let us notice that if no mask is defined or if a mask with all 1s is selected,
the scheduler will take under consideration all the RBGs in the scheduling process.

UFA in 5G-LENA can be simulated with the :ref:`notchingExample` example described
in detail in :ref:`Examples` section, while the notching functionality is tested
with the UNIT Test :ref:`notchingTest` described in :ref:`Validation` section.


.. _sec-random-access:

Random Access
=============

The NR model includes a model of the Random Access procedure based on
some simplifying assumptions, which are detailed in the following for
each of the messages and signals described in the specs [TS36321]_.

   - **Random Access (RA) preamble**: in real LTE/NR systems this
     corresponds to a Zadoff-Chu (ZC)
     sequence using one of several formats available and sent in the
     PRACH slots which could in principle overlap with PUSCH.
     PRACH Configuration Index 14 is assumed, i.e., preambles can be
     sent on any system frame number and subframe number.
     The RA preamble is modeled using the NrControlMessage class,
     i.e., as an ideal message that does not consume any radio
     resources. The collision of preamble transmission by multiple UEs
     in the same cell are modeled using a protocol interference model,
     i.e., whenever two or more identical preambles are transmitted in
     same cell at the same TTI, no one of these identical preambles
     will be received by the gNB. Other than this collision model, no
     error model is associated with the reception of a RA preamble.

   - **Random Access Response (RAR)**: in real LTE/NR systems, this is a
     special MAC PDU sent on the DL-SCH. Since MAC control elements are not
     accurately modeled in the simulator (only RLC and above PDUs
     are), the RAR is modeled as an NrControlMessage that does not
     consume any radio resources. Still, during the RA procedure, the
     NrGnbMac will request to the scheduler the allocation of
     resources for the RAR using the FF MAC Scheduler primitive
     SCHED_DL_RACH_INFO_REQ. Hence, an enhanced scheduler
     implementation (not available at the moment) could allocate radio
     resources for the RAR, thus modeling the consumption of Radio
     Resources for the transmission of the RAR.

   - **Message 3**:  in real LTE/NR systems, this is an RLC TM
     SDU sent over resources specified in the UL Grant in the RAR. In
     the simulator, this is modeled as a real RLC TM RLC PDU
     whose UL resources are allocated by the scheduler upon call to
     SCHED_DL_RACH_INFO_REQ.

   - **Contention Resolution (CR)**: in real LTE/NR system, the CR phase
     is needed to address the case where two or more UE sent the same
     RA preamble in the same TTI, and the gNB was able to detect this
     preamble in spite of the collision. Since this event does not
     occur due to the protocol interference model used for the
     reception of RA preambles, the CR phase is not modeled in the
     simulator, i.e., the CR MAC CE is never sent by the gNB and the
     UEs consider the RA to be successful upon reception of the
     RAR. As a consequence, the radio resources consumed for the
     transmission of the CR MAC CE are not modeled.


Figure :ref:`fig-mac-random-access-contention` and
:ref:`fig-mac-random-access-noncontention` shows the sequence diagrams
of respectively the contention-based and non-contention-based MAC
random access procedure, highlighting the interactions between the MAC
and the other entities.


.. _fig-mac-random-access-contention:

.. figure:: figures/rach/mac-random-access-contention.*
   :align: center

   Sequence diagram of the Contention-based MAC Random Access procedure


.. _fig-mac-random-access-noncontention:

.. figure:: figures/rach/mac-random-access-noncontention.*
   :align: center

   Sequence diagram of the Non-contention-based MAC Random Access procedure



.. only:: latex

    .. raw:: latex

        \clearpage
