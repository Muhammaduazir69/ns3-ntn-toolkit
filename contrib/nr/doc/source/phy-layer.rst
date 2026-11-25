.. Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
..
.. SPDX-License-Identifier: GPL-2.0-only

.. highlight:: cpp

PHY layer
*********
This section describes in detail the different models supported and developed at PHY layer.


Frame structure model
=====================
In NR, the 'numerology' concept is introduced to flexibly define the frame structure in such a way that it can work in both sub-6 GHz and mmWave bands. The flexible frame structure is defined by multiple numerologies formed by scaling the subcarrier spacing (SCS) of 15 kHz. The supported numerologies (0, 1, 2, 3, 4)  correspond to SCSs of 15 kHz, 30 kHz, 60 kHz, 120 kHz, and 240 kHz. The SCS of 480 kHz is under study by 3GPP, but the simulator can support it. Theoretically, not all SCS options are supported in all carrier frequencies and channels. For example, for sub 6 GHz, only 15 kHz, 30 kHz, and 60 kHz are defined. Above 6 GHz, the supported ones are 60 kHz, 120 kHz, and 240 kHz. Also, for numerology 2 (i.e., SCS = 60 KHz), two cyclic prefixes (CP) overheads are considered: normal and extended. For the rest of the numerologies, only the normal overhead is taken into consideration.

.. _tab-numerologies-3gpp:

.. table:: Numerologies defined in 3GPP NR Release-15

   ===========   =========================   =============
   Numerology    Subcarrier spacing in kHz   Cyclic prefix
   ===========   =========================   =============
    0            15                          normal
    1            30                          normal
    2            60                          normal, extended
    3            120                         normal
    4            240                         normal
   ===========   =========================   =============

In the time domain, each 10 ms frame is split in time into ten subframes, each of duration of 1 ms. Every subframe is split in time into a variable number of slots, and each slot is composed of a fixed number of OFDM symbols. In particular, the length of the slot and the number of slots per subframe depend on the numerology, and the length of the OFDM symbol varies according to the numerology and CP. The number of OFDM symbols per slot is fixed to 14 symbols for normal CP, and to 12 OFDM symbols for extended CP.

In the frequency domain, the number of subcarriers per physical resource block (PRB) is fixed to 12, and the maximum number of PRBs, according to Release-15, is 275. With a particular channel bandwidth, the numerology defines the size of a PRB and the total number of PRBs usable by the system. PRBs are grouped into PRB groups at MAC scheduling time.

:numref:`fig-frame` shows the NR frame structure in time- and frequency- domains for numerology 3 with normal CP and a total channel bandwidth of 400 MHz.

.. _fig-frame:

.. figure:: figures/numerologies-1.*
   :align: center
   :scale: 35 %

   NR frame structure example for numerology 3, normal CP, and 400 MHz bandwidth

The implementation in the 'NR' module currently supports the NR frame structures and numerologies shown in Table :ref:`tab-numerologies`. Once the numerology is configured, the lengths of the symbol, the slot, the SCS, the number of PRBs within the bandwidth, and the number of slots per subframe, are dynamically determined at runtime, based on Table :ref:`tab-numerologies`.

.. _tab-numerologies:

.. table:: Implemented NR numerologies

   ===========   ==================   ==================     ================   =========================   ================
   Numerology    Slots per subframe   Symbol length (μs)     Slot length (ms)   Subcarrier spacing in kHz   Symbols per slot
   ===========   ==================   ==================     ================   =========================   ================
    0            1                         71.42                1                  15                          14
    1            2                         35.71                0.5                30                          14
    2            4                         17.85                0.25               60                          14
    3            8                         8.92                 0.125              120                         14
    4            16                        4.46                 0.0625             240                         14
    5            32                        2.23                 0.03125            480                         14
   ===========   ==================   ==================     ================   =========================   ================

In the 'NR' module, to support a realistic NR simulation, we accurately model (as per the standard) the numerology-dependent slot and OFDM symbol granularity. We use the event scheduling feature of ns-3 to model the time advancement. Starting from time 0, we insert (and process) events that represent the advancement of the time. One of such events is the starting slot boundary, where the processing follows a logical order that involves the MAC, then the scheduler, before returning the control to the PHY. Here, allocations are extracted, and, for each assignment, a new event is inserted in the simulator. The last added event is the end of the slot, which in turn will invoke a new starting slot boundary event.

Two factors influence the processing of events and allocations. The first is the availability of the channel: in NR, the channel is always available for transmission, while in the unlicensed spectrum, this may not be true. If the channel is not available, the event machine will go directly to the next slot boundary (with some details that are not explained here). The second is the MAC-to-PHY processing delay: fixed at 2 slots in the implementation (``NrPhy::GetL1L2CtrlLatency``), it indicates that the MAC is working ahead of the PHY to simulate the time needed for each component of the chain to perform its work. For example, in numerology 0, PHY level at Frame 10, Subframe 0, Slot 0, will call MAC to realize the allocations of Frame 10, Subframe 2, Slot 0.

Some of the details of what is explained above is present in the papers [WNS32018-NR]_, [CAMAD2018-NR]_, along with some performance evaluations.


FDM of numerologies
===================
An additional level of flexibility in the NR system can be achieved by implementing the multiplexing of numerologies in the frequency domain. As an example, ultra-reliable and low-latency communications (URLLC) traffic requires a short slot length to meet strict latency requirements, while enhanced mobile broadband (eMBB) use case in general aims at increasing throughput, which is achieved with a large slot length. Therefore, among the set of supported numerologies for a specific operational band and deployment configuration, URLLC can be served with the numerology that has the shortest slot length and eMBB with the numerology associated with the largest slot length. To address that, NR enables FDM of numerologies through different BWPs, to address the trade-off between latency and throughput for different types of traffic by physically dividing the bandwidth in two or more BWPs. In :numref:`fig-bwp`, we illustrate an example of the FDM of numerologies. The channel is split into two BWPs that accommodate the two numerologies multiplexed in the frequency domain. The total bandwidth :math:`B` is then divided into two parts of bandwidth :math:`B_u` for URLLC and :math:`B_e` for eMBB, so that :math:`B_u+B_e \le B`.

.. _fig-bwp:

.. figure:: figures/bwp.*
   :align: center
   :scale: 80 %

   FDM of numerologies example

In the 'NR' module, the user can configure FDM bands statically before the simulation starts. This is a critical design assumption based on two main reasons. First, the 'NR' module relies on the channel and the propagation loss model that is not able to allow runtime modifications of the physical configuration parameters related to time/frequency configuration (such as the system bandwidth, the central carrier frequency, and the symbol length). Thus, until the current channel model is modified to allow these runtime configuration changes, it will not be possible to perform semi-static reconfiguration of BWPs. The second reason is that in the simulator, the RRC messaging to configure the default BWP, as well as the BWP reconfiguration, are not supported yet. See implementation details and evaluations in [WNS32018-NR]_, which is inspired by [CA-WNS32017]_.


Duplexing schemes
=================
The 'NR' simulator supports both TDD and FDD duplexing modes in a flexible manner. Indeed, a gNB can be configured with multiple carriers, some of them being paired (for FDD), and others being TDD. Each carrier can be further split into various BWPs, under the assumption that all the BWPs are orthogonal in frequency, to enable compatibility with the channel instances. The gNB can simultaneously transmit and receive from multiple BWPs. However, from the UE side, we assume the UE is active in a single BWP at a time.


TDD model
#########
NR allows different slot types: DL-only ("DL" slots), UL-only ("UL" slots), and Flexible ("F" slots). Flexible slots have a certain number of DL symbols, a guard band, and a certain number of UL symbols. For the DL-only and UL-only case, the slots have, as the name suggests, only DL or only UL symbols. A TDD pattern in NR, repeated with a pre-configured periodicity, is a set of the previously defined slot types.

In the 'NR' module, the TDD pattern is represented by a vector of slot types, where the length and the content of such vector are user-defined. In the case of Flexible slots, the first and the last OFDM symbols are reserved for DL CTRL and UL CTRL, respectively (e.g., DCI and UCI). The symbols in between can be dynamically allocated to DL and UL data, hence supporting dynamic TDD. In the case of DL-only slots, the first symbol is reserved for DL CTRL, and the rest of the symbols are available for DL data. In the case of UL-only slots, the last symbol is reserved for UL CTRL, and the rest of the symbols are available for UL data.

The model also supports the special slot type ("S" slots) to emulate LTE. In those slots, the first symbol is reserved for DL CTRL, the last slot is reserved for UL CTRL, and the rest of the symbols are available for DL data.

Note there are not limitations in the implementation of the TDD pattern size and structure. However, one must ensure there is no a large gap in between two DL slots, or two UL slots, so that different timers (e.g., RRC, HARQ, CQI) do not expire simply by the TDD pattern.


FDD model
#########
In the 'NR' module, FDD duplexing is modeled through the usage of two paired bandwidth parts, where one is dedicated to transmitting DL data and control, and the other for the transmission of the UL data and control. The user configures each bandwidth part with a DL-only (or UL-only) pattern; the linking between the two bandwidth parts for the correct routing of the control messages is then derived automatically (the cell advertises its UL carrier in SIB1, and the UE derives its primary UL bandwidth part and routing from it). As an example, the HARQ feedback for a DL transmission will be uploaded through the UL-only bandwidth part, but it applies to the DL-only bandwidth part: the linking is needed for correctly routing that message from one bandwidth part to the other. Setups with several carrier pairs can still install the mapping manually through ``SetOutputLink()``.

The paired bandwidth parts may be configured with different numerologies; the control-plane procedures (random access, RRC signalling and handover) across such a pairing are exercised by the ``nr-handover-rach-config`` test suite.

How the time looks like in both schemes
#######################################
In both schemes, the time starts at the beginning of the slot. The GNB PHY retrieves the allocations made by MAC for the specific slot, and extracts them one by one. Depending on the allocation type, the PHY schedules the variable TTI type. For instance, most probably in DL or F slot, the first symbol is allocated to the CTRL, so the GNB starts transmitting the CTRL symbol(s). The UE begins as well receiving these CTRLs, thanks to the fact that it (i) knows the type of the slot, and (ii) at the registration time it discovers how many symbols are reserved for the DL CTRL. In this (these) symbol(s), the UE receives the DCI. Based on the received DCIs, it can schedule multiple variable TTI to (i) receive data if it received DL DCI or (ii) send data if it received UL DCI. In UL or F slots, at the end of the slot, there will be a time in which the UE will be able to transmit its UL CTRL data. The GNB specifies this time at the registration time, and it is considered that it will be the last operation in the slot.

When the slot finishes, another one will be scheduled in a similar fashion.


CQI feedback
============
NR defines a Channel Quality Indicator (CQI), which is reported by the UE and can be used for MCS index selection at the gNB for DL data transmissions. NR defines three tables of 4-bit CQIs (see Tables 5.2.2.1-1 to 5.2.2.1-3 in [TS38214]_), where each table is associated with one MCS table. In the simulator, we support CQI Table1 and CQI Table2 (i.e., Table 5.2.2.1-1 and Table 5.2.2.1-2), which are defined based on the configured error model and corresponding MCS Table.

Before nr-3.0, we only supported the generation of a *wide-band* CQI that is computed based on the SISO data channel (PDSCH). Such value is a single integer that represents the entire channel state or better said, the (average) state of the resource blocks that have been used in the gNB transmission (neglecting RBs with 0 transmitted power).

The CQI index to be reported is obtained by first obtaining an SINR measurement and then passing this SINR measurement to the Adaptive Modulation and Coding module (see details in AMC section) that maps it to the CQI index. Such value is computed for each PDSCH reception and reported after it.

In case of UL transmissions, there is not explicit CQI feedback, since the gNB directly indicates to the UE the MCS to be used in UL data transmissions. In that case, the gNB measures the SINR received in the PUSCH, and computes based on it the equivalent CQI index, and from it the MCS index for UL is determined.

Since nr-3.0, we also support *wide-band* CQI computed based on the MIMO data channel (PSDCH).

Since nr-4.0, we also support *wide-band* and *sub-band* CQI for MIMO data channel (PDSCH) and/or the CSI-RS+CSI-IM (see more in :ref:`CSI-RS and CSI-IM`).

CQI feedback for MIMO is detailed in section :ref:`Search for the optimal precoding matrix`.


Power allocation
================
In the simulator, we have two types/models for power allocation.
The first model assumes a uniform power allocation over the whole set of RBs
that conform the bandwidth of the BWP. That is, power per RB is fixed. However,
if a RB is not allocated to any data transmission, the transmitted power is null, and no interference is generated in that RB.
The second model assumes a uniform power allocation over the active set of RBs,
i.e., over the set of RBs used by the transmitter (e.g., gNB in DL or UE in UL). In this case,
the power per RB is the same over the active RBs, but it is not fixed over different
slots, as it depends on the actual number of RBs being used for the transmission.
The model to use can be configured through the attribute ``PowerAllocationType``
at NrGnbPhy and NrUePhy.


Interference model
==================
The PHY model is based on the well-known Gaussian interference models, according
to which the powers of interfering signals (in linear units) are summed up together
to determine the overall interference power. The useful and interfering signals,
as well as the noise power spectral density, are processed to calculate the SNR,
the SINR, the RSSI (in dBm) and the RSRP (in dBm).

Also, such powers are used to determine if the channel is busy or empty. For that,
we are creating two events, one that adds, for any signal, the received power and
another that subtracts the received power at the end time. These events determine
if the channel is busy (by comparing it to a threshold) and for how long.


Spectrum model
==============
In the simulator, radio spectrum usage follows the usual way to represent radio
transmission in the ns-3 simulator [baldo2009]_. The core is an object that represents
the channel characteristic, including the propagation, following the 3GPP specifications
[gpp-channel-dev]. In the simulation, there will be as many channel models as the user needs,
remembering that two (or more) channel models cannot overlap over the spectrum frequencies.
In the NR nodes, there will be as many physical layers as the number of channel models;
each physical layer communicates to its channel model through a spectrum model instance
that owns a model of the physical layer antenna. The combination of the sender's and
receiver's antenna gain (given by the configured beam and the antenna element radiation pattern),
the propagation loss, and the channel characteristics, provide the value of the
received power spectral density for each transmitted signal. The interference among
different nodes is calculated using the MultiModelSpectrumChannel described in [baldo2009]_.
In this way, we can simulate dynamic spectrum access policies, as well as dynamic TDD schemes,
considering downlink-to-uplink and uplink-to-downlink interference.


Data PHY error model
====================
The PHY abstraction of NR based systems is a complex task due to the multiple
new features added to NR. In NR, in addition to number of RBs, the number of
OFDM symbols can also be variably allocated to a user, which in combination with
wide-bandwidth operation significantly increases the number of supported
transport block sizes (TBSs). The inclusion of LDPC coding for data channels (i.e., PDSCH and PUSCH)
with multiple lifting sizes and two types of base graphs increases the complexity of
the code block segmentation procedure at PHY. Moreover, NR supports
various configurations for MCS tables, and modulation orders up to 256-QAM.
All these features have been considered to model NR performance appropriately.

The 'NR' module includes a PHY abstraction model for error modeling that is compliant with the
latest NR specifications, including LDPC coding,
MCS up to 256-QAM, different MCS Tables (MCS Table1 and MCS Table2),
and NR transport block segmentation [TS38214]_ [TS38212]_. Also, the developed PHY
abstraction model supports HARQ
based on Incremental Redundancy (IR) and on Chase Combining (CC), as we will present in
the corresponding section.

Let us note that the attribute ``ErrorModelType`` configures the type of error modelling, which can be set to NR (ns3::NrEesmCcT1, ns3::NrEesmIrT1, ns3::NrEesmCcT2, ns3::NrEesmIrT2) or to LTE (ns3::NrLteMiErrorModel, default one) in case one wants to reproduce LTE PHY layer. In the NR case, the HARQ method and MCS table are configured according to the selected error model, e.g., ns3::NrEesmCcT1 uses HARQ-CC and MCS Table1.

The error model of the NR data plane in the 'NR' module is developed according to standard
link-to-system mapping (L2SM) techniques. The L2SM choice is aligned with the
standard system simulation methodology of frequency-selective
channels. Thanks to L2SM we are able to maintain a good
level of accuracy and at the same time limiting the computational complexity
increase. It is based on the mapping of single link layer performance obtained
by means of link level simulators to system (in our case network) simulators.
In particular a link-level simulator is used for generating the performance
of a single link from a PHY layer perspective, in terms of code block
error rate (BLER), under specific conditions. L2SM allows the usage
of these parameters in more complex scenarios, typical of system/network-level
simulators, where we have more links, interference and frequency-selective fading.

To do this, a proprietary simulator of InterDigital Inc., compliant with NR specifications,
has been used for what concerns the extraction of link-level performance
by using the Exponential Effective SINR (EESM) as the L2SM mapping function.

The overall NR PHY abstraction model that is implemented in the 'NR' module is shown in
:numref:`fig-l2sm`. The L2SM process receives inputs consisting of a vector
of SINRs per allocated RB, the MCS selection (including MCS index and the MCS
table to which it refers), the TBS delivered to PHY, and the HARQ history. Then,
it provides as output the BLER of the MAC transport block.
The model consists of the following blocks: SINR compression, LDPC base graph (BG) selection,
segmentation of a transport block into one or multiple code blocks
(known as code block segmentation), mapping of the effective SINR to BLER
for each PHY code block (denoted as code BLER),
and mapping of code BLERs to the transport BLER.

.. _fig-l2sm:

.. figure:: figures/l2sm-1.*
   :align: center
   :scale: 60 %

   NR PHY abstraction model

The HARQ history depends on the HARQ method. In HARQ-CC, the HARQ history contains
the SINR per allocated RB, whereas for HARQ-IR, the HARQ history contains the last
computed effective SINR and number of coded bits of each of the previous retransmissions.
Given the SINR vector and the HARQ history, the effective SINR is computed according to
EESM. The optimization of EESM is performed using the NR-compliant link-level simulator.
The LDPC BG selection follows NR specifications, which uses TBS and MCS selection, are detailed next.
Once the BG selection is known, the code block segmentation (if needed) is performed
to derive the number of code blocks and the number of bits in each code block,
which is also known as code block size (CBS), also as per NR specs. Given the effective SINR,
the ECR, the MCS selection, and the CBS, the corresponding code BLER can be
found using SINR-BLER lookup tables obtained from the NR-compliant link-level simulator.
Finally, based on the number of code blocks and the code BLER, the transport BLER
of the transport block is obtained. In what follows we detail the different blocks and
NR features supported by the model.

**MCS**: NR defines three tables of MCSs: MCS Table1 (up to 64-QAM),
MCS Table2 (up to 256-QAM), and MCS Table3 (up to 64-QAM with low spectral efficiency),
which are given by Tables 5.1.3.1-1 to 5.1.3.1-3 in [TS38214]_.
A base station can indicate the table selection to a UE either
semi-statically or dynamically, and the MCS index selection is communicated to
the UE for each transmission through the DCI.
Each MCS index defines an ECR, a modulation order,
and the resulting spectral efficiency (SE).

In the 'NR' module, MCS Table1 and MCS Table 2 can be selected.
The MCS Table1 includes from MCS0 (ECR=0.12, QPSK, SE=0.23 bits/s/Hz)
to MCS28 (ECR=0.93, 64-QAM, SE=5.55 bits/s/Hz), whereas the MCS Table2
has MCS indices from MCS0 (ECR=0.12, QPSK, SE=0.23 bits/s/Hz) to MCS27
(ECR=0.93, 256-QAM, SE=7.40 bits/s/Hz).
As shown in :numref:`fig-l2sm`, the MCS Table (1 or 2) and the
MCS index (0 to 28 for MCS Table1, and 0 to 27 for MCS Table2) are
inputs for the NR PHY abstraction.

**LDPC BG selection**: BG selection in the 'NR' module is based on the following
conditions  (as per Sections 6.2.2 and 7.2.2 in TS 38.212) [TS38212]_. Assuming :math:`R` as the ECR of the selected MCS
and :math:`A` as the TBS (in bits), then,

* LDPC base graph 2 (BG2) is selected if :math:`A \le 292` with any value of :math:`R`, or if :math:`R \le 0.25` with any value of :math:`A`, or if :math:`A \le 3824` with :math:`R \le 0.67`,
* otherwise, the LDPC base graph 1 (BG1) is selected.

**Code block segmentation**: Code block segmentation for LDPC coding in
NR occurs when the number of total bits in a transport block including
cyclic redundancy check (CRC) (i.e., :math:`B = A + 24` bits) is larger than the maximum CBS, which is 8448
bits for LDPC BG1 and 3840 bits for LDPC BG2.
If code block segmentation occurs, each transport block is split into :math:`C` code blocks of
:math:`K` bits each, and for each code block, an additional CRC sequence of :math:`L=24`
bits is appended to recover the segmentation during the decoding process.
The segmentation process takes LDPC BG selection and LDPC lifting size
into account, the complete details of which can be found in [TS38212]_, and the
same procedure has been included in the 'NR' module (as per Section 5.2.2 in TS 38.212).

**SINR compression**: In case of EESM, the mapping
function is exponential and the effective SINR for single transmission depends
on a single parameter (:math:`\beta`).
More precisely, the effective SINR for single transmission is obtained as:

:math:`SINR_{\text{eff}} = {-}\beta \ln \Big( \frac{1}{|\upsilon|}\sum_{n \in \upsilon} \exp\big({-}\frac{\text{SINR}_n}{\beta}\big)\Big)`,

where :math:`\text{SINR}_n` is the SINR value in the n-th RB, :math:`\upsilon`
is the set of allocated RBs, and :math:`\beta` is the parameter that needs to be
optimized.

In EESM, given an experimental BLER measured
in a fading channel with a specific MCS, the :math:`\beta` value (and so the mapping function)
is calibrated
such that the effective SINR of that channel approximates to the SINR that
would produce the same BLER, with the same MCS, in AWGN channel conditions.
In order to obtain the optimal mapping functions, we used the NR-compliant
link-level simulator and use a calibration technique described in [calibration-l2sm]_.
We use tapped delay line (TDP) based fading channel models recommended by 3GPP in [TR38900]_.
A collection of LOS (TDL-D) and NLOS (TDL-A) channel models ranging in delay
spread from 30 ns to 316 ns are used. For NR, SCS of 30 KHz and 60 KHz are simulated.
The details of the link-level simulator as well as the optimized :math:`\beta` values
for each MCS index in MCS Table1 and MCS Table2 are detailed in [nr-l2sm]_,
as included in the 'NR' simulator.

**Effective SINR to code BLER mapping**: Once we have the effective SINR
for the given MCS, resource allocation, and channel model,
we need SINR-BLER lookup tables to find the corresponding code BLER.
In order to obtain SINR-BLER mappings, we perform extensive simulations using our
NR-compliant link-level simulator. Such curves are included in the 'NR' simulator in form
of structures.

For each MCS (both in MCS Table1 and Table2), various resource allocation
(with varying number of RBs from 1 to 132 and varying number of OFDM symbols from 1 to 10)
are simulated. Given the resource allocation,
the corresponding value of block size, LDPC BG selection,
and LDPC lifting size can be derived. In our link-level simulator,
the block size remains below the maximum CBS (i.e., 8448 bits  for  LDPC  BG1
or 3840 bits  for  LDPC  BG2), since code block segmentation is integrated into
the proposed NR PHY abstraction model to speed up the simulation rate.

Note that the SINR-BLER curves obtained from the link-level simulator
are quantized and consider a subset of CBSs. Accordingly, in the 'NR' module,
we implement a worst case approach to determine the code BLER value by
using lower bounds of the actual CBS and effective SINR.
In the PHY abstraction for HARQ-IR, for simplicity and according to the obtained curves,
we limit the effective ECR by the lowest ECR of the MCSs that have the same modulation
order as the selected MCS index. The coded bits beyond that ECR are credited as a
repetition gain of the effective SINR (attribute ``CreditRateBelowModulationFloor`` of
``NrEesmIr``, enabled by default), as detailed in the HARQ section.

**Transport BLER computation**: In case there is code block segmentation, there is
a need to convert the code BLER found from the link-level simulator's lookup table
to the transport BLER for the given TBS.
The code BLERs of the :math:`C` code blocks (as determined by the code block segmentation)
are combined to get the BLER of a transport block as:

:math:`TBLER = 1- \prod_{i=1}^{C} (1-CBLER_i) \approxeq 1- (1-CBLER)^C`.

The last approximate equality is implemented in the 'NR' simulator, which
holds because code block segmentation in NR generates code blocks of roughly equal sizes.


Beamforming model
=================

Beamforming model supports two types of beamforming algorithms: ideal and realistic.
Ideal BF methods determine the BF vectors based on either the
assumption of the perfect knowledge of the channel (e.g., cell scan method),
or the exact positions of the devices (e.g., DoA method), and they do not consume
any time/frequency overhead to design the BF vectors.
On the other hand, realistic BF methods, are expected to select the best BF
based on some real measurements, e.g., estimate the channel based on SRSs.

For each type of beamforming methods, there is a beamforming helper, that helps the
user to create BF tasks. For this purpose are created ``IdealBeamformingHelper`` and
``RealisticBeamformingHelper``, which implement
``BeamformingHelperBase`` interface.
When a UE is attached to a gNB, the beamforming helper creates a BF task.
The BF task is composed of a pair of connected gNB and UE devices for which
the BF helper will manage the update of the BF vectors by calling
``GetBeamformingVectors`` of the configured BF algorithm.

In :numref:`fig-rbf-impl`, we show the class diagram of the beamforming model.

.. _fig-rbf-impl:

.. figure:: figures/rbf-impl.*
   :align: center
   :scale: 50 %

   Diagram of beamforming model, dependencies on 3GPP channel related classes, and ``NrGnbPhy``.


The main difference between ``IdealBeamformingHelper`` and
``RealisticBeamformingHelper`` is that ideal helper triggers the update of
BF vectors of all devices at the same time based on
the configured periodicity through the ``BeamformingPeriodicity`` attribute
of the ``IdealBeamformingHelper`` class.
On the other hand, ``RealisticBeamformingAlgorithm`` triggers the update of
the BF vectors when the configured trigger event occurs, and then
only the BF vectors of the pair of devices for which the SRS measurement has been
reported (pair of gNB and UE) are being updated.


**Ideal beamforming**

All ideal BF algorithms inherit ``IdealBeamformingAlgorithm`` class which
implements the ``BeamformingAlgorithm`` interface and thus must override the function
``GetBeamformingVectors`` which determines the BF vectors to be used on a pair of devices, i.e., gNB and UE.

The 'NR' module supports different ideal methods:
beam-search or cell-scan method (``CellScanBeamforming``),
LOS path or DoA method (``DirectPathBeamforming``),
LOS path at gNB and quasi-omni at UE (``DirectPathQuasiOmniBeamforming``),
beam-search at gNB and quasi-omni at UE (``CellScanQuasiOmniBeamforming``), and
quasi-omni at gNB and LOS path at UE (``QuasiOmniDirectPathBeamforming``).

*  ``CellScanBeamforming`` implements a type of ideal BF algorithm that
   searches for the best pair of BF vectors (providing a highest average SNR)
   from the set of pre-defined BF vectors assuming
   the perfect knowledge of the channel.
   For the beam-search method, our simulator supports abstraction of the
   beam ID through two angles (azimuth and elevation).
   A new interface allows you to have the beam ID available at MAC layer for
   scheduling purposes.

*  ``DirectPathBeamforming`` assumes knowledge of the pointing angle in between devices,
   and configures transmit/receive beams pointing into the LOS path direction.

*  ``DirectPathQuasiOmniBeamforming`` uses the LOS path for configuring gNB beams,
   while configures quasi-omnidirectional beamforming vectors at UEs (for transmission and reception).

*  ``QuasiOmniDirectPathBeamforming`` configures quasi-omnidirectional BF vectors at gNB,
   while uses the LOS path for configuring UE beams (for transmission and reception).

*  ``CellScanQuasiOmniBeamforming`` configures cell-scan BF vectors at gNB and
   quasi-omni BF vectors at UE.

Previous models were supporting also long-term covariance matrix based method
(``OptimalCovMatrixBeamforming``) which is currently not available due to
incompatibility with the latest ns-3 3GPP channel model.
Modifications are needed in order to port this method from the
old 5G-LENA code-base and adapt it to the latest ns-3 3gpp channel model
(Contributions are welcome!). ``OptimalCovMatrixBeamforming`` determines
the optimal transmit and receive beam based on the perfect knowledge
of the channel matrix.

**Realistic beamforming**

To implement a new realistic BF algorithm, we have created a separate class called
``RealisticBeamformingAlgorithm`` which relies on SRS SINR/SNR measurements to determine BF vectors.
Similarly to previously mentioned ``CellScanBeamforming``, there is a set of
pre-defined BF vectors, but the knowledge of the channel is not perfect,
and depends on the quality of reported SRS measurement. The SRS measurement (SINR or SNR)
can be configured through the ``UseSnrSrs`` attribute of ``RealisticBeamformingAlgorithm``.
Basically, if the SNR is used, no interference is assumed in SRS; meanwhile if SINR measurement is used,
the worst-case inter-cell interference is assumed (i.e., SRS that use the same SRS resource will interfere,
independently of the orthogonality of the employed Zadoff-Chu sequences).
The BF vector trigger update event can be either SRS count event
(e.g., after every N SRSs are received, the BF vectors are updated),
or based on the delay event after SRS reception (e.g., :math:`\delta`
time after each SRS reception).
The type of event and its parameters can be configured through ``RealisticBfManager`` class.
Hence, in order to use realistic BF functionality it is necessary to install
``RealisticBfManager`` at gNBs PHY instead of the default ``BeamManager`` class.
The configuration of trigger event and its parameters can be done per
gNB instance granularity, but can be easily extended to be done per UE.

For each BF task, an instance of realistic BF algorithm is created,
which is then connected to ``NrSpectrumPhy`` SRS SINR/SNR trace to receive SRS reports.
Realistic BF algorithm is also connected to its helper through a callback to
notify it when BF vectors of a device pair need to be updated
(based on configuration and SRS reports).
When BF vectors need to be updated, the function ``GetBeamformingVector``
or realistic BF algorithm is called, which calls ``GetEstimatedLongTermComponent``
for each pair of pre-defined beams of the receiver and transmitter in order to
estimate the channel quality of each of them based on the SRS reports.
The estimation of the channel is done based on the abstraction model explained in the following
section.
Finally, ``CalculateTheEstimatedLongTermMetric`` calculates the metric that is used to select the
best BF pair.

In :numref:`fig-rbf-impl`, we show the diagram of the classes that are used for realistic
BF based on SRS measurements, the dependencies among classes, and the most important
methods. E.g., we can see that `RealisticBeamformingAlgorithm`
needs to access to `ThreeGppChannelModel` to obtain the channel matrix in order to perform the estimation of the channel based on SRS report.


**Abstraction model for SRS-based channel estimation**

Assume a single-antenna system. Let
:math:`h` denote the (complex-valued) small-scale fading channel
between a UE and a gNB. Then, the estimation of the small-scale fading channel
at the gNB can be modeled as in [SigProc5G]_ :

:math:`\hat{h} = \alpha (h+e)`,

where :math:`\alpha` is a scaling factor to maintain normalization of
the estimated channel, and :math:`e` is the white complex Gaussian
channel estimation error.
The estimation error is assumed to be characterized by zero-mean and variance
:math:`\sigma_e^2`.

The variance of the error is given by:

:math:`\sigma_e^2 = \frac{1}{(SINR \cdot \Delta)}`,

where SINR is the received linear SINR (or SNR) of SRS at the gNB and :math:`\Delta`
is the gain obtained from time-domain filtering during the channel estimation.
According to 3GPP analysis of SRS transmission, :math:`\Delta`
is set to 9 dB [SigProc5G]_. The scaling factor is given by:

:math:`\alpha = \sqrt{\frac{1}{(1+\sigma_e^2)}}`.

Then, the channel matrix estimate can be used to compute transmit/receive BF
vectors, as part of the beam management.


SRS transmission and reception
==============================

SRS transmission typically spans over 1, 2 or 4 consecutive OFDM symbols at the end
of the NR slot. 5G-LENA implements such behaviour in the time domain by allowing
different configurations. In the frequency domain, in order to allow frequency multiplexing,
SRS is typically transmitted over only a subset of subcarriers, defined by the
configuration, e.g., each 2nd or each 4th subcarrier is used for SRS transmission.
However, since the minimum transmission granularity in 5G-LENA module is a RB in frequency domain,
all subcarriers are used for SRS transmission.
:numref:`fig-srs-5glena` shows the slot structure and the symbols over which the
SRS transmission spans, assuming a repeated TDD pattern structure of
[DL F UL UL UL] (i.e., one DL slot, followed by one flexible slot and three UL
slots and that SRS transmissions occur in F slots (i.e., slots number 1 and 6 in the figure).
In 5G-LENA, flexible slots consist of DL/UL control symbols and
a variable number of DL and UL symbols for data; DL slots carry only DL control
and DL data; and UL slots consist of UL data and UL control parts.

.. _fig-srs-5glena:

.. figure:: figures/srs-ext3.*
   :align: center
   :scale: 50 %

   Example of SRS transmissions of 4 different UEs (maximum 1 UE SRS transmission per slot, as per 5G-LENA design), considering SRS periodicity equal to 20 slots. Numerology considered is :math:`\mu=0`. F stands for frame and SF for subframe.


In 5G NR, SRS parameters, such as periodicity and offset, are typically configured
by RRC and notified to UE [TS38331]_. Another option is to have gNB MAC scheduler to
determine the SRS periodicity/offset and then to notify UE through DCI format 2\_3 on
which resources SRS should be transmitted [TS38212]_. The latter option, scheduling-based SRS,
is a more dynamic approach and allows more flexible SRS parameter and periodicity assignment,
e.g., when there are less UEs, a lower periodicity value can be used, while when there
are more UEs, the gNB MAC scheduler can dynamically increase the periodicity and then
update the offsets accordingly. We have implemented scheduling-based SRS.

To allow dynamic SRS scheduling and adjustment of SRS periodicity/offset of all UEs,
we introduced ``NrMacSchedulerSrs`` and ``NrMacSchedulerSrsDefault`` into 5G-LENA model.

``NrMacSchedulerSrs`` is an interface that is used by the NR gNB MAC scheduler to obtain
the SRS offset/periodicity for a UE. There can be various implementations
of this interface that would simulate different algorithms for SRS
offset/periodicity generation.
In ``NrMacSchedulerSrsDefault``, we provide one possible implementation.
Each time a new UE is attached it is called the function ``AddUe``
that returns the offset/periodicity configuration. When scheduler detects
that the SRS periodicity is too small for the number of UEs it calls the
``IncreasePeriodicity``, which picks up the next periodicity value from the list
of standard  values, i.e., 2, 4, 5, 8, 10, 16, 20, 32, 40, 64, 80, 160, 320, 640,
1280, 2560 slots [TS38331]_.
Scheduling-based SRS is more flexible approach than SRS configuration through
RRC. E.g., in 4G-LENA SRS configuration is through RRC and static, which requires
that a user needs to  configure SRS periodicity based on the maximum expected number
of UEs in the simulation scenario. To allow dynamic SRS periodicity adaptation in
5G-LENA, it was necessary to set a constraint which is that at most 1 UE can send
the SRS in a single slot.

Configuration parameters related to SRS transmissions are specified in ``NrMacSchedulerNs3``
class. The user can configure the number of SRS symbols that will be allocated
for SRS transmission through the attribute ``SrsSymbols``. Additionally,
the user can configure whether SRS will be transmitted only in flexible slots,
or in both flexible and UL slots by setting the attribute ``EnableSrsInUlSlots``.


.. _UplinkPowerControl:

Uplink power control
====================

Uplink Power Control (ULPC) allows an eNB to adjust the transmission power
of an UE, and as such it plays a critical role in reducing inter-cell Interference.
In LTE and NR, the standardized procedure can have two forms: open and closed loop,
where closed loop relies on open loop functionality, and extends it with control
coming from eNB. Open loop can be entirely implemented at UE side, while the
closed loop depends on the algorithm and the logic implemented at eNB.
Open loop in general is aimed to compensate the slow variations of the received
signal (i.e., path loss and shadowing), while CLPC is used to further adjust
the UEs’ transmission power so as to optimize the overall system performance.
ULPC determines a power for different types of transmissions, such as, PUSCH,
PUCCH, and SRS.

As a starting point for the development of ULPC feature for LTE/NR we have used
implementation that was already available in ns-3 simulator in LTE module
(see the full description here: ns-3 LTE Uplink Power Control Design [lte-ulpc]_.
However, this class only supports PUSCH and SRS power control, while there is
no support for PUCCH. Since in the goal is to have a high fidelity simulations
with realistic uplink transmissions, including PUCCH, CLPC for PUCCH is a
mandatory feature. Whatsoever in ns-3 LTE module, this was not considered
important since in ns-3 LTE models all uplink control messages are modeled
as ideal (do not consume resources, and hence no error model). Moreover,
ns-3 LTE ULPC implements only TS 36.213, which is limited only to a specific
set of frequencies. A newer TS 38.213 extends TS 36.213 and allows its
application in a wider range of frequencies. In our extended model,
we have added support for an independent reporting of Transmit Power Command
(TPC) for PUSCH/SRS and PUCCH.

ULPC is implemented in ``NrUePowerControl`` class which computes and updates
the power levels for PUSCH, SRS transmissions and PUCCH. It supports open and closed loop modes.
According the open loop the UE transmission power depends on the estimation
of the downlink path loss and channel configuration. On the other hand,
closed loop, additionally allows the gNB to control the UE transmission
power by means of explicit TPC included in the DCI. In closed Loop, two modes are available: the absolute mode, according
to which the txPower is computed with absolute TPC values, and the accumulation
mode, which instead computes the txPower using accumulated TPC values. When
the MAC scheduler creates DCI messages, it calls the GetTpc function to ask
for TPC values that should be sent to each UE.

NrUePowerControl is inspired by LteUePowerControl, but most of the parts
had to be extended or redefined. Comparing to LteUePowerControl, the
following features are added:

- PUCCH power control,
- low bandwidth and enhanced coverage BL/EC devices,
- independent TPC reporting for PUSCH and PUCCH,
- TS 38.213 technical specification for NR uplink power control (PUSCH, PUCCH, SRS power control)
- upgrade the API to reduce time/memory footprint that could affect significantly large scale simulations.


Moreover, NrUePowerControl includes a full implementation of LTE and NR
uplink power control functionalities, by allowing a user to specify in
which mode the power control will execute: LTE/LAA (TS 36.213) or NR (and TS 38.213)
uplink power control (i.e., by using TSpec attribute of NrUePowerControl).
As a results, NrUePowerControl supports the following:

- PUSCH power control implementation for LTE and NR
- PUCCH power control implementation for LTE and NR
- SRS power control implementation for LTE and NR
- CLPC implementation (accumulation and absolute modes)


**LTE PUSCH power control**

The formula for LTE PUSCH is provided in Section 5.1.1.1 of TS 36.213.
There are two types of formulas, for simultaneous transmission of PUSCH and PUCCH,
and separated. Currently we have implemented the option when the
transmissions of PUCCH and PUSCH are not simultaneous, since this is
the current model design of both LTE (LENA v1) and NR (LENA v2) modules.
The following formula defines the LTE PUSCH power control that we implemented
in NrUePowerControl class:


.. _fig-ulpc-pusch-36213:

.. figure:: figures/ulpc/pusch-1.*
   :align: center
   :scale: 35 %


*  :math:`P_{CMAX,c}(i)` is the UE configured maximum output transmit power defined as defined in 3GPP 36.101. (Table 6.2.2-1)
   in a subframe :math:`i` for the serving cell :math:`c`, and  default value for :math:`P_{CMAX,c}(i)` is 23 dBm.

*  :math:`M_{PUSCH,c}(i)` is the bandwidth of the PUSCH resource assignment expressed in number
   of resource blocks used in a subframe :math:`i` and serving cell :math:`c`.

*  :math:`P_{O\_PUSCH,c}(j)` is a parameter composed of the sum of a component :math:`P_{O\_NOMINAL\_PUSCH,c}(j)`
   provided from higher layers :math:`j={0,1}` and a component :math:`P_{O\_UE\_PUSCH,c}(j)` provided by higher
   layers for :math:`j={0,1}` for serving cell :math:`C`. SIB2 message needs to be extended to carry these two
   components, but currently they can be set via attribute system using ``NrUePowerControl`` class attributes: ``PoNominalPusch``
   and ``PoUePusch``.

*  :math:`\alpha_{c} (j)` is a 3-bit parameter provided by higher layers for serving cell :math:`c`.
   For :math:`j=0,1`, :math:`\alpha_c \in \left \{ 0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1 \right \}` .
   For :math:`j=2`, :math:`\alpha_{c} (j) = 1`. This parameter is configurable by attribute system
   by setting ``Alpha`` attribute of ``NrUePowerControl`` class.

*  :math:`PL_{c}` is the downlink pathloss estimate calculated at the UE for the serving cell :math:`c` in dB
   and :math:`PL_{c} = P_{RS} – P_{RSRP}`, where :math:`P_{RS}` is provided by higher layers.
   P_{RSRP} is filtered by higher layers. :math:`P_{RS}` is provided in SIB2 message.

*  :math:`\Delta_{TF,c}(i)` is calculated based on :math:`K_{s}` which is provided by the
   higher layers for each serving cell. When :math:`K_{s} = 1.25` then :math:`\Delta_{TF,c}(i)`
   is calculated by using the following formula:
   :math:`\Delta_{TF,c}(i) = 10\log_{10}((2^{BPRE\cdot K_s}-1)\cdot\beta_{offset}^{PUSCH} )`.
   On the other hand, when :math:`K_{s} = 0`, :math:`\Delta_{TF,c}(i) = 0`.
   Currently, the latter option is set by default, and alternatively, the value could be dynamically
   set through set function according to the previously mentioned formula and attribute settings.

*  :math:`f_{c}(i)` is component of Closed Loop Power Control. It is the current PUSCH power control
   adjustment state for serving cell :math:`c`.

   If Accumulation Mode is enabled :math:`f_{c}(i)` is given by:

   .. math::

      f_{c}(i) = f_{c}(i-1) + \delta_{PUSCH,c}(i - K_{PUSCH})

   where :math:`\delta_{PUSCH,c}` is a correction value, also referred to as a TPC command and is included
   in PDCCH with DCI; :math:`\delta_{PUSCH,c}(i - K_{PUSCH})` was signalled on PDCCH/EPDCCH with DCI for
   serving cell :math:`c` on subframe :math:`(i - K_{PUSCH})`; :math:`K_{PUSCH} = 4` for FDD.

   If UE has reached :math:`P_{CMAX,c}(i)` for serving cell :math:`c`, positive TPC commands for serving cell
   :math:`c` are not accumulated. If UE has reached minimum power, negative TPC commands are not accumulated.
   Minimum UE power is defined in TS 36.101 section 6.2.3. Default value is -40 dBm.

   If Accumulation Mode is not enabled :math:`f_{c}(i)` is given by:

      .. math::

         f_{c}(i) = \delta_{PUSCH,c}(i - K_{PUSCH})

   where: :math:`\delta_{PUSCH,c}` is a correction value, also referred to as a TPC command and is included
   in PDCCH with DCI; :math:`\delta_{PUSCH,c}(i - K_{PUSCH})` was signalled on PDCCH/EPDCCH with DCI for
   serving cell :math:`c` on subframe :math:`(i - K_{PUSCH})`; :math:`K_{PUSCH} = 4` for FDD.

   Mapping of TPC Command Field in DCI format 0/3/4 to absolute and accumulated :math:`\delta_{PUSCH,c}`
   values is defined in TS36.231 section 5.1.1.1 Table 5.1.1.1-2.




**NR PUSCH power control**

NR PUSCH formula is provided in Section 7.1.1 of TS 38.213. This formula is analog to the LTE PUSCH
power control, except that is more generic and that the TPC accumulation state is calculated differently.
NR PUSCH formula depends of the numerology (0 - 4, 0 for LTE) and this makes the formula more generic,
and flexible for different subcarrier spacing configurations:


.. _fig-ulpc-pusch-38213:

.. figure:: figures/ulpc/pusch_38213.*
   :align: center
   :scale: 35 %


*  :math:`P_{CMAX,f,c}(i)` is the UE configured maximum output transmit power defined in [8-1, TS 38.101-1], [8-2, TS38.101-2]
   and [TS38.101-3] for carrier :math:`f` for serving cell :math:`c` in PUSCH transmission occasion :math:`i`. Default
   value for :math:`P_{CMAX,f,c}(i)` is 23 dBm.

*  :math:`P_{O\_PUSCH,b,f,c}(j)` is a parameter composed of the sum of a component :math:`P_{O\_NOMINAL\_PUSCH,b,f,c}(j)`
   provided from higher layers and a component :math:`P_{O\_UE\_PUSCH,b,f,c}(j)` provided by higher layers for serving
   cell :math:`C` where :math:`j \in \left \{0, 1, ..., J-1 \right \}`. These attributes can be set in the same way as
   for TS 36.213 PUSCH per each bandwidth part independently.

*  :math:`M_{RB,b,f,c}^{PUSCH}(i)` is the bandwidth of the PUSCH resource assignment expressed in number
   of resource blocks for PUSCH transmission occasion :math:`i` on active UL BWP :math:`b` of carrier :math:`f` and
   serving cell :math:`c`. :math:`\mu` is numerology used for SCS configuration defined in [TS 38.211].

*  :math:`\alpha_{b,f,c} (j)` is a 3-bit parameter provided by higher layers. Currently, allowed values
   for this parameters are the same as for TS 36.213 :math:`\alpha_{c} (j)`, and can be configured in
   the same way (See previous section).

*  :math:`PL_{b,f,c}` is the downlink pathloss estimate in dB that is calculated at the UE for the active DL BWP :math:`b` if
   carrier :math:`f` and serving cell :math:`c`. The calculation of pathloss is the same as explained in the previous section.

*  :math:`\Delta_{TF,b,f,c}(i)` is calculated in the same way as in previous section, i.e.,
   :math:`K_{s} = 1.25` then :math:`\Delta_{TF,b,f, c}(i)` is calculated in the following way:
   :math:`\Delta_{TF,b,f,c}(i) = 10\log_{10}((2^{BPRE\cdot K_s}-1)\cdot\beta_{offset}^{PUSCH})`.
   Otherwise, when :math:`\Delta_{TF,b,f,c}(i)`, then :math:`\Delta_{TF,b,f, c}(i) = 0`. :math:`\Delta_{TF,b,f,c}(i)`
   can be dynamically set through the set function of ``NrUePowerControl``.


*  :math:`f_{b,f,c}(i,l)` is component of Closed Loop Power Control. It is the PUSCH power control adjustment state
   :math:`l` for active UL BWP :math:`b` of carrier :math:`f` of serving cell :math:`c` and PUSCH transmission
   occasion :math:`i`.


   If Accumulation Mode is enabled :math:`f_{b,f,c}(i,l)` is given by:

      .. math::

         f_{b,f,c}(i,l) = f_{b,f,c}(i-i_0,l) + \delta_{PUSCH,c}(i - K_{PUSCH})

   where: :math:`\delta_{PUSCH,c}` is a correction value, also referred to as a TPC command and is included
   in PDCCH with DCI; :math:`\delta_{PUSCH,c}(i - K_{PUSCH})` was signalled on PDCCH/EPDCCH with DCI for
   serving cell :math:`c` on subframe :math:`(i - K_{PUSCH})`; :math:`K_{PUSCH} = 4` for FDD.

   If UE has reached :math:`P_{CMAX,c}(i)` for serving cell :math:`c`, positive TPC commands for serving cell
   :math:`c` are not accumulated. If UE has reached minimum power, negative TPC commands are not accumulated.
   Minimum UE power is defined in TS36.101 section 6.2.3. Default value is -40 dBm.

   If Accumulation Mode is not enabled :math:`f_{c}(i)` is given by:


.. _fig-ulpc-fc-38213:

.. figure:: figures/ulpc/fc_38213.*
   :align: center
   :scale: 35 %


*  :math:`\sum_{m=0}^{C(D_i)-1} \delta_{PUSCH,b,f,c}` is a sum of TPC command values in a set :math:`D_i` of TPC
   command values with cardinality :math:`C(D_i)` that the UE receives between :math:`K_{PUSCH}(i-i_0) -1`
   symbols before PUSCH transmission occasion :math:`i-i_0` and :math:`K_{PUSCH}(i)` symbols before PUSCH
   transmission occasion :math:`i` on active UL BWP :math:`b` of carrier :math:`f` and serving cell :math:`c`
   for PUSCH power control adjustment state :math:`l`, where :math:`i_0` is the smallest integer for which
   :math:`K_{PUSCH}(i-i_0)` symbols before PUSCH transmission occasion :math:`i-i_0` is earlier than
   :math:`K_{PUSCH}(i)` symbols before PUSCH transmission occasion :math:`i`.
   This definition is quite different from the one that we have seen in TS 36.213 PUSCH,
   hence this is probably the component in formula that could make an important difference in power adjustment
   when choosing among TS 36.213 and TS 38.213 formula in NrUePowerControl class.
   The difference with respect to the TS 36.213 formula for accumulation is that this formula is being calculated per
   transmission occasion, while accumulation component in TS 38.213 is being constantly updated as
   TPC commands arrive regardless when of the transmission occasion event happens.

   On the other hand, if accumulation mode is not enabled :math:`f_{b,f,c}` is given by the following
   expression:

   .. math::

      f_{b,f,c}(i,l) = \delta_{PUSCH,b,f,c}(i,l),

   where :math:`\delta_{PUSCH,b,f,c}(i,l)` is the absolute values that is given in Table 7.1.1-1 of TS 38.213.
   The following table illustrates which absolute and accumulated :math:`\delta_{PUSCH,b,f,c}` corresponds to
   each TPC command.

.. table:: TPC commands

   +---------------+-------------------------------------------+---------------------------------------+
   | TPC command   |  Accumulated :math:`\delta_{PUSCH,b,f,c}` | Absolute :math:`\delta_{PUSCH,b,f,c}` |
   +===============+===========================================+=======================================+
   |       0       |                   -1                      |                 -4                    |
   +---------------+-------------------------------------------+---------------------------------------+
   |       1       |                    0                      |                 -1                    |
   +---------------+-------------------------------------------+---------------------------------------+
   |       2       |                    1                      |                  1                    |
   +---------------+-------------------------------------------+---------------------------------------+
   |       3       |                    3                      |                  4                    |
   +---------------+-------------------------------------------+---------------------------------------+


**PUCCH power control**

Similarly to PUSCH power calculation there is a lot of similarities in the formulas for PUCCH between
TS 36.213 and TS 38.213. Hence, we will not enter in the details to explain each of the components since
their equivalents were already explained in the previous sections, such as :math:`P_{CMAX,c}(i)` and :math:`P_{CMAX,f,c}(i)`,
:math:`P_{O\_PUCCH,c}(j)` or :math:`P_{O\_PUCCH,b,f,c}(j)`, which are, for example, equivalent to
:math:`P_{O\_PUSCH,c}(j)` or :math:`P_{O\_PUSCH,b,f,c}(j)`, respectively.
Also, an interested reader is referred to technical specifications (TS 36.213 and TS 38.213)
for more detailed explanations. Formula for TS 36.213 PUCCH is provided in Section 5.1.2.1 of TS 36.213,
while formula for NR PUCCH power control is provided in Section 7.1.2. of TS 38.213.
Both of these are shown in continuation, and as such are implemented in NrUePowerControl class.
Note that with respect to PUSCH there is no absolute mode of TPC feedback for PUCCH, hence, accordingly,
only accumulation mode is implemented. Similarly to PUSCH implementation,
the value :math:`\Delta_{TF,b,f,c}(i) = 0` by default is 0 (assuming :math:`K_s=0`) or
could be dynamically set through set function according to corresponding formula.
PUCCH ULPC formula according to 36.213 is calculated by the following formula:

.. _fig-ulpc-pucch-36213:

.. figure:: figures/ulpc/pucch_36.213.*
   :align: center
   :scale: 35 %

PUCCH ULPC formula according to 38.213 is given in the following:

.. _fig-ulpc-pucch-38213:

.. figure:: figures/ulpc/pucch_38213.*
   :align: center
   :scale: 35 %


**SRS power control**

LTE SRS power control formula is provided in Section 5.1.3.1 of TS 36.213,
while  NR SRS power control formula is provided in Section 7.1.3 of TS 38.213.
We will again skip repeating the explanation of each of the components of the formula
as the equivalents were already explained before. Reader should note that these
formulas rely on PUSCH power control, e.g., LTE SRS power control relies on
:math:`P_{O\_PUSCH,c}(j)` and :math:`fc(i)`.
In the following we provide formulas that are implemented in NrUePoweControl
for LTE and NR SRS transmissions. SRS ULPC formula according to 36.213 is given by:


.. _fig-ulpc-srs-36213:

.. figure:: figures/ulpc/srs_ts36213.*
   :align: center
   :scale: 35 %


:math:`P_{SRS\_OFFSET,c}(m)` is semi-statically configured by higher layers for m=0,1 for
serving cell c. For SRS transmission given trigger type 0 then m=0,1 and for SRS
transmission given trigger type 1 then m=1.
For K_{s} = 0 :math:`P_{SRS\_OFFSET,c}(m)` value is computed with equation:

   .. math::

      P_{SRS\_OFFSET,c}(m)value = -10.5 + P_{SRS\_OFFSET,c}(m) * 1.5 [dBm]


SRS ULPC formula according to 38.213 is as follows:

.. _fig-ulpc-srs-38213:

.. figure:: figures/ulpc/srs_ts38213.*
   :align: center
   :scale: 35 %


**Closed loop power control (CLPC)**

As we could see in previous formulas there is a difference in the way the accumulation
of TPC commands is performed in LTE and NR. In LTE it happens synchronously, always considering
TPC command that was received in (i−KPUSCH) subframe, while for NR it is necessary to take
into account different TPC commands depending on the transmission occasion i−i0 and how many
symbols have passed since last PDCCH and the first symbol of the current transmission occasion.
In :ref:`fig-ulpc-7` we illustrate a sequence diagram of how LTE and NR CLPC are
implemented and at which point the accumulation state is being updated for each of them.
From the sequence diagram we can see that once the NrUePowerControl receives
TPC command LTE CLPC updates the accumulation state, while NR CLPC only saves
the value and it updates it only at the next transmission occasion.


.. _fig-ulpc-7:

.. figure:: figures/ulpc/nr-clpc.*
   :align: center
   :scale: 35 %

   LTE/NR CLPC collaboration diagram: TPC command being sent by NrGnbPhy with DCI,
   and the TPC command reception, reporting to NrUePowerControl and applying for
   the next transmission occasion.

HARQ
====
The NR scheduler works on a slot basis and has a dynamic nature [TS38300]_.
For example, it may assign different sets of OFDM symbols in time and RBs
in frequency for transmissions and the corresponding redundancy versions.
However, it always assigns an integer multiple of RBs consisting of 12
resource elements in the frequency domain and 1 OFDM symbol in the time domain.
In our module, for simplicity, we assume that retransmissions
(including the first transmission and the corresponding redundancy versions)
of the same HARQ process use the same MCS and the same number of RBs,
although the specific RBs' time/frequency positions within a slot may vary
in between the retransmissions. Also, the SINRs experienced
on each RB may vary through retransmissions. As such, HARQ affects both the PHY
and MAC layers.

The redundancy version (RV) field carried in the DCIs and reported in the PHY
traces is used here as the transmission-attempt index of the HARQ process: it is
0 for the first transmission and increases by one for each retransmission, rather
than being restricted to the values 0 to 3.

The 'NR' module supports two HARQ methods: Chase Combining (HARQ-CC)
and Incremental Redundancy (HARQ-IR).

At the PHY layer, the error model has been extended to support HARQ with
retransmission combining.
Basically, it is used to evaluate the correctness of the blocks received and
includes the messaging algorithm in charge of communicating to the HARQ entity
in the scheduler the result of the combined decodifications. The EESM for
combined retransmissions
varies with the underline HARQ method, as detailed next.

**HARQ-CC:** In HARQ-CC, every retransmission contains the same coded bits
(information and coding bits). Therefore, the effective code rate (ECR)
after the q-th retransmission remains the same as after the first transmission.
In this case, the SINR values of the corresponding resources are summed across
the retransmissions, and the combined SINR values are used for EESM. After q
retransmissions, in the 'NR' simulator, the effective SINR using EESM
is computed as:

:math:`SINR_{\text{eff}} = {-}\beta \ln \Big( \frac{1}{|\omega|}\sum_{m \in \omega} \exp \big({-}\frac{1}{\beta}\sum_{j=1}^q \text{SINR}_{m,j}\big)\Big)`,

where :math:`\text{SINR}_{m,j}` is the SINR experienced by the m-th RB in the j-th
retransmission, and :math:`\omega` is the set of RBs to be combined.

**HARQ-IR:** In HARQ-IR, every retransmission contains different coded bits than
the previous one. The different retransmissions typically use a different set of
coding bits. Therefore, both the effective SINR and the ECR need to be
recomputed after each retransmission.
The ECR after q retransmissions is obtained in the 'NR' simulator as:

:math:`\text{ECR}_{\text{eff}} = \frac{X}{\sum_{j=1}^q C_j}`,

where X is the number of information bits and :math:`C_j` is the number of
coded bits in the j-th retransmission.
The effective SINR using EESM after q retransmissions is computed over the coded bits of
all of them:

:math:`\text{SINR}_{\text{eff}} = {-}\beta \ln \Big( \frac{1}{\sum_{j=1}^q |\omega_j|}\sum_{j=1}^q \sum_{m \in \omega_j}\exp\big({-}\frac{\text{SINR}_{m,j}}{\beta}\big)\Big)`,

where :math:`\text{SINR}_{m,j}` is the SINR experienced by the m-th RB in the j-th
retransmission and :math:`\omega_j` is the set of RBs of the j-th retransmission. The
effective SINR does not grow when the retransmissions have the same SINR: the combining gain
comes from the ECR, and the BLER is read from the SINR-BLER curve of the MCS whose ECR is the
closest above :math:`\text{ECR}_{\text{eff}}` among the MCSs of the modulation order of the
transmission. Below the lowest ECR of that modulation order,
:math:`\text{ECR}_{\text{min}}`, the curve no longer changes, so further retransmissions
would gain nothing, and HARQ-IR would be worse than HARQ-CC, while incremental redundancy is
never worse than Chase combining. The coded bits beyond :math:`\text{ECR}_{\text{min}}` are
therefore credited as a repetition (Chase) gain, a lower bound of their incremental redundancy
gain:

:math:`\text{SINR}_{\text{eff}} \leftarrow \text{SINR}_{\text{eff}} \, \frac{\text{ECR}_{\text{min}}}{\text{ECR}_{\text{eff}}}`
when :math:`\text{ECR}_{\text{eff}} < \text{ECR}_{\text{min}}`.

This credit (attribute ``CreditRateBelowModulationFloor`` of ``NrEesmIr``) is enabled by
default. With it, HARQ-IR is at least as good as HARQ-CC for the 16-QAM and 64-QAM MCSs, and
equal to it when the first transmission uses the lowest MCS of its modulation order. For the
QPSK MCSs it can still fall short of HARQ-CC near the waterfall: their SINR-BLER curves are
closer to each other than their code rates, so moving down to a lower MCS gains less than the
ratio of the code rates.


At the MAC layer, the HARQ entity residing in the scheduler is in charge of
controlling the HARQ processes for generating new packets and managing the
retransmissions both for the DL and the UL. The scheduler collects the HARQ
feedback from gNB and UE PHY layers (respectively for UL and DL connection)
by means of the FF API primitives ``SchedUlTriggerReq`` and ``SchedUlTriggerReq``.
According to the HARQ feedback and the RLC buffers status, the scheduler generates
a set of DCIs including both retransmissions of HARQ blocks received erroneous
and new transmissions, in general, giving priority to the former.
On this matter, the scheduler has to take into consideration one constraint
when allocating the resource for HARQ retransmissions, it must use the same
modulation order of the first transmission attempt. This restriction comes from
the specification of the rate matcher in the 3GPP standard [TS38212]_, where
the algorithm fixes the modulation order for generating the different blocks
of the redundancy versions.

The 'NR' module supports multiple (20) stop and wait processes to allow continuous data flow. The model is asynchronous for both DL and UL transmissions. The transmissions, feedback, and retransmissions basically depend on the processing timings, the TDD pattern, and the scheduler. When ``EnableHarqReTx`` is true, the number of retransmissions per HARQ process is bounded by ``MaxHarqReTx``. Setting ``EnableHarqReTx`` to false disables retransmissions regardless of the value of ``MaxHarqReTx``.


MIMO
====
In real systems, devices capable of performing MIMO spatial multiplexing can use more than one stream to transmit,
e.g., a gNB in the downlink can send multiple streams to those UEs that support MIMO spatial multiplexing and are able to decode
multiple streams simultaneously. MIMO technology is known in textbooks for several decades, and it is used in 4G-LTE and 5G-NR,
and has been in Wi-Fi products for more than 20 years. For optimal MIMO performance, the gNB should apply a precoding matrix
(at the transmitter) that determines how the signal is aligned relative to the channel, and the UE (at the receiver) should
apply a receive filter to suppress inter-stream interference and recover each stream. Usually, the precoding matrix that provides
the maximum SINR for the given channel matrix is selected [Palomar2006]_ and the receive filter is usually designed to reduce the
mean-square error between the transmitted and decoded data symbols, for which the MMSE-IRC Interference Rejection Combining
receiver is usually adopted in 3GPP, as it provides a good balance between performance (mean-square error reduction) and
implementation complexity (linear receiver). In 3GPP, MIMO spatial multiplexing is permitted and enabled thanks to the CSI
feedback, which includes the Precoding Matrix Indicator (PMI), the Rank Indicator (RI), and the Channel Quality Indicator (CQI).
The selection of the precoding matrix that gives the best performance (maximum SINR) is done by the UE, and reported through the
PMI through an index of a set of predefined precoding matrix from a codebook, as part of the CSI feedback message to the gNB.
In addition, the UE also reports the RI as part of the CSI feedback, which indicates to the gNB how many streams to use for that UE.
Even if RI=1, we can still use precoding to combine the signals to/from multiple antenna ports in an optimal way. Each antenna
port is then indeed aligned to the channel through beamforming. 3GPP 5G NR, differently from LTE, allows sending up to 4 streams
in the same TB. Indeed, in 3GPP, the MIMO operations are enabled by the adoption of antenna arrays (usually modeled as dual-polarized
linear antenna arrays [TR38901]) and the introduction of antenna ports concept, in which basically, for an antenna array of
multiple antenna elements, multiple antenna elements are combined into one antenna port for digital processing (precoding),
while analog processing (beamforming) is applied for the antenna elements within one antenna port.

The MIMO model adopted in 5G-LENA can combine spatial multiplexing (with up to four streams per user, and 32 antenna ports)
and beamforming (which applies for each of the streams). Up to four streams are encoded in the same TB. PMI, RI and CQI
are implemented and included as part of the CSI feedback. It follows the 3GPP codebook-based Type I model for precoding [TS38214]_
and assumes MMSE-IRC receiver. For precoding and rank selection, an exhaustive search is implemented. The number of streams is
called the rank in the code, which affects the TBS and other performance characteristics. The inter-stream interference is correctly
computed through matrix processing, and this is why the use of more than 2 streams requires the Eigen library to compute operations
like matrix inverse, SVD, etc. As the SINR and interference computations are correctly modeled, following [Palomar2006]_, and multiple
streams are fit into one TB, this allows using the SISO error model for MIMO error modeling, by vectorizing the 2D SINR (RBs, rank)
into 1D SINR (RBs x rank).

In the following, we explain the design choices and implementation details to enable MIMO. This includes, 1) adding the "rank"
parameter to many interfaces throughout the code, 2) the MIMO interference and SINR calculations, as well as the interfaces to pass
the results to other classes, 3) the computation of transport block error rates (TBLER) based on the MIMO SINR, 4) the search for
the optimal precoding matrix, which the UE needs to send as a feedback to the gNB in the PMI, as well as the 3GPP-compliant precoding
matrix codebook, and 5) the enabling of the new MIMO methods and using the feedback at the gNB.

Rank
#############
Firstly, the interfaces are extended to allow passing the rank number ( the number of MIMO layers).
In this way, for example, already existing functions for the calculation of the transport block size for SISO could be
easily updated to be used for both, SISO and MIMO (in this section we refer to SISO as the single stream transmission,
although multiple antennas are supported, and MIMO for the multiple-stream transmission). Also, packet traces are
extended to include the rank number.

The MIMO interference and SINR calculations
###########################################
The NR model for the interference calculation is extended to support the calculation of the MIMO interference and
MIMO SINR calculations. The main class for the calculation of the interference in the NR module is ``NrInterference`` class.
This class is extended with new functions for the computation of the interference-and-noise covariance matrix and SINR.
These functions are ``CalcOutOfCellInterfCov``, ``CalcCurrInterfCov``, ``AddInterference``, and ``ComputeSinr``.
``CalcOutOfCellInterfCov`` computes the interference signals from all out-of-cell interferers.
``CalcCurrInterfCov`` prepares ``NrInterference`` class for MU-MIMO by supporting the calculation of the interference signals
by also considering the interferers from the same cell. For example, in the MU-MIMO UL, UEs from the same cell could act as interferers.
``AddInterference`` adds the covariance of the signal to an existing covariance matrix.
Finally, ``ComputeSinr`` computes the SINR as follows:

  1) the actual interference-and-noise covariance for the signal is computed,
  2) the signal is transformed into a different representation where the interference-and-noise covariance is an identity matrix
     (aka whitening transformation),
  3) a dummy precoding matrix is created when none exists, and
  4) the SINR based on the MSE matrix is computed as explained in [Palomar2006]_.

To support all these MIMO operations, it was not enough to use a single dimensional ``SpectrumValue`` type that has been traditionally
used in ``NrInterference`` for SISO. To support an efficient storage and computations of MIMO operations new classes were defined,
such as ``NrCovMat``, ``NrIntfNormChanMat`` and ``NrSinrMatrix``.
``NrCovMat`` stores the interference-plus-noise covariance matrices of a MIMO signal, with one matrix page for each frequency bin.
This class also provides some functions for efficient computations on covariance matrices.
Its functions ``CalcIntfNormChannel`` performs interference whitening [interf-whitening]_.
``NrIntfNormChanMat`` stores the interference-whitened channel matrix, the channel matrix after normalizing/whitening the
interference. Its function ``ComputeSinrForPrecoding`` computes the SINR based on MSE.
Finally, ``NrSinrMatrix`` stores the MIMO SINR matrix whose dimensions are the rank and the number of RBs.
MIMO implementation requires Eigen3 [eigen3]_, a C++ template library for linear algebra: matrices,
vectors, numerical solvers, and related algorithms.
However, Eigen library is not always available. To allow the compilation even when Eigen is not available a CMake switch is added:

  a) when Eigen is enabled, the file nr-mimo-matrices-eigen.cc is compiled
  b) when Eigen is disabled, the file nr-mimo-matrices-no-eigen.cc is compiled (the implementations just contain a single NS_FATAL_ERROR).
     In this case, users can still compile but can only use SISO, they will get this error only when trying to use MIMO.
     The functions used from Eigen library could be in the future implemented in ns-3 to reduce dependency of ns-3 and
     the nr module on Eigen library. Then nr-mimo-matrices-no-eigen.cc could be implemented to call these ns-3 alternatives of Eigen
     functions.

To support the multi-dimensional MIMO signals a new interference chunk processor called ``NrMimoChunkProcessor`` is introduced.
This class mirrors the original ``LteChunkProcessor`` that is originally used in ``NrInterference`` for SISO.
``LteChunkProcessor`` is not sufficient for MIMO because it can only store a frequency-domain vector of SINR values whereas
MIMO requires a 2D matrix with the dimensions: number of RBs  and number of MIMO layers.
``LteChunkProcessor`` stores the sum of the different signals' power spectral density values and
performs the averaging once the function ``End`` is called. Such SINR averaging in the time-domain limits the fidelity.
In general, each received signal may have different number of MIMO layers, hence combining the SINR of different signals is not
trivial. To avoid all this, ``NrMimoChunkProcessor`` keeps a list with full information of all different signals and
no averaging is performed. The averaging now must be implemented in the error model which opens the door also for different possible
implementations, e.g., error model may apply exponential effective SINR both over time and frequency.
Hence the ``NrMimoChunkProcessor`` only looks like ``LteChunkProcessor``, but is actually mainly used as a storage to pass
the information to other entities that can perform a different computations by exploiting the full information of all different signals.
``NrMimoChunkProcessor`` provides two kind of callbacks:

    * MIMO SINR: one 2D matrix for each different time-domain chunk, which is used by the error model to compute TBLER, and
    * Interference covariance matrices for each different time-domain chunk are passed to CQI generating functions and used
      are used with channel matrix to compute the precoding matrix PMI feedback.

Since nr-3.2, ``LteChunkProcessor`` has been ported to NR as ``NrChunkProcessor``.

Computation of TBLER based on the MIMO SINR
###########################################

A new function called ``GetTbDecodificationStatsMimo`` is added to ``NrErrorModel`` to determine if a transport block was received
successfully. ``GetTbDecodificationStatsMimo`` performs a simple weighted average over potentially multiple different signal values
received over time to get a single SINR matrix. The SINR matrix is then linearized to a vector and passed to the existing error
model for SISO by calling a function ``GetTbDecodificationStats``. Effectively, ``GetTbDecodificationStatsMimo`` is a
translation layer between the new MIMO code and the existing SISO error model.

When using MIMO one should configure the ``AmcModel`` as ``ErrorModel``. The ``ShannonModel`` is not yet supported with MIMO,
some additions are needed to ``NrAmc`` to allow its usage.

Search for the optimal precoding matrix
#######################################

``NrPmSearchFull`` class is implemented to find the optimal precoding matrix, rank indicator, and corresponding CQI,
and creates a CQI/PMI/RI feedback message. ``NrPmSearchFull`` uses exhaustive search for 3GPP Type-I codebooks.
Optimal rank is considered as the rank that maximizes the achievable TB size when using the optimal PMI. To determine the
rank indicator the algorithm loops through all ranks (the number of MIMO layers), and for each rank it computes PMI,
and it computes the maximum supported MCS and associates TB size, and finally it selects the rank that results in the highest TB size.
The optimal WB/SB PMI values are periodically updated based on the configured update intervals, that can be configured using
two attributes in NrUePhy class: ``WbPmiUpdateInterval`` and ``SbPmiUpdateInterval``.
When a PMI update is requested, the optimal precoding matrices are updated using exhaustive search over all possible
precoding matrices specified in a codebook that is compatible with 3GPP TS 38.214 Type-I. The procedure based on exhaustive search
loops over all possible sub-band precoding matrices and computes the SINR that would be achieved by each precoding matrix,
and selects the precoder resulting in the highest average SINR.
Finally, the feedback message is created that includes the optimal rank, the corresponding optimal precoding matrix and CQI.

``NrPmSearch`` is the base class and ``NrPmSearchFull`` is one possible specialization that finds PMI, RI and CQI. One could create
another specialization of ``NrPmSearch`` that would implement a different algorithm to find PMI and RI values.

The size of the sub-bands depends on the channel bandwidth, both in numbers of PRBs. It should be set accordingly to 3GPP
TS 38.214 Table 5.2.1.4-2 via the attribute ``NrPmSearch::SubbandSize``.

As of release 4.0, additional PMI selection techniques were included. These can be selected by setting
``NrHelper::PmSearchMethod`` attribute to ``ns3::NrPmSearchIdeal``, ``ns3::NrPmSearchFast``, ``ns3::NrPmSearchSasaoka``,
``ns3::NrPmSearchMaleki``.

* ``NrPmsearchIdeal`` extracts the theoretical ideal precoding matrix from the channel matrix via SVD decomposition,
  then selects the number of columns (equivalent to the rank) that maximizes the TBS through brute force.

* ``NrPmSearchFast`` uses a RI selection technique, settable via the attribute ``NrPmSearch::RankTechnique``, to skip
  all other rank computations.

  * Available RI techniques include: ``SVD``, based on SVD decomposition;
    ``WaterFilling``, based on power allocation; ``Sasaoka``, based on increment of channel capacity ratio.

    * Rank selections ``SVD`` and ``WaterFilling`` must be calibrated per scenario using
      the calibration factor ``NrPmSearch::RankThreshold``.
    * After determining the rank, the PMI sub-indices I1 and I2 are searched.

  * The I1 index, which corresponds to wide-band component, is searched using the channel matrix average.
  * The I2 index, which corresponds to sub-band component, is searched using the channel matrix averaged per sub-band.

* ``NrPmSearchSasaoka`` uses the ``Sasaoka`` RI selection technique, then searches for the PMI that maximizes
  the mutual information, instead of TBS targeted by other techniques. Both techniques are proposed in [Sasaoka2019]_.

* ``NrPmSearchMaleki`` implements a search-free PMI selection exploiting intrinsic characteristics of
  the 3GPP Type I codebooks proposed in [Maleki2023]_.
  Since the PMI search is fast, the RI is determined by brute-force search.


MIMO activation
###############
``NrHelper`` is the class that is responsible of setting the ``NrPmSearch`` algorithm to ``NrUePhy`` instance, and the configuration of the corresponding parameters,
such as the type of the search algorithm, the type of the codebook, and the rank limit. ``NrHelper`` also creates ``NrMimoChunkProcessor``
and adds the necessary callbacks. These callbacks are:

 * ``NrSpectrumPhy::UpdateMimoSinrPerceived`` which is called to provide MIMO SINR feedback
 *  ``NrUePhy::GenerateDlCqiReportMimo`` which is called to provide MIMO signal to functions that perform PMI search and create CQI/PMI/RI feedback

Until nr-4.1, enabling MIMO in the simulation required setting the ``EnableMimoFeedback`` attribute of the ``NrHelper`` to true.
The ``EnableMimoFeedback`` enables MIMO feedback including PMI/RI/CQI, while ``RankLimit`` limits the possible RI value
(e.g., to 1 stream). So, even if RI is limited to 1, the usage of MIMO feedback can provide benefits because of the PMI feedback.

Since nr-4.2, MIMO feedback is enabled based on the ``CsiFeedbackFlags`` attribute of the ``NrHelper``.
It is implied based on the presence of ``CQI_PDSCH_MIMO`` or ``CQI_CSI_IM``, as checked by the ``NrHelper::IsMimoFeedbackEnabled()`` function.
To configure PMI search parameters (such as rank limit, PMI search method, the codebook,) ``NrHelper`` provides a function ``SetupMimoPmi``.


CSI-RS and CSI-IM
=================
5G-LENA simulated CSI-RS and CSI-IM in order to allow for CSI feedback that is similar to the one explained in 3GPP
standard, in addition to the existing PDSCH-based CSI feedback. The CSI-RS signal cannot be modeled realistically
as the minimum granularity in the frequency domain is a physical resource block. For this reason, this signal cannot
be simulated in PDSCH channel since then we would have unrealistic overhead that does not exist in real implementation.
To avoid this issue, we have simulate CSI-RS in PDCCH, i.e., at the end of the PDCCH duration. CSI-RS signal is being
transmitted towards each UE with the best analog beam towards that UE, by using the configured beamforming algorithm
(e.g. direct path beamforming, cell scan, etc).

A side effect of relying on the existing PDSCH-based feedback is that it is aperiodic, meaning interference
can only be measured on scheduled resources for a given UE. These resources are allocated based on availability and
data-plane traffic, which severely impacts the quality of CSI feedback under high load and multiple UEs.
To address this, current schedulers allocate RBGs to UEs reporting wide-band CQI 0 (out-of-range), allowing
them to update their CSI feedback on allocated bands and avoid being starved of resources throughout the
simulation due to a single poor report.

CSI-RS/CSI-IM resolves this issue by periodically transmitting a non-precoded channel state information
reference signal (CSI-RS) as described in Section 7.4.1.5 of [TS38211]_. CSI-RS measurements are used to
obtain the **frequency domain spectrum channel matrix** (whose dimensions are the number of RX ports,
the number of TX Ports, and the number of resource blocks (RBs)). After receiving the CSI-RS,
CSI-IM interference measurements can be performed on the PDSCH to produce the **interference covariance
matrix**, detailed in Section 5.2.2.4 of [TS38214]_. These measurements, combined with the channel matrix,
are used to generate CSI feedback, including wide-band and sub-band CQI, as well as digital precoding
information like Rank Indicator (RI) and Precoding Matrix Indicator (PMI) (Section 5.2.2 in [TS38214]_).
However, CSI-RS and CSI-IM add computational complexity due to the extra spectrum and propagation model
calculations required.

A new type of the signal parameters representation called ``NrSpectrumSignalParametersCsiRs``
implements the CSI-RS signal. To reduce the computational complexity of this implementation and to avoid that the
spectrum and propagation loss models are being called for the CSI-RS signals that are not intended for the specific
devices, it is created a new spectrum filter called ``NrCsiRsFilter``, that will directly discard such signals
inside of spectrum, and they will not reach ``NrSpectrumPhy`` objects of the devices for which they were not intended.
``NrCsiRsFilter`` filters the signal being received if the receiving SpectrumPhy is not of type ``NrSpectrumPhy``,
which can happen e.g., when two technologies coexist (e.g. NR and Wi-Fi). Additionally, ``NrCsiRsFilter`` filters
out the signal if the receiving device is not a UE device. ``NrCsiRsFilter`` determines whether CSI-RS signal is
intended for a specific UE device thanks to holding RNTI of the UE for which it was intended.

A second spectrum filter, called ``NrSidelinkFilter``, discards the signals exchanged between two gNBs or
between two UEs, so that the spectrum and propagation loss models are only called for the gNB-to-UE and
UE-to-gNB links. This reduces the computational complexity of scenarios with many devices, since the number
of gNB-gNB and UE-UE links grows quadratically with the number of gNBs and UEs, respectively. 3GPP calibration
scenarios typically only consider the links between UEs and gNBs, so this filter can be used to speed up
such simulations without affecting their results. Signals sent or received by a SpectrumPhy that is not of
type ``NrSpectrumPhy`` (e.g., in coexistence scenarios with Wi-Fi) are never discarded by ``NrSidelinkFilter``.

Since the gNB-gNB and UE-UE interference is ignored, ``NrSidelinkFilter`` should only be used with a TDD
pattern composed of pure DL and UL slots, where such interference does not arise. It also discards every
UE-to-UE transmission, including sidelink, so it must not be used when evaluating sidelink scenarios.

A third spectrum filter, called ``NrCoexistenceFilter``, discards the signals exchanged between an NR device and
a device of another technology (e.g., Wi-Fi), that is, between an ``NrSpectrumPhy`` and a SpectrumPhy of any other
type. Signals exchanged between two NR devices, or between two non-NR devices, are not affected. It is useful when
several technologies share a spectrum channel, but their mutual interference is not of interest, since the
spectrum and propagation loss models are then only called for the links within each technology. Conversely, it
removes the interference between NR and the other technologies, so it must not be used when evaluating
coexistence scenarios.

The filters installed on the spectrum channels created by ``NrChannelHelper`` are selected by its attribute
``AddFilters``, a bitmask of ``NrChannelHelper::Filter`` values. By default, only ``NrCsiRsFilter`` is
installed (``NrChannelHelper::CSIRS``). Other filters (``NrChannelHelper::SIDELINK`` for ``NrSidelinkFilter``
and ``NrChannelHelper::COEXISTENCE`` for ``NrCoexistenceFilter``) are installed by setting the attribute before
the spectrum channels are created. For example, ``NrCsiRsFilter`` and ``NrSidelinkFilter`` are installed as follows:

.. sourcecode:: cpp

   Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
   channelHelper->SetAttribute("AddFilters",
                               UintegerValue(NrChannelHelper::CSIRS | NrChannelHelper::SIDELINK));
   channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
   channelHelper->AssignChannelsToBands({band});

The ``nr-transmit-filter`` test suite verifies that each combination of filters discards exactly the
expected signals.

CSI-RS is being transmitted periodically, and the configured period should be a multiply of gNB PHY TDD pattern.
This is because it can be transmitted only during the slots that contain the DL CTRL (these slot types are downlink,
flexible, and special). Periodicity is configured per gNB by using the attribute ``CsiRsPeriodicity`` of ``NrGnbPhy``,
and by default is 10 slots, which for numerology 0 corresponds to having CSI-RS transmitted each 10 ms.
Apart from ``CsiRsPeriodicity``, there is CSI-RS offset, which is automatically assigned in a round robin fashion
to all UEs attached to a specific gNB, by skipping the UL slots, and by taking value from 0 to ``CsiRsPeriodicity``,
and then when all offset values are assigned for the following users it start again from the 0 value. This means that
if we have more users then slots in the CSI-RS periodicity period, some of the UEs will receive their CSI-RS in
the same slot, in the PDCCH. CSI-RS is being transmitted for each UE by using the best analog beam toward that UE
and it is being transmitted over all the bandwidth.

In order to obtain the interference, the CSI-IM is modeled in PDSCH. The duration of CSI-IM can
be configured by using the attribute ``CsiImDuration`` of ``NrUePhy``. By default, its duration is
OFDM symbol. The measured interference can be averaged by using a moving average by configuring the
alpha parameter through ``AlphaCovMat`` attribute of ``NrUePhy``. Increasing the ``CsiImDuration``
potentially increases the number and power of received interferers, by listening the channel for more time,
giving time to neighboring cells to change the direction of their beams.

To configure the CSI feedback type, use the attribute ``NrHelper::CsiFeedbackFlags``, which supports
multiple configurations through a combination of flags. Each flag is defined by a single bit, as follows:

- 0b0000 means that no CSI feedback will be provided;
- 0b0001 (or ``CQI_PDSCH_MIMO``) enables existing PDSCH-based CSI;
- 0b0010 (or ``CQI_CSI_RS``) enables CSI-RS-based channel matrix estimation;
- 0b0100 (or ``CQI_CSI_IM``) enables CSI-IM interference covariant matrix measurements;
- 0b1000 (or ``CQI_PDSCH_SISO``) legacy PDSCH-based CSI without spatial channel models.

Valid configurations, **requiring data-plane traffic to the measuring UE** to measure interference:

- ``CQI_PDSCH_SISO``, legacy PDSCH-based CSI for non-spatial models;
- ``CQI_PDSCH_MIMO``, existing PDSCH-based CSI feedback;
- ``CQI_PDSCH_MIMO|CQI_CSI_RS``, combines PDSCH-based feedback with CSI-RS for a more frequently updated channel matrix.

Valid configurations, **not requiring data-plane traffic to the measuring UE** to measure interference:

- ``CQI_CSI_RS|CQI_CSI_IM``, CSI-IM measurements on PDSCH, as triggered by CSI-RS, independent of resource allocation;
- ``CQI_PDSCH_MIMO|CQI_CSI_RS|CQI_CSI_IM``, combines PDSCH-based feedback with CSI-IM measurements on unallocated
  resources triggered by CSI-RS.

Invalid configurations include:

- ``CQI_CSI_IM`` and ``CQI_CSI_IM|CQI_PDSCH_MIMO``, because CSI-IM relies on CSI-RS signals to trigger measurements.

Radio Link Failure (RLF)
========================

In real NR networks, Radio link failure (RLF) can happen due to several reasons.
It can be triggered if a UE is unable to decode PDCCH due to poor signal quality,
upon maximum RLC retransmissions, RACH problems and other reasons. 3GPP only
specifies guidelines to detect RLF at the UE side, in [TS36331]_ and [TS36133]_.
On the other hand, the gNB implementation is expected to be vendor specific.
To implement the RLF functionality in ns-3, we have assumed the following
simplifications:

 * The RLF detection procedure at eNodeB is not implemented. **Instead, a direct
   function call by using the SAP between UE and gNB RRC (for both ideal and real
   RRC) is used to notify the gNB about the RLF**.
 * No RRC connection re-establishment procedure is implemented, thus, the UE
   directly goes to the IDLE state upon RLF. This is in fact as per the standard
   [TS36331]_ sec 5.3.11.3, since, at this stage the NR module does not support
   the Access Stratum (AS) security.

The above mentioned RLF specifications can be divided into the following two
categories:

 #. RLF detection
 #. Actions upon RLF detection

In the following, we will explain the RLF implementation in context of these
two categories.

RLF detection implementation
############################

The RLF detection at the UE is implemented as per [TS36133]_, i.e., by monitoring
the radio link quality based on the reference signals (which in the simulation
is equivalent to the PDCCH) in the downlink. Thus, it is independent of the method
used for the downlink CQI computation.

The RLF detection starts once the RRC connection is established between UE and
gNodeB, i.e., UE is in "CONNECTED_NORMALLY" state; upon which the RLF parameters
are configured (see ``NrUePhy::DoConfigureRadioLinkFailureDetection``). In real
networks, these parameters are transmitted by the gNB using IE UE-TimersAndConstants or
RLF-TimersAndConstants. However, for the sake of simplification, in the simulator
they are presented as the attributes of the ``NrUePhy`` and ``NrUeRrc`` classes.
Moreover, what concerns the carrier aggregation, i.e., when a UE is configured
with multiple component carriers, the RLF detection is only performed by the
primary component carrier, i.e. component carrier id 0
(see ``NrUePhy::DoNotifyConnectionSuccessful``). In ``NrUePhy`` class, CQI
calculation is triggered for every downlink subframe received,
and the average SINR value is measured across all resource blocks. For the RLF
detection, these SINR values are averaged over a downlink frame and if the result
is less than a defined threshold Qout (default: -5dB), the frame cannot be decoded
(see``NrUePhy::RadioLinkFailureDetection``). The Qout threshold corresponds to 10%
block error rate (BLER) of a hypothetical PDCCH transmission taking into account
the PCFICH errors [R4-081920]_. Once, the UE is unable to decode
20 consecutive frames, i.e., the Qout evaluation period (200ms) is reached, an
out-of-sync indication is sent to the UE RRC layer (see ``NrUeRrc::DoNotifyOutOfSync``).
Else, the counter for the unsuccessfully decoded frames is reset to zero. At the
``NrUeRrc``, when the number of consecutive out-of-sync indications matches with the
value of N310 parameter, the T310 timer is started and NrUePhy is notified to start
measuring for in-sync indications (see ``NrUePhy::DoStartInSyncDetection``). We note
that, the UE RRC state is not changed till the expiration of T310 timer. If the
resultant SINR values averaged over a downlink frame is greater than a defined
threshold Qin (default: -3.9dB), the frame is considered to be successfully
decoded. Qin corresponds to 2% BLER [R4-081920]_ of a hypothetical PDCCH transmission
taking into account the PCFICH errors. Once the UE is able to decode 10
consecutive frames, an in-sync indication is sent to the UE RRC layer
(see ``NrUeRrc::DoNotifyInSync``). Else, the counter for the successfully decoded
frames is reset to zero. If prior to the T310 timer expiry, the number of
consecutive in-sync indications matches with N311 parameter of ``NrUeRrc``, the UE
is considered back in-sync. At this stage, the related parameters are reset to
initiate the radio link failure detection from the beginning
(see ``NrUePhy::DoConfigureRadioLinkFailureDetection``). On the other hand, If the
T310 timer expires, the UE considers that a RLF has occurred
(see ``NrUeRrc::RadioLinkFailureDetected``).

Figure :ref:`fig-nr-ue-rlf` summarizes this radio-link-failure state machine: the N310 / T310 /
N311 out-of-sync / in-sync logic, the two possible outcomes selected by the
``NrUeRrc::UseRrcReestablishment`` attribute, and the optional RLC-AM max-retx trigger
(``RlcMaxRetxTriggersRlf``, off by default) that declares an RLF directly, cancelling any pending
T310 (TS 38.331 5.3.10.3).

.. _fig-nr-ue-rlf:

.. figure:: figures/rrc/nr-ue-rlf.*
   :align: center

   UE radio-link-failure detection and timers (T310 / N310 / N311).

Actions upon RLF
################

Once the T310 timer is expired, a UE is considered to be in RLF; upon which the
UE RRC:

 * Sends a request to the gNB RRC to remove the UE context
 * Moves to "CONNECTED_PHY_PROBLEM" state
 * Notifies the UE NAS layer about the release of RRC connection.

Then, after getting the notification from the UE RRC the NAS does the following:

 * Delete all the QoS Flows (previously TFTs)
 * Reset the bearer counter
 * Restore the bearer list, which is used to activate the QoS flows for the next
   RRC connection. This restoration of the QoS flows is achieved by maintaining an
   additional list, i.e., ``m_qosFlowsToBeActivatedListForReconnection`` in NrEpcUeNas
   class
 * Switch the NAS state to OFF by calling NrEpcUeNas::Disconnect
 * Tells the UE RRC to disconnect

The UE RRC, upon receiving the call to disconnect from the ``NrEpcUeNas`` class,
performs the action as specified by [TS36331]_ 5.3.11.3, and finally leaves the
connected state, i.e., its RRC state is changed from "CONNECTED_PHY_PROBLEM" to
"IDLE_START" to perform cell selection as shown in figure :ref:`fig-nr-ue-procedures-after-rlf`.

..

   TODO: Add UE RRC states figure, and update above text

   perform cell selection as shown in figures
   `fig-nr-ue-rrc-states` and :ref:`fig-nr-ue-procedures-after-rlf`.

At this stage, the NR module does not support the paging functionality, therefore,
to allow a UE to read SIB2 message after camping on a suitable cell after RLF, a
work around is used in ``NrUeRrc::EvaluateCellForSelection`` method. As per this
workaround, the UE RRC invokes the call to ``NrUeRrc::DoConnect`` method, which
enables the UE to switch its state from "IDLE_CAMPED_NORMALLY" to "IDLE_WAIT_SIB2",
thus, allowing it to perform the random access.

.. _fig-nr-ue-procedures-after-rlf:

.. figure:: figures/nr-ue-procedures-after-rlf.*
   :scale: 95 %
   :align: center

   UE procedures after radio link failure

The gNB RRC, after receiving the notification from the UE RRC starts the procedure
of UE context deletion, which also involves the deletion of the UE context removal
from the EPC :ref:`fig-nr-ue-context-removal-from-epc` and the gNB stack
:ref:`fig-nr-ue-context-removal-from-gnb-stack`. We note that, the UE context
at the MME is not removed since, QoS flows are only added at the start of a
simulation in MME, and cannot be added again unless scheduled for addition
during a simulation.

.. _fig-nr-ue-context-removal-from-epc:

.. figure:: figures/nr-ue-context-removal-from-epc.*
   :scale: 80 %
   :align: center

   UE context removal from EPC

.. _fig-nr-ue-context-removal-from-gnb-stack:

.. figure:: figures/nr-ue-context-removal-from-gnb-stack.*
   :scale: 80 %
   :align: center

   UE context removal from gNB stack
