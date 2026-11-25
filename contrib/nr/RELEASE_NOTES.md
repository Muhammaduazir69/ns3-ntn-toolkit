<!--
Copyright (c) 2022 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)

SPDX-License-Identifier: GPL-2.0-only
-->

5G-LENA Release Notes                         {#releasenotes}
=====================

This file contains release notes for the NR module (most recent releases first).

All of the ns-3 documentation is accessible from the ns-3 website:
http://www.nsnam.org including tutorials: https://www.nsnam.org/documentation/

Consult the file CHANGES.md for more detailed information about changed
API and behavior across releases.

Release NR-v5.2
---------------

Availability
------------
November 20, 2026.

Cite this version
-----------------
DOI: 10.5281/zenodo.24391398

New user-visible features
-------------------------
- The number of HARQ retransmissions is now configurable through the new
  ``NrMacSchedulerNs3::MaxHarqReTx`` attribute. When ``EnableHarqReTx`` is
  enabled, retransmission chains are bounded by transmission attempts rather
  than by ``rv == 3``, so RV can exceed 3 when ``MaxHarqReTx`` exceeds 3.
- Added ``NrSidelinkFilter``, a spectrum transmit filter that drops gNB-to-gNB
  and UE-to-UE NR signals before the spectrum channel computes their propagation
  loss and 3GPP channel matrices, so only gNB-to-UE and UE-to-gNB links are
  evaluated. It reduces the simulation cost of deployments with many nodes, at
  the price of ignoring gNB-gNB and UE-UE interference; it is intended for TDD
  patterns made of pure DL and UL slots, and must not be used in sidelink
  scenarios, since it also drops sidelink transmissions. Signals from or to
  non-NR devices (e.g., Wi-Fi in coexistence scenarios) are not filtered. It is
  disabled by default and is enabled with the new ``NrChannelHelper::AddFilters``
  attribute (``NrChannelHelper::CSIRS | NrChannelHelper::SIDELINK``), set before
  the spectrum channels are created. Covered by the new ``nr-transmit-filter``
  test suite.
- Added ``NrCoexistenceFilter``, a spectrum transmit filter that drops the
  signals exchanged between NR and non-NR devices (e.g., Wi-Fi), so that the
  spectrum channel only evaluates the links within each technology. It is meant
  for simulations where several technologies share a spectrum channel but their
  mutual interference is not of interest, and must not be used in coexistence
  scenarios. It is disabled by default and is enabled through
  ``NrChannelHelper::AddFilters`` with ``NrChannelHelper::COEXISTENCE``. Covered
  by the ``nr-transmit-filter`` test suite.
- Added ``NrRanEpcHelper``, which installs the gNB side of the S1 and X2
  interfaces without any core network node, so that other core networks (e.g., a
  connector to a 5G core network such as Open5GS) are plugged in through
  ``NrRanEpcHelper::ConnectGnbToCore()``. ``NrNoBackhaulEpcHelper`` now derives
  from it.
- A simulation can now have several core networks (EPC helpers), and their UEs
  can roam between them (``NrNoBackhaulEpcHelper::AddRoamingPartner()``): a core
  network attaches the subscribers of its roaming partners with their QoS flows
  from their home network, and the network that served the UE before releases it
  in its gNB (S1-AP UE Context Release) and deletes its session. With local
  breakout, the UE gets IPv4 and IPv6 addresses of the visited network; with
  home-routed roaming, its session goes through an S8 link to the PGW of its
  home network, and it keeps its home addresses. The UEs move between the
  networks through radio link failures and cell selection. Covered by the new
  ``nr-core-roaming`` test suite.
- The GTP-U packets between the gNBs and the SGW now carry the PDU Session
  Container extension header with the QFI, in both directions, as on the N3
  interface of 5G core networks (4 more bytes per packet). ``NrGtpuHeader`` now
  supports any chain of GTP-U extension headers.
- The AI-based schedulers now observe the UEs instead of their logical channels,
  take the numerology of the carrier (``Numerology`` attribute of
  ``NrMacSchedulerOfdmaAi`` and ``NrMacSchedulerTdmaAi``), and
  ``gsoc-nr-rl-based-sched`` comes with DDQN, A2C, DDPG and PPO agents, PDCP
  discarding options, and a CI job.
- Added unstructured PDU sessions (3GPP TS 23.501, clause 5.6.1), which carry
  one non-IP network-layer protocol that the network does not parse, also when
  roaming.
- Added the ``nr-external-core-roaming`` example, in which the UEs of a
  simulation roam to an external 5G core network (Open5GS) through nr-bridge,
  with the scripts to set it up in ``utils/open5gs-roaming``.
- ``TrafficGenerator`` can add a ``SeqTsSizeHeader`` to its packets
  (``EnableSeqTsSizeHeader`` attribute, ``TxWithSeqTsSize`` trace source), and
  ``TrafficGeneratorNgmnVoip`` now sends one packet per encoder frame during
  talk spurts.
- When a transport block is decoded with the MIMO SINR, ``RxPacketTrace.txt``
  now reports that SINR, and the error model traces carry it too.

Bugs fixed
----------
- Fixed ``gsoc-leo-demo-example`` attaching the ground terminal to a satellite
  below its horizon. ``NrHelper::AttachToClosestGnb()`` compared the ground
  terminal's topocentric ``GetPosition()`` with the satellites' ECEF positions,
  so it picked the satellite on the far side of the Earth. The ground terminal
  now uses ``GeocentricEcefMobilityModel`` (#284).
- Fixed the simulation aborting with "cannot add the same kind of tag twice" when
  a packet reached a packet socket that had already delivered it. The tags a
  packet socket stamps on delivery were left on the packet the gNB EPC
  application relays to the S1-U interface, from where they were copied onto
  every RLC segment and carried into the reassembled SDU. They are now removed
  when the packet is taken off the NR packet socket. Covered by the new
  ``nr-epc-gnb-relayed-tag`` test suite.
- Fixed the gNB RRC reusing the C-RNTI of a UE context destroyed while its
  handover was still in progress. ``AddUe()`` hands out the first C-RNTI absent
  from the UE map, so the value became available again while the X2 messages of
  the interrupted handover were still in flight, and a UE CONTEXT RELEASE
  arriving afterwards released whichever UE had inherited it. Such a C-RNTI is
  now reserved until the handover concludes, or at the latest for the new
  ``NrGnbRrc::RntiReservationDuration`` (500 ms by default).
- Fixed the UE MAC terminating the simulation with an ``std::out_of_range``
  exception when an uplink grant arrived for a logical channel that had been
  removed from the MAC. The RLC entity of a channel dropped on handover, or by
  the ``DoReset()`` preceding a re-establishment, keeps its service access point
  and can still report a buffer status. That report was recorded even though no
  ``NrMacSapUser`` remained to serve it, leaving an LCID in the buffer status map
  with no entry in the logical channel map; the next uplink grant selected it and
  ``SendNewStatusData()``, ``SendRetxData()`` or ``SendTxData()`` threw while
  looking it up. Such reports are now discarded when they arrive.
- Fixed ``NrMacSchedulerTdma::DoReshapeAllocation()`` looping without terminating
  when reshaping a HARQ retransmission whose resource count exceeded the range of
  its symbol counter, which is reachable at wide bandwidths, and dividing by zero
  when no RBGs were free. The reshape no longer requires an exact symbols x RBGs
  factorisation of the original resource count: it uses the fewest symbols that
  hold at least that many resources, so retransmissions with no exact shape in the
  slot are no longer held back. Covered by the new ``nr-test-sched-reshape`` test
  suite.
- Fixed the SRS ignoring uplink power control. ``NrUePhy::UlSrs()`` transmitted
  the SRS over the whole band with the power left by the previous PUSCH or PUCCH,
  so its power spectral density depended on what was sent before it. When
  ``NrUePhy::EnableUplinkPowerControl`` is set, the SRS power is now computed by
  ``NrUePowerControl::GetSrsTxPower()``, which also makes the ``ReportSrsTxPower``
  trace fire. ``P0_SRS``, used with ``TSpec`` set to ``TS38.213``, is now the
  ``NrUePowerControl::P0Srs`` attribute instead of a hard-coded 0 dBm. Covered by
  the ``nr-uplink-power-control-test`` test suite.
- Fixed the uplink link adaptation using the SISO SINR while the gNB decodes the
  PUSCH with the post-MMSE MIMO SINR. With a channel that supports MIMO, the
  PUSCH CQI is now generated by ``NrGnbPhy::GenerateMimoDataCqiReport()`` from
  the same SINR the error model uses, so the scheduler sees the receive combining
  and interference rejection gain of multi-port gNBs. Covered by the new
  ``nr-ul-mimo-cqi`` and ``nr-ul-mimo-cqi-wiring`` (built with Eigen) test suites.
- Fixed ``NrMacSchedulerOfdma`` checking uplink allocations against the downlink
  notch mask when creating UL DCIs. They are now checked against the uplink one.
  Covered by new UL cases of the ``nr-test-notching`` test suite.
- Fixed UL HARQ retransmissions each taking their own symbols over the whole band.
  ``NrMacSchedulerHarqRr::ScheduleUlHarq()`` now places a retransmission on the
  symbols of one already scheduled in the slot when both have the same symbol
  count, their UEs share a beam and their RBGs do not overlap, so retransmissions
  that were frequency multiplexed in their first transmission are no longer
  serialized in time or deferred to a later slot. Covered by the
  ``nr-test-sched-harq`` test suite.
- Fixed DL and UL HARQ retransmissions leaving the rest of the band empty with
  the OFDMA schedulers. New data of the other UEs in the beam of the
  retransmissions is now scheduled on the RBGs they leave free, over the same
  symbols, before the remaining symbols of the slot are allocated. UEs are only
  placed there when this does not leave the remaining symbols unused. Covered by
  the ``nr-test-sched-harq`` test suite.
- Fixed DL and UL HARQ retransmissions being deferred to a later slot when their
  symbols did not fit in the ones left, or, in the DL, their RBGs were taken by
  another retransmission of their beam, although their resources did. Such a
  retransmission is now reshaped into the fewest symbols left that hold its
  resources, keeping its MCS and TBS. Covered by the ``nr-test-sched-harq`` test
  suite.
- Fixed the OFDMA schedulers placing all the UEs on the RBGs left free by the first
  HARQ retransmission span of a beam, leaving the following spans empty and some UEs
  without enough resources for their data. The UEs are now spread over all the spans
  of the beam. Covered by the ``nr-test-sched-harq`` test suite.
- Fixed ``ResourceAssignmentMatrix::GetAssignedResourcesToUe()`` reporting every
  resource as ``ResourceType::EMPTY``. It now reports the type each resource was
  assigned with. Covered by the ``nr-resource-assignment-matrix`` test suite.
- Fixed ``TrafficGenerator`` sending more than the size of a packet burst: its
  last packet had the full packet size instead of the bytes left, so a 0.5 MB
  file of ``TrafficGeneratorFtpSingle`` with 1448-byte packets sent 512592
  bytes.
- Fixed ``HexagonalGridScenarioHelper`` placing the indoor UEs at continuous
  heights: their floor is now an integer, uniform in 1 to N_fl, with N_fl
  uniform in 4 to 8 (TR 38.901 Table 7.4.1-1).
- Fixed ``HexagonalGridScenarioHelper`` dropping the UEs of each sector in the
  disc inscribed in its hexagon, shrunk by the minimum BS-UT distance: about a
  third of the cell, the corners and the borders between sectors, was never
  populated, which made the geometry about 3 dB too favorable at the median.
  The UEs are now dropped uniformly in the hexagon of their cell, at the
  minimum BS-UT distance or more from their site.
- Fixed UEs that connect to a cell without ``NrHelper::AttachToGnb()``, after a
  handover or a radio link failure, getting neither CSI-RS nor a beam from
  their new gNB. Only the gNB of ``AttachToGnb()`` registered a UE, so its new
  gNB served it with a quasi-omni beam and transmitted no CSI-RS towards it,
  the UE kept computing its CSI with the last CSI-RS of its previous cell, and
  the beamforming kept steering the UE towards its previous gNB. ``NrHelper``
  now registers a UE at the gNB of every cell it connects to.
- Fixed the UE keeping the CSI-RS channel and the interference covariance of
  its previous cell or bandwidth. A UE now discards them when it synchronizes
  with a cell, resets after a radio link failure, or changes its number of
  RBs, and computes its CSI with the PDSCH channel until the first CSI-RS of
  its new cell. A CSI-IM triggered by a CSI-RS received before a bandwidth
  change used to abort the simulation on a spectrum model mismatch.
- Fixed the MIMO SINR with interferers without precoding, like the DL control
  and the CSI-RS of neighbor cells, which counted every transmit port at the
  full transmit power: an interferer of N ports weighed N times its power (12
  dB for a dual-polarized gNB with 8 horizontal ports), while the desired
  signal and precoded interferers carry the transmit power once. The power of
  a signal without precoding is now split evenly across its transmit ports.
- Fixed the CSI-IM based CSI using the interference covariance measured with
  the CSI-RS instead of the averaged one when ``NrUePhy::AlphaCovMat`` is not 1.
- Fixed HARQ-IR (``NrEesmIrT1`` and ``NrEesmIrT2``) gaining nothing from the
  retransmissions that bring the effective code rate below the lowest code rate
  of the modulation order of the transport block: the 3rd and 4th transmissions
  of a 64-QAM block, and every retransmission of a block at the lowest MCS of
  its modulation order, decoded with the same probability as the 2nd or the
  1st, so HARQ-IR was worse than HARQ-CC and a block that failed twice was
  practically lost. The rate below the floor is now credited as a repetition
  gain of the effective SINR (``NrEesmIr::CreditRateBelowModulationFloor``,
  enabled by default).
- Fixed ``NrRlcUm`` losing SDUs with ``OutOfOrderDelivery`` enabled. The PDUs
  were reassembled in their arrival order by the in-order reassembly, which
  takes an SN gap as a loss: every PDU overtaken by another, as by a HARQ
  retransmission, discarded the SDUs segmented across its boundaries, and in
  the dense urban calibration two thirds of the FTP files lost data. The
  segments of an SDU split across PDUs are now held by SN and reassembled once
  all of them are received, in any order.
- Fixed ``NrRlcUm`` leaving bytes in its queue that the UE never requested a
  grant for (#298). It now reports its buffer status after every transmission
  opportunity that leaves data in its queue. Covered by the new
  ``nr-ul-stranded-rlc-bytes`` test suite.
- Fixed DL HARQ retransmissions being lost in a slot without DL symbols, because
  UL took them all, or when their timer expired while they waited. They now wait
  for a later slot. Covered by the new ``nr-dl-harq-retx-no-symbols`` test
  suite.
- Fixed two UL HARQ retransmissions of a UE being placed on the same symbols.
  The UE sent them one after the other, overran its slot, and the simulation
  aborted with "Cannot RX UL CTRL while receiving DATA". Covered by the
  ``nr-test-sched-harq`` test suite.
- Fixed the HARQ beam round-robin serving the same beam first in every slot,
  and the UL HARQ retransmissions not following it. Covered by the
  ``nr-test-sched-harq`` test suite.
- Fixed the UE MAC admitting an RLC STATUS PDU without reserving its MAC
  subheader and the SHORT_BSR. The UE MAC now also sizes the UL MAC subheaders
  and the SHORT_BSR exactly, and fills the bytes left in a TB with a padding
  BSR. Covered by the new ``nr-ue-mac-status`` test suite.
- Fixed the UE not sending its UL CTRL, and the DL HARQ feedback in it, in an
  SRS slot that followed a slot in which it had sent UL CTRL (#292): the channel
  access granted for one slot was kept, so the next slot requested none. Covered
  by the new ``nr-harq-feedback-timing`` test suite.
- Fixed the order of the control messages of the gNB depending on heap
  addresses, which made simulations not reproducible across runs (#290).
- Fixed the ``gsoc-leo-demo-example`` computing the beamforming vectors before
  orienting the satellite antennas (#287).
- Fixed ``NrFhControl`` adding the per-UE overhead instead of subtracting it in
  ``GetMaxRegAssignable()`` and ``GetMaxMcsAssignable()``, which wrapped around
  and sent every UE at MCS 0 with ``OptimizeMcs``; never releasing the UEs with
  HARQ retransmissions, which kept their BWP active; reading the antenna ports
  of BWP 0 instead of the BWP of the allocation; and throwing
  ``std::out_of_range`` for a BWP with only HARQ UEs. Covered by the new
  ``nr-test-fh-control`` test suite.
- Fixed the initial association not setting the RSRP and the best beam of each
  gNB from the strongest beam of that gNB, scaling the received power by the gNB
  transmit power in dBm a second time, and activating the UE panel of the
  strongest beam of all gNBs instead of the one of the associated gNB. Covered
  by the new ``nr-initial-association`` test suite.
- Fixed the UE aborting its ongoing reception when it reapplied the numerology
  of every MIB it received. Covered by the new ``nr-numerology-reception`` test
  suite.
- Fixed the UE applying an RRC reconfiguration of a connection it had already
  left, e.g., a handover command sent after it declared a radio link failure.
- Fixed the subband PMI update of the PMI searches when a reinitialized search
  had no precoding to refine; it now falls back to a full update. Fixed the
  subband PMIs and the beam mapping of the Maleki search.
- Fixed the gNB sending CSI-RS towards RNTI 0, which every UE without an RNTI
  accepted, and the UE scheduling its CSI-IM check with a negative delay with a
  propagation delay.
- Fixed the spectrum channel matrix of signals without one, e.g., with the
  ``TwoRay`` channel model, being normalized by the transmitter PSD a second
  time.
- Fixed the TypeId names of ``NrMacSchedulerOfdmaSymbolPerBeam`` and
  ``NrMacSchedulerOfdmaSymbolPerBeamLB``, which had a duplicated prefix, and
  registered the ``NrChannelHelper`` TypeId.
- Fixed a segmentation fault during the disposal of ``NrPhy``, and
  ``NrUePhy::EndVarTti()`` warning at every slot about clamping a delay that was
  exactly zero.

Release NR-v5.1
---------------

Availability
------------
August 6, 2026.

Cite this version
-----------------
DOI: 10.5281/zenodo.21770621

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-20.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 25.10 (Questing Quokka) with g++15 and clang-20.
- ARM
  - Ubuntu 25.10 (Oracular Oriole) with g++15 and clang-20.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release is compatible with ns-3.48.

New user-visible features
-------------------------
- FDD cells now advertise their UL carrier in SIB1 (the equivalent of
  ``frequencyInfoUL`` in TS 38.331), and the UE derives its primary UL BWP and
  bandwidth-part routing from it automatically. Manual
  ``NrUeNetDevice::PrimaryUlIndex`` and ``SetOutputLink()`` configuration is no
  longer needed for cells with a single DL/UL carrier pair, and was removed
  from the examples and tests.
- Added support for handover between FDD cells, including cells with inverted
  UL/DL carrier roles and different per-carrier numerologies: the handover
  command carries the target cell's UL carrier and RACH configuration, and the
  UE re-tunes and re-synchronizes its UL BWP to the target cell. Covered by the
  new ``nr-handover-rach-config`` test suite (TDD and FDD, ideal and real RRC).
- The bandwidth-part configurations and the QoS flow mapping are now advertised
  in the dedicated RRC configuration delivered to the UE.
- Added mobility state estimation to ``NrUeRrc`` (TR 36.839): the UE counts its
  recent handovers over a sliding window, classifies itself as normal, medium or
  high mobility, and scales the A3 time-to-trigger accordingly. Disabled by
  default; controlled by the ``MseEnable``, ``MseCountWindow``,
  ``MseHystNormal``, ``MseThreshMedium``, ``MseThreshHigh``, ``MseSfMedium`` and
  ``MseSfHigh`` attributes, plus ``MseFixedScale`` to apply a fixed scale from
  the start. Covered by the new ``nr-mobility-state-ttt`` test suite.
- Added an optional TR 36.839 handover failure model (``NrUeRrc``
  ``Tr36839HandoverFailure``, disabled by default): a handover command that
  arrives while the source link is already below Qout (T310 running) declares
  radio link failure instead of rescuing the link, so the handover failure rate
  can be calibrated against the TR 36.839 reference curves. Covered by the new
  ``nr-tr36839-handover-failure`` test suite. The companion
  ``Tr36839HoFailureMinT310Elapsed`` attribute grades the too-late threshold by
  the elapsed T310 time.
- Added simulation-level handover metrics: the ``NrUeRrc``
  ``RadioLinkFailureCause`` trace records why every radio link failure happened
  (with the elapsed T310 time and the time since the last successful handover),
  and the ``NrGnbRrc`` ``HandoverTotalTime`` trace reports the total handover
  time from the A3 trigger at the source gNB to completion. Connected-mode
  failures are now counted as genuine radio link failures (T310 expiry,
  failures during handover); a too-late handover command is booked as a
  handover failure instead, matching the TR 36.839 definitions.
- Added optional handover admission and ping-pong guards, disabled by default:
  ``NrA3RsrpHandoverAlgorithm`` ``MinTargetRsrpDbm`` and ``MinTargetRsrqDb``
  reject candidate targets below the configured RSRP/RSRQ floors, and
  ``NrGnbRrc`` ``HandoverMinTimeOfStay`` suppresses a handover back to the cell
  the UE just left. Covered by the new ``nr-handover-target-admission`` test
  suite.
- Added MIB-wait cell reselection for force-camped UEs (``NrUeRrc``
  ``MibWaitReselectTimeout`` and ``MibWaitReselectMaxAttempts``): a UE stuck in
  IDLE_WAIT_MIB on a cell whose MIB it never decodes now reselects the
  strongest untried measured cell and camps again. Covered by the new
  ``nr-mib-wait-reselection`` test suite.
- Added ``RandomDirectionDisc2dMobilityModel``, a random-direction mobility
  model bounded by a circle rather than a rectangle, and the
  ``HexagonalGridScenarioHelper::SetUeDiscRadius()`` method to place and move
  UEs inside a disc centred on the central site. Covered by the new
  ``nr-random-direction-disc-2d`` test suite.
- Added the ``NrHelper::AttachWindow`` attribute (disabled by default) to
  stagger the initial UE attachment over a configurable window,
  de-synchronising the RACH thundering herd when many UEs attach at once.
  Covered by the new ``nr-attach-window`` test suite.
- Added ``HexagonalGridScenarioHelper::SetRingOneOnlyPicos()`` to deploy
  picocells only around the six ring-1 sites, the placement the 3GPP TR 36.839
  no-wraparound dense-A layout calls for.
- ``CQI_CSI_RS`` is now accepted alone as a ``CsiFeedbackFlag`` value: the
  channel matrix is updated from each CSI-RS reception, so CSI feedback no
  longer depends on PDSCH data traffic being present.
- The RLC was reworked towards TS 38.322 conformance: AM status, polling and
  window handling, UM receive-window behaviour, and re-establishment resets
  were fixed and are covered by the extended ``nr-test-rlc-am-status``,
  ``nr-test-rlc-um-rx`` and ``nr-test-rlc-header`` suites, and the TM mode now
  fires a ``TxDrop`` trace on transmit-buffer overflow (new
  ``nr-test-rlc-tm`` suite). Every PDU is stamped with a transmitting-entity
  identity so that stale PDUs of a re-created peer entity still in flight after
  a handover are discarded instead of corrupting reassembly.
- The CI pipeline gained a weekly valgrind memcheck job and a weekly
  MemorySanitizer job (whose findings are fixed in this release), and a
  lizard-based code-linting check that fails a merge request when it raises the
  maximum cyclomatic complexity or adds duplicate code blocks relative to the
  upstream default branch (introduced late in the v5.0 cycle).

Bugs fixed
----------
- Fixed a permanent uplink stall when the bootstrap BSR transport blocks are
  lost. The UE MAC only re-armed its scheduling request when the last received
  UL DCI had HARQ process 0 or rv 3, so once the few bootstrap grants were spent
  without a decoded BSR no scheduling request was ever sent again and the gNB
  stopped granting. The scheduling request is now re-armed whenever new buffer
  status arrives while the UE is backlogged, as the UE keeps requesting
  resources while a BSR is pending and no UL-SCH resources are available
  (TS 38.321 clauses 5.4.4 and 5.4.5). The scheduler no longer records the
  request as 12 bytes of buffer in every uplink logical channel group of the
  UE. A scheduling request carries neither a buffer size nor a logical channel
  group, and only a buffer status report tells the scheduler what the UE has
  and where, so the request is now tracked as a pending bootstrap grant, which
  makes the UE eligible for uplink scheduling without making any of its logical
  channels look active. Previously a backlogged UE repeating the request
  downgraded the buffer reported by an earlier BSR and kept its signalling
  group permanently active, which flattened the uplink priorities of the QoS
  schedulers. The rate at which the request is repeated is now bounded by the
  new ``NrUeMac::SrProhibitTimer`` attribute (``sr-ProhibitTimer``, 10 ms by
  default): while it runs the request stays pending instead of being
  transmitted on every slot, and it is stopped when a grant that carried the
  buffer status drains the buffer. A request that goes unanswered for
  ``NrUeMac::SrTransMax`` transmissions is cancelled, and the UE starts a random
  access procedure to recover uplink resources instead of asking forever.
  Covered by the new ``nr-ul-sr-retransmission`` test suite.
- Fixed ``NrRlcUm`` and ``NrRlcTm`` stopping their buffer status report timer
  whenever a new SDU arrived from PDCP. New data is what triggers a buffer
  status report (TS 38.321 clause 5.4.5), never a reason to stop reporting, and
  the timer is only armed again by a transmission opportunity: once the uplink
  stalled, the last cancellation was final and the buffer status the gNB held
  went stale. ``NrRlcAm`` already behaved correctly.
- Fixed the scheduler dereferencing an invalid UE map entry when a DL CQI report
  arrived for a UE that had already been released from the cell (handover, RRC
  release, radio link failure). The lookup was guarded only by an assertion, so
  the dereference happened in optimized builds; such stale reports are now
  logged and dropped, as was already the case for the UL CQI and UL HARQ paths.
- Fixed MIMO SINR chunks being selected by RNTI alone when decoding a transport
  block. RNTIs are per-cell values that are reused across cells, so following a
  handover a chunk measured against the previous cell could be matched to a
  transport block expected from the new one. Where the two cells had granted a
  different rank, the averaged SINR matrix no longer matched the expected rank and
  decoding aborted. Chunks now carry the cell they were received from and are
  filtered on it.
- Fixed an abort when a data signal arrived misaligned with a reception already in
  progress. A cell or RNTI change part-way through a reception, as happens on
  handover or after radio link failure, does not stop that reception, so the next
  allocation from the new cell could arrive on a different symbol or with a
  different length and trip the time-alignment assertion. Such a signal is now
  discarded, and still contributes interference.
- Fixed the handover command not applying the target cell's RACH configuration
  to the UE MAC, which sized the RA response window and bounded the preamble
  retransmissions from the source cell's (or no) configuration.
- Fixed ``BwpManagerGnb/Ue::SetOutputLink()`` silently ignoring a second call
  for the same source BWP, which made it impossible to re-point a link.
- Fixed a read of an uninitialized numerology in ``NrUeRrc::StartConnection()``
  when the UE was forced to camp without decoding SIB1; the FR1-safe RACH lock
  duration is now used.
- Fixed FDD operation of the ``cttc-nr-traffic-ngmn-mixed`` example: uplink
  traffic was routed to the DL-only BWP and silently dropped. Also made the
  example abort on a simulation time too short for the applications to run,
  instead of crashing in the statistics post-processing.
- Fixed receivers permanently misreporting interference after a saturating
  signal expired: a very strong signal absorbed the co-resident ordinary
  signals when added to the running sum of received power (floating-point
  absorption), and an infinite signal turned the sum into NaN, so every later
  SISO SINR chunk at that receiver was garbage. The sum is now rebuilt from
  the tracked list of live signals at each chunk evaluation. Covered by the
  new ``nr-interference-saturating-signal`` test suite.
- Fixed a new transmission (NDI set) on a HARQ process being combined with stale
  HARQ history left by an earlier corrupted transport block, when the previous
  retransmission chain ended without a successful reception. The history is now
  flushed on new data. Covered by the new ``nr-test-harq-flush`` test suite.
- Fixed TDMA schedulers and OFDMA uplink scheduling starving a cell when the
  number of active UEs at MCS 0 reached the schedulable resources of a slot:
  every UE received an allocation too small to transmit, all DCIs were
  silently discarded, and without data there was no CSI feedback to raise the
  MCS. Sub-minimum-TBS allocations are now reaped and redistributed, mirroring
  the OFDMA downlink behaviour. Covered by the
  ``nr-system-test-scheduler-many-ues`` test suite with 100 UEs.
- Fixed the DCI RBG bitmask being validated against the RB count instead of
  the RBG count.
- Fixed an abort when a UE that had declared radio link failure re-selected a
  cell and sent a fresh RRC connection request while the gNB still held its
  context in an active state; the request is now ignored and the state timeout
  releases the context. Covered by the new ``nr-stray-rrc-request`` test suite.
- Fixed the RLC reassembly aborting on aliased sequence numbers after a
  handover, and the duplicate delivery to PDCP of an out-of-order PDU whose
  sequence number slid back inside the reordering window.
- Fixed a reference cycle that kept the whole UE stack (NrUeNetDevice,
  NrEpcUeNas, NrSpectrumPhy) alive for the lifetime of the process; the
  back-pointers and callbacks are now released on dispose.
- Fixed uses of uninitialised memory reported by MemorySanitizer, and made the
  tests destroy the simulator to release still-reachable memory.
- Fixed the ``gsoc-leo-demo`` example: it now forces LOS, points the UE
  antenna to the sky, and over-drives the uplink in the smoke test.

Release NR-v5.0
---------------

Availability
------------
July 3, 2026.

Cite this version
-----------------
DOI: 10.5281/zenodo.21165297

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-20.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 25.10 (Questing Quokka) with g++15 and clang-20.
- ARM
  - Ubuntu 25.10 (Oracular Oriole) with g++15 and clang-20.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release is compatible with ns-3.48.

Important news
--------------
- FDD users should apply the ns-3 patch from merge request
  https://gitlab.com/nsnam/ns-3-dev/-/merge_requests/2929, which solves an
  important bug that could cause UEs in NLOS condition to receive 30 dB less
  power (we measured up to a 1.4x decrease in the number of corrupted UL
  transport blocks after applying it). TDD users are not affected.
- Users are encouraged to apply the ns-3 patch from merge request
  https://gitlab.com/nsnam/ns-3-dev/-/merge_requests/2883 for much faster
  3GPP channel computations.
- The NR module now builds on Windows: internal symbols are hidden and the
  public API is exported through the new ``NR_EXPORT`` macro.
- Major overhaul of the handover, RRC and radio-link-failure machinery:
  inter-frequency and inter-numerology handovers, a same-cell primary-BWP
  switch mechanism, Cell Individual Offset (CIO) handover biasing, msg3 HARQ
  retransmissions, RRC real mode fixes, FDD support in the RLF/RRC procedures,
  and much wider test coverage (X2 handover, inter-frequency handover, BWP
  switch, RLF over RRC ideal/real in TDD and FDD).
- The UE measurement chain was corrected: RSRP is averaged in the linear
  domain (fixing reports stuck at range 0 in multi-cell scenarios with data
  traffic), the RSRP reporting range mapping was updated to 5G-NR, RSRQ is now
  interference-aware, and the beam-scan and SRS channel-estimation models of
  the beamforming algorithms were fixed.
- The fronthaul control model now supports selecting the functional split
  (FS 6, 7.1, 7.2 and 7.3), with per-split fronthaul throughput computations.
- The documentation was heavily extended: new per-layer design chapters (PHY,
  MAC, RLC, PDCP, SDAP, RRC, NAS, EPC and the S1/S5/S11/X2 interfaces),
  sequence diagrams and state machines generated from committed sources, and
  traffic-pattern figures generated from simulation runs.

New user-visible features
-------------------------
- Added the ``cttc-nr-demo-sionna-rt`` example, which runs an end-to-end NR
  deployment with ``NrChannelHelper`` configured to use the Sionna RT channel
  model. The example exposes Sionna RT scene and path-solver parameters on the
  command line and is built only when Sionna RT dependencies are available.
- Added the ``gsoc-leo-demo-example`` NTN example (GSoC 2025), integrating LEO
  satellite mobility with the 3GPP NTN channel and propagation models, with
  representative direct-to-mobile, VSAT and backhaul deployment presets.
- Added a same-cell primary-BWP switch mechanism driven by per-carrier RSRP
  measurements (``NrUeRrc::BwpSwitchHysteresis``).
- Added inter-frequency handover support with the corresponding measurement
  configuration, and inter-numerology handover continuity.
- Added Cell Individual Offset (CIO/CRE) support for handover biasing,
  including negative offsets (TR 36.839 Set 5).
- Added msg3 HARQ retransmissions (``NrMacSchedulerNs3::Msg3MaxRetx``).
- Added the option to declare RLF on RLC-AM maximum retransmissions
  (``NrUeRrc::RlcMaxRetxTriggersRlf``, TS 38.331 Section 5.3.10.3).
- Added fronthaul functional split selection (``NrFhControl::FunctionalSplit``)
  and fronthaul capacity handling for full-duplex fronthaul links.
- Unified RNG stream assignment behind ``NrHelper::AssignStreams()`` for
  reproducible simulations.

Bugs fixed
----------
- Fixed the code block segmentation count in NrAmc::CalculateTbSize: the
  number of code blocks was computed with an integer division truncated
  before applying ceil (TS 38.212 Section 5.2.2 mandates a real-valued
  division), undercounting the code block CRC overhead and over-estimating
  the usable transport block size.
- Fixed UE RSRP measurement overflow (sample counter wrap) and dB-domain
  averaging corruption, which produced out-of-range RSRP values and permanently
  stalled A3-triggered handovers in multi-cell scenarios with data traffic.
- Fixed RLC-UM reassembly delivering corrupt SDUs after t-Reordering expiry,
  the RLC-UM PDCP discard being a no-op, and invalid framing-info transitions
  (#272).
- Fixed the receive-side zenith step in the ideal and realistic beam scans
  (#276).
- Fixed the realistic beamforming SRS channel estimation to match the
  documented reference model (time-domain filtering gain and total error
  variance).
- Fixed a DL HARQ symAvail double-count underflow (#278).
- Fixed a heap overflow in the HARQ round-robin beam ordering.
- Fixed the RA-preamble sentinel so idle UEs do not match stray RARs.
- Fixed the radio environment map helper to use the origin antenna element
  type when copying antennas (#274).
- Fixed X2 status serialization/deserialization, the X2 interfaces setup, and
  the RRC bandwidth ASN.1 (de)serialization (FR1/FR2 CHOICE encoding).
- Fixed non-deterministic behavior from unstable sorts and standard shuffles
  (#250).
- Fixed the outdoor UE speed being stuck at the indoor value in the hexagonal
  grid scenario helper.
- Fixed endless rescheduling of delayed measurement reports and handovers.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items

Release NR-v4.2
---------------

Availability
------------
March 5, 2026.

Cite this version
-----------------
DOI: 10.5281/zenodo.18875770

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-20.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 25.10 (Questing Quokka) with g++15 and clang-20.
- ARM
  - Ubuntu 25.10 (Oracular Oriole) with g++15 and clang-20.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release is compatible with ns-3.47.

Important news
--------------
- A 10 year old bug in RLC UM, inherited from LTE, was solved. It could crash simulations with a PDCP
  assert related to dcBit==DATA_PDU. It resulted from an incorrect handling of reassembling out-of-window PDUs,
  resulting in a stale cached PDU outside the PDU, that was then reassembled to the wrong sequence of PDUs after
  the reception window wrapped around.
- The BWPId and CellId schemes were modified. Now there is a single CellId per gNB,
  and BWPIds are local to that gNB.
- ARFCNs are now used instead of frequency. Whenever BWPs of two different devices are compared,
  their respective BWPIds should be retrieved based on the ARFCN match. Note that final frequency
  used during the simulation may differ from the specified one by the user, if not representable by the ARFCN,
  as defined in TS 38.104 and patent EP4021097A1.
- Classes renamed:
  - `NrEpcTft` was renamed to `NrQosRule`
  - `NrEpcTftClassifier` was renamed to `NrQosRuleClassifier`
  - `NrEpsBearer` was renamed to `NrQosFlow`
  - `NrEpsBearerTag` was renamed to `NrQosFlowTag`
  - `QCI` was renamed to `fiveQi`
- The hardcoded eviction factor in `NrMacSchedulerOfdma` was replaced with an exhaustive solution to
  prevent stopping allocation earlier, if there are enough available resources, even if their usage
  cause an MCS drop.
- Indoor and outdoor calibration parameters were updated to match 3GPP SU-MIMO reference studies (e.g. RP-180524).
- MCS tables were moved from `nr-eesm-t1.h/cc` and `nr-eesm-t2.h/cc` to `nr-mcs-tables.h/cc`. Precision of effective
  code rates, spectral efficiency and CQI tables were increased.
- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------
- The QFI field was added to QoS rules
- The class `PhasedArrayAngleConvention` was introduced to control whether the angles passed to Kronecker
  beamforming vectors follow the 3GPP convention, or the ULA convention in ns-3.

Bugs fixed
----------
- (02da6eef) Bug fixed in the realistic beamforming algorithm: added missing conjugate transpose of
  the RX beamforming vector (See ns-3 #1314).
- (bc99102e) Logical channel priorities now return the respective 5QI priority instead of the 5QI value.
- (4b8498a4) `NrNoBackhaulEpcHelper::SetupRemoteHost()` now returns the remote host address instead of the PGW.
- (99090a26) Set 5QI for SRB1.
- (160331d2) Improve documentation of `NrAmc::CalculateTbSize()` and `NrAmc::GetPayloadSize`.
- Set random streams of:
  - (0f1c967b) `cttc-realistic-beamforming`
  - (3b1433fd) `nr-ideal-beamforming-test`
  - (2c01aa7f) `PmSearch` in `NrHelper`
  - (ff414885) `nr-test-epc-e2e-data`
  - (aeec59b5) `nr-phy-patterns`
- (5a8743ff, 599d434a, 86e774f0) Ensure each antenna pair is used by a single channel at `NrRadioEnvironmentMapHelper`
  and `cttc-channel-randomness.cc` example.
- (8107c1a5) Fix RLC UM receive window, by reassembling the oldest `SequenceNumber` outside the reception window,
  instead of the lowest `SequenceNumber` in the buffer.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items


Release NR-v4.1.1
-----------------

Availability
------------
Available since October 16, 2025.

Cite this version
-----------------
DOI: 10.5281/zenodo.17366429

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-20.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 25.10 (Questing Quokka) with g++15 and clang-20.
- ARM
  - Ubuntu 25.10 (Oracular Oriole) with g++15 and clang-20.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release is compatible with ns-3.46.

Important news
--------------
- This release includes one important change, which is using the upstreamed hexagonal wraparound model in ns-3.46.

- This release includes important fixes, such as:
  - To avoid misconfigurations, the attribute ``NrHelper::EnableMimoFeedback`` was removed.
    The MIMO feedback is now implicitly enabled/disabled based on ``NrHelper::CsiFeedbackFlags``.

- A lot of code changes are related to the upgrade of clang-format (17->20) and clang-tidy (17->19),
  used with ns-3.46.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------

Bugs fixed
----------

- (9cb800b2) Imply MIMO feedback based on `NrHelper::CsiFeedbackFlags` to prevent misconfiguration such as `NrHelper::EnableMimoFeedback=True` and `NrHelper::CsiFeedbackFlags=CQI_PDSCH_SISO`.
- (1d5a12ce) Set standard compliant sub-band size for DenseAmimoIntel settings.
- (518e061d) Do not call ConstructSelf from objects derived from Object.
- (2fa3e26d) Add missing flag to properly load outdoor calibration settings.
- (3c7a8b44) Enable wraparound by default in Intel R1-1707360 based scenarios.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items

Release NR-v4.1
---------------

Availability
------------
Available since July 7, 2025.

Cite this version
-----------------
DOI: 10.5281/zenodo.15830121

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-19.
  - Ubuntu 20.04 with g++10.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 24.10 (Oracular Oriole) with clang-19.
- ARM
  - Ubuntu 24.10 (Oracular Oriole) with clang-19.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release will be compatible with ns-3.45.

Important news
--------------
- This release includes some features, such as:
  - Resource assignment matrix (currently used to ensure there are no overlapping allocated resources);
  - Wraparound model and support for hexagonal deployments, which allows for higher interference without requiring the simulation of additional rings, which can drastically speed up simulations;
  - FastFadingConstantPositionMobilityModel, a static mobility model with non-zero velocity to exercise channel fading code;
  - Indoor and Outdoor calibration examples were updated to reflect RP-180524;
  - Add new random TDMA and OFDMA schedulers, to maximize the chance of interference between network nodes (used for testing);
  - Add CI jobs to test nr on MacOS, and to perform periodic regression testing with outdoor calibration.

- This release includes important fixes, such as:
  - Schedulers now have memory, maintaining their fairness over time;
  - Downlink HARQ is now transmitted in scenarios where there are more active transmissions than symbols.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------

Bugs fixed
----------

- (bd722156) Prevent downlink HARQ from never transmitting (in case there are more active transmissions than symbols).
- (dd44a226) Use CreateObject<> for Object derived classes. Create<>() results in uninitialized attributes.
- (484144cd) Discard out-of-range signals (all-zero PSD) at nr-spectrum-phy (a result from DistanceBasedThreeGppSpectrumPropagationLossModel).
- (a5cf6fde) Add assert to check for required for phased array propagation loss model in RSRP-based initial association.
- (b3274e23) Fix data DCIs that were not reflecting notching masks.
- (242f43d4) Fix fronthaul dropping policy, by preventing the transmission of data DCI if control DCI is discarded.
- (83b2c1b3) Fix temporal fairness of PF and QoS schedulers.
- (8b92cd08) Fix temporal fairness of RR schedulers.
- (e23a296e) Allow NR to work without beam managers set, to support legacy channel and antenna models.
- (8a4a3cc4) Prevent starvation of UEs with MCS0 on OFDMA schedulers.
- (b0343c64) Reap and redistribute insufficient scheduled resources to other UEs.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items

Release NR-v4.0
---------------

Availability
------------
Available since May 15, 2025.

Cite this version
-----------------
DOI: 10.5281/zenodo.15422217

Supported platforms
-------------------
This release has been tested on the following platforms:
- x86_64
  - Arch Linux with g++-15 and clang-19.
  - Ubuntu 20.04 with g++10.
  - Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
  - Ubuntu 23.04 with g++13.
  - Ubuntu 24.10 (Oracular Oriole) with clang-19.
- ARM
  - Ubuntu 24.10 (Oracular Oriole) with clang-19.
  - MacOS Sequoia 15.4.1 with AppleClang 17.

This release will be compatible with ns-3.44.

Important news
--------------
- This release includes major features, such as support for:
  - CSI-RS signals for channel estimation;
  - CSI-IM for inter-cell interference measurements;
  - multiple single-panel antenna arrays per UE;
  - sub-band CSI feedback;
  - sub-band aware scheduler;
  - alternative symbols per beam scheduler;
  - multiple RI/PMI selection alternatives;
  - Kronecker beamforming;
  - initial association based on maximum RSRP instead of shortest distance;
  - reinforced-learning based scheduler contributed by Hyerin Kim during GSoC 2024;
  - NYUSIM, FTR and legacy channel models and the new channel helper contributed by João Albuquerque during GSoC 2024.

- This release also contain important fixes. See the bugs fixed section in this document and CHANGES.md for more details.

- Previously, CSI feedback was based only on PDSCH, and there was a workaround to ensure the channel was estimated even in resources allocated to different users. That workaround has been removed. See CHANGES.md for more details.

- It is no longer necessary to call NrHelper::UpdateDeviceConfigs(); please remove from any examples or tests.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------

- We introduced CSI-RS signals and CSI-IM interference measurements to complement PDSCH-based CQI feedback. These can be configured through the `NrHelper::CsiFeedbackFlags` attribute. Example `cttc-nr-mimo-demo.cc` is extended to show how to configure different types of the feedback.
- We introduced a new attachment algorithm `AttachToMaxRsrpGnb` which follows 3GPP configuration to attach UE to the gNB with highest RSRP within indicates handoff margin.
- We introduced a new beamforming algorithms, `KroneckerBeamforming` and `KroneckerQuasiOmniBeamforming` which provides Kronecker based beamforming whereas in the `KroneckerBeamforming` both gNB and UE use Kronecker, and in the `KroneckerQuasiOmniBeamforming` gNB use Kronecker vector and UE used Omni direction to do Beamforming.
- As part of the GSoC 2024 contribution from João Albuquerque, we introduced a new helper, `NrChannelHelper`, which provides a simple interface for configuring spectrum channels with various channel models, including NYUSIM, Fluctuating Two-Ray (FTR), and 3GPP. Previously, channel configuration was done along with band creation, where the user selected the scenario for use in the 3GPP model as an argument. In addition to extending the module to support NYUSIM and FTR channels, we have also created a helper class to assist with channel configuration `NrChannelHelper`. Users can now use this helper class to configure channels. The `gsoc-nr-channel-models.cc` example demonstrates how to configure different spectrum channels for end-to-end simulations. These channels can be 'legacy' (e.g., Friis) or not (e.g., NYUSIM, 3GPP, FTR), via an adaptation to transform the receiver's PSD (Power Spectral Density) into a channel matrix approximation. The `nr-channel-setup-test.cc` test checks if the NrChannelHelper API can correctly create a specified channel, which will be created using well-defined combinations of scenarios, channel models, and channel conditions.
- The ns-3 implementation of NYUSIM channel model is added as a feature of the NR module. The original files are installed in utils directory, with some version updates. Acknowledgments to Hitesh Poddar (NYU WIRELESS) and Tomoki Yoshimura (Sharp Laboratories of America), authors of the NYUSIM channel model code implementation in ns-3. (Original code: https://github.com/hiteshPoddar/NYUSIM_in_ns3/tree/main)
- As part of the GSoC 2024 contribution from Hyerin Kim, we introduced a new Reinforced-Learning based MAC schedulers, `NrMacSchedulerTdmaAi` and `NrMacSchedulerOfdmaAi`, using the `ns3-gym` module.
- We introduced sub-band CQI reporting, in addition to additional Rank Indicators (RI)/Precoding Matrix Indicator (PMI) selection techniques, as alternatives to the `NrPmSearchFull`, such as `NrPmSearchIdeal`, `NrPmSearchFast`, `NrPmSearchSasaoka` and `NrPmSearchMaleki`.
- We introduced sub-band awareness to the MAC schedulers, which can be controlled via the attribute `NrMacSchedulerNs3::McsCsiSource`.
- We introduced new symbols per beam scheduling options, which can be controlled via  the attribute ``NrMacSchedulerOfdma::SymPerBeamType``.
- We introduce a multi panel antenna approach in spectrum phy which allows UE to have more than 1 panels which could be 1 of them activated at time. This feature is included in `AttachToMaxRsrpGnb` and `KroneckerQuasiOmniBeamforming`.


Bugs fixed
----------
- (0305c3b1) With CSI-RS+CSI-IM, users are not starved due to CQI 0 preventing RBG allocation when there was no data transmission and PDSCH-based CQI feedback was used.
- (2ae58438) UEs numerology is now properly set via MIB, in case UEs were not explicitly attached to gNBs via `NrHelper::AttachToGnb()`.
- (9c479cec) UEs getting stuck in the ACTIVE state after multiple BSR transmission losses have been fixed by adjusting the UEs' state machine transitions and using the BSR expiration timer. If the timer expires without receiving any grant in response to the BSR and HARQ is not active, an SR is retransmitted. If the timer expires while HARQ is active, the UE checks whether all HARQ retransmission attempts have been completed. If so, an SR is retransmitted; otherwise, it waits for HARQ to finish all retransmissions before sending an SR again (see issue #230).
- (61ab4eb4) MSG3 is now transmitted only on UL and F slots.
- (9cf6c894) RRC setup for FDD has been fixed, but it requires proper configuration. See CHANGES.md for more details.
- (4955e72c) RandomPRB-based downsampling could generate an illegal offset during sub-band downsampling.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items


Release NR-v3.3
----------------

Availability
------------
Available since October 15, 2024

Cite this version
-----------------
DOI: 10.5281/zenodo.13929095

Supported platforms
-------------------
This release has been tested on the following platforms:
- Arch Linux with g++-14 and clang-18 (with GNU stdlibc++ and LLVM libc++).
- Ubuntu 20.04 with g++10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
- Ubuntu 23.04 with g++13.
- Ubuntu 23.10 (Mantic Minotaur) with clang-16.

This release is compatible with ns-3.42.

Important news
--------------
- This release includes Fronthaul Control mechanisms, that allow
  to simulate a limited-capacity fronthaul (FH) link based on the
  Fronthaul Capacity (m_fhCapacity) set by the user in the example
  script, and to apply FH Control methods (m_fhControlMethod) in order
  to restrict user allocations, if these they do not fit in the
  available FH capacity. O-RAN 7.2x functional split is assumed.
  The methods supported are the Dropping, Postponing, Optimize MCS
  and Optimize RBs, however it can be easily extended to apply
  additional methods. The Fronthaul Control takes decisions based
  on the model applied, the available fronthaul capacity, the applied
  modulation compression and the number of active UEs in each cell
  (either with new or HARQ data). This last information is updated
  after each scheduling process is finalized, through the MAC layer.
  The interaction of the Fronthaul Control with the MAC and high-PHY
  layers takes place through a set of SAP interfaces that allow the
  bidirectional exchange of information.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------
- Fronthaul Control mechanisms are available in the NR module.
  The ``cttc-nr-fh-xr`` example shows how to use and configure the
  Fronthaul Control in a scenario employing XR, CG and VoIP traffic.
  For a more detailed description see the NR manual and Doxygen.

Bugs fixed
----------
- (2b84532c) Set the correct UL output file name for the ``NrMacSchedulingStats``.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items


Release NR-v3.2
---------------

Availability
------------
Available since September 25, 2024

Cite this version
-----------------
DOI: 10.5281/zenodo.13837253

Supported platforms
-------------------
This release has been tested on the following platforms:

- Arch Linux with g++-14 and clang-18 (with GNU stdlibc++ and LLVM libc++).
- Ubuntu 20.04 with g++10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
- Ubuntu 23.04 with g++13.
- Ubuntu 23.10 (Mantic Minotaur) with clang-16.

This release will be compatible with ns-3.42.

Important news
--------------

- NR is now independent from the upstream LTE module from ns-3.
  To simplify the porting process, we migrated most source files from LTE, including RLC, RRC, PDCP, EPC.
  However, keep in mind we are not adding supported features at this time.
  We hope to enable more features in future releases.

- We introduced three new CI jobs with additional tests.
  The first CI job checks for now deprecated emacs lines.
  The second job checks if the nr works normally without eigen (which is required by MIMO).
  The third job tests nr with LLVM's libc++. This helps to detect defects
  caused by relying on implementation specific behavior of different C++ standard libraries.

- The ``NrHelper::EnableHarq`` attribute was removed for being misleading and causing bugs.
  If you want to avoid retransmissions, use ``NrMacSchedulerNs3::EnableHarqReTx`` instead.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------

Bugs fixed
----------

- (5b5f2d27) Fix Codebook type configuration using the NrHelper::SetupMimoPmi()
- (e1272b43) Print missing CQI field and remove extra tabulation
- (3617c735) Do not account for CRC when estimating potential throughput
- (3f93030d) Remove duplicate multiplication of RBGs and symbols per beam
- (c2ea8533) Remove misleading HarqEnabled flag from NrHelper and add test case
- (3618a80a) Sort control message lists before merging
- (a68f143c) Update type of rbNum to prohibit overflow and wrong SINR trace
- (738b08bc) Prevent out-of-bounds access when rntiIt == m_rlcAttached.end()
- (ed645749) Prevent negative values of effective SINR
- (63c863ce) examples: Disable cttc-nr-mimo-demo when eigen is not found

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items


Release NR-v3.1
----------------

Availability
------------
Available since July 19, 2024

Cite this version
-----------------
DOI: 10.5281/zenodo.12773723

Supported platforms
-------------------
This release has been tested on the following platforms:
- Arch Linux with g++-13 and clang-16.
- Ubuntu 20.04 with g++10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
- Ubuntu 23.04 with g++13.
- Ubuntu 23.10 (Mantic Minotaur) with clang-16.

This release is compatible with ns-3.42.

Important news
--------------
- This release comes with a completely new PMI Type-I Single Panel codebook for MIMO.

- Performance optimizations were included for faster simulations.

- The code base is updated for C++ conformance and aligned with ns-3.42.

- Many bugs were fixed. One of them is significant, however it is limited to NR-U users.
  The interference events were counted incorrectly after a refactoring merged in nr-3.0.
  This caused the NrSpectrumPhy to change state prematurely due to sensing the channel
  as empty when there was a transmission with received power bigger than the threshold.

- We introduced two new CI jobs. The first to improve uniformization of commit messages.
  The second catches memory related bugs, allowing for us or contributors to fix them
  before finding them on the wild, crashing simulations or generating bogus results.

- Remember to follow the instructions from the README.md file, i.e., to checkout
  the correct release branch of both ns-3 and the NR module. The information about
  compatibility with the corresponding ns-3 release branch is stated in the
  `README.md` file.

New user-visible features
-------------------------
- A new MIMO Type-I Single Panel codebook is available in the NR module.
  The ``NrCbTypeOneSp`` codebook allows the MIMO model introduced in
  nr-3.0 to support up to 32 antenna ports and up to rank 4. See the
  ``cttc-nr-mimo-demo.cc`` file for an example on how to configure
  the new codebook. More details are available at the nr-manual and Doxygen.

- Sub-band downsampling/upsampling techniques were implemented, allowing
  for much faster simulations using large antenna panels with many ports.

Bugs fixed
----------
- (edd72f85) Fixed the number of rows when creating a dummy precoding matrix
- (2c98da82) Abort when size of packet isn't supported by UDP
- (d827c1df) Detect overflow in active tx count
- (a9c6af8b) Assigned random streams to fix traffic-generator-test
- (1092d90d) Reverted changes to NrInterference accounting of events
- (88973614) Fixed serialized size of NrRadioBearerTag
- (884b811e) Extend 16-bit numbers to 32-bit, to prevent overflows in Cantor computation
- (9b59d43c) Prevent memory leak by freeing up SpectrumValue globals at the end of the simulation
- (1df0862b) Prevent memory leak by explicitly disposing Phys not aggregated to a Node
- (3cddaa57) Removed the deprecated attribute 'UseFixedRi' leftovers
- (9da24622) Removed the deprecated attribute 'NrSpectrumPhyList' leftovers
- (6b182dca) Fix config paths used by DlCtrlPathloss/DlDataPathloss
- (1e47feb1) Fix static and monolib build of nr
- (0fda3f52) Use sqlite prepare statement to prevent leak
- (0989a1eb) Fix NrSpectrumPhy disposal to include m_rxSpectrumModel and m_txPsd
- (69a76a94) Fix RealisticBeamformingHelper disposal

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items

Release NR-v3.0
----------------

Availability
------------
Available since February 16, 2024

Cite this version
-----------------
DOI: 10.5281/zenodo.10670856

Supported platforms
-------------------
This release has been tested on the following platforms:
- Arch Linux with g++-13 and clang-16.
- Ubuntu 20.04 with g++-9 and 10, and clang-10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.
- Ubuntu 23.04 with g++13.
- Ubuntu 23.10 (Mantic Minotaur) with clang-16.

This release is compatible with ns-3.41.

Important news
--------------
- This release comes with a completely new MIMO model implementation.
Comparing to the previous dual-polarized implementation, this new model offers a
much more general approach and can be easily extended to support different types of MIMO,
which was not the case with the previous implementation which was limited to a
particular case of MIMO, i.e., DualPolarized-MIMO (DP-MIMO), supporting
only 2 streams with no possibility for extension. Old DP-MIMO implementation
used some assumptions related to the mapping of the streams to ports that were
limiting its application, i.e., each stream was mapped to the antenna elements
with a specific polarization. This means that there was a hard limitation on the
assignment of streams to a specific port and to the specific antenna elements of
one polarization. Also, it used a not realistic inter-stream interference rejection
model. Instead, the new MIMO implementation will support general MIMO, through
the inclusion of 3GPP-compliant PMI (precoding matrix indicator) and RI (rank indicator),
assuming MMSE-IRC (interference rejection combining) receiver for inter-stream
interference suppression, as adopted in 3GPP. Such general MIMO includes as particular
case the old DP-MIMO, but with a more generic and realistic model for DP-MIMO in
which the streams/port/antennas mapping is not predefined but defined through
digital precoding and in which a more accurate model for the inter-stream
interference calculation is considered. The new MIMO is flexible and can be easily
extended for more streams/ranks/ports.

- The current MIMO implementation requires Eigen3 library (https://libeigen.gitlab.io).

- When creating an MR, 5G-LENA users will be able to use CI/CD minutes belonging
to the NR module project.

- The code base is updated and a bit modernized, e.g. to use standard integer types,
simpler bool checks, spelling errors are fixed, automatic clang-tidy fixes are
applied, for loops are modernized, python formatter settings were added, etc.

- Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both ns-3 and the NR module. The information about
compatibility with the corresponding ns-3 release branch is stated in the
`README.md` file.


New user-visible features
-------------------------
- A new MIMO model is available in the NR module. The ``cttc-nr-mimo-demo`` example
shows how to use and configure the new MIMO model. See the NR manual and Doxygen
for the full description of the new MIMO model and APIs. The new APIs are
also listed CHANGES.md.

- The NR module pipelines have been improved in multiple aspects:
  - GCC 13 CI jobs are added.
  - Ubuntu rolling CI jobs are added to weekly tests.
  - Cmake format CI job is added.
  - Black and isort formatter settings for python were added and applied.
  - Added python format CI job.
  - Added spell-checking CI job.
  - Clang-tidy CI job is added.
  - The documentation build and deployment are split now, and the deployment is limited to master.
  - Merge request pipelines are enabled.

Bugs fixed
----------
None.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items


Release NR-v2.6
----------------

Availability
------------
Available since November 30, 2023

Cite this version
-----------------
DOI: 10.5281/zenodo.10246105

Supported platforms
-------------------
This release has been tested on the following platforms:
- Arch Linux with g++-13 and clang-16.
- Ubuntu 20.04 with g++-9 and 10, and clang-10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.

Recommended ns-3 release: ns-3.40

Important news
--------------
This release is compatible with ns-3.40.

Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both, ns-3 and the NR module. The information
about compatibility with the corresponding ns-3 release branch is stated in the
`README.md` file.

New user-visible features
-------------------------
- This NR release comes with the first NR module tutorial created by Giovanni
Grieco in the scope of GSoC 2023 project `IUNS-3 5G NR: Improving the Usability of ns-3's 5G NR Module`
mentored by Tom Henderson, Katerina Koutlia, and Biljana Bojovic. This tutorial
focuses on a `cttc-nr-demo` example and explains the internal functionality of the
NR RAN by providing a detailed, layer-by-layer insights on the packet lifecycle
as they traverse the RAN. The tutorial highlights all the points in the NR protocol
stack where packets may be delayed or dropped, and how to log and trace such events.

- Overall enhancement of the logging in the NR module. For many log messages
the log level is refactored to provide easier overall logging, which allows to
focus on the main events in the RAN protocol stack. In many places the logging
information is expanded and improved its comprehension.

- Added `Tx` and `Rx` trace sources in NrNetDevice to allow easier tracing of
the events when the packet is transmitted or received.

- The references in the examples are updated to point to the latest 3GPP documents.

Bugs fixed
----------
- #157 Changed default value of NumHarqProcess to 16.
- #166 Avoid Time overflow in nr-ch-access-manager.cc.
- Fixed to use NS_LOG_FUNCTION instead of NS_LOG_INFO for function calls.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items



Release NR-v2.5
----------------

Availability
------------
Available since July 26, 2023

Cite this version
-----------------
DOI: 10.5281/zenodo.8188631

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-9 or later
- clang-6 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- Arch Linux with g++-13 and clang-13.
- Ubuntu 20.04 with g++-9 and 10.
- Ubuntu 22.04 with g++11 and 12 and clang-11 and 14.

Recommended ns-3 release: ns-3.39

Important news
--------------
This release is compatible with ns-3.39.

Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both, ns-3 and the NR module. The information
about compatibility with the corresponding ns-3 release branch is stated in the
`README.md` file.

New user-visible features
-------------------------
- New QoS schedulers are included that perform scheduling by taking into account the
QoS requirements of QoS flows. `NrMacSchedulerTdmaQos` and `NrMacSchedulerOfdmaQos`
classes are responsible for setting the scheduler and access mode types when desired
by the user and updating the DL and UL metrics of each UE. `NrMacSchedulerUeInfoQos`
performs the sorting of the UEs (based on DL and UL metrics).

- New design for LC bytes assignment that allows the implementation of various
algorithms. A new base class is added, known as `NrMacSchedulerLcAlgorithm` that
allows the implementation of various algorithms for the LC byte assignment. Two
algorithms are implemented. `NrMacSchedulerLcRR` that includes the original
implementation of assigning bytes to LCs in RR fashion and `NrMacSchedulerLcQos`
that shares bytes among the active LCs by taking into account the resource type
and the e_rabGuaranteedBitRate of a flow.

Bugs fixed
----------
None.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr/-/work_items



Release NR-v2.4
----------------

Availability
------------
Available since April 5, 2023

Cite this version
-----------------
DOI: 10.5281/zenodo.7807983

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-9 or later
- clang-8 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- ArchLinux with g++-9, 10 and 11, and clang-8, 9, 10, 11 and 12.

Recommended ns-3 release: ns-3.38

Important news
--------------
Starting with this NR release, the `NR` module is compatible with REUSE software
(https://reuse.software/).
The old way of licensing the code is replaced by REUSE format and syntax.
The `NR` module pipelines have been extended to include `REUSE` job.

This release adds new traffic models to the NR module, NGMN and 3GPP Extended
Reality (XR) models.

With this release we have started to test the reproducibility of the examples,
e.g. the examples that are part of this reproducibility testing will fail if
the result is not as expected for the specific parameters configuration.

The `NR` module now has its `DOI` number which can be found in `RELEASE_NOTES.md`.

The `NR` module has a new pipeline job that checks the compatibility with the
latest ns-3 clang-format.

Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both, ns-3 and the NR module. E.g., the current
NR module Release 2.4 is compatible with the ns-3.38 release branch. The
information about compatibility with the corresponding ns-3 release branch
is stated in the `README.md` file.

New user-visible features (old first)
-------------------------
- The testing of the `NR` module has been extended to test the reproducibility of
the NR module examples results. To achieve this we have extended some of the `NR`
examples to include the verification of the produced results in terms of KPIs,
such as throughput and/or delay. This verification can help us
detect when some of the changes either in ns-3 modules or in the NR
features implementation or configurations affects and changes the original
performance of the `NR` module. The goal of this testing is not only to
guarantee the reproducibility, but also to allow an early detection of the
bugs in the code or in the configuration. The examples that are currently
covered to some extent with this reproducibility check are:
`cttc-nr-notching.cc`, `cttc-3gpp-channel-example`, `cttc-3gpp-channel-nums-fdm`,
`cttc-3gpp-channel-nums`, and `cttc-nr-demo`.

- The `NR` module now includes a new traffic generators framework that allows to
simulate NGMN traffic applications for mixed traffic scenarios and advanced
and multi-flow 3GPP XR traffic applications, such as Virtual Reality (VR),
Augmented Reality (AR), and Cloud Gaming (CG) applications.
The traffic models are included in `nr/utils/traffic-generators`,
with a goal to port them in the future to the ns-3 applications
module. To do that, we currently lack of tests for the 3GPP traffic generators.
This NR traffic generators framework adds the following traffic models to the
`NR` module: NGMN FTP, NGMN video streaming, NGMN gaming, NGMN VoIP, 3GPP AR Model
3A 3 streams: pose/control, scene/video and audio/data, 3GPP VR downlink 1 stream:
scene/video, 3GPP VR downlink 2 streams: scene/video and audio/data, 3GPP VR
uplink: pose/control, 3GPP CG downlink 1 stream: scene/video, 3GPP CG downlink
2 streams: scene/video and audio/data, and 3GPP CG uplink: pose/control.
The traffic generator framework can be easily extended to include more traffic
types. For more information about these models, please take a look in the NR
manual sections `NGMN mixed and 3GPP XR traffic models`, `Examples` and
`Test for NGMN traffic models`. New examples are added to demonstrate the usage
of the new traffic models. These are: `traffic-generator-example.cc`,
`cttc-nr-traffic-ngmn-mixed.cc`, and `cttc-nr-traffic-generator-3gpp-xr.cc`. Also,
`traffic-generator-test` is added to test traffic generator framework, and it
currently supports testing of NGMN traffic types.

Bugs fixed
----------
- Detected and fixed a bug when postponing transmissions in NR-U simulations.

- Fixed an error in PointInFTPlane constructor, i.e., m_rbg was defined
as `uint32_t`, but the constructor parameter was using `uint8_t`.

* Fixed an issue with `S` slots in `NrGnbPhy` and `NrPhy` to treat correctly `S`
slots when evaluating if they can be used for DL or UL transmissions.

* Fixed a bug in the `NrMacSchedulerNs3` in functions `DoScheduleDl` and
`DoScheduleUl` when updating the active DL and UL users list.

* Fixed a bug in `NrGnbPhy::StartSlot` that was not allowing to use a flexible
and configurable number of `CTRL` symbols per slot.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr



Release NR-v2.3
----------------

Availability
------------
Available since November 23, 2022

Cite this version
-----------------
DOI: 10.5281/zenodo.7780747

Supported platforms
-------------------
The supported platforms are the same as for the NR-v2.1 release, except that
the recommended ns-3 release is ns-3.37.

This release has been tested on the following platforms:
- ArchLinux with g++-9, 10 and 11, and clang-8, 9, 10, 11 and 12.

Important news
--------------
The module follows now the clang-format C++ code style. Clang-format can be easily
integrated with modern IDEs or run manually on the command-line.
Supported versions are:
- Clang-format-14
- Clang-format-15
- Clang-format-16

For more information please refer to
https://gitlab.com/nsnam/ns-3-dev/-/blob/master/doc/contributing/source/coding-style.rst

Whitespaces are also checked with:

```
$ python3 ns-3-dev/utils/trim-trailing-whitespace.py --check nr
```

This module can be updated with the usual:

```
$ git pull
```

command.

Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both, ns-3 and the NR module. E.g., the NR module
Release 2.2 is compatible with the ns-3.36.1 release branch, while the NR module
Release 2.3 is compatible with the ns-3.37 release branch.

The information about compatibility with the corresponding ns-3 release branch
is stated in this (RELEASE_NOTES.md) document in the "Supported platforms"
section for each NR release (starting from the NR Release 1.3).

New user-visible features (old first)
-------------------------
- Upgrade nr to `ThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity`
changes.
- Added new example called `cttc-nr-3gpp-calibration` used for the calibration
of the simulator under 3GPP outdoor reference scenarios.
- Added `DlDataSnrTrace`, `DlCtrlPathloss` and `DlDataPathloss` trace sources in
NrSpectrumPhy.
- `NrUePhy` now includes the RSRP measurements of a UE.
- PHY traces are extended with a function to set the results folder path.
- `HexagonalGridScenarioHelper` is extended to define the max UE distance from the
closest site for the `HexagonalGridScenarioHelper::CreateScenarioWithMobility`
function. Moreover, the simTag and the results folder can now be set.
- The antenna orientation in the `NodeDistributionScenarioInterface::GetAntennaOrientationDegrees`
is changed from 60, 180, 300 degrees to 30, 120, 270.
- `GridScenarioHelper` includes now a function to set the starting position of the grid.
- Included some performance enhancements, such as to remove from `NrEesmErrorModel`
unnecessary copy, to allow the `NrErrorModel` to be passed and fetched as an object
and to reduce the execution of tests and examples_to_run.py.

Bugs fixed
----------
- Sfnsf frame number is expanded to 32-bit to prevent rollover
- Fixed how the HARQ feedback from multiple streams is combined in `NrUePhy`.
- Fixed and modified the code for MAC UL/DL RLC TX/RX/PDU queues.
- Included a fix in NrHelper to avoid reassigning a stream due to incorrect pointer.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr



Release NR-v2.2
----------------

Availability
------------
Available since June 03, 2022

Supported platforms
-------------------
The supported platforms are the same as for the NR-v2.1 release, except that
the recommended ns-3 release is ns-3.36.1.

Important news
--------------
This module can be updated with the usual

```
$ git pull
```

command.

Remember to follow the instructions from the README.md file, i.e., to checkout
the correct release branch of both, ns-3 and the NR module. E.g., the NR module
Release 2.1 is compatible with the ns-3.36 release branch, while the NR module
Release 2.2 is compatible with the ns-3.36.1 release branch.

The information about compatibility with the corresponding ns-3 release branch
is stated in this (RELEASE_NOTES.md) document in the "Supported platforms"
section for each NR release (starting from the NR Release 1.3).

New user-visible features (old first)
-------------------------

None.

Bugs fixed
----------

None.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr



Release NR-v2.1
---------------

Availability
------------
Available since May 06, 2022

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-9 or later
- clang-8 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- ArchLinux with g++-9 and 10 clang-8, 9, 10, and 11

Recommended ns-3 release: ns-3.36.

Important news
--------------
This module can be updated with the usual

```
$ git pull
```

command. We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------
- Added new distance-based 3GPP spectrum propagation loss model
- Added the Get function to obtain the pointer to PHY traces
- Added scenario with UE mobility in `HexagonalGridScenarioHelper` class
- Added option to set random antenna height in centrain percentage of UEs in
`HexagonalGriScenarioHelper`
- Extended `HexagonalGridScenarioHelper` to allow installing the hexagonal scenario
with the 4th and the 5th ring (needed for the wrap around calibration)
- Added new attribute to `NrMacSchedulerNs` to allow enabling or disabling HARQ ReTx
- `NrRadioEnvironmentMapHelper` is extended to provide the progress report at std::
cout, i.e., 1%, 10%, ..., 100%, and provides the estimation of the time left
- Added CQI column in RxPacketTraceUe
- Added SIR calculation and plot in `NrRadioEnvironmentMapHelper`
- Added CellScan algorithm based on azimuth and zenith in class called
`CellScanBeamformingAzimuthZenith` in `ideal-beamforming-algorithm.h/cc`
- Added new trace for reporting DL SINR CTRL
- Extended and improved RLC and PDCP traces to include simple traces per RX/TX
side, and combined/merged end-to-end traces.



Bugs fixed
----------
- Fixed how to consider RLC overhead when updating the TX queues in MAC scheduler
- Fixed attachment for loop in `lena-lte-comparison` example
- Fixed wrong assignation of NetDeviceContainer vector for REM in `lena-lte-comparison`
example
- Fixed REM sector index bug in lena-lte-comparison.cc example
- Fix the buffer size calculation in `NrMacSchedulerLcg` to consider correctly the
RLC overhead
- Improved formatting of the RLC/PDCP traces

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr



Release NR-v2.0
--------------

Availability
------------
Available since April 21, 2022

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-9 or later
- clang-8 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- ArchLinux with g++-9 and 10 clang-8, 9, 10, and 11

Recommended ns-3 release: ns-3.36. (If ns-3.36 is not available yet, use ns-3
master branch.)

Important news
--------------
This module can be updated with the usual

```
$ git pull
```

command. We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------
- The NR module now supports DP-MIMO spatial multiplexing. The NR DP-MIMO model
currently supports MIMO with 2 streams for the downlink, however, in the future,
the MIMO model will be extended to support more streams and the operation in the
uplink.
The current, NR DP-MIMO model exploits dual-polarized antennas and their
orthogonality to send the two data streams, by exploiting polarization diversity.
The model does not rely on abstraction, as is it case with LTE MIMO,
and thus can more properly model the propagation differences between the two
streams and account for rank adaptation. So, one of the important new features
that enters with this release along the NR DP-MIMO is the rank adaptation.
The major modifications to support the DP-MIMO feature are inside the PHY and
MAC layers of the NR module.


Bugs fixed
----------


Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v1.3
---------------

Availability
------------
Available since April 7, 2022.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-9 or later
- clang-8 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- ArchLinux with g++-9 and 10 clang-8, 9, 10, and 11

Recommended ns-3 release: ns-3.36.

Important news
--------------
In past, the NR module releases were not bound to any particular ns-3 release
because we were trying to keep in sync with the latest advancements in ns-3-dev
(master). Since ns-3 has had many disruptive API changes since its release 3.35
towards the upcoming release 3.36, and it also changed the build system from waf to
cmake, many 5G-LENA users have reported issues related to the unsuccessful
compilation of NR v1.2 with the latest ns-3 master.

Hence, starting from this NR release we decided to bind each NR release to
the specific ns-3 release by recommending the ns-3 release with which has been
tested the specific NR release.

Additionally, due to many accumulated changes in the NR module, that were waiting
for the next ns-3 release, we have decided to split all these upcoming changes into
several incremental releases to facilitate the 5G-LENA users to upgrade
their code gradually.

As a first step towards that goal, we have created this NR release v1.3 to
provide to the 5G-LENA users a version of the NR module that is compatible with
the upcoming ns.3-36.

(This release has been tested with ns-3-dev master with commit 4a98f050, and is
expected to be fully compatible with ns-3.36).

This module can be updated with the usual

```
$ git pull
```

command.

We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------
- Switch to cmake build system
- Upgrade nr to spectrum changes that entered with the MR 686
- Upgrade nr to lte changes that entered with the MR 810

Bugs fixed
----------

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr



Release NR-v1.2
--------------

Availability
------------
Available since June 4, 2021.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note: not all features available on all platforms):
- g++-7 or later
- clang-8 or later
- (macOS only) Xcode 10.1 or later

This release has been tested on the following platforms:
- ArchLinux with g++-7, 8, 9, and 10 clang-8, 9, 10, and 11

Important news
--------------
This module can be updated with the usual

```
$ git pull
```

command. We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------
- File Transfer Protocol (FTP) model 1 traffic model is included.

Bugs fixed
----------
- The computation of the effective SINR for the error modeling under HARQ-IR uses
now an updated formula that accounts for pure IR with no repetition of coding bits.
- There were cases in which multiple UEs could be assigned the same SRS offset value,
because the generation of the possible SRS offset was including multiple 0 values.
Now, the generation of the available values for the SRS offsets has been updated
to not contain multiple 0 values.
- Realistic beamforming algorithm with trigger event configured as delay update
uses the actual channel at SRS reception moment for real BF update with delay.
- There have been reported cases where an assert in PHY was triggered due to the
fact that the Allocation Statistics were not in accordance with the real allocation.
This happened because the `SlotAllocInfo` structure, and in particular the
`m_numSymAlloc` field, was not updated accurately when a UE didn’t get a
DCI (i.e., when the TBS is less than 7 bytes). Scheduler, now correctly updates
the `m_numSymAlloc` field and the `usedSym` variable in `NrMacSchedulerNs3::DoScheduleDlData`
and `NrMacSchedulerNs3::DoScheduleUlData` when DCI is not created.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v1.1
--------------

Availability
------------
Available since March 2, 2021.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.8 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux with g++-9.2.1, clang-9.0.1, and Python 3.8.1
- Ubuntu 18.04 (64 bit) with g++-8.3.0 and Python 2.7.12/3.5.2

Important news
--------------
This module can be updated with the usual

```
$ git pull
```

command. We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------
- The scheduler can selectively leave particular RBG empty. This feature is
  called notching, and is used when multiple gNBs are collaborating to avoid
  interferences over a spectrum part.
- Added SRS allocation, transmission, and reception. Added an SRS message that
  takes 4 symbols (in the default configuration) within some periodicity (default
  at 80 slots). SRS are dynamically scheduled by the gNB (with an interface and
  an example specialized scheduler for it, `NrMacSchedulerSrsDefault`), and its
  allocation is signaled to the UE through a DCI. This is used by the UE to
  transmit SRS.
- `RealisticBeamformingAlgorithm` class is added. It implements a
  beamforming algorithm that determines the beamforming vectors of the transmitter
  and the receiver based on the SINR SRS.
- Uplink power control functionality implemented through the `NrUePowerControl`
  class, supporting UL power control for PUSCH, PUCCH, and SRS.
- IPV6 is now supported. That is, the end-to-end connections between the UEs
  and the remote hosts can be IPv4 or IPv6.

Bugs fixed
----------
- BeamManager called the function with the name "ChangeToOmniTx" of 3gpp
  antenna. This was causing that the CTRL was not being passed through 3gpp
  spectrum propagation model, but only through the propagation loss model.
- `GridScenarioHelper` fixes to correctly place nodes.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v1.0
--------------

Availability
------------
Available since Sept. 16, 2020.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.8 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux with g++-9.2.1, clang-9.0.1, and Python 3.8.1
- Ubuntu 18.04 (64 bit) with g++-8.3.0 and Python 2.7.12/3.5.2

Important news
--------------
The v1.0 can now be installed in the ns-3-dev repository, or any ns-3 version
starting from ns-3.31. This module can be updated with the usual

```
$ git pull
```

command. We hope you will have fun and good times in using our module!

New user-visible features (old first)
-------------------------

- Renamed all mmwave- classes, tests, examples, helpers, to nr-. Renamed all
  the classes by replacing the prefix `MmWave` with `Nr`.
- Renamed the `Enb` part in `Gnb` (e.g., `NrEnbPhy` -> `NrGnbPhy`)
- Processing delays N0, N1, N2 are introduced as attributes of NrEnbPhy,
  respectively for the DL DCI delay, DL HARQ feedback delay, and UL DCI delay.
  The values K0, K1, K2 (definition in the 3GPP standard) are then calculated
  and communicated to the UE in the DCI. The N2Delay parameter replaces the old
  UlSchedDelay parameter.
- Removed PhyMacCommon. Its attributes are now divided among different
  classes. Please check CHANGES.md for the list.
- Separated NrEesmErrorModel in four different classes: NrEesmIrT1, NrEesmIrT2,
  NrEesmCcT1, NrEesmCcT2. These classes encapsulate the properties (harq method,
  table) that were an attribute of NrEesmErrorModel.
- Added the LENA error model. To be used only in conjunction with a OFDMA
  scheduler, and without beams. This error model has the same performance and
  values as the error model used in the lte/ module. The reference file is
  `lena-error-model.h`.
- Added the attribute `RbOverhead` to the NrGnbPhy and NrUePhy to set the
  bandwidth overhead to keep in consideration when calculating the number of
  usable RB. By default, it is set to 0.04, while in the previous versions the
  effect was like a value set to 0.0 (0.0 means that there are no guard bands,
  and the entire bandwidth is usable).
- Starting with this release the simulator is using new ns-3-dev 3ggp
  channel, spectrum, propagation, channel condition and antenna models
  that are implemented in spectrum, propagation and antenna modules of
  ns-3-dev. To allow usage of this new channel and antenna models, we have
  introduced a new BeamManager class which is responsible configuration of
  beamforming vectors of antenna arrays. BeamManager class is also responsible
  of configuring quasi-omni beamforming vector for omni transmissions.
  Since real beamforming management methods are still not implemented
  in our module, there are available ideal beamforming methods: cell scan
  and direct path. User can configure ideal beamforming method by using
  attribute of IdealBeamformingHelper which is in charge of creating
  the corresponding beamforming algorithm and calling it with configured
  periodicity to generate beamforming vectors for pairs of gNBs and UEs.
  BeamManager class is then responsible to cache beamforming vectors for
  antenna. For example, at gNB BeamManager for each connected UE device
  there will be cached the beamforming vector that will be used for
  communication with that UE. In the same way, the BeamManager at UE
  serves the same purpose, with the difference that it will be normally just one
  element in the map and that is toward its own gNB.
- Starting with this release the default behaviour will be to calculate
  interference for all the links, and will not be any more possible to exclude
  UE->UE and GNB->GNB interference calculations.
- NrHelper is now refactored to take into account Multi-Cell Configurations
  (i.e., different configuration for different cells).
- Introduced the TDD pattern: every gNb can use a different pattern, specified
  by a string that identify a sequence of slot types (e.g.,
  "DL|UL|UL|DL|DL|DL|").
- Introduced FDD operational mode for a Bandwidth Part.
- A new Component Carrier/Bandwidth Part helper has been added in file
  `cc-bwp-helper.h`. With this class, it is easy to divide the spectrum in
  different regions.
- Added the support for NR MAC sub-headers and subPDU.
- Added SHORT_BSR as MAC CE element, that goes with MAC data, and is evaluated
  by the error model upon delivery.
- A new NrRadioEnvironmentMapHelper has been implemented for the generation of
  DL and UL REM maps.

Bugs fixed
----------

- Removed legacy and invalid mmwave-* examples, that were inherited from mmWave codebase.
- The code performing LBT at UE side always assumed a DL CTRL symbol inside a slot.
  With TDD, that may not happen, and the code has been updated to not
  always assume a DL CTRL.
- TDMA Scheduler was assuming that there are always UE to schedule. That may not be true
  if these UEs have scheduled an HARQ retransmission (and hence unable to
  receive a new data DCI). Code updated to remove this assumption.
- gNb was scheduling first DL and then UL, so when it was calculating the symbols to be
  assigned to DL, it was not taking into account the UL symbols. Code has been updated to
  schedule first UL and then DL.
- N1 (used to schedule DL HARQ Feedback) could not be set to zero, because the way the
  calculation was carried out was resulting in negative delay. Calculation has been updated
  to take into account the case of N1=0.
- The Harq timer is now set to 0 every time a retransmission is scheduled and
  transmitted (before, it was just incremented)
- The BwpManagerGnb now redirects BSR and SR to the source bandwidth part, as
  the UE has already done the selection based on the configured QCI.
- LBT at the gNb side is now scheduled (and done) every time there is a DL CTRL
  to transmit, as in FDD configuration, other BWP can inject messages after
  the start of the slot.
- NrLteErrorModel contained a bug that prevented the calculation of the right
  error value.
- Scheduler contained a bug that was forcing, in some situation, a double SR
  scheduling, stopped by a FATAL_ERROR.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v0.4
--------------

Availability
------------
Available since Thu Feb 13 2020.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux with g++-9.2.1, clang-9.0.1, and Python 3.8.1
- Ubuntu 18.04 (64 bit) with g++-8.3.0 and Python 2.7.12/3.5.2

Important news
--------------
- This release is aligned with CTTC's ns-3-dev commit id
217b410c lte-rlc: TM is now sending more than one packet per transmission opportunity
that is on top of the nsnam ns-3-dev master commit id
578c107eb internet: fix packet deduplication test .
To upgrade CTTC's ns-3-dev, please run the following (save any non-official
commit, as they will be deleted):

$ cd /path/to/cttc/ns-3-dev
$ git reset --hard HEAD~200
$ git pull

This module can be updated with the usual

$ git pull


New user-visible features (old first)
-------------------------------------

- (error model) new BLER-SINR tables for MCS table1 and table2
- (performance) Various performance improvements


Bugs fixed
----------
- (scheduler) Correctly schedule beams of users that got a HARQ retx space.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v0.3
--------------

Availability
------------
Available since Tue Aug 27 2019.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux 2018.10.08 with g++-8.2.1 and Python 3.7.1
- Ubuntu 16.04 (64 bit) with g++-5.4.0 and Python 2.7.12/3.5.2

New user-visible features (old first)
-------------------------

- (error model) Added the NrEesmErrorModel class. It models the NR PHY
abstraction according to LDPC coding, block segmentation, and including
MCS/CQI table 1 and 2. The user can select the HARQ method (HarqCc or HarqIr)
as well as the MCS/CQI table of NR to be used (McsTable1 or McsTable2), through
new attributes. In this release, the BLER-SINR tables are not completed yet,
and so it is recommended not to use this error model.

- (3gpp channel model) 3gppChannelModel can now be used by any other module,
i.e. it is not any more mmwave specific spectrum propagation model. This means
that any subclass of NetDevice can be attached to a channel using this
SpectrumPropagationModel. An additional requirement is that the technology uses
AntennaModel that is implementing AntennaArrayBasicModel interface. This
feature is a basic prerequisite for simulating co-existence of different
technologies that are using the 3gpp channel model implementation.

- (3gpp channel model) The beamforming phase has been extracted from the model,
and it is now a duty of the NetDevice. The gNB phy has now a new attribute to
configure the periodicity of the beamforming. Please note that it is still ideal,
i.e., it does not require any simulated time to be performed.

- (3gpp channel model) gNB-gNB and UE-UE pathloss and channel computation can be
allowed (through a new attribute) for Indoor Hotspot scenarios.

- (spectrum phy) gNB-gNB and UE-UE interferences can be enabled (through a new
attribute).

- (RRC) Now all carriers are registered to RRC, to transmit system information
through all the bandwidth parts.

- (SCHED) The scheduler now is informed of RACH preamble messages, and reserve
some space in the DL CTRL symbol to send the RAR messages.

- Added traces that indicate the transmission or the reception of CTRL
messages. For instance, take a look to *EnbMacRxedCtrlMsgsTrace* or
*EnbMacTxedCtrlMsgsTrace* in the Gnb MAC file.

Bugs fixed
----------
- (scheduler) Fixed the use of a static MCS value in the schedulers
- (spectrum-phy) While looping the packets in the packet burst,
  to send the feedback, extract the RNTI for each loop. Before, it was
  asserting when the RNTI changed. The code does not depend on the
  RNTI previous values, so it should be safe.
- (phy) At the beginning, fill some slot (number configured by UL sched delay
  param) with UL CTRL symbol.

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr


Release NR-v0.2
---------------

Availability
------------
Available since Fri Feb 1 2019.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux 2018.10.08 with g++-8.2.1 and Python 3.7.1
- Ubuntu 16.04 (64 bit) with g++-5.4.0 and Python 2.7.12/3.5.2

New user-visible features (old first)
-------------------------

- (mmwave) Removed any reference to Rlc Low Latency
- (mmwave) The code that was previously under the directory src/mmwave has been
  moved to src/nr.
- (nr) Aligned ComponentCarrierGnb, MmWaveEnbMac, MmWaveUePhy to ns-3-dev
- (nr) Aligned the BwpManager and the various helper/example to the bearer
  definitions in Rel. 15
- (nr) Removed unsupported MmWaveBeamforming and MmWaveChannelMatrix classes.
- (nr) Added a 3GPP-compliant antenna class.
- (nr) Added a 3GPP-compliant UL scheduling request feature.

Bugs fixed
----------

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.cttc.es/ns3-new-radio/nr
