<!--
Copyright (c) 2022 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)

SPDX-License-Identifier: GPL-2.0-only
-->

5G-LENA Changes       {#changes}
===============

NR module: API and model change history
---------------------------------------

<!-- This ChangeLog is updated in the reverse order with the most recent changes coming first.  Date format:  DD-MM-YYYY -->

ns-3 is an evolving system and there will be API or behavioral changes
from time to time. Users who try to use scripts or models across
versions of ns-3 may encounter problems at compile time, run time, or
may see the simulation output change.

We have adopted the development policy that we are going to try to ease
the impact of these changes on users by documenting these changes in a
single place (this file), and not by providing a temporary or permanent
backward-compatibility software layer.

A related file is the RELEASE_NOTES.md file in the top level directory.
This file complements RELEASE_NOTES.md by focusing on API and behavioral
changes that users upgrading from one release to the next may encounter.
RELEASE_NOTES attempts to comprehensively list all of the changes
that were made. There is generally some overlap in the information
contained in RELEASE_NOTES.md and this file.

The goal is that users who encounter a problem when trying to use older
code with newer code should be able to consult this file to find
guidance as to how to fix the problem.  For instance, if a method name
or signature has changed, it should be stated what the new replacement
name is.

Note that users who upgrade the simulator across versions, or who work
directly out of the development tree, may find that simulation output
changes even when the compilation doesn't break, such as when a
simulator default value is changed.  Therefore, it is good practice for
_anyone_ using code across multiple ns-3 releases to consult this file,
as well as the RELEASE_NOTES.md, to understand what has changed over time.

This file is a best-effort approach to solving this issue; we will do
our best but can guarantee that there will be things that fall through
the cracks, unfortunately.  If you, as a user, can suggest improvements
to this file based on your experience, please contribute a patch or drop
us a note on ns-developers mailing list.

## Changes from NR-v5.1 to v5.2

### New API:
- Add the ``NrMacSchedulerNs3::MaxHarqReTx`` attribute to configure the maximum number of HARQ retransmissions per transport block.
- Add the ``RxPacketTraceParams::m_harqProcessId`` field, the HARQ process ID of the received transport block. It is available to ``RxPacketTraceUe`` and ``RxPacketTraceGnb`` trace sinks; it is not written to ``RxPacketTrace.txt``.
- Add the ``TrafficGenerator::EnableSeqTsSizeHeader`` attribute and ``TxWithSeqTsSize`` trace source, which add a ``SeqTsSizeHeader`` (sequence number, timestamp and size) to the generated packets, as ``OnOffApplication`` and ``PacketSink`` do, and the ``TrafficGeneratorNgmnVoip::SidEnabled`` attribute, which can disable the SID packets sent in the inactive state (enabled by default).
- Add unstructured PDU session support (3GPP TS 23.501 Section 5.6.1), carrying a single non-IP network-layer protocol whose payload the network does not parse. This adds the ``NrPduSessionType`` enum, the ``NrQosRule::Unstructured()`` factory, ``NrHelper::ActivateUnstructuredQosFlow()``, and the ``NrEpcHelper::ActivateUnstructuredQosFlow()`` and ``NrEpcHelper::GetUnstructuredSessionDevice()`` methods. Such a session needs no address on the UE and is reached from the external network through a device of its own on the PGW. With local breakout roaming, the PGW of the visited network gives the session an egress device of its own when the UE attaches there; with home-routed roaming, the session keeps the egress device of the home network.
- Add ``NrSidelinkFilter``, a ``SpectrumTransmitFilter`` that discards gNB-to-gNB and UE-to-UE NR signals, skipping their propagation loss and channel matrix computation. Signals involving non-NR devices are not filtered. It drops sidelink transmissions, so it must not be installed in sidelink scenarios.
- Add ``NrCoexistenceFilter``, a ``SpectrumTransmitFilter`` that discards the signals exchanged between NR and non-NR devices, skipping their propagation loss and channel matrix computation. It removes the interference between NR and the other technologies, so it must not be installed in coexistence scenarios.
- Add the ``NrChannelHelper::AddFilters`` attribute, a bitmask of ``NrChannelHelper::Filter`` values (``CSIRS``, ``SIDELINK``, ``COEXISTENCE``) selecting the transmit filters installed on the spectrum channels it creates. It defaults to ``CSIRS`` only, which keeps the previous behavior, and must be set before the channels are created.
- Add roaming between the core networks of a simulation, with local breakout: ``NrNoBackhaulEpcHelper::AddRoamingPartner()`` makes two core networks roaming partners, ``NrEpcMmeApplication::UpdateLocation()`` and ``NrEpcMmeApplication::CancelLocation()`` exchange the subscription of a UE and release its session in the network that served it before, and the ``Attach`` and ``CancelLocation`` trace sources of ``NrEpcMmeApplication`` report the attachments and the deletions of the sessions. Add ``NrNoBackhaulEpcHelper::GetMmeApplication()``.
- Add home-routed roaming between the core networks of a simulation: ``NrNoBackhaulEpcHelper::AddRoamingPartner()`` takes a ``homeRouted`` parameter, which connects the SGW of each network to the PGW of the other one with an S8 link (``NrNoBackhaulEpcHelper::S8LinkDataRate``, ``S8LinkDelay`` and ``S8LinkMtu`` attributes), and ``NrGtpcCreateSessionRequestMessage`` carries the PGW S5/S8 address for the control plane (``GetPgwAddress()``, ``SetPgwAddress()``) selected by the MME.
- Add ``NrEpcMmeApplication::ExternalUpdateLocation()`` and ``NrEpcMmeApplication::SetExternalCancelLocationCallback()``, with which a network outside of the simulation (e.g., an external 5G core network roaming with the home network through a SEPP) serves a subscriber of a core network of the simulation.
- Add the S1-AP UE Context Release procedure: ``NrEpcS1apSapGnb::UeContextReleaseCommand()``, ``NrEpcS1apSapMme::UeContextReleaseComplete()``, and ``NrEpcGnbS1SapUser::UeContextReleaseCommand()``, with which the gNB application asks the RRC to release the UE. Classes implementing these SAPs must implement the new methods.
- Add the ``NrNoBackhaulEpcHelper::UeIpv4Network`` and ``NrNoBackhaulEpcHelper::UeIpv4Mask`` attributes, the IPv4 network of the UEs of a core network (7.0.0.0/8 by default).
- Add the GTP-C Delete Session procedure (``NrGtpcDeleteSessionRequestMessage``, ``NrGtpcDeleteSessionResponseMessage``), with which the MME deletes the session of a UE in the SGW and the PGW, and ``NrEpcPgwApplication::HasUe()``, ``NrEpcPgwApplication::GetUeAddress()`` and ``NrEpcPgwApplication::GetUeAddress6()``.
- Add the ``NrUePowerControl::P0Srs`` attribute (-80 dBm by default, from -202 to 24 dBm as the ``p0`` field of ``SRS-Config`` in TS 38.331), the ``P0`` of the SRS power control formula of TS 38.213 clause 7.3.1, used when ``TSpec`` is ``TS38.213``.
- Add ``NrGnbPhy::DeregisterUe()``, removing a UE from the UEs of the gNB, including its CSI-RS.
- Add the ``NrEesmIr::CreditRateBelowModulationFloor`` attribute, enabled by default (see Changed Behavior).
- Add the ``Numerology`` attribute to ``NrMacSchedulerOfdmaAi`` and ``NrMacSchedulerTdmaAi``.

### Changes to Existing API
- Following the ns-3 spectrum change that passes beamforming vectors explicitly (ns-3 issue #1369), ``NYUSpectrumPropagationLossModel`` and ``DistanceBasedThreeGppSpectrumPropagationLossModel`` overrides of ``DoCalcRxPowerSpectralDensity()`` and ``GetLongTerm()`` take the two beamforming vectors as parameters, and callers of ``CalcRxPowerSpectralDensity()`` and ``DoCalcRxPowerSpectralDensity()`` now pass them (typically ``array->GetBeamformingVector()``) instead of relying on the arrays' current state.
- ``NrAsSapUser::RecvData()`` now takes the QoS Flow ID of the flow the packet arrived on as a second parameter, so that the UE can recover the network-layer protocol of an unstructured PDU session on the downlink. Callers of the previous single-argument form must pass the QFI, or zero for an IP session, whose protocol is read from the packet itself.
- The ``NrEpcGnbApplication`` constructor now takes an additional ``Ptr<Socket>`` for unstructured PDU sessions, inserted between the ``nrSocket6`` and ``cellId`` parameters.

- Add ``NrRanEpcHelper``, the new base class of ``NrNoBackhaulEpcHelper``, which installs the gNB side of the S1 and X2 interfaces without creating any core network node. Subclasses connect each gNB to their core network in ``NrRanEpcHelper::ConnectGnbToCore()``. ``NrNoBackhaulEpcHelper::DoAddX2Interface()`` is now inherited from ``NrRanEpcHelper``, and the ``X2Link*`` attributes are now defined by ``NrRanEpcHelper``.
- Add ``NrEpcS1apSapGnb::ErabToBeSetupItem::gnbTeid``, the downlink TEID of a QoS flow at the gNB. When it is not set (zero), the downlink uses the same TEID as the uplink (``sgwTeid``), as with the built-in core network.
- Add support for GTP-U extension headers in ``NrGtpuHeader`` (``AddExtensionHeader()``, ``GetExtensionHeaders()``), and for the PDU Session Container extension header (``SetPduSessionContainer()``, ``HasPduSessionContainer()``, ``GetPduType()``, ``GetQfi()``), which carries the QFI on the N3 interface.
- ``NrGtpuHeader::SetExtensionHeaderFlag()`` and ``NrGtpuHeader::SetNextExtensionType()`` are removed: the E flag and the next extension header type are now derived from the extension headers added with ``AddExtensionHeader()``.
- The following ``NrMacSchedulerNs3`` methods are now ``final``, as they depend on its private state and cannot be meaningfully overridden: ``DoCschedCellConfigReq()``, ``DoCschedUeConfigReq()``, ``DoSchedDlRlcBufferReq()``, ``DoSchedUlMacCtrlInfoReq()``, ``DoSchedDlCqiInfoReq()``, ``DoSchedUlCqiInfoReq()``, ``DoCschedUeReleaseReq()``, ``DoCschedLcConfigReq()``, ``DoCschedLcReleaseReq()``, ``DoSchedDlTriggerReq()``, ``DoSchedUlTriggerReq()``, ``DoSchedUlSrInfoReq()``, ``DoSchedSetMcs()``, ``DoSchedDlRachInfoReq()``, ``GetDlCtrlSyms()``, ``GetUlCtrlSyms()`` and ``IsMaxSrsReached()``. Schedulers that derive from ``NrMacSchedulerNs3`` and override any of them no longer compile.
- The ``NumMainInterfererGnb`` and ``PrimaryCarrierIndex`` attributes of ``NrInitialAssociation``, and ``NrHelper::InitialAssocParams::primaryCarrierIndex``, are ``uint8_t`` instead of ``double``.
- ``NrMacSchedulerNs3::ScheduleUlHarq()`` and ``NrMacSchedulerHarqRr::ScheduleUlHarq()`` take the map of the active UL HARQ processes as their third parameter.
- The TypeIds of the symbol-per-beam allocators are renamed from ``ns3::NrMacSchedulerONrMacSchedulerOfdmaSymbolPerBeam`` and ``ns3::NrMacSchedulerONrMacSchedulerOfdmaSymbolPerBeamLB`` to ``ns3::NrMacSchedulerOfdmaSymbolPerBeam`` and ``ns3::NrMacSchedulerOfdmaSymbolPerBeamLB``.
- The AI-based schedulers observe UEs instead of logical channels: ``NrMacSchedulerUeInfoAi::LcObservation`` is replaced by ``NrMacSchedulerUeInfoAi::UeObservation``, the ``NrMacSchedulerUeInfoAi`` constructor takes the numerology as its second parameter, and the ``NrMacSchedulerAiNs3GymEnv`` constructor takes the number of UEs instead of the number of flows.

### Changed Behavior

- When another core network serves a UE, the MME of the network that served it before sends a UE Context Release Command to the gNB of the UE, which releases its RRC connection if it still has it.
- The PGW sends the downlink packets and the GTP-C messages of a session to the SGW that created it, rather than to the SGW of its network, and the SGW sends those of a session to the PGW selected by the MME. The Create Session Request has one more F-TEID IE (the PGW S5/S8 address, which is empty for the PGW of the network of the SGW).
- A simulation can have several core networks: ``NrHelper`` connects the gNBs to the core network of the EPC helper set when they are installed, and activates the QoS flows of each UE in the core network of the EPC helper set when the UE was installed (its home network), instead of the EPC helper set last. The core networks after the first one use their own addresses for their internal links (``10.<index>.0.0`` for S1-U, and so on) and remote host networks (``1.<index>.0.0/16``); the first one keeps the previous addresses.
- The GTP-U packets between the gNBs and the SGW now carry the PDU Session Container extension header, with the QFI, in both directions (4 more octets per packet), and the gNB maps the downlink packets to their QoS flow with it.
- ``NrGtpuHeader`` now omits the optional fields when none of the E, S and PN flags is set, and no longer sets the spare bit of the first octet (TS 29.281).
- A HARQ process now ends by transmission attempts instead of ``rv == 3``. The PHY clears the soft buffer when a new transport block arrives (NDI == 1) instead of when ``rv == 3``. RV can exceed 3 when ``MaxHarqReTx`` exceeds 3.
- ``TrafficGeneratorNgmnVoip`` now sends one voice packet per encoder frame (``EncoderFrameLength``, 20 ms by default) during talk spurts, as the NGMN VoIP model specifies, instead of one every ``VoicePayload`` * 8 / 12200 seconds (26.2 ms with the default 40-byte payload). With the default attributes the average offered rate rises from about 6.5 kbit/s to about 8.4 kbit/s (16 kbit/s in talk spurts, 0.75 kbit/s of SID packets in silence, voice activity factor 0.5); the ``traffic-generator`` test expectation is updated accordingly.
- ``TrafficGenerator`` now accepts a next packet size of zero from a generator, scheduling the next packet without sending one, and logs a warning instead of asserting when the socket fails to send.
- ``NrHelper::AttachToGnb()`` now activates the default IP QoS flow only for a UE that has an IPv4 or IPv6 stack. A UE reaching the network over an unstructured PDU session alone has no IP stack and gets no default flow. Previously, the default flow was always activated whenever the EPC was used.
- The gNB EPC application now removes the ``PacketSocketTag`` and the ``DeviceNameTag`` from a packet received on the NR packet socket before relaying it to the S1-U interface. A packet socket stamps every packet it delivers with the parameters of that delivery, and those tags used to travel on with the relayed packet: they were copied onto each RLC segment by ``Packet::CreateFragment()``, carried into the reassembled SDU by ``NrRlc::ReassembleSdus()``, which delivers the held first segment and keeps its packet tags, and ended up aborting the simulation when the packet reached a packet socket that stamped it a second time. Covered by the new ``nr-epc-gnb-relayed-tag`` test suite, which walks that chain step by step.
- The gNB RRC now keeps the C-RNTI of a UE context destroyed while a handover was still in progress out of the allocation pool until that handover concludes. ``NrGnbRrc::AddUe()`` assigns the first C-RNTI absent from the UE map, so one freed by an interrupted handover (a target that cannot allocate a preamble, or a UE context removal requested during the procedure) was immediately available again while the X2 messages naming it were still in flight; a UE CONTEXT RELEASE arriving afterwards addressed whichever UE had inherited the value and tore down its connection. The reservation is lifted when the UE CONTEXT RELEASE is received, and in any case after the new ``NrGnbRrc::RntiReservationDuration`` attribute (500 ms by default) so that a handover which never concludes cannot hold a C-RNTI forever.
- The UE MAC now discards a buffer status report whose logical channel is not configured in the MAC, instead of recording it. The RLC entity of a logical channel that the MAC has dropped (on handover, or through the ``DoReset()`` that precedes a re-establishment) keeps the service access point it was given and can still report a buffer status; such a report can never be served, because no ``NrMacSapUser`` is left to receive the transmission opportunity. Recording it left an LCID in the buffer status map with no counterpart in the logical channel map, which ``NrUeMac::SendNewStatusData()``, ``SendRetxData()`` and ``SendTxData()`` then looked up unconditionally, terminating the simulation with an ``std::out_of_range`` exception when the next uplink grant selected that channel.
- ``NrHelper::AssignStreams()`` now assigns streams to the NYU and FTR (``TwoRay``) channel objects, which previously drew from automatically assigned streams, and to the channel condition model shared with the fading model when no propagation loss model is installed. ``NYUPropagationLossModel`` and ``NYUChannelModel`` now also assign the streams of their shadowing (normal) and gamma random variables, and ``NYUChannelModel`` draws its Poisson and binomial values from its uniform random variable instead of a ``std::mt19937`` seeded by ``std::random_device``, which made NYU simulations non-reproducible even with a fixed seed and run number. The output of simulations using these channel models changes accordingly.
- With ``NrUePhy::EnableUplinkPowerControl`` set, the UE now transmits the SRS with the power computed by ``NrUePowerControl::GetSrsTxPower()``. It used to reuse the transmit power of the last PUSCH or PUCCH (or the ``NrUePhy::TxPower`` attribute before any uplink transmission), spread over the whole band, so SRS-based SINR and beamforming estimates changed with the preceding transmission. Without uplink power control the SRS still uses ``NrUePhy::TxPower``.
- With a channel that supports MIMO, the PUSCH CQI used for uplink link adaptation is now computed from the post-MMSE MIMO SINR that the gNB uses to decode the transport block, averaged over the reception time (and over the layers in dB), instead of from the SISO SINR. Gains from receive combining and interference rejection at multi-port gNBs are now reflected in the uplink MCS, and the ``NrGnbPhy::UlSinrTrace`` trace reports that SINR. Without such a channel the SISO SINR is still used.
- ``NrMacSchedulerHarqRr::ScheduleUlHarq()`` places a UL HARQ retransmission on the symbols of a retransmission already scheduled in the slot when both have the same number of symbols, the UEs share a beam and their RBGs do not overlap. Each retransmission keeps the RBGs, MCS and TBS of its first transmission. Previously every retransmission took its own symbols, so frequency-multiplexed retransmissions (as produced by the OFDMA schedulers) were serialized in time, or deferred when the slot ran out of symbols.
- With the OFDMA schedulers, new UL data is now scheduled on the RBGs left free by the UL HARQ retransmissions of the slot, over their symbols, for UEs in the beam of those retransmissions. As a UE gets a single grant per slot, a UE is only placed there when its data fits, or when the UEs left behind have enough data to fill the symbols after the retransmissions. Previously the rest of the band stayed empty while a retransmission was sent, and new data only got the symbols left after all retransmissions.
- With the OFDMA schedulers, new DL data is now scheduled on the RBGs left free by the DL HARQ retransmissions of the slot, over their symbols, for UEs in the beam of those retransmissions. A UE with a retransmission in the slot still gets no new data, since it cannot receive two PDSCHs that overlap in time. As with the UL, a UE is only placed there when its data fits, or when the UEs left behind have enough data to fill the symbols after the retransmissions. Previously the rest of the band stayed empty while a retransmission was sent, and new data only got the symbols left after all retransmissions.
- ``NrMacSchedulerHarqRr::ScheduleUlHarq()`` reshapes a UL HARQ retransmission that cannot share the symbols of another one, and whose symbols do not fit in the ones left in the slot, into the fewest of them that hold its RBGs times symbols over the UL band. It keeps its MCS and TBS. Previously such a retransmission was deferred to a later slot.
- ``NrMacSchedulerHarqRr::ScheduleDlHarq()`` reshapes a DL HARQ retransmission whose RBGs are taken by another retransmission of its beam, or whose symbols do not fit in the ones left in the slot, into the fewest of the symbols left after every beam that hold its RBGs times symbols over the DL band. It keeps its MCS and TBS. Previously such a retransmission was deferred to a later slot unless ``NrMacSchedulerHarqRr::ConsolidateHarqRetx`` was enabled.
- With the OFDMA schedulers, the UEs scheduled on the RBGs left free by HARQ retransmissions are now spread over all the retransmission spans of their beam. Each UE, smallest buffer first, goes to the span in which it gets the most resources, assuming the free RBGs of a span are split evenly among its UEs, as long as that share carries its data and the data of the UEs already there. Previously the spans were filled one at a time, and the first one took every UE whose data fitted in its free resources as a whole, so the following spans could stay empty, and a UE could get less than its data once the RBGs were split.
- ``NrHelper`` no longer connects the gNB RRC to ``NrEpcGnbApplication`` and ``NrEpcX2``; this is now done by ``NrEpcHelper::AddGnb()`` when it receives an ``NrGnbNetDevice``. Custom ``NrEpcHelper`` subclasses must connect the RRC to an entity implementing ``NrEpcGnbS1SapProvider`` (and to an X2 entity, if X2 is supported). Subclasses of ``NrNoBackhaulEpcHelper`` and ``NrPointToPointEpcHelper`` are not affected.
- ``NrHelper`` now registers a UE at the gNB of every cell the UE connects to (the ``ConnectionEstablished`` and ``HandoverEndOk`` traces of ``NrUeRrc``), and deregisters it from its previous gNB, so that a UE that changes cell after a handover or a radio link failure gets CSI-RS and a beam from its new gNB. Beamforming tasks only run for the gNB that serves the UE. Previously only the gNB of ``NrHelper::AttachToGnb()`` registered the UE.
- HARQ-IR (``NrEesmIrT1`` and ``NrEesmIrT2``) now credits the effective code rate below the lowest code rate of the modulation order of the transport block as a repetition gain of the effective SINR, ``SINR_eff * ECR_min / ECR_eff``, through the new ``NrEesmIr::CreditRateBelowModulationFloor`` attribute (enabled by default; set it to false for the previous behavior). Previously such retransmissions gained nothing, so a 64-QAM block that failed twice, or any block at the lowest MCS of its modulation order, practically never recovered and HARQ-IR was worse than HARQ-CC. Results with HARQ-IR and retransmissions change.
- The spectrum channel matrix built by ``NrSpectrumPhy`` for signals that arrive without one (e.g., with the ``TwoRay`` channel model) is no longer normalized by the transmitter PSD. As with the matrices produced by ``ThreeGppSpectrumPropagationLossModel``, it now carries the received power, so the MIMO SINR used to decode transport blocks is computed against the noise and interference on the same scale, instead of being overestimated by the inverse of the TX PSD (which made those transport blocks practically error-free).
- When a transport block is decoded with the MIMO SINR, the SINR reported in ``RxPacketTraceParams`` (``RxPacketTrace.txt``) is now the per-layer MIMO SINR passed to the error model, averaged over the allocated RBs and layers, instead of the SISO SINR derived from the received PSD. The SISO value aggregates the power of all layers and ignores inter-layer interference, so it did not match the reported TBLER, e.g., overestimating the SINR of rank-2 transmissions.
- ``NrRlcUm`` now reports its buffer status after every transmission opportunity that leaves data in its queue. The reported size counts one header per SDU while each segment takes one, so the UE MAC could consider the queue empty with data left in the RLC, pad the rest of the grant and report an empty buffer, delaying the rest of the SDU by ``NrRlcUm::BufferStatusReportTimer``. The output of simulations using RLC UM changes accordingly.
- The scheduler now keeps the DL HARQ retransmissions of a slot without symbols for DL, because UL took them all, for a later slot. They used to be dropped, leaving the HARQ process waiting for its timer and the RLC waiting for a PDU that would never arrive. A HARQ process waiting for its retransmission also no longer expires by the timer, which only covers feedback that never arrives.
- The initial association sets the RSRP and the best beam of each gNB from the strongest beam of that gNB, no longer multiplies the received power by the gNB transmit power in dBm, which the channel already includes, and activates the UE panel that best receives the associated gNB, instead of the panel that best receives the strongest beam of all the scanned gNBs before the path loss. The RSRP of each gNB follows its transmit power and the associated gNB may change.
- The UE now releases the UL CTRL channel access of a slot in which it sends nothing, so it sends its UL CTRL, and the DL HARQ feedback in it, in an SRS slot that follows a slot with UL CTRL (#292).
- The UE MAC now reserves the MAC subheader and the SHORT_BSR of an RLC STATUS PDU before admitting it, sizes the UL MAC subheaders and the SHORT_BSR exactly, and fills the bytes left in a TB with a padding BSR.
- The gNB PHY now sends its control messages in the order they were produced (queued messages such as MIB and SIB1 first, then DL and UL DCIs), instead of an order that depended on heap addresses (#290). Simulation outputs change.
- The UE now only updates its number of RBs from a MIB when the numerology changes, and the numerology of the BWP a connected UE is served on changes only with its configuration, so a MIB no longer aborts an ongoing reception.
- The UE now ignores an RRC reconfiguration of a connection it has already left, e.g., a handover command sent after it declared a radio link failure.
- The PMI searches fall back to a full update when a subband update has no precoding to refine. The Maleki search computes different subband PMIs and beam mapping.
- The gNB no longer transmits CSI-RS towards UEs without an RNTI, and a UE without an RNTI ignores CSI-RS.
- ``NrMacSchedulerHarqRr::ScheduleUlHarq()`` no longer places two UL retransmissions of a UE on the same symbols.
- ``NrMacSchedulerNs3::SortUlHarq()`` now calls ``NrMacSchedulerHarqRr::SortUlHarq()``, which sorts as ``SortDlHarq()`` does, so a subclass can sort the UL HARQ retransmissions on its own.
- The HARQ beam round-robin of ``NrMacSchedulerHarqRr`` now moves the beam served first to the back of its queue in every slot, with separate DL and UL queues, and serves the UL HARQ retransmissions in beam round-robin order too. The same beam was served first in every slot.
- ``NrFhControl`` now subtracts the per-UE overhead from the capacity in ``GetMaxRegAssignable()`` and ``GetMaxMcsAssignable()``, releases a UE with a HARQ retransmission once it is scheduled, uses the antenna ports of the BWP of each allocation, and returns 0 active UEs for a BWP with only HARQ UEs. ``OptimizeMcs`` and ``OptimizeRBs`` give much higher MCSs and RBs than before.

## Changes from NR-v5.0 to v5.1

### New API:
- Add ``NrUeMac::GetRachConfig()`` to inspect the RACH configuration currently applied to a UE MAC.
- Add the ``NrUeMac::SrProhibitTimer`` attribute (``sr-ProhibitTimer`` of TS 38.331, 10 ms by default), the minimum time between two transmissions of the scheduling request. While it runs the pending request is held rather than dropped, and is sent on the first occasion after it expires; set it to zero to transmit on every occasion.
- Add the ``NrUeMac::SrTransMax`` attribute (``sr-TransMax`` of TS 38.331, 64 by default), the maximum number of transmissions of a pending scheduling request before the request is cancelled and a random access procedure is started to recover uplink resources.
- Add the ``NrRlcUm::BufferStatusReportTimer`` and ``NrRlcTm::BufferStatusReportTimer`` attributes (10 ms by default), the period at which the buffer status is reported again while the transmission buffer is not empty. ``NrRlcAm`` already had one.
- Add the ``ulNumerology`` and ``ulCarrierFreq`` fields to ``NrRrcSap::ServingCellConfigCommon`` (broadcast in SIB1 and carried in the handover command), advertising the cell's UL carrier (the equivalent of ``frequencyInfoUL`` in TS 38.331). For TDD carriers they point at the carrier itself; for FDD cells they point at the dedicated UL-only carrier.
- Add ``BwpManagerGnb::HasOutputLink()`` to check whether an output mapping is installed for a BWP.
- Add ``NrUeRrc::SetUpdateBwpOutputLinkFn()`` (wired automatically by ``NrHelper``), through which the UE RRC keeps the ``BwpManagerUe`` output links aligned with the primary DL/UL BWP pair whenever it changes (initial camping, handover).
- Add mobility state estimation attributes on ``NrUeRrc`` (``MseEnable``, ``MseCountWindow``, ``MseHystNormal``, ``MseThreshMedium``, ``MseThreshHigh``, ``MseSfMedium``, ``MseSfHigh``, ``MseFixedScale``), scaling the A3 time-to-trigger with the UE mobility state per TR 36.839. Disabled by default.
- Add ``NrUeRrc::Tr36839HandoverFailure`` and ``Tr36839HoFailureMinT310Elapsed`` attributes modelling the TR 36.839 handover failure when the handover command arrives out-of-sync (T310 already running). Disabled by default.
- Add the ``NrUeRrc::RadioLinkFailureCause`` and ``NrGnbRrc::HandoverTotalTime`` trace sources, reporting the cause of each radio link failure and the total handover time from A3 trigger to completion.
- Add target-admission floors ``MinTargetRsrpDbm`` and ``MinTargetRsrqDb`` on ``NrA3RsrpHandoverAlgorithm`` and the reversal guard ``HandoverMinTimeOfStay`` on ``NrGnbRrc``, all disabled by default.
- Add ``NrUeRrc::MibWaitReselectTimeout`` and ``MibWaitReselectMaxAttempts`` attributes, making a force-camped UE that never decodes the MIB reselect the strongest untried measured cell. Disabled by default (zero timeout).
- Add ``RandomDirectionDisc2dMobilityModel``, a random-direction mobility model bounded by a disc, and ``HexagonalGridScenarioHelper::SetUeDiscRadius()`` to place and move UEs inside a disc centred on the central site (a zero radius keeps the legacy per-sector placement).
- Add the ``NrHelper::AttachWindow`` attribute, deferring the k-th ``AttachToGnb()`` call by ``AttachWindow`` times the base-2 radical inverse of k to stagger the initial attachment of many UEs. Disabled by default.
- Add ``HexagonalGridScenarioHelper::SetRingOneOnlyPicos()``, restricting picocell deployment to the six ring-1 sites (TR 36.839 no-wraparound dense-A layout). Disabled by default.
- Accept ``CQI_CSI_RS`` alone as a ``NrHelper`` ``CsiFeedbackFlag`` value; it provides no interference measurement but keeps the channel matrix updated without PDSCH traffic.
- Add a ``TxDrop`` trace source to the RLC TM mode, fired when a PDU is dropped on transmit-buffer overflow.
- Stamp every RLC PDU with a monotonically assigned transmitting-entity identity, carried in the ``NrRlcTag``; ``NrRlc::AcceptPduFromPeerEntity()`` discards PDUs of an older epoch and re-establishes the receiving side (per TS 38.322 5.1.2.3) when a newer one appears. Untagged PDUs bypass the check.

### Changed Behavior
- The UE MAC now retransmits the scheduling request whenever new buffer status arrives while it is backlogged, as the UE keeps requesting resources while a BSR is pending and no UL-SCH resources are available (TS 38.321 clauses 5.4.4 and 5.4.5); previously the scheduling request was re-armed only when the last received UL DCI had HARQ process 0 or rv 3, so when the bootstrap BSR transport blocks were lost the uplink stalled permanently. The scheduler no longer records a scheduling request as 12 bytes of buffer in every uplink logical channel group of the UE; a request carries neither a size nor a group, so it is now tracked as a pending bootstrap grant in ``NrMacSchedulerUeInfo::m_srBytesPending``, which makes the UE eligible for uplink scheduling without making any of its logical channels look active, and is cleared as soon as a grant is issued or a BSR arrives. Previously a backlogged UE repeating the request downgraded the buffer reported by an earlier BSR and kept its signalling group permanently active, which flattened the uplink priorities of ``NrMacSchedulerOfdmaQos`` and ``NrMacSchedulerTdmaQos``. The rate at which the request is repeated is now bounded by the new ``NrUeMac::SrProhibitTimer`` attribute (TS 38.321 clause 5.4.4): while it runs the request stays pending instead of being transmitted on every slot, and it is stopped when a grant that carried the buffer status drains the buffer, so a request triggered by data arriving later is not held back. A request that goes unanswered for ``NrUeMac::SrTransMax`` transmissions is now cancelled, and the UE starts a random access procedure to recover uplink resources instead of asking forever; ``NrUeRrc`` accepts the outcome of a random access started while already connected instead of aborting on it. Covered by the new ``nr-ul-sr-retransmission`` test suite.
- ``NrRlcUm`` and ``NrRlcTm`` no longer stop their buffer status report timer when a new SDU arrives from PDCP. New data is what triggers a buffer status report (TS 38.321 clause 5.4.5), never a reason to stop reporting, and the timer is only armed again by a transmission opportunity: once the uplink stalled, the last cancellation was final and the reported buffer status went stale. ``NrRlcAm`` already behaved this way.
- The dedicated RRC configuration delivered to the UE now advertises the bandwidth-part configurations and the QoS flow mapping; previously the UE learned them only through SIB broadcasts or helper-side setup.
- FDD configuration is now automatic: the gNB advertises its UL carrier in SIB1, the UE derives its primary UL BWP from it at cell selection (and ``NrHelper::AttachToGnb()`` derives it from the gNB configuration for the ideal path), the UE BWP routing follows the primary DL/UL pair, and the gNB routes the outgoing messages of UL-only carriers through its first DL-capable carrier unless an explicit mapping was installed. Manual ``NrUeNetDevice::PrimaryUlIndex`` and ``SetOutputLink()`` configuration is no longer needed for cells with a single DL/UL carrier pair; mixed TDD/FDD carrier-aggregation setups whose pairing does not involve the primary carrier still configure it manually (see ``cttc-lte-ca-demo``).
- ``BwpManagerGnb::SetOutputLink()`` and ``BwpManagerUe::SetOutputLink()`` now overwrite an existing mapping for the same source BWP; previously a second call for the same source was silently ignored.
- MIMO SINR chunks are now matched to a transport block by cell as well as by RNTI. An RNTI identifies a UE only within its own cell and the values are reused across cells, so after a handover a chunk measured against the previous cell could be selected for a transport block expected from the new one; where the two cells had granted a different rank the resulting SINR matrix disagreed with the expected rank and decoding aborted. ``MimoSinrChunk`` and ``MimoSignalChunk`` gained a ``cellId`` field for this purpose.
- A data signal that arrives while a reception is already in progress, but which is not aligned with it in start time or duration, is now discarded instead of aborting the simulation. Only the first reception schedules the end-of-reception event and the SINR of every simultaneously decoded signal is integrated over that one window, so the signals must be time-aligned; a cell or RNTI change in the middle of a reception (handover, or re-synchronisation after radio link failure) could deliver a misaligned one. The discarded signal is still accounted for as interference.
- Handover now applies the target cell's RACH configuration (carried in the handover command) to the UE MAC; previously the MAC kept whatever RACH configuration the source cell's SIB2 had installed, or none at all when the primary UL moved to a BWP that never decoded system information. Covered by the new ``nr-handover-rach-config`` test suite.
- Handover between FDD cells now works, including cells with inverted UL/DL carrier roles and different per-carrier numerologies: the handover command carries the target's UL carrier frequency, bandwidth and numerology, the UE re-synchronizes its UL PHY to the target cell, and the target gNB reserves the non-contention preamble on the MAC of the carrier where the random access actually arrives.
- A UE forced to camp without decoding SIB1 now uses the FR1-safe RACH lock duration instead of reading an uninitialized numerology.
- The ``cttc-nr-traffic-ngmn-mixed`` example now routes the UE traffic flow through the UL BWP in FDD (previously all uplink traffic was sent to the DL-only BWP and silently dropped, breaking FDD operation), aborts on a simulation time too short for the applications to run, and computes the median flow delay only over flows that received packets.
- A new transmission (NDI set) on a HARQ process now flushes any HARQ history left on that process by an earlier corrupted transport block. The history feeds both incremental redundancy (IR) and chase combining (CC) error models; previously, when a retransmission chain ended without a successful reception or an rv == 3 transmission (e.g., with HARQ retransmissions disabled), the stale history survived process reuse and the new transport block was incorrectly combined with it. Covered by the new ``nr-test-harq-flush`` test suite.
- The TDMA schedulers and the OFDMA uplink scheduling now reap allocations whose transport block size is below the minimum required to create a DCI (10 bytes in downlink, 12 bytes in uplink) and redistribute those resources among the other below-minimum UEs, mirroring the reaping already performed by the OFDMA downlink scheduling. Previously, when the number of simultaneously active UEs was greater than or equal to the schedulable resources in a slot and the UEs were still at the initial MCS 0, every UE received an allocation too small to transmit, all the DCIs were silently discarded, and the cell starved indefinitely (no data implies no CSI feedback, which implies the MCS never left 0). The `nr-system-test-scheduler-many-ues` test suite covers this scenario with 100 UEs.
- A receiver no longer keeps a permanent memory of an expired saturating signal in its interference sum. ``NrInterferenceBase`` kept the total received power as a running sum, exact in real arithmetic but not in floating point: a strong enough signal absorbed the co-resident ordinary signals when added (S + s rounds to S), so the paired subtractions left the sum permanently biased by the absorbed amount, and an infinite signal (overflowed linear received power) turned it into NaN. Every later SISO SINR and interference chunk at that receiver was then computed from the corrupted sum, even when nothing abnormal was on the air anymore. The sum is now rebuilt from the tracked list of live signals at every chunk evaluation, as the MIMO covariance path already did, and healed whenever a subtraction leaves a negative or non-finite residue. Covered by the new ``nr-interference-saturating-signal`` test suite.
- The RLC UM and AM receiving modes were corrected towards TS 38.322 conformance: AM status report generation (NACK fit in the configured status PDU size), polling and window-based retransmission blocking, UM receive-window rewind and wrap-around advance, and the re-establishment reset of both modes were fixed, and duplicate PDUs of an already-delivered SN no longer reach PDCP twice in out-of-order delivery mode. ``MaxTxBufferSize = 0`` now means an unlimited buffer in ``NrRlcUm`` and ``NrRlcTm``, consistent with ``NrRlcAm``.
- Connected-mode failures are now accounted per the TR 36.839 definitions: only T310 expiry and failures during a handover count as radio link failures, and a too-late handover command counts as a handover failure. Previously the too-late case was booked as a radio link failure, over-counting the RLF rate, and the handover failure rate double-counted it (and could exceed 100%). The ``RadioLinkFailureCause`` and ``HandoverTotalTime`` traces expose the new accounting.
- The DCI RBG bitmask is now validated against the RBG count of the bandwidth part instead of its RB count; a bitmask whose set bits fit the RB count but not the RBG count was previously accepted and read out of bounds.
- ``NrEpcUeNas`` and ``NrSpectrumPhy`` now release their back-pointer and bound callback to the owning ``NrUeNetDevice`` on dispose, breaking a reference cycle that kept the whole UE stack alive for the lifetime of the process.
- A UE that re-selects a cell and sends a fresh RRC connection request after declaring radio link failure no longer aborts the simulation when the gNB still holds its context in an active state; the stray request is ignored and the state timeout releases the context.

## Changes from NR-v4.2 to v5.0

### New API:
- Add ``NrHelper::AssignStreams()`` taking a single parameter structure (EPC, gNB/UE nodes and devices, and their base streams), unifying and replacing the previous scattered RNG stream assignment so that simulations are reproducible across runs. See commit 2e444dfa.
- Add a same-cell primary-BWP switch mechanism: the UE RRC measures RSRP per carrier and can switch its primary BWP within the serving cell, controlled by the new ``NrUeRrc::BwpSwitchHysteresis`` attribute, with a ``BwpManagerUe`` callback exposing the UE primary BWP index. See commits 15d565e7, 363afad1, 02499113 and 4daf7d08.
- Add ``NrGnbRrc::HandoverDecisionDelay`` and ``NrGnbRrc::HandoverTriggeringDelay`` attributes to model the handover decision and triggering latencies.
- Add the ``NrGnbNetDevice::NrHandoverAlgorithm`` attribute to configure the handover algorithm per device.
- Add Cell Individual Offset (CIO) support for handover biasing (cell range expansion), including a ``Hysteresis`` attribute on ``NrA3RsrpHandoverAlgorithm`` and a signed A3 offset to support negative CIO values (TR 36.839 Set 5). See commits 4735ccfc and 590741fe.
- Add msg3 (UL RRC Connection Request) HARQ retransmission, controlled by the new ``NrMacSchedulerNs3::Msg3MaxRetx`` attribute. See commit 0da21153.
- Add ``NrUePhy::Qin`` and ``NrUePhy::Qout`` attributes to configure the radio-link-monitoring in-sync/out-of-sync thresholds.
- Add the ``NrUeRrc::RlcMaxRetxTriggersRlf`` attribute (default false) to declare RLF when RLC-AM reaches its maximum retransmissions, per TS 38.331 Section 5.3.10.3. See commit 0c1e88ed.
- Add the ``NrFhControl::FunctionalSplit`` attribute to select the fronthaul functional split (``FS_6``, ``FS_7_3``, ``FS_7_2`` -- the default -- and ``FS_7_1``), with the corresponding per-split fronthaul throughput computation. See commits a48d380f and 4554c34f.
- Add the ``NrGnbPhy::TestDropRachPreambles`` attribute (testing support).
- Public API classes are now annotated with the ``NR_EXPORT`` macro, and internal symbols are hidden, to support building the module on Windows. See commit 5aa128de.

### Changes to Existing API
- The ``NumRbPerRbg`` attribute was moved from ``NrGnbMac`` to ``NrHelper``. See commit f3b5e3ed.
- The ``RbOverhead`` attribute was moved from ``NrGnbPhy`` to ``NrHelper`` and is applied at setup time only. See commit 4f40bd6c.
- The misspelled ``NrRlcUm::OutOfOfOrderDelivery`` attribute was renamed to ``OutOfOrderDelivery``.

### Changed Behavior
- NrAmc::CalculateTbSize now computes the number of code blocks with a real-valued division before applying ceil, as mandated by TS 38.212 Section 5.2.2; the integer division previously truncated the ratio, undercounting the code blocks and their CRC overhead, so transport blocks slightly larger than a code block now yield a marginally smaller usable TB size.
- UE RSRP is now accumulated in linear mW and converted to dBm only when reporting, the measurement sample counters were widened to 32 bits, zero-power PSS samples are skipped, and the RRC layer-3 filter ignores non-finite values. This fixes measurement corruption (reports stuck at reporting range 0) in multi-cell scenarios with data traffic; reported RSRP values in windows mixing different signal levels are slightly higher than before. See commit 6691525f.
- The RSRP reporting range mapping was updated from LTE to 5G-NR. See commit b6462ad0.
- The UE RSRQ measurement is now interference-aware (RSSI built from noise plus inter-cell interference). See commit f08b3c06.
- The ideal (cell scan) and realistic beamforming algorithms now scan the receive-side beams with the receive-side zenith step; previously the transmit-side step was used, incorrectly scanning the UE codebook whenever the two arrays had a different number of rows (fixes #276).
- The realistic beamforming SRS channel estimation now applies the documented time-domain filtering gain (Delta = 9 dB) and the normalization factor alpha, and draws the complex error with the documented total variance. See commit fea12c41.
- RLC-UM no longer delivers corrupt SDUs after t-Reordering expiry (stranded partial SDUs are discarded and the reassembly window is resynchronized), and the PDCP discard path actually drops the expired SDU instead of only tracing it. See commits 391f6bb9 and 070c7076 (and fixes #272).
- Handover: added inter-frequency and inter-numerology handover support (measurements, data-plane continuity and BWP re-tuning), the QoS flow ID is copied to the target on handover, X2-U data received outside the forwarding window is tolerated, and the endless rescheduling of delayed reports and handovers was fixed.
- Radio link failure: buffers, interference matrices, cell ID and CSI reporting are properly reset after RLF, RLF reselection works across numerologies, and FDD operation was added to the RLF and RRC tests.
- Initial cell selection now scans all configured BWPs, a forced camping request overrides an in-progress cell selection, and the UE listens with numerology zero by default.
- Scheduler: unstable sorts were replaced by stable ones and ``std::shuffle`` by the ns-3 shuffle for cross-platform reproducibility (fixes #250), transmission opportunities are assigned to control logical channels before data, MAC subheader overheads are accounted in buffer estimates, minimum control/data allocation sizes are enforced, and signaling messages are prioritized and not segmented.
- MIB/SIB1 transmissions are deferred to DL-capable slots, SIB1 parameters can be reconfigured via RRC, and the MIB numerology is routed per-carrier by ARFCN.
- Fixed a DL HARQ ``symAvail`` double-count underflow (fixes #278), a heap overflow in the HARQ round-robin beam ordering, and the RA-preamble sentinel so idle UEs do not match stray RARs.
- The radio environment map helper now uses the origin antenna element type when copying antennas (fixes #274).
- Messages are routed between BWPs using the ARFCN, and the signaling radio bearers are installed in the MAC/PHY stacks of all BWPs.

### Changes to build system
- This release is compatible with ns-3.48, and the minimum CMake version was bumped to 3.25.2 to match ns-3. See commits 5152ffa9 and fe00e81c.
- The module builds on Windows: internal symbols are hidden and the public API is exported through ``NR_EXPORT``. See commit 5aa128de.
- The ``cttc-nr-demo-sionna-rt`` example is built only when Sionna RT dependencies are detected and the platform is not Windows.
- The documentation build generates the figure artifacts from their committed ``.dia``/``.seqdiag``/``.dot`` sources instead of shipping generated images, is self-contained (no ns-3 source tree required), and the CI documentation job stops at the first Sphinx error and verifies that the expected outputs were generated.

## Changes from NR-v4.1.1 to v4.2

### New API:
- Add ``Get/SetBwpId()`` functions to ``BandwidthPartGnb``. These will be used instead of ``Get/SetCellId()``,
  minimizing confusion between the use of BwpId as "Physical" CellIds.
- Add ``NrGnbNetDevice::GetArfcnBwpId()`` function to retrieve device-local BWPId (offset) matching a specific ARFCN. See commit ef6a9043.
- Add ``NrUeNetDevice::GetBwpArfcn()`` function to retrieve the ARFCN of the specified BWPId. See commits ef6a9043 and fea08a4d.
- Add ``NrPhy::FrequencyHzToArfcn()`` and ``NrPhy::ArfcnToFrequencyHz()`` functions to compute ARFCN from frequency and vice-versa. See commit be20638d.
- Add ``NrMacSchedulerOfdma::EstimateTotalTbCapacity()`` function that temporarily allocates all remaining resources to a UE,
  checks the maximum achievable TB size, and then frees up resources. This function replaces the previously hardcoded eviction factor. See commit f2f00d78.

### Changes to Existing API
- Changed std:vector<uint16_t> cellIds parameters with a single cellId. In LTE we had multiple cells per gNB netdevice,
  while in NR we don't. All BWPs will now refer to the same RRC, as configured in the RRC. This affects primarily
  ``NrEpcHelper``, ``NrHelper``, ``NrNoBackhaulEpcHelper``, ``NrPointToPointEpcHelper`` and tests. See commit 8d50bc6e.
- ``NrQosRule`` replaces previous use of ``NrEpcTft` to reflect 5G terminology
- ``NrEpcTftClassifier`` class was renamed to ``NrQosRuleClassifier`` to reflect 5G terminology
- ``NrEpsBearer`` class was renamed to ``NrQosFlow`` to reflect 5G terminology.  Public API (class method names, 5QI values) that used to refer to ``EpsBearer`` now refers to ``QosFlow``
- ``NrEpcBearerTag`` class was renamed to ``NrQosFlowTag`` to reflect 5G terminology
- Replaced ``DlEarfcn`` and ``UlEarfcn`` with a single ``Arfcn``. See commit ef6a9043.
- The function ``NrGnbNetDevice::GetCellIds()`` was replaced with ``NrGnbNetDevice::GetBwpIds()``. See commit 8d50bc6e.
- The function ``NrGnbNetDevice::GetBwpId()`` was replaced with ``NrGnbNetDevice::GetCellId()``. See commit 8d50bc6e.
- The function ``NrGnbNetDevice::GetCellIdDlBandwidth()`` was replaced with ``NrGnbNetDevice::GetBwpDlBandwidth()``. Same for UL. See commits 8d50bc6e.
- The function ``NrGnbNetDevice::GetCellIdDlEarfcn()`` was replaced with ``NrGnbNetDevice::GetBwpArfcn()``. Same for UL. See commits 8d50bc6e and ef6a9043.

### Changed Behavior
- The numeration of BWPs was changed, so that BWP Ids match the order they are installed.
- The iteration order of rules used to classify packets to QoS flows, in the QosRuleClassifier
  (previously EpcTftClassifier), has changed.  The default bearer is still checked last, but a
  precedence-based ordering (ascending precedence according to TS 24.501) is now supported, and
  for rules that do not have precedence explicitly set, they are now evaluated in the order that
  they were added, rather than in reverse order (previously).
- The assignments of Data Radio Bearer ID, Logical Channel ID, and Qos Flow ID (formerly EPS Bearer ID)
  have been slightly changed; most notably, DRBID now aligns with LCID instead of LCID being assigned to (DRBID + 2)
- NrMacSchedulerLcRR previously divided the number of available bytes in a TxOp equally to all active LCs.
  Now it distributes them in RR fashion, so LCs with fewer bytes to transmit use less bytes, and LCs with more
  bytes to transmit get more if all other LCs with fewer bytes are fully satisfied.
- ARFCNs are now computed according to 3GPP standard, and are used instead of frequencies.

---

## Changes from NR-v4.1 to v4.1.1

### New API:

- ``NrHelper::IsMimoFeedbackEnabled()`` function was added to check whether MIMO feedback should be enabled or not, based on ``NrHelper::CsiFeedbackFlags``.
- ``NrChannelHelper::SetWraparoundModel()`` function was added to automatically aggregate a wraparound model to created channels.
- ``GetVirtualMobilityModel()`` function was added to get the wrapped position of an UE related to another position, given a channel which may have
  or not an aggregated wraparound model. This is necessary in cases where we do not effectively transmit in the channel (which now transparently
  applies wraparound for real transmissions), such as in ``NrInitialAssociation`` and ideal beamforming techniques.

### Changes to Existing API

- ``NrHelper::EnableMimoFeedback`` attribute was removed. To check if MIMO feedback should be enabled, call the new ``NrHelper::IsMimoFeedbackEnabled()`` function.
- ``WraparoundThreeGppSpectrumPropagationLossModel`` and ``HexagonalWraparoundModel`` were removed.
  Now wraparound is transparently applied by the ``MultiModelSpectrumChannel`` and ``SingleModelSpectrumChannel`` if a wraparound model is aggregated to it.

### Changed Behavior

---

## Changes from NR-v4.0 to v4.1

### New API:

- ``ThreeGppFtpM1Helper`` has two new functions ``GetMaxFilesNumPerUe()`` and ``SetMaxFilesNumPerUe()`` that allows users to limit the maximum number of files sent per UE.
- ``NrInitialAssociation`` and ``KroneckerQuasiOmniBeamforming`` have two additional attributes to set beam column and row angles: ``ColumnAngles`` and ``RowAngles``. These can also be set with the new functions ``ParseColBeamAngles()`` and ``ParseRowBeamAngles()``.
- ``KroneckerBeamforming`` has four additional attributes to set beam column and row angles for transmitter and receiver: ``TxColumnAngles``, ``TxRowAngles``, ``RxColumnAngles``, and ``RxRowAngles``. These can also be set with the new functions ``ParseColTxBeamAngles()``, ``ParseRowTxBeamAngles()``, ``ParseColRxBeamAngles()``, and ``ParseRowRxBeamAngles()``.
- ``NrMacSchedulerNs3`` has a new function ``ReshapeAllocation()``, which calls ``NrMacSchedulerTdma::DoReshapeAllocation()`` to perform DCI consolidation, using as many RBGs as possible without reducing MCS.
- New class ``ResourceAssignmentMatrix``, which can create a resource assignment matrix, which can set beam per symbol, plus RNTI and usage type per RBG. When asserts are enabled, the matrix is used to validate ``NrMacSchedulerNs3::ScheduleDl()`` and ``NrMacSchedulerNs3::ScheduleUl()`` allocations. The matrix can also be printed for visual inspection.

### Changes to Existing API


### Changed Behavior
- Schedulers now have memory, and maintaining their fairness over time. As a result, users should observe less UEs with 0 throughput.
- Downlink HARQ is now scheduled using a round-robin per beam fashion.
- To minimize symbols used while maintaining a high MCS, downlink HARQ now passes through a consolidation step implemented in function ``NrMacSchedulerNs3::ReshapeAllocation()``.

---

## Changes from NR-v3.3 to v4.0

### New API:

- Introduced multi-panel antenna support in SpectrumPhy.

- Added a new attachment algorithm `NrInitialAssociation` based on maximum RSRP instead of distance.
  It can be automatically setup via the new function `NrHelper::AttachToMaxRsrpGnb()`.

- Introduced new beamforming algorithms: `KroneckerBeamforming` and `KroneckerQuasiOmniBeamforming`.

- Added a new helper class, `NrChannelHelper`, to simplify the implementation, configuration, and assignment of spectrum channels with various channel models for band creation. This extends the module to support NYUSIM, Fluctuating Two-Ray (FTR), and 3GPP channel models.

- Introduced a new reinforcement learning–based scheduler in both TDMA and OFDMA variants: `NrMacSchedulerTdmaAi` and `NrMacSchedulerOfdmaAi`. These require additional fields to hold the observation state, provided by the new `NrMacSchedulerUeInfoAi` class. The schedulers communicate with the ns3-gym Python model via `NrMacSchedulerAiNs3GymEnv`.

- Channel state information (CSI) can now be obtained using periodic CSI-RS signaling and CSI-IM measurements. A new signal type, `NrSpectrumSignalParametersCsiRs`, was introduced to represent CSI-RS signals. The type of CSI feedback can be configured using the `NrHelper::CsiFeedbackFlags` attribute.

- Introduced a new spectrum filter, `NrCsiRsFilter`, to reduce the computational complexity of CSI-RS signaling by filtering signals for `SpectrumPhy` instances that should not receive them.

- Added a new function, `NrMacSchedulerUeInfo::GetDlMcs()`, which returns either the wideband MCS or an MCS estimate based on the sub-band CQI of allocated RBGs.

- Added several new functions to `NrMacSchedulerOfdma` to simplify resource allocation: `AttemptAllocationOfCurrentResourceToUe()`, `AllocateCurrentResourceToUe()`, `DeallocateCurrentResourceFromUe()`, `ShouldScheduleUeBasedOnFronthaul()`, and `DeallocateResourcesDueToFronthaulConstraint()`.

- Introduced new symbols-per-beam scheduling options. These can be set using the attribute NrMacSchedulerOfdma::SymPerBeamType, with available values: LOAD_BASED (default), ROUND_ROBIN, and PROPORTIONAL_FAIR.

- A new function, `IsMaxSrsReached()`, was added to `NrGnbRrc`, `NrGnbMac`, `NrMacScheduler`, and `NrMacSchedulerSrs`, allowing the RRC to use the flexible SRS timings implemented in 5G-LENA instead of requiring manual configuration via `NrGnbRrc::SrsPeriodicity`.

- Added two new functions, `NrEpcHelper::SetupRemoteHost` and `NrEpcHelper::SetupRemoteHost6`, to replace the previously copy-and-pasted code for setting up remote hosts in IPv4 and IPv6 networks, respectively.

- Added a new attribute, `NrPmSearch::EnforceSubbandSize`, which controls whether sub-band sizes follow 3GPP specifications, providing greater flexibility.

- Introduced the attribute `NrNetDevice::ReceiveErrorModel` to support packet-drop error models, along with a new trace source, `NrNetDevice::Drop`, for tracking dropped packets.

- Added new functions to `NrUeRrc: SetPrimaryUlIndex()`, `GetPrimaryUlIndex()`, `SetPrimaryDlIndex()`, and `GetPrimaryDlIndex()`—allowing explicit selection of downlink and uplink primary carriers.

- Added new classes to implement alternative symbols-per-beam scheduling policies:
  - `NrMacSchedulerOfdmaSymbolPerBeamLB` (default, load-based),
  - `NrMacSchedulerOfdmaSymbolPerBeamRR` (round-robin),
  - `NrMacSchedulerOfdmaSymbolPerBeamPF` (proportional fair).

### Changes to Existing API
- Removed the scenario configuration from band creation. Band initialization and channel attribute setting methods were removed from NrHelper.

- Added a new attribute, `NrRlcUm::OutOfOrderDelivery`, corresponding to the TS 36.322 Section 5.1.2.2.3 rlc-OutOfOrderDelivery variable (default: false).

- Merged the `NrMacSchedulerCQIManagement::DlWBCQIReported()` and `DlSBCQIReported()` functions into a single function: `DlCqiReported()`.

- NrAmc pointers previously passed via parameters to `NrMacSchedulerUeInfo` functions are now set during UeInfo creation.

- Functions `NrUeInfo::GetDlRBG()`, `GetUlRBG()`, `GetDlSym()`, and `GetUlSym()` now return vectors instead of single values to support sub-band–aware schedulers.

- Added a new field to `NrHelper::AntennaParams` to configure antenna downtilt angles.

- Removed the class `CellScanBeamformingAzimuthZenith`. The corresponding method `BeamManager::SetSectorAz(double azimuth, double zenith)` was removed in favor of the refactored `CellScanBeamforming`.

- Changed the type of the sector parameter in `BeamManager::SetSector()` from uint16_t to double to support oversampling.

- Removed the attribute `NrGnbRrc::SrsPeriodicity` in favor of `NrMacSchedulerSrsDefault::StartingPeriodicity`.

### Changed Behavior
- `CellScanBeamforming` and `RealisticBeamformingAlgorithm` were refactored to scan all azimuth and elevation sectors based on the number of antenna elements and the oversampling factor.

- Since release 3.0, UEs received unintended transmissions in NrSpectrumPhy to estimate the channel on unused PRBs. This workaround (`isIntendedRx = true`) has now been removed, as CSI-RS and CSI-IM are sufficient for full-bandwidth estimation. Using only `NrHelper::CsiFeedbackFlags = CQI_PDSCH_MIMO` now estimates the channel exclusively on allocated RBGs.

- Since release 3.2, MSG3 could be incorrectly transmitted in DL and S slots.
  This has been fixed, and MSG3 scheduling is now restricted to F and UL slots.

- Since release 3.2, FDD setup was incorrectly mixing DL and UL carriers. This has been fixed. When using FDD:

  - Ensure that two bandwidth parts are used.
  - Explicitly set the uplink primary index.
  - Set appropriate TDD patterns.

```
// 1x10MHz in TDD, 2x5MHz in FDD
uint8_t bands = 1 + m_isFdd;
auto bwAndBWPPair = m_nrHelper->CreateBandwidthParts({{2.8e9, 10e6 / bands, bands}}, "UMa");
NrHelper::GetGnbPhy(gnbDevs.Get(i), 0)
          ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|"));
if (m_isFdd)
{
    Config::SetDefault("ns3::NrUeNetDevice::PrimaryUlIndex", UintegerValue(1));
    NrHelper::GetGnbPhy(gnbDevs.Get(i), 0)
              ->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
    NrHelper::GetGnbPhy(gnbDevs.Get(i), 1)
              ->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
}
```

---

## Changes from NR-v3.2 to v3.3

### New API:

- A new class ``NrFhControl`` is added that implements the Fronthaul Control that
  allows the simulation of a limited-capacity fronthaul (FH) link. This class interacts
  with the PHY and MAC layers through the ``NrFhPhySapProvider``, ``NrFhPhySapUser``,
  ``NrFhSchedSapProvider`` and ``NrFhSchedSapUser`` SAPs.

- Added ``BwpManagerAlgorithm::GetAlgorithm()`` to retrieve the ``BwpManagerAlgorithm``.

### Changes to existing API:

- The ``NrHelper`` is extended to include the function ``EnableFhControl`` to enable
  the Fronthaul Control. Moreover, attributes of the Fronthaul Control can be set
  through the ``SetFhControlAttribute`` function.

- The ``NrMacSchedulerNs3``, ``NrMacSchedulerOfdma``, ``NrMacSchedulerHarqRR`` and
  ``NrGnbPhy`` are extended to support various Fronthaul Control mechanisms for
  discarding data or limiting allocations.

### Changed behavior:

- (37dea723) In the OFDMA access mode, allocate, at least, the number of RBs necessary
  to transmit the minimum TBS.

- (2dd513a7) Delay the shuffling of the SRS offset until the stream assignment.

---

## Changes from NR-v3.1 to v3.2

### New API:


### Changes to existing API:
- Removed the ``NrHelper::HarqEnabled`` flag because it was considered a misleading
feature. Unlike the feature available in the ns-3 LTE module, which completely
disables/enables HARQ logic, this feature solely disables HARQ feedback.
Currently, the only similar option available in the NR module is
``ns3::NrMacSchedulerNs3::EnableHarqReTx`` which enables/disables HARQ retransmissions.
- All previously ``Lte`` prefixed configuration paths, classes and functions should now be use the ``Nr``
prefix. All configuration paths, classes and functions containing ``Enb`` should now use ``Gnb``.
  - Before: ``Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/NewUeContext", MakeBoundCallback(&NrBearerStatsConnector::NotifyNewUeContextEnb, this));``
  - After: ``Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/NewUeContext", MakeBoundCallback(&NrBearerStatsConnector::NotifyNewUeContextGnb, this));``

### Changed behavior:
- ``IMSI``s are now aligned with ``NodeId``, instead of using an independent counter.

---

## Changes from NR-v3.0 to v3.1

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.41 to ns-3.42.

### New API:
- SU-MIMO model is extended with a new class called ``NrCbTypeOneSp`` that
provides the implementation of Type-I Single-Panel Codebook 3GPP TS 38.214,
Rel. 15, Sec. 5.2.2.2.1 supporting up to 32 ports, rank 4, and codebook mode 1.

### Changes to existing API:
-

### Changed behavior:
-

---

## Changes from NR-v2.6 to v3.0

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.40 to ns-3.41.

With this release we also move from the old DP-MIMO model to a new more general
MIMO model, hence there are many API changes, i.e., those that were part of the
old DP-MIMO model are mostly removed, while there are many new APIs that are
added to support the new MIMO model. In the following are listed the most
important API changes.

### New API:
- A new chunk processor ``NrMimoChunkProcessor`` is added to store the results of the
``NrInterference`` in the two different types of chunks ``MimoSinrChunk`` and ``MimoSignalChunk``.
- A new ``NrCbTypeOne`` class is added as an interface class for the precoding matrix
codebooks defined in 3GPP TS 38.214, Sec. 5.2.2.2.1. The class provides an interface
for the wideband precoding matrix index i1 and the subband precoding matrix index i2.
- A new ``NrCbTwoPort`` class is added that implements the case of 2 antenna ports
given in Table 5.2.2.2.1-1.
- A new class ``NrPmSearch`` is added as an interface class for searching optimal precoding
matrices and creating full CQI/PMI feedback. Provides configuration for common parameters.
- A new class ``NrPmSearchFull`` provides an implementation of the ``NrPmSearch`` interface
that uses exhaustive search over all possible precoding matrices specified in a codebook that
is compatible with 3GPP TS 38.214 Type-I to find the optimal PMI and RI values,
and creates a CQI/PMI/RI feedback message.
- A new class ``NrMimoSignal`` is added to create an alternative representation of the MIMO signal.
``NrMimoSignal`` averages over the multiple different interference signals over time
and it stores (consolidates) the signals towards multiple UEs into a single channel matrix.
- New functions are added to ``NrInterference`` to pass the MIMO signals: ``AddSignalMimo``,
``StartRxMimo`` and ``DoSubtractSignalMimo``. Additionally, ``NrInterference`` is extended
with functions that are used to compute the interference-and-noise covariance matrix.
These functions are: ``CalcOutOfCellInterfCov``, ``CalcCurrInterfCov``, and ``AddInterference``.
Finally, a function ``ComputeSinr`` is added to compute MIMO SINR.
- New matrices that are used for MIMO calculations are defined in file nr-mimo-matrices.h/cc.
These matrices are: `NrCovMat`, `NrIntfNormChanMat`, and `NrSinrMatrix`.
- ``NrErrorModel`` is extended to include ``GetTbDecodificationStatsMimo`` which is called
by the ``NrSpectrumPhy`` to determine if the transport block was received correctly.
- ``NrAmc`` is extended to include additional methods to compute the maximum supported
MCS when MIMO is used. These functions are ``GetMaxMcsForErrorModel``, and ``GetWbCqiFromMcs``.
- In ``NrMacSchedulerUeInfo`` ``m_cqi`` is renamed to ``m_wbCqi``.
- The ``GenerateDlCqiReportMimo`` function is added to ``NrUePhy`` to generate DL CQI,
channel quality precoding matrix, and rank indicators (PMI, and RI).
``CheckUpdatePmi`` is added to ``NrUePhy`` to allow limiting the frequency of PMI updates in
the simulator to achieve better simulation performance.
- In ``NrSpectrumPhy``, a new ``AddDataMimoChunkProcessor`` function is added  to
set the MIMO chunk processor, and a new callback function ``UpdateMimoSinrPerceived``
is added to store the MIMO SINR chunks for all received signals at the end of
interference calculations. A private function ``GetMimoSinrForRnti`` is added to
filter MIMO SINR for the RNTI of the expected signal.
- A new structure ``PmCqiInfo`` is used as the CQI feedback message
that contains the optimum CQI, RI, PMI, and full precoding matrix.
- A new structure ``AntennaParams`` is added to NrHelper to allow easier configuration of
the antenna parameters of the UE and gNB.
- New functions ``SetupGnbAntennas`` and ``SetupUeAntennas`` are added to ``NrHelper``
to allow easier configuration of antenna parameters. A new function ``SetupMimoPmi`` is added to allow
easier configuration of PMI search in MIMO operation.
- ``BandwidthPartInfo`` configuration is possible outside of helper.

### Changes to existing API:
- The rank number is added to many functions related to transport block and transport
block calculations. It is added also to the output trace file of ``NrPhyRxTrace``.
- In ``NrMacSchedulerUeInfo``, ``m_cqi`` renamed to ``m_wbCqi``.
- In ``NrSpectrumPhy`` RNTI is added to the ``ExpectedTb`` struct.
- ``SfnSf`` has a new function ``GetEncodingWithSymStartRnti`` that allows to encode
RNTI uses 16 bits (48-63), frame number uses 24 bits(24-47), subframe number uses
8 bits (16-23), slot number uses 8 bits (8-15), numerology uses 3 bits (5-7),
and sym start uses 5 bits (0-4).
- In OFDMA DL, multiple transmissions to different UEs are no longer
represented by a single signal (aka OFDMD DL trick).
To allow that many interfaces are extended with RNTI parameter.
As already mentioned ``GetEncodingWithSymStartRnti`` allows a mapping of a single
packet burst to a single signal for a specific UE. In ``NrGnbPhy`` it is
implemented the scaling of the transmitted power based on the number of occupied
resources per UE. The ``SetSubChannel`` function is called individually for each
simultaneous transmission. In ``NrGnbPhy::FillTheEvent()`` is removed the logic that
skips all the allocations other than the first one. ``NrGnbPhy::m_lastBfChange`` is
used to prevent improper changes of analog beamforming vectors. In ``NrSpectrumPhy``
are skipped receptions of signals that are intended for other UEs. However, the
variable that controls whether the UE will consider the signal as intended for
him is currently hardcoded to true ``isIntendedRx = true;`` to allow correct
generation of CQI feedback from data signals, even from those for other UEs in
the same cell. This workaround will be removed once proper CSI-RS signaling
is implemented. In ``NrSpectrumPhy::StartTxDataFrames``, asserts that were
not allowing multiple signals at the same time are removed.
- ``CreateQuasiOmniBfv`` is updated to consider multiple ports and polarizations.
- ``NrUePhy::GetSpectrumPhy`` does not have any parameter. It returns the only
``NrSpectrumPhy`` that is attached to that ``NrUePhy`` instance.
- Functions that were created to support DP-MIMO are removed. These are:
``NrUePhy::SetFixedRankIndicator``, ``NrUePhy::GetFixedRankIndicator``, ``NrUePhy::UseFixedRankIndicator``,
``NrUePhy::SetRiSinrThreshold1``, ``NrUePhy::GetRiSinrThreshold1``, ``NrUePhy::SetRiSinrThreshold2``,
``NrUePhy::GetRiSinrThreshold2``, and ``NrUePhy::SelectRi``. Also,
``NrSpectrumPhy::SetInterStreamInterferenceRatio`` is removed.
- ``BeamConfId`` is removed. Instead is used ``BeamId``.
- ``ThreeGppChannelModelParam`` is removed since its main purpose was testing the DP-MIMO.
- Old `cttc-nr-mimo-demo` example that was using DP-MIMO, was replaced by a new
example with the same name that uses a new MIMO implementation.
- A "stream" parameter is removed from ``NrHelper``'s functions
``InstallUeDevice``, ``InstallGnbDevice``, ``CreateGnbPhy``, ``CreateUePhy``,
``InstallSingleUeDevice`` and ``InstallSingleGnbDevice``.
``dlHarqCallback`` parameter is added back to ``CreateUePhy``
- Parameter ``streamId`` removed from ``NrGnbPhy::GenerateDataCqiReport``.
The index parameter was removed from ``NrGnbPhy::SendDataChannels``. The number of
active streams parameter is removed from ``SetSubChannels``.
Changed parameter type of the ``NrUePhy::CreateDlCqiFeedbackMessage`` function.
Removed the parameter that indicates the number of active streams from
``NrUePhy::SetSubChannelsForTransmission`` and ``NrUePhy::GetTxPowerSpectralDensity``.
- Callback functions for the HARQ feedback are moved back from ``NrUePhy`` to
``NrSpectrumPhy``. ``NrSpectrumPhy`` now has again the following functions:
``SetPhyDlHarqFeedbackCallback``. Function ``NrUePhy::NotifyDlHarqFeedback`` is removed.
- ``HarqProcessInfoSingleStream`` is removed.
- The ``DciInfoElementTdma`` constructor is updated to remove the notion of multiple
streams, i.e. the scalar parameters are used for tbs, ndi, and rv, instead of
``std::vector``.
- The ``NONE`` state is removed from ``HarqStatus`` enumeration.
- The parameter stream is removed from the ``NrPhySap::SendMacPdu`` function.
- APIs of ``BeamformingHelperBase``, ``RealisticBeamformingHelper`` and
``IdealBeamformingHelper`` are updated to remove the notion of the stream. Also,
``RealisticBeamformingAlgorithm`` and ``IdeaBeamformingAlgorithm`` child classes
have been updated accordingly.
- Unused code is removed from ``cttc-nr-traffic-3gpp-xr`` example.

### Changed behavior:
- ``cttc-nr-demo`` example's parameters are updated to be according 3GPP TR 38.901 UMi-Street Canyon. Also,
the logged values in the NR tutorial are updated accordingly with the updated results.
- ``NrHelper`` now creates an ``NrPmSearch`` object for each UE, and sets the corresponding parameters.
- ``NrHelper`` creates the ``NrMimoChunkProcessor`` and adds the callback to
``UpdateMimoSinrPerceived`` for MIMO SINR feedback, and to ``GenerateDlCqiReportMimo``
to provide the MIMO signal to the PMI search.
- ``NrMacSchedulerCQIManagement`` now processes the RI and PMI feedback.
- The MAC schedulers (OFDMA, TDMA, HARQ, NrMacSchedulerNs3) now use the RI and precoding matrix
from the feedback when scheduling a new downlink transmission, and set the precoding matrix in the DCI.
- In ``NrPhyMacCommon`` the precoding matrix information is added to several structures, e.g., ``DlCqiInfo``.
- In ``NrSpectrumPhy``, the downlink precoding matrix is set to the transmission
parameters according to the DCI.

---

## Changes from NR-v2.5 to v2.6

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.39 to ns-3.40.

### New API:

- Added `Tx` and `Rx` trace sources in NrNetDevice to allow easier tracing of
the events when the packet is transmitted or received.

### Changes to existing API:

None.

### Changed behavior:

- With the #157 fix the default number of HARQ processes for the downlink and
the uplink has changed from 20 to 16.

- In general, the NR module logging is refactored. Many log messages have now
different logging levels. E.g., instead of NS_LOG_INFO is used NS_LOG_FUNCTION.
There are new log messages at the different layers at gNB and UE.

---

## Changes from NR-v2.4 to v2.5

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.38 to ns-3.39.

### New API:

* New QoS MAC schedulers are implemented that perform scheduling by taking into
account the QoS requirements of QoS flows. The implemented scheduler classes are
the `NrMacSchedulerTdmaQos` and `NrMacSchedulerOfdmaQos`. These classes are
responsible for setting the scheduler and access mode types when desired by the
user and updating the DL and UL metrics of each UE. The sorting of the UEs
(based on these metrics) is done by the newly implemented class `NrMacSchedulerUeInfoQos`.

* New design for the LC bytes assignment. A new base class is added, known as
`NrMacSchedulerLcAlgorithm` that allows the implementation of various algorithms
for the LC bytes assignment. We implement two classes, the `NrMacSchedulerLcRR`
and the `NrMacSchedulerLcQos`. The former is the original implementation of
assigning bytes to LCs in RR fashion (see method AssignBytesToLC in previous releases).
The latter includes a new algorithm that shares bytes among the active LCs by taking
into account the resource type and the e_rabGuaranteedBitRate of a flow.

### Changes to existing API:

* Extend the `BwpManagerAlgorithm` to support Release 18 5QIs.

* Code is updated based on the lte module extension for delay-critical GBR.

* Method `AssignBytesToLC` of `NrMacSchedulerNs3` is moved to a class (`NrMacSchedulerLcRR`).

* Deprecate use of `V4PingHelper` (now `PingHelper`) to be compatible with ns-3.

* NrHelper undefined method `GetBandwidthPartMap` is dropped.

### Changed behavior:

---

## Changes from NR-v2.3 to v2.4

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.37 to ns-3.38, including updating to the new and updated interfaces in
antenna and spectrum module (ns-3 !1046 and ns-3 !1269).

### New API:

* The `NR` module now includes a new traffic generators framework that allows to
simulate NGMN traffic applications for mixed traffic scenarios and advanced
and multi-flow 3GPP XR traffic applications, such as Virtual Reality (VR),
Augmented Reality (AR), and Cloud Gaming (CG) applications.
The traffic models are included in `nr/utils/traffic-generators`,
with a goal to extend them and in future port them to the ns-3 applications
module. In particular, this NR release adds the following traffic applications:
`TrafficGenerator3gppAudioData`, `TrafficGenerator3gppGenericVideo`,
`TrafficGeneratorPoseControl`, `TrafficGeneratorNgmnFtpMulti`,
`TrafficGeneratorNgmnGaming`, `TrafficGeneratorNgmnVideo`, and `TrafficGeneratorNgmnVoip`.
Additionally, the framework offers `TrafficGeneratorHelper` that helps
installing traffic generator application on a client node. And, the model
offers also `XrTrafficMixerHelper` that helps mixing 3GPP traffic applications
to simulate more complex multi-stream 3GPP XR models such as 3GPP AR Model 3A
that is composed of 3 streams (pose/control, scene/video, audio/data),
3GPP VR composed of 2 streams (scene/video and audio/data), and 3GPP CG downlink
composed of 2 streams (scene/video and audio/data). Usage of these new APIs in
the NR module can be seen in the examples `cttc-nr-traffic-ngmn-mixed.cc`, and
`cttc-nr-traffic-generator-3gpp-xr.cc`.

* Added RSRP trace source to `NrUePhy`.

* Added function `GetDlCtrlSymbols` to `NrGnbMac` that allows to obtain the number
of symbols used for `CTRL`.

### Changes to existing API:

* Removed unused folders: `BeamFormingMatrix`and `Raytraycing`. If you were maybe
using these folders and you would like that we return them back to the `NR`
module please let us know.

### Changed behavior:

* Detected and fixed a bug when postponing transmission in NR-U simulations,
i.e., when channel was busy and `PushFrontSlotAllocInfo` was being called to
postpone currently scheduled transmission, a wrong function for encoding the
SfnSf was being called. It was called `GetEncodingWithSymStart` instead of
`GetEncForStreamWithSymStart`. Thanks to George Frangulea for helping to find and
resolve this bug.

* Fixed an error in PointInFTPlane constructor, i.e., m_rbg was defined
as `uint32_t`, but the constructor parameter was using `uint8_t`.

* Fixed an issue with `S` slots in `NrGnbPhy` and `NrPhy`, whose previous
implementation was not considering correctly that `S` slot could contain the
`UL` control for the HARQ, and also it was not considering `S` slot as an
option for the `DL` and neither the `UL` in the functions `HasDlSlot` and
`HasUlSlot`.

* Fixed a bug in the `NrMacSchedulerNs3` in functions `DoScheduleDl` and `DoScheduleUl`
when updating the active DL and UL users list. Since these two maps were not
being updated correctly, in some situations the function `GetSymPerBeam` was
being called with activeDlUe or activeUlUE map containing all empty elements,
and then the assert was being hit in `GetSymPerBeam` function:
`NS_ASSERT(symAvail >= symUsed);`.

* Fixed `NrGnbPhy::StartSlot` to allow a flexible and configurable number of
`CTRL` symbols per slot. Removed an assumption of only a single DL CTRL symbol
per slot which was valid for the first versions of the NR module.

* Previously offered `FileTransferApplication` is now implemented in
`TrafficGeneratorFtpSingle`. Also, previously offered `FileTransferHelper` is now
offered in `TrafficGeneratorHelper`. FTP M1 continues to be supported by the `NR`
module, just that `ThreeGppFtpM1Helper` is using now `TrafficGeneratorHelper` and
`TrafficGeneratorFtpSingle`. Hence, the `NR` user still uses `ThreeGppFtpM1Helper`,
but this class is now using a new traffic generator framework.

---

## Changes from NR-v2.2 to v2.3

This release contains the upgrade of the supported ns-3 release, i.e., upgrade
from ns-3.36.1 to ns-3.37, including updating to clang format and with clang-tidy.
In addition, whitespaces trailing is also checked.

### New API:

* Added new example called `cttc-nr-3gpp-calibration` used for the calibration
of the simulator under 3GPP outdoor reference scenarios.

* Added `DlDataSnrTrace`, `DlCtrlPathloss` and `DlDataPathloss` trace sources in
NrSpectrumPhy.

* `NrUePhy` now includes the RSRP measurements of a UE.

### Changes to existing API:

* Changed parameter type of the function
`DistanceBasedThreeGppSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity`.

* PHY traces are extended with a function to set the results folder path.

* `HexagonalGridScenarioHelper` is extended to allow the configuration of a
variable that defines the maximum distance between a UE and the closest site, through
the `HexagonalGridScenarioHelper::SetMaxUeDistanceToClosestSite` function.
This function can be used only in conjunction with the
`HexagonalGridScenarioHelper::CreateScenarioWithMobility`.

* `HexagonalGridScenarioHelper` is also extended to set the results folder path
and a simTag for the generated gnuplot file. This is useful so that different
simulations will not override the results.

* `GridScenarioHelper` includes now a function to set the starting position of the grid.

* `Nrhelper` now avoids re-assigning a stream due to incorrect pointer.

* Remove from `NrEesmErrorModel` redundant SpectrumValue copy that can cause a
significant drop in the performance unnecessarily.

* Allow the `NrErrorModel` to be passed and fetched as an object in `NrSpectrumPhy`.

* Sfnsf frame number is expanded to 32-bit to prevent rollover


### Changed behavior:

* The antenna orientation in the `NodeDistributionScenarioInterface::GetAntennaOrientationDegrees`
is changed from 60, 180, 300 degrees to 30, 120, 270.

* Fixed how the HARQ feedback from multiple streams is combined in `NrUePhy`.
Now it takes into account the use case when there are different HARQ IDs,
because there can be ReTX DCI, and TX DCI for the same UE.

* Fixed and modified the code for MAC UL/DL RLC TX/RX/PDU queues.
The code at gNB MAC that keeps track of DL/UL RLC queues has been reworked, due
to some inconsistencies related to the handling of information about RLC UE queues.
In some cases, due to these misalignments, the gNB MAC was not assigning sufficient
resources to a UE. Moreover, the UE MAC did not account correctly related to the
MAC header.

---

## Changes from NR-v2.1 to v2.2

This release contains only the upgrade of the supported ns-3 release,
i.e., upgrade from ns-3.36 to ns-3.36.1.

### New API:

None.

### Changes to existing API:

None.

### Changed behavior:

None.

---

## Changes from NR-v2.0 to v2.1

### New API:
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

### Changes to existing API:

### Changed behavior:
- Calculate CQI based on Average Spectral Efficiency in `nr-amc.cc`
- Stop assigning resources in UL TDMA if TB size >= buffSize in `nr-mac-scheduler-ue-info.cc`
- Changed how to consider RLC overhead when updating the TX queues in MAC scheduler
- Changed the buffer size calculation in `NrMacSchedulerLcg` to consider correctly the
RLC overhead

---

## Changes from NR-v1.3 to v2.0

### New API:

* `NrUePhy` has a new function `GetSpectrumPhy` that returns the `NrSpectrumPhy`
instance corresponding to the provided index. By default it returns 0th
`NrSpectrumPhy` instance, since `NrUePhy` must have at least 1 `NrSpectrumPhy`
installed. But, can have more instances, depending on the supported number
of streams for DP-MIMO. Additionally,
`NrUePhy` has new public functions related to rank adaptation: `SetFixedRankIndicator`,
`GetFixedRankIndicator`, `UseFixedRankIndicator`, `SetRiSinrThreshold1`, `GetRiSinrThreshold1`,
`SetRiSinrThreshold2`, and `GetRiSinrThreshold2`. Additionally, it has one new
private function called `SelectRi` for selecting the RI that will be reported to
gNB. `NrSpectrumPhy` has new function for configuring the inter-stream interference,
called `SetInterStreamInterferenceRatio`.

* `BeamConfId` is a new class that is added to uniquely identify the beam
configuration of a `NrUePhy` or `NrGnbPhy` supporting DP-MIMO, hence up to two streams.
Previously, OFDMA scheduler was assigning at the same varTti only users belonging
to the same beam (identified by BeamId), because there was maximum 1 beam per
`NrUePhy` or `NrGnbPhy` instance. Now, since with DP-MIMO we can have 2 beams per
PHY instance, it is used `BeamConfId` in OFDMA scheduling to select the users
that can be scheduled at the same varTti.

* Added new `NrStatsCalculator` statistic base class, and its specialization class called
 `NrMacSchedulingStats` class that provides statistics for DL and UL MAC scheduling,
 and it also includes the stream ID. Added new `NrSchedulingCallbackInfo` structure
 for holding the scheduling information.

 * `NrGnbMac` has a new trace source called `UlScheduling`, while `DlScheduling`
 trace source is extended to provide more detailed scheduling information.

 * Added new `ThreeGppChannelModelParam` class that inherits `ThreeGppChannelModel`
 class and allows the parametrization of the correlation coefficient. This
 class is added for research purposes to allow parametrizing the cross correlation
 correlation parameter and thus study the impact of this parameter on
 inter-stream interference among the signals of different polarizations, and
 to compare it with the original 3GPP cross polarization correlation parameter
 value and resulting interference.

 * Added new example called `cttc-nr-mimo-demo` that allows the configuration,
 usage and testing of DP-MIMO feature in nr module.

 * Added API for new pathloss traces. Extended CTRL traces to include SRS Tx and
 RX information. Added stream ID in DL/UL RX traces (`NrPhyRxTrace`).

 * `NrHelper` functions for enabling NR traces are now part of NrHelper public API.
 These new public functions are: `EnableDlPhyTraces`, `EnableUlPhyTraces`,
 `EnableGnbPacketCountTrace`, `EnableUePacketCountTrace`, `EnableTransportBlockTrace`,
 `EnableGnbPhyCtrlMsgsTraces`, `EnableUePhyCtrlMsgsTraces`, `EnableGnbMacCtrlMsgsTraces`,
 `EnableUeMacCtrlMsgsTraces`, `EnableRlcTraces`, `EnablePdcpTraces`, `GetPdcpStats`,
 `EnableDlMacSchedTraces`, `EnableUlMacSchedTraces` and `EnablePathlossTraces`.

### Changes to existing API:

* `NrHelper`'s public API functions `InstallUeDevice`, `InstallGnbDevice` have one
additional parameter that defines how many streams will support the device.
The value of this new parameter is by default 1 so it does not impact the usual
usage of these functions. `NrHelper`'s private API functions `CreateGnbPhy`,
`CreateUePhy`, `InstallSingleUeDevice` and `InstallSingleGnbDevice` have now one
additional parameter that allows to configure the number of streams (antenna
arrays or subpartitions) supported by the corresponding PHY or device.
Also, `NrHelper`'s private function `CreateUePhy` has now one less parameter,
i.e., dlHarqCallback parameter is removed. These changes should not affect
5G-LENA users who do not modify/inherit/extend `NrHelper`'s code.

* `NrGnbPhy` function `GenerateDataCqiReport` has one more parameter `streamId` to
indicate for which stream is reported CQI. Private function `NrGnbPhy::SendDataChannels`
has one more parameter that is the index of the `NrSpectrumPhy` of that `NrGnbPhy`
over which will be sent the data. Private function `SetSubChannels` has one more
parameter that says the number of active streams.

* Changed parameter type of the `NrUePhy::CreateDlCqiFeedbackMessage` function.
Private `NrUePhy::SetSubChannelsForTransmission` function has one more parameter that
indicates the number of active streams, i.e, the number of antenna arrays
(subpartitions) over which will be split the total configured TX power.
`ReportCurrentCellRsrpSinr` extended to include the stream ID.
`NrUePhy::GetTxPowerSpectralDensity` has one more parameter that indicates the number
of active streams (antenna subpartitions or subarrays) over which the total TX
power will be split.

* Callback functions for the HARQ feedback are moved from `NrSpectrumPhy` to `NrUePhy`.
`NrUePhy` now has the the following functions: `SetPhyDlHarqFeedbackCallback` and
 `NotifyDlHarqFeedback`.

* Old struct that represents HARQ information, called `NrDlHarqProcessInfo` has
been replaced with `HarqProcessInfoSingleStream` that is used for the same purpose.
Now, new `NrDlHarqProcessInfo` contains a vector of `HarqProcessInfoSingleStream`,
one instance per spectrum phy (per antenna array or antenna subpartition).

* `DciInfoElementTdma` constructor is changed to support more streams, i.e.,
instead of scalar parameters for tbs, ndi and rv, now there are `std::vector`
parameters for the same purpose. `HarqStatus` enumeration extended to support
new `NONE` state, apart from previous `NACK` and `ACK`.

* `NrPhySap` function `SendMacPdu` now has one more parameter that indicates the
index of the NrSpectrumPhy (stream) which will be used for the transmission.
The default value of this parameter is 0, so this change should not affect
regular usage of this function call.

* APIs of `BeamformingHelperBase`, `RealisticBeamformingHelper` and
`IdealBeamformingHelper` are changed to allow beamforming per stream (antenna array of
`NrPhy`), and are also refactored to simplify function calls.

*`BeamManager` is installed per `NrSpectrumPhy` instance and not per `NrPhy` instance
as before. Each `BeamManager` takes care of a single antenna array or antenna
subpartition.

* APIs of `RealisticBeamformingAlgorithm` and all `IdeaBeamformingAlgorithm` child
classes have been updated accordingly.

### Changed behavior:

---

## Changes from NR-v1.2 to v1.3

### New API:

* `NrGnbNetDevice` now has function `GetCellIds` that returns the list of
cell IDs belonging to that gNB. This function is added for the
compatibility with the ns-3 LTE 810 MR.
* `NrUePhy` implements `DoGetCellId` and `DoGetDlEarfcn` for the API
compatibility with the ns-3 LTE 810 MR.

### Changes to existing API:

* `NrSpectrumPhy` and `BeamManager` have a function called `GetAntenna`
instead of `GetAntennaArray` that they had previously.
* `NrPhy`, `NrUePhy`, `NrGnbPhy`, do not have any more function called
`GetAntennaArray`.

### Changed behavior:

---

## Changes from NR-v1.1 to v1.2

### New API:

* Added "file-transfer-application.cc/h" and "file-transfer-helper.cc/h" that
implement File Transfer Protocol (FTP) applications. In addition, 3GPP FTP model
1 helper has been added that configures FTP Model 1 traffic model. These FTP and
3GPP FTP M1 implementations could be used by other modules, but they still need
some development, so for the moment they will be included in the nr module.
* New `NrHarqTest` is included to test HARQ-IR and HARQ-CC combining methods (thanks
to Andrey Belogaev).

### Changes to existing API:

* `m_numSymAlloc` field of `SlotAllocInfo` structure and the `usedSym` variable
in `NrMacSchedulerNs3::DoScheduleDlData` and `NrMacSchedulerNs3::DoScheduleUlData`
are now correctly updated when a UE does not get a DCI (i.e., when the TBS is less than 7 bytes).
* PHY traces have been modified to allow post-processing. Moreover, a simulation
tag can be added in order to generate different file names when Simulation Campaigns
are performed.
* `cttc-fh-compression.cc` example has been extended with a possibility to use
FTP traffic model.
* Extended APIs to compute SRS SNR and enable real BF based on SRS SNR measurements
(previously real BF was performed based on SRS SINR measurements, now it can be done
based on either SRS SNR or SRS SINR). In particular, `NrInterference` has been
extended to compute SNR of received data, a new callback is included in `NrSpectrumPhy`
to compute the SNR of SRS, and real BF algorithm has been extended to use SNR report.
The new attribute `UseSnrSrs` of `RealisticBeamformingAlgorithm` denotes whether
the SRS measurement will be SNR or SINR (and defaults to true, i.e., SNR is used).


### Changed behavior:

* If HARQ-IR is used, the performance may change because its error modeling uses
a new formula for the computation of the effective SINR.
* The available SRS offsets are correctly generated so that none of the UEs get
the same SRS offset.
* If realistic beamforming algorithm with trigger event configured with delay update
is used, the performance may change because it now uses the actual channel at SRS
reception moment for real BF update with delay.
* By default, SRS Symbols is set to 1 symbol. In the previous release, the default
value was 4 symbols.

---

## Changes from NR-v1.0 to v1.1

### New API:

* Added attribute "SrsSymbols" in `NrMacSchedulerNs3`, to indicate how many
  symbols are available to the SRS message.
* Added attribute "EnableSrsInUlSlots" in `NrMacSchedulerNs3` to allow SRS only
  in F slots, or both in F and UL slots.
* Added attribute "EnableSrsInFSlots" in `NrMacSchedulerNs3` through which SRS
  can be disabled in F slots. If SRS is disabled in both UL and F slots, then
  SRS will be completely disabled.
* Added new beamforming algorithm called `RealisticBeamformingAlgorithm`
  which determines the beamforming vector of the transmitter and receiver based
  on the SINR of SRS.
* Added `RealisticBeamformingHelper` that needs to be used when
  `RealisticBeamformingAlgorithm` is being configured in order to
  schedule beamforming updates and to collect the information of SRS SINR
  reports that are necessary for the `RealisticBeamformingAlgorithm`
  execution.
* Added attribute `PowerAllocationType` to `NrGnbPhy` and `NrUePhy`, which allows
  configuring the power allocation type. Currently, two types of power allocation
  are supported: uniformly over all bandwidth (all RBs) or uniformly over active
  (used) RBs.
* Added `NrUePowerControl` class that implements NR UL PC power control
  functionality. Support UL PC power control for PUSCH, PUCCH and SRS, and can
  operate in two different modes: TS 36.213 and TS 38.213 modes. Uplink power
  control is disabled by default.
* Proportional Fair scheduler in UL direction is added.
* New examples: cttc-nr-notching.cc and cttc-realistic-beamforming.cc
* New tests: nr-test-notching.cc, nr-uplink-power-control-test.cc, and
  nr-realistic-beamforming-test.cc


### Changes to existing API:

* Added `NrHelper::AssignStreams` function that is a central function in NR module for
  assigning streams. The exceptions are:
  `HexagonalGridScenarioHelper` and `GridScenarioHelper` whose randomness is not
  controlled from NrHelper::AssignStreams function, but instead it needs to be called
  separately if one wants to assign streams for scenarios. Also, class `RealisticBeamformingAlgorithm`
  is not controlled from `NrHelper::AssignStreams`, but it should be from future upgrades.
  To allow assigning streams and control randomness in NR module all functions that have
  some random variables are extended with `AssignStreams` function. Classes that are extended
  include: `NrUeMac`, `NrSpectrumPhy`, `NrMacSchedulerSrsDefault` (also parent classes
  `NrMacSchedulerNs3` and `NrMacScheduler`), `RealisticBeamformingAlgorithm`, `NrHelper`,
  `GridScenarioHelper`, `HexagonalGridScenarioHelper`.
* Added functions InH_OfficeOpen_nLos, InH_OfficeOpen_Los, InH_OfficeMixed_nLoS,
  InH_OfficeMixed_LoS, InitUmaBuildings, InitUmiBuildings, InitV2VHighway,
  InitV2VUrban channel modeling in cc-bwp-helper.h/cc and nr-helper.cc
* Attribute `IdealBeamformingMethod` of `IdealBeamformingHelper` class
  is renamed to `BeamformingMethod`
* The scheduler now support the setting of the notched mask, through
  void SetDlNotchedRbgMask (const std::vector<uint8_t> &dlNotchedRbgsMask);
  for the DL and
  void SetUlNotchedRbgMask (const std::vector<uint8_t> &ulNotchedRbgsMask);
  for the UL
* Added != operator function to `BeamId` class.
* IPV6 is now supported. That is, the end-to-end connections between
  the UEs and the remote hosts can be IPv4 or IPv6. The classes,
  `NrNetDevice`, `NrGnbNetDevice`, and `NrUeNetDevice` are updated accordingly.
* Previously we had two overloaded functions `GetSpectrumModel` one was with
  the parameters: double bandwidth, double centerFrequency, uint8_t numerology
  and another with parameters: uint32_t numRbs, double centerFrequency,
  double subcarrierSpacing. These two functions are unified, and only the latter
  one is left.
* `nr-phy.h` does not contain any more default configurations related to the
  bandwidth. Default configurations is now set for UE by `DoSetInitialBandwidth`,
  and for gNB it should be set with UpdateConfig call.
* Initial bandwidth at UE depends on the numerology and it will be set to 6 RBs until
  configured through RRC.
* The example lte-lena-comparison.cc has been extended with more command line
  parameters.
* Some tests are renamed so that all tests start with prefix `nr-`.


### Changed behavior:

* If a notching mask is set, the scheduler will avoid to allocate the RBG in
  which the mask (a vector of integers) is set to zero.
* When PF scheduler is configured, UL is also PF (previously RR UL was considered)
* Newly added `NrHelper::AssignStreams` function may change default stream
  assignment that was being assigned previously in NR examples. All examples
  are updated to use `NrHelper::AssignStreams` to fix random streams in NR module.
* By default is configured power allocation over active RBs. Before this release, by
  default was uniform power allocation over all RBs. Power allocation type can be
  configured by using `PowerAllocationType` attribute of `NrGnbPhy` and `NrUePhy`.
* Newly added SRS allocation, transmission and reception will occupy periodically
  4 symbols in a slot over a certain periodicity, in UL or F slots.
* If real beamforming is configured, then the beamforming update will be based
  on SRS SINR reports, instead of with a given periodicity (as it was before
  in the case of ideal beamforming).

---

## Changes from NR-v0.4 to v1.0

### New API:

* Added `ReportPowerSpectralDensity` trace source in NrUePhy.
* Added `SlotDataStats` and `SlotCtrlStats` trace source in NrGnbPhy.
* Added `RBDataStats` trace source in NrGnbPhy.
* Added `RxDataTrace` trace source in NrSpectrumPhy.
* Extended the GridScenarioHelper with the introduction of the
  HexagonalGridScenarioHelper class. This new class creates a hexagonal grid
  deployment consisting of up to 19 sites with 3 sectors of 120º each,
  resulting in 57 hexagonal cells. The location of sites and UEs can be
  represented with gnuplot.
* Added GetSpectrumModel to the SAP interface between PHY and MAC, so the CQI
  calculation can see that value.
* Added `Pattern` attribute to NrGnbPhy, to set the TDD Pattern.
* Added IdealBeamformingHelper class that is used to configure ideal beamforming
  algorithm and the periodicity of performing the ideal beamforming methods.
  Attribute `IdealBeamformingMethod` will be used to configure the beamforming
  algorithm, and `BeamformingPeriodicity` to configure the beamforming
  periodicity.
* Added IdealBeamformingAlgorithm class and its subclasses that are used to
  configure beamforming vectors for the pairs of devices, normally between gNB
  and UE, but is possible to use it in future for UE to UE.
* Added BeamManager class at gNB and UE phy that is responsible for caching
  beamforming vectors to use when communicating with connected devices. It is
  also responsible for configuring quasi-omni beamforming vector for omni
  transmissions. This class should be used by gNB and UE to control its
  ThreeGppAntennaArrayModel instances.
* Added attribute `NumRefScPerRb` in NrAmc, to indicate the number of reference
  subcarriers per RB.


### Changes to existing API:

* Functions `NrGnbPhy::ReceiveUlHarqFeedback` and `NrUePhy::ReceiveLteDlHarqFeedback`
  are renamed to `NrGnbPhy::ReportUlHarqFeedback` and `NrUePhy::EnqueueDlHarqFeedback`, respectively.
* Removed attribute `NumberOfComponentCarriers` and `UseCa` from NrHelper.
* Removed `SetScheduler` method from NrHelper.
* NrUePhy includes now traces for the DL DCI and the corresponding DL HARQ Feedback
* Renamed MmWaveEnbNetDevice attribute `ComponentCarrierMap` into BandwidthPartMap
* Removed `CentreFrequency` attribute from MmWavePhyMacCommon: it is now Set
  by the CcBwpHelper while dividing the spectrum.
* Removed `N0Delay` and `N1Delay` attributes from MmWavePhyMacCommon: they are
  now in NrGnbPhy.
* Removed `TbDecodeLatency` attribute from MmWavePhyMacCommon: it is now
  an attribute of `NrGnbPhy` and `NrUePhy`.
* Moved `NumRbPerRbg` attribute from MmWavePhyMacCommon to NrGnbMac.
  This attribute is still needed by UE PHY, thus for the moment this attribute
  cannot be reconfigured once being set. Once that DCI bitmask is changed to
  work per the RB granularity, this attribute can be reconfigured.
* Removed `CellId` attribute from NrGnbNetDevice.
* Removed `CcId` attribute from MmWavePhyMacCommon.
* Removed `MacSchedType` attribute from MmWavePhyMacCommon.
* Removed `Bandwidth`, `Numerology`, `SymbolsPerSlot` attribute from
  MmWavePhyMacCommon. These attribute are now of NrGnbPhy, while the UEs get
  these values from RRC or from direct call by the helper at the attaching
  moment.
* Removed `NumHarqProcess` and `HarqDlTimeout` from MmWavePhyMacCommon.
  `NumHarqProcess` is now attribute of NrGnbnbMac and NrUeMac, while
  `HarqDlTimeout` is completely removed since the timeout is equivalent to the (maximum) number of harq processes.
* The number of DL and UL CTRL symbols can be configured now in
  NrMacSchedulerNs3 through the attributes `DlCtrlSymbols` and `UlCtrlSymbols`.
* Removed attribute `L1L2CtrlLatency`, and fixed it to 2 in NrPhy.
* Removed `EnableAllInterferences` attribute from MmWaveSpectrumPhy which was
  used to enable or disable interference calculations for all links.
* Replaced the message DCI_TDMA with UL_DCI and DL_DCI, to differentiate among
  the allocations.
* The EESM error model class has been separated, and the attributes HarqMethod
  and McsTable removed. Now, to configure different error model based on EESM,
  you have to specify directly the Harq method and the table name in the
  error model name: the new classes are NrEesmIrT1, NrEesmIrT2, NrEesmCcT1,
  NrEesmCcT2.
* The AMC inside the scheduler has been separated into an UL and a DL part.
  Methods in the helper have been added to help the user configuring that part.
  In particular, have a look at SetGnbDlAmcAttribute and SetGnbUlAmcAttribute.
* Removed attribute `Ber` in NrAmc, which is now set based on the error model
  in use.
* Renamed all mmwave- classes, tests, examples, helpers, to nr-.

### Changed behavior:

* K0, K1, K2 Delays are removed from the phy-mac common, instead they are
  implemented as parameters of the DCI. In the DCI message, the gNb reports the
  K{0,1,2} delay instead of the sfn number.
* UE receives DL data according to K0 and sends UL data according to K2
  (passed from the gNb in the DL and UL DCI, respectively).
* UE schedules the DL HARQ Feedback according to K1 delay (passed from the
  gNb to the UE in the DL DCI).
* Gnb first schedules UL and then DL.
* RACH Preamble is sent without applying any delay (i.e. L1l2CtrlLatency)

---

## Changes from NR-v0.3 to NR-v0.4

### New API:

### Changes to existing API:

* Replaced SINR-BLER curves for MCS Table1 and Table2 of the NR error model
  with more accurate (and complete) values.

### Changed behavior:


---

## Changes from NR-v0.2 to NR-v0.3

### New API:

* Added NrErrorModel as interface for any error model. It is used by NrAmc and
MmWaveSpectrumPhy.
* Added attribute *ErrorModelType* to the class NrAmc, which must be kept in
sync with the attribute *ErrorModelType* of MmWaveSpectrumPhy
* Added attribute *ErrorModelType* to the class MmWaveSpectrumPhy
* Added class NrEesmErrorModel to model the NR PHY abstraction according to LDPC
coding, block segmentation, and including MCS/CQI table 1 and 2. It has an attribute
to select the HARQ method (HarqCc or HarqIr) and another attribute to select the
MCS/CQI table of NR to be used (McsTable1 or McsTable2). In this release, the
BLER-SINR are not completed yet, and so it is recommended not to use this model.
* Added attributes AntennaNumDim1, AntennaNumDim2, AntennaArrayType to
MmWaveEnbPhy and MmWaveUePhy. And these two classes are responsible for an
antenna array installation. This means that there is an instance of
AntennaArrayBasicModel per MmWavePhy.
* MmWavePhy has new functions Set/Get functions for configuring and
obtaining AntennaNumDim1, AntennaNumDim2, AntennaArrayType attributes. The
configuration of these parameters is allowed through MmWavePhy object since
we would like to have access to these attributes through the Config system
and that we can specify different values for an antenna for MmWaveUePhy and
MmWaveEnbPhy. Function to set AntennaArrayType can be called only for the
initial configuration. If it is called after the antenna object is created the
program will be halted and the user will be informed of the error.
* AntennaArray3gppModel has new attribute RandomUeorientation. If set to true
the 3D antenna orientation will be generated randomly throwing a dice by
using a UniformRandomVariable.
* MmWave3gppChannel has a new attribute UpdateBeamformingVectorsIdeally which
determines if the update of the beamforming vectors will be ideal, i.e. if
the beamforming vectors will be adjusted every time that the channel is updated.
* AntennaArrayBasicModel has a new function GetBeamformingVectorUpdateTime
that returns the last time at which the beamforming vector toward a specified
device has been updated.
* EnbPhy now can perform beamforming, in a very initial form (every connected ue
will perform the beamforming, and it will take 0 simulated time). The periodicity
of this beamforming depends on the value of the new attribute *BeamformingPeriodicity*
of the class MmWaveEnbPhy
* MmWaveSpectrumPhy now has two more traces: TxCtrlTrace and TxDataTrace, to extract
information about how the channel is occupied
* Now MmWaveSpectrumPhy can enable interference for all links, also UE->UE and GNB->GNB,
configured through the attribute EnableAllInterferences.
* Now MmWave3gppChannel can enable generation of propagation losses for all links,
also UE->UE and GNB->GNB, configured through the attribute EnableAllChannels. This
feature, nowadays, applies *only* to Indoor Hotspot scenarios, for which the 3GPP pathloss
model is valid.
* Introduced Listen-Before-Talk after MAC. The interface of a channel access manager
is inside the file *nr-ch-access-manager.h*.

### Changes to existing API:

* Renamed MmWaveAmc into NrAmc
* Renamed MmWaveMiErrorModel in NrLteMiErrorModel, and adapted it to the new
NrErrorModel interface
* MmWave3ppChannelModel does not anymore have an attribute for
MmWavePhyMacCommon class and thus it does not have any more the corresponding
functions to Set/Get the corresponding instance, SetConfigurationParameters
and GetConfigurationParameters, respectively. It instead has a new attribute
that needs to be configured at the beginning of the simulation.
This attribute is Bandwidth. The two most important attributes of this class
are now CenterFrequency and Bandwidth.
* AntennaArrayBasicModel has now a pointer to a SpectrumModel.
* MmWaveSpectrumValueHelper.
* Common functions of MmWaveEnbPhy and MmWaveUePhy,
CreateTxPowerSpectralDensity and CreateNoisePowerSpectralDensity are moved to
MmWaveUePhy and renamed to GetNoisePowerSpectralDensity and
GetTxPowerSpectralDensity to distinguish them from the original functions that
belong to MmWaveSpectrumValueHelper.
* MmWaveSpectrumValueHelper does not depend any more on MmWavePhyMacCommon
class. Functions that had up to know this class as an input, now have the
minimum subset of parameters that are used in the corresponding function.
* MmWaveSpectrumValueHelper does not have anymore empty function for
CreateTxPowerSpectralDensity that had as an input a power map (powerTxMap).
* MmWave3gppChannel can now be used by any other module, it is not any more
only mmwave specific spectrum propagation model. This means that any subclass
of NetDevice can be attached to a channel using this SpectrumPropagationModel.
An additional requirement is that the technology uses AntennaModel that is
implementing AntennaArrayBasicModel interface. The
dependencies from mmwave module-specific classes are removed, e.g. dependency on
MmWaveEnbNetDevice, MmWaveUeNetDevice, MmWaveUePhy, MmWaveEnbPhy.
* Removed MmWaveMacSchedulerNs3 attribute McsDefaultDl/McsDefaultUl. Its
functionality is now taken by the attribute StartingMcsDl/StartingMcsUl
* Renamed MmWavePointToPointEpcHelper into NrPointToPointEpcHelper. All
attributes that before were in the form "ns3::MmWavePointToPointEpcHelper"
must now be referred to "ns3::PointToPointEpcHelper". In fact, NrPointToPointEpcHelper
is now inheriting from PointToPointEpcHelper.
* AntennaArrayModel has the beamforming vector that emulates omni reception and
transmission.
* Removed attribute *BeamformingEnabled* and *UpdateBeamformingVectorsIdeally*
from MmWave3gppChannel.
* Removed attribute *EnbComponentCarrierManager* from MmWaveHelper. The only allowed
CCManager is now BwpManagerGnb.
* Removed attribute *UeComponentCarrierManager* from MmWaveHelper. The only allowed
manager is now UeManagerGnb.
* Removed attribute *Bandwidth* from MmWave3gppChannel.

### Changed behavior:
* BeamSearchBeamforming and LongTermCovMatrixBeamforming functions of
3gppChannelModel can now be called from the outside of this class. This means
that the update of the beamforming vectors can be triggered at any time and
does not have to be related to the channel update event.
* TxPower default value for a UE passed from 30.0 dBm to 2.0 dBm
* TxPower default value for a gNb passed from 30.0 dBm to 2.0 dBm
* MmWave3gppChannel supports calculation of the beamforming gain when one of
devices is in omni mode. This is possible because AntennaArrayModel has new
beamforming vector that emulates omni reception and transmission.
* The Slot Indication is asked to the MAC at the beginning of the slot, before
doing everything else at PHY level.
* UE and ENB PHY does not rely on the order of the allocation to understand if
an allocation is DL/UL, DATA/CTRL. Also, we support now the possibility to start
an allocation at a symbol != 0.
* The scheduler is now informed of RACH messages, even if (for the moment) it
does nothing to allocate space for UL RRC message, it reserve some ideal
space in the DL CTRL link to send the RAR response.
* The UE will ignore the UL CTRL slot if there are no messages to send. Practically speaking,
the UE will not set power on the RBG, not generating interference.
* The GNB will not put power during the DL CTRL symbol if there aren't messages
to send to the UEs.
* The control message timings are implemented in such a way that the delay can
be adjusted in a flexible manner. In particular, when a control message is passed
from PHY to MAC or from MAC to PHY, the user can adjust the delay according to each
release specifications. As default we apply a delay of 1 ms, except of some cases
(e.g. RAR msg), where the delay is set to 2 ms.

---

## Changes from NR-v0.1 to NR-v0.2

### New API:

* A new AntennaArray3gppModel is introduced that inherits all the features of the AntennaArrayModel, but it considers 3GPP directional antenna elements instead of ISO antenna elements.
* 3gppChannelModel has a new attribute "speed" for configuring the speed. Previous and currently the default behaviour is that 3gppChannelModel calculates the relative speed between the transmitter and the receiver based on their positions. However, this parameter can be configured when the static scenario is being used but is desired to imitate small scale fading effects that would exist in a mobile scenario.
* New traces sources are added to the Interference class for collecting the SNR and RSSI values.

### Changes to existing API:

* The supported bearers QCI are only the ones defined in Rel. 15.
* The Ue NoiseFigure is not anymore hard-coded value, instead can be configured through the new MmWaveUePhy attribute called "NoiseFigure".
* MmWaveEnbDevice and MmWaveUeDevice have new attributes AntennaNumDim1 and AntennaNumDim2 that allow the configuration of a rectangular antenna. Previously only squared antenna was supported.
* GnbAntennaArrayModelType and UeAntennaArrayModelType attributes are added to MmWaveHelper that allow the configuration of the antenna type (ISO vs 3gppp) of gNb and UE, respectively.
* AntennaArrayModel has a new attribute AntennaOrientation that can be used to configure the antenna orientation which can be horizontal in XY plane, or vertical in ZY plane.
* The attributes _MmWaveEnbNetDevice::MmWaveEnbPhy_ and _MmWaveEnbNetDevice::MmWaveEnbMac_ have been removed as they are replaced by the component carrier map. GetPhy () and GetMac () for UE and gNB NetDevices are now deleted, instead it is necessary to use the version with the ccId.
* Times in _PhyMacCommon_ class are now expressed with ns3::Time.
Removed attribute _PhyMacCommon::WbCqiPeriod_ as it was unused.

### Changed behavior:
* The SR is now sent only for Component Carrier ID == 0. The Enb Mac will receive it and forward to the CC manager; then, the CCM will route it to the most appropriate MAC, so the assignation for the SR will be done on the appropriate CC.
* In any given slot, the scheduler will now ignore UEs with new data to transmit if it assigned resources for retransmitting an HARQ process.
*  Default value of the UpdatePeriod parameter of the MmWave3gppChannel is changed to be 1 ms. Note that this dramatically slows down the simulation execution since the channel matrix will be updated every 1 ms. Previously, the default parameter was 0, which means that unless configured differently the channel matrix was not being updated.
* Previously there was a single propagation loss model, no matter how many BWPs there are in the simulation. Now, there is a propagation loss model instance per BWP. Long story short. MmWaveHelper instantiates a channel instance per BWP. The channel of each BWP is configured by using its own instance of the MmWavePhyMacCommon channel parameters. Since different BWPs are on different frequencies each BWP shall have its own propagation loss model that is configured with the frequency of the corresponding BWP.
* The default value of the UpdatePeriod parameter of the MmWave3gppChannel is returned to 0 ms. This is because it is detected that there are many occasions when the update of the channel matrix is not needed in the simulation example or the test, hence when the update is enabled by default the execution time of these simulations unnecessarily is slowed down.
* The SINR traces in MmWaveSpectrumPhy are now printed based on the TB of the UE, and not the entire band as it was before.
* The scheduler now assigns the maximum amount of data that is possible to the various LCs. Before, if there was only one LC with only 3 bytes, the assignation would have been of 3 bytes, despite of the TBS. Now, all the TBS can be assigned to him. We also manage cases in which multiple LC or LCG are active, but (as before) only in a round-robin fashion.

---

## Changes from NR-v0.0 to NR-v0.1

We did not track changes since the beginning. However, the main features we added to the mmWave module (from which we forked) are the FDM of numerologies and a new and improved MAC scheduling layer.

### New API:

* ND

### Changes to existing API:

* ND

### Changed behavior:

* ND
