# ntn-traffic

> The traffic + measurement backbone of the toolkit: an ORAN-NTN QoS-flow application suite with an in-band measurement header, an AI-native KPM flow monitor, and a helper that stands up a **real mmwave NR NTN cell** (SpectrumPhy + MAC + RLC/PDCP + RRC + EPC) under SGP4 satellite mobility — plus the classic 3GPP HTTP / NRTV / CBR traffic models.
> Part of **ns3-ntn-toolkit** — [README](../../README.md) / [INSTALL](../../INSTALL.md).

## Overview

`ntn-traffic` is the module every other toolkit module composes for a real data plane. It now ships three layers:

1. **`NtnRealStackHelper`** — builds a genuine NR-style air interface from the in-tree `mmwave` module (SpectrumPhy, MAC scheduler, HARQ, AMC, RLC/PDCP, RRC, EPC) between caller-supplied satellite (gNB) and ground (UE) nodes that carry their own mobility models (SGP4, TR 38.811, HAPS). The link is NTN-ized: free-space path loss valid at LEO range, a mmWave-NR PHY at an S-band carrier (FR2 numerology — 60 kHz SCS — **not** a 3GPP NR-NTN FR1 band/numerology; the 2.0 GHz carrier has no assigned 3GPP band number, sitting in the n256 uplink, and the 50 MHz default bandwidth exceeds the 20/30 MHz NTN-FR1 max), the "satellite EIRP" applied as a conducted Tx-power scalar to a terrestrial 8×8 `UniformPlanarArray` gNB with SVD beamforming (array gain added separately by the spectrum model — **not** a reflector beam with a 3 dB footprint or a TR 38.821-derived link budget), HARQ off by default. Every headline KPI is **measured**, not computed: SINR/TBLER from the PHY `RxPacketTraceUe` trace, throughput/delay/jitter/loss from in-band header bytes at the application sinks.
2. **The ORAN application suite** (`NtnOranApplication`, `NtnOranSink`, `NtnOranPayloadHeader`, `NtnCommandAndControlApp`) — QoS-flow-aware traffic sources whose every packet carries its 5QI / S-NSSAI / QFI identity and measurement primitives (sequence number, TX timestamp) as **real serialized bytes**, so per-flow KPIs survive GTP re-encapsulation through the EPC. This suite replaces bare `OnOffApplication` as the standard traffic source across the toolkit's modules and examples.
3. **`NtnOranAiFlowMonitor`** — an AI-native measurement layer over the ORAN flows, built on the real ns-3 FlowMonitor infrastructure: per-flow KPM time series under official 3GPP TS 28.552 / O-RAN E2SM-KPM metric names, sliding-window AI feature vectors, EWMA z-score anomaly events, and XML/CSV/InfluxDB/E2 exporters.

The module also retains NTN-oriented traffic generators with NTN-appropriate defaults: the **3GPP HTTP** model (over a satellite bent-pipe / regenerative link), the **NRTV** near-real-time video model over TCP/UDP, a CBR application, a legacy `TrafficTimeTag` for per-packet latency tracking (superseded by the in-band header timestamp), and the lightweight `NtnRealisticTrafficHelper` point-to-point data plane used by earlier examples.

## What's new (June 2026 — AI-Native ORAN-NTN update)

See the toolkit [CHANGELOG](../../CHANGELOG.md).

- **ORAN application suite** with the in-band `NtnOranPayloadHeader` (24-byte wire header), six 3GPP traffic profiles with 5QI-correct defaults, a measuring sink (one-way delay, RFC 3550 jitter, sequence-gap loss, throughput from received bytes), and a platform command-and-control telemetry app with a battery model.
- **`NtnOranAiFlowMonitor`** — ORAN-flow classifier/probe pair plugged into the real ns-3 `FlowMonitor`, KPM series under TS 28.552 names, AI feature windows, anomaly detection, four exporters.
- **`NtnRealStackHelper`** — the real-radio NTN cell helper with `InstallOranFlow()`, `EnableOranFlowMonitor()`, a satellite **payload-option model** (Transparent / RegenerativeRu / RegenerativeRuDu / FullGnb, each with its own user-plane delay composition), and `SetFeederGeometry()` for live feeder-delay updates from the actual satellite/gateway mobility.
- **TR 38.821 calibration example** validating the measured radio against the official 3GPP link-budget methodology (Set-1 LEO-600, S-band, handheld UE).
- `OnOffApplication` is no longer used by toolkit module examples; `NtnOranApplication` QoS flows are the standard traffic source (the legacy `NtnRealisticTrafficHelper` P2P path still uses OnOff internally).

## Models, helpers & key classes

### ORAN application suite (`model/`)

- **`NtnOranPayloadHeader`** — the in-band user-plane header carried by every ORAN packet as real bytes (so QoS identity and measurement primitives ride *inside* the GTP tunnel, immune to FlowMonitor byte-tag stripping). Wire format (24 bytes, network order): version (u8), payload type (u8), per-flow sequence number (u32), TX timestamp in ns (u64), 5QI (u8, TS 23.501), S-NSSAI SST (u8) + SD (u24, TS 23.003), QFI (u8, 6 bits used, TS 38.415), srcId (u16), dstId (u16). Payload types: `EMBB_VIDEO`, `URLLC_CMD`, `MMTC_READING`, `CNC_TELEMETRY`, `KPM_REPORT`, `FH_SAMPLE`, `VOICE`, `BACKGROUND`.
- **`NtnOranApplication`** — attribute-driven traffic source with TS 23.501 Table 5.7.4-1 profiles, each presetting 5QI/QFI/payload type and cadence (explicit attributes override): `CONVERSATIONAL_VOICE` (5QI 1, deterministic 20 ms vocoder cadence), `EMBB_VIDEO` (5QI 2, frame bursts at `FrameRate`, bytes from `DataRate`), `URLLC_PERIODIC` (5QI 82, small deterministic-period commands), `MMTC_PERIODIC` (NB-IoT-style periodic sensor reports), `POISSON_BACKGROUND` (5QI 9, exponential inter-arrivals), `CBR_SATURATING` (back-to-back CBR at `DataRate`, eMBB full-buffer). `SetFlowIdentity(fiveQi, sst, sd, srcId, dstId)` stamps the slice/QoS identity.
- **`NtnOranSink`** — measuring server. Per QoS flow (keyed by the in-band srcId + 5QI + S-NSSAI) it computes one-way delay (RX time minus in-band TX timestamp), RFC 3550 interarrival jitter, loss from sequence-number gaps, and throughput from received bytes over the flow's active span — all measured at the application from bytes that crossed the real radio and GTP tunnel. Also parses `NtnCncTelemetry` records (`GetLatestTelemetry`).
- **`NtnCommandAndControlApp`** — platform C&C telemetry as a real application: each period it samples the node's real ns-3 `MobilityModel` (SGP4 satellite / HAPS / UAV), derives attitude from the velocity frame, updates a linear battery model (`BatteryCapacityWh` / `PowerDrawW`), and sends the telemetry record after an `NtnOranPayloadHeader` (payload type `CNC_TELEMETRY`, 5QI 69 mission-critical signalling). Late or lost C&C is a real failure mode.
- **`NtnStaticExtraLossModel`** — a runtime-settable scalar excess loss (dB, negative = gain) as a real `PropagationLossModel`, for chaining onto the live channel via `NtnRealStackHelper::AddExtraPropagationLoss()`; lets scenario events (blockage, RIS engage/release, beam-pointing loss) show up in the *measured* SINR/TBLER.

### AI-native flow monitor (`model/ntn-oran-ai-flow-monitor.h`)

- **`NtnOranFlowClassifier`** (`: FlowClassifier`) — flows keyed by the ORAN QoS identity in real packet bytes (srcId + dstId + 5QI + S-NSSAI), the way a DRB/QoS flow is keyed in TS 38.415, instead of the IP 5-tuple.
- **`NtnOranFlowProbe`** (`: FlowProbe`) — one probe per measurement point (traffic source, each sink); reports every packet into a real `ns3::FlowMonitor` with the in-band sequence number as packet id.
- **`NtnOranAiFlowMonitor`** — the KPM/AI layer: per-flow KPI time series at a configurable granularity period (default 1 s) under official names — `DRB.UEThpDl`, `DRB.RlcSduDelayDl`, `DRB.PacketLossRateDl`, `DRB.PdcpSduVolumeDl`, plus `L1M.RS-SINR` / `TB.TotNbrDl` / `TB.ErrTotNbrDl` from the PHY trace when attached to `NtnRealStackHelper`; sliding-window AI feature vectors (`GetFeatures`: mean/slope of throughput, delay, loss, SINR, plus jitter) for xApps / `ns3-ai-ntn`; an EWMA z-score anomaly detector per flow per metric (`RegisterAnomalyCallback`); exporters: FlowMonitor XML (`SerializeToXmlFile`), wide CSV (`WriteCsv`), InfluxDB line protocol (`WriteInfluxLp`), and E2SM-KPM-shaped indication callbacks (`RegisterE2Consumer`) for the `oran-ntn` E2 node.

### Real-radio helper (`helper/ntn-real-stack-helper.h`)

- **`NtnRealStackHelper`** — the headline API for radio-true scenarios:
  - Config (before `Build`): `SetSimTime`, `SetOutputDir`, `SetRunTag`, `SetCarrierFrequencyHz` (default 2 GHz S-band), `SetBandwidthHz`, `SetSatEirpDbm`, `SetUeTxPowerDbm`, `SetBackhaulDelay`, `SetHarqEnabled` (off by default — terrestrial HARQ timers break over the slant), `SetNtnHarqProfile(true)` (optional NTN-stretched HARQ profile: turns HARQ on and rescales the only HARQ timing knobs mmwave exposes — `MmWavePhyMacCommon::HarqDlTimeout` and `NumHarqProcess` — from the slant geometry of the nodes passed to `Build()`, budgeting 4 HARQ rounds over the radio round trip, LEO-600 one-way ~2.2 ms at zenith; the default remains exactly today's behavior, HARQ off), `SetRlcAmEnabled`, `SetUplink`, `SetGates`, `SetStrictGates`.
  - **Payload options** (`SetPayloadOption`): user-plane extra delay composed from the live feeder slant range — `Transparent` (bent pipe, 2 × slant/c), `RegenerativeRu` (O-RU on sat, Open-FH 7.2x over the feeder: slant/c + 0.25 ms), `RegenerativeRuDu` (O-DU on sat, F1 midhaul: slant/c + 0.15 ms), `FullGnb` (full gNB on sat, GTP backhaul: slant/c + 0.05 ms — the default). `SetFeederGeometry(satMobility, gwMobility)` re-evaluates the EPC backhaul delay every second from the real feeder geometry; `ComputePayloadExtraDelay(slantRangeM)` exposes the model.
  - **Uplink timing — NTN K_offset consumption** (`SetKOffsetConsumption(bool)`, default OFF, nr backend): when on, `Build()` derives the NTN cell-specific K_offset from the *service-link round-trip* geometry — `ComputeKOffsetSlots()` = `ceil(RTT / slot) + 1`, matching the SIB19 `cellSpecificKoffset` derivation — and programs it into the scheduler's UL DCI→PUSCH gap by extending `NrGnbPhy::N2Delay` (TS 38.213 §4.2). This pushes each UE's uplink grant past the round trip so the delayed downlink no longer collides with the UE's uplink slot ("Cannot TX while RX"), and is the unlock that lets `SetAirInterfaceDelay(true)` run **single-UE / downlink-heavy uplink** under a real air-interface propagation delay. `GetConsumedKOffsetSlots()` returns the K_offset applied (0 when off); `GetGnbN2Delay(gnbIdx, bwp)` reads back the N2Delay actually programmed on a built gNB PHY (= base N2Delay + consumed K_offset). **Scope/honesty:** this consumes the populated K_offset into UL PUSCH timing only; *fully* enabling multi-UE uplink additionally needs per-UE Timing Advance and NTN-aware SRS/PUCCH timing that the vendored nr v3.3 lacks (deferred to the ns-3.48 / nr v5.0 migration). It required raising the vendored nr `N2Delay` cap from 4→320 slots (ntn-patch 05 in `contrib/nr/ntn-patches/`).
  - Build & traffic: `Build(gnbNodes, ueNodes)` (both must already carry a `MobilityModel`), `InstallTraffic(profile, start, stop)` with pre-canned mixes (`NbIotPeriodic`, `EmbbStreaming`, `UrllcPings`, `ConversationalVoice`, `MixedBouquet`) — all backed by `NtnOranApplication` flows — or `InstallOranFlow(ueIdx, fiveQi, sst, sd, profile, start, stop)` for one explicit QoS flow with full slice identity.
  - Measurement: **`EnableAiFlowMonitor(outputPrefix)` is the canonical KPM wiring** — one call (any time after `Build()`, before or after installing traffic) creates the `NtnOranAiFlowMonitor`, wires the PHY source, attaches every flow the helper installed (including flows installed later), and auto-exports `<outputPrefix>_kpm_series.csv` / `.lp` at end of simulation; `GetAiFlowMonitor()` returns the monitor for anomaly callbacks / E2 consumers. The older `EnableOranFlowMonitor()` (manual attach + manual export, must be called after traffic install) is kept for existing callers. `Collect()` aggregates after `Simulator::Run()`; accessors `GetMeanDlSinrDb`, `GetMeanDlTbler`, `GetDlCorruptFraction`, `GetRxThroughputMbps`, `GetMeanDelayMs`, `GetMeanJitterMs`, `GetAppLossRatio`, `GetCellMeanSinrDb`, and per-UE `GetUeRnti` / `GetUeRecentSinrDb` / `GetUeMeanSinrDb` / `GetUeRecentTbler` / `GetUeRxBytes` for CHO/RIC/slice logic.
  - Extensibility: `AddExtraPropagationLoss(Ptr<PropagationLossModel>)` chains module physics (THz absorption, Sionna RT, A2G TR 38.811) onto the real channel; `RegisterPeriodicCallback(period, cb)` runs module logic on the real event queue; `GetMmWaveHelper` / `GetEpcHelper` / `GetRemoteHost` / device containers for custom wiring.
  - `WriteHealthReport()` emits `sim_health.csv` whose `HealthGates` assert trace provenance: minimum transport blocks decoded at the UE PHY, a measured app-throughput floor, SINR-from-PHY-trace and active-error-model requirements (strict mode fails the run on a missed gate). Its `air_interface` tag is **derived from the actual carrier** (backend `nr`/`mmwave` × band `fr1`/`fr2`/`thz` per TS 38.104: FR1 ≤ 7.125 GHz, FR2 ≤ 71 GHz, `thz` above) rather than a hardcoded `fr1`, so e.g. a THz example at 100 GHz is labelled `nr-thz-ntn` instead of mislabelling as FR1.

### Lightweight P2P data plane (`helper/ntn-realistic-traffic-helper.h`)

- **`NtnRealisticTrafficHelper`** — the original helper that drops a UDP data plane over point-to-point links (InternetStack + per-UE P2P + FlowMonitor) into any scenario so `Simulator::Run()` exercises packets instead of returning instantly. Config (`SetSimTime`, `SetProfile`, `SetGates`, ...), `InstallUes(n)` or `AttachExistingUes(...)`, then `Wire()`; `RegisterPeriodicCallback` hooks; `UpdateUeLink(ueIndex, oneWayDelay, per)` couples per-UE link delay and packet-error rate to live orbital geometry (see `doc/DYNAMIC_LINK_COUPLING.md`); `WriteHealthReport()` emits `sim_health.csv` with event-activity gates. Profiles: `NbIotPeriodic`, `EmbbStreaming`, `UrllcPings`, `DigitalTwinTelemetry`, `MixedBouquet` (default). No radio physics — for radio-true KPIs use `NtnRealStackHelper`.

### Classic traffic models (`model/`, `stats/`)

- 3GPP HTTP (`three-gpp-http-satellite-client`, `three-gpp-http-variables`) with `three-gpp-http-satellite-helper`; NRTV (`nrtv-header`, `nrtv-tcp-client`, `nrtv-tcp-server`, `nrtv-udp-server`, `nrtv-variables`, `nrtv-video-worker`) with `nrtv-helper`; `cbr-application` with `cbr-helper`. Application KPI collectors under `stats/` (`ApplicationStatsHelper` delay/throughput family).
- **Legacy / experimental** (kept for compatibility, not exercised by any current example or test): `TrafficTimeTag` (per-packet latency byte-tag — superseded by the in-band `NtnOranPayloadHeader` timestamp, which survives GTP re-encapsulation) and `HistogramPlotHelper` (gnuplot histogram helper). Plotting helper `client-rx-trace-plot` is still used by the NRTV example. Prefer `NtnOranSink` / `NtnOranAiFlowMonitor` for new measurement code.

## Examples

Listed in `examples/CMakeLists.txt`; all build to `build/contrib/ntn-traffic/examples/`.

### ntn-oran-qos-flows

The flagship ORAN-NTN example. One real LEO cell (`NtnRealStackHelper`: mmwave SpectrumPhy + MAC + RLC/PDCP + RRC + EPC, SGP4 satellite at zenith receding with genuine dynamics) carries four 3GPP QoS flows side by side, each an `NtnOranApplication` with its own 5QI / S-NSSAI in real packet bytes — UE0 5QI 1 conversational voice, UE1 5QI 2 eMBB video bursts, UE2 5QI 82 URLLC commands, UE3 5QI 9 mMTC reports — plus the satellite's own C&C telemetry (5QI 69) flowing to an SMO endpoint over a feeder link with the physical zenith delay. UEs move under TR 38.811 mobility classes (`NtnTr38811MobilityHelper`, MixedContinental). The AI-native KPM layer runs over all four flows via the canonical `EnableAiFlowMonitor("ntn-oran-qos-flows")` wiring.

```sh
./ns3 run "ntn-oran-qos-flows --simSeconds=40"
```

- **Outputs:** a live per-flow delay table and anomaly events on stdout; end-of-run per-flow measured KPIs (rx packets, one-way delay, jitter, loss, throughput per 5QI/S-NSSAI), last C&C telemetry, and a measured cell summary (SINR, TBLER, throughput); files `ntn-oran-qos-flows_kpm_series.csv` / `.lp` (InfluxDB line protocol; auto-exported by `EnableAiFlowMonitor`) in the working directory, plus `oran_flow_monitor.xml` and `sim_health.csv` in `--outputDir` (default `ntn-oran-qos-flows-output`).
- **Key args:** `--simSeconds`, `--leoAltKm` (default 550), `--freqGHz` (default 2, S-band carrier — mmWave-NR FR2 numerology, not a 3GPP NR-NTN FR1 band/numerology), `--satEirpDbm`, `--outputDir`.

### ntn-tr38821-calibration

Calibrates the toolkit's measured radio against the official 3GPP **TR 38.821 Sec. 6.1.3** link-budget methodology — study case Set-1 LEO-600, S-band downlink, handheld UE (EIRP density 34 dBW/MHz → 48.77 dBW over 30 MHz; G/T −31.62 dB/K; reference CNR 15.78 dB at 90° elevation). The satellite is a real SGP4 element at 600 km; every 2 s the example computes the TR closed-form CNR at the live slant range and compares it with the SINR measured off the PHY trace. Two calibration gates: the measured-minus-TR offset must stay constant across the pass (std < 1.5 dB — the offset itself is the known beamforming array gain of the spectrum model, reported, not hidden), and the measured SINR decay between zenith and pass end must match the TR FSPL delta within 1 dB.

```sh
./ns3 run "ntn-tr38821-calibration --simSeconds=120"
```

- **Outputs:** a per-sample table (slant range, TR CNR, measured SINR, offset) and a final `CALIBRATED`/`FAIL` verdict on stdout; `sim_health.csv` in `--outputDir`. Exit code is non-zero on a failed gate, so it doubles as a CI check.
- **Key args:** `--simSeconds`, `--outputDir`.

### ntn-real-stack-smoke

Minimal validation of `NtnRealStackHelper`: a single LEO gNB (SGP4 Walker serving satellite) over a handful of TR 38.811-mobility ground UEs, eMBB streaming over the real radio, measured SINR/TBLER/throughput reported via the health gates.

```sh
./ns3 run "ntn-real-stack-smoke --simTime=10 --numUes=4 --altKm=600"
```

- **Outputs:** a one-line measured summary and `sim_health.csv` in `--outputDir` (default `ntn-real-stack-smoke-out`).
- **Key args:** `--simTime`, `--numUes`, `--altKm`, `--satEirpDbm`, `--freqGhz`, `--bwMhz`, `--outputDir`.

### ntn-nr-fr1-demo

Proves the **5G-LENA (`nr`) FR1 NTN radio spine** on the `NtnRealStackHelper` NR backend: a real NR data plane at FR1 numerology (30 kHz SCS) on an S-band (2.0 GHz) carrier with 20 MHz bandwidth — the FR1 regime the FR2-locked `mmwave` path cannot reach. Topology: one LEO gNB at ~600 km with a few ground UEs directly below.

```sh
./ns3 run "ntn-nr-fr1-demo --simTime=2 --numerology=1"
```

- **Outputs:** measured NR summary on stdout (mean DL SINR / TBLER, PHY transport blocks, throughput).
- **Key args:** `--simTime` (s, def 2), `--numUes` (def 3), `--altitudeKm` (def 600), `--numerology` (0 = 15 kHz, 1 = 30 kHz; def 1), `--satEirpDbm` (def 70), `--freqGhz` (def 2), `--bwMhz` (def 20, the NTN-FR1 max), `--outputDir` (def `./`).

### ntn-nr-deep-integration-demo

Exercises the four **NR deep-integration enablers** the toolkit adds to `NtnRealStackHelper` (nr backend), each turning a piece of 5G-LENA machinery into a *measured* quantity: **D** native NR stats + measured MCS / MIMO rank / PRB utilisation + NTN-stretched HARQ pool; **C** QoS slices (three 5QIs → three BWPs + the OfdmaQos scheduler + per-5QI dedicated bearers); **B** real MIMO via `SetupMimoPmi` (measured rank); **A** the armed A3-RSRP + X2 handover. Two satellites (gNBs) plus ground UEs.

```sh
./ns3 run "ntn-nr-deep-integration-demo"          # all four enablers
./ns3 run "ntn-nr-deep-integration-demo --slices=0" # isolate single-BWP
```

- **Outputs:** a measured four-enabler summary on stdout (mean DL SINR / TBLER / MCS / MIMO rank / PRB, per-slice per-BWP SINR / TB counts, handover count) and the native NR PDCP/RLC/MAC/PHY stat files (`NrDlMacStats.txt`, `NrDl*RlcStats*`, `NrDl*PdcpStats*`, `RxPacketTrace.txt`) in the working directory.
- **Key args:** `--slices` (per-slice BWPs / Enabler C, def true), `--simTime` (s, def 10), `--numUes` (def 6), `--altitudeKm` (def 600), `--satEirpDbm` (def 70), `--bwMhz` (def 30 → 3 × 10 MHz BWPs), `--outputDir` (def `./nr-deep-demo/`).
- **Note:** the handover count is **0** here by design — the neighbour satellite sits only 20 km to the side over a short sim, so both slant ranges hug 600 km and the RSRP hysteresis is never crossed. For a *firing* handover on a realistic pass, see **ntn-nr-handover-pass** below.

### ntn-nr-handover-pass

The dedicated proof of **Enabler A**: a real NR inter-satellite **A3-RSRP + X2 handover** on a realistic 600 km LEO pass. The serving satellite starts overhead the UE and flies off toward the horizon at true LEO ground-track speed (~7.5 km/s) while the neighbour rises to overhead; the UE's *measured* neighbour RSRP (sensed on each cell's PSS) crosses the serving cell by the hysteresis, the A3 event fires, and the serving gNB triggers an X2 handover. `GetHandoverCount()` counts the completions.

```sh
./ns3 run "ntn-nr-handover-pass"
```

- **Outputs:** the A3 trigger and a measured summary on stdout — sat altitude/speed, A3 hysteresis/TTT, mean DL SINR, PHY transport blocks, and **`[A] Handovers done: N`** (N = 1 at the shipped defaults). Add `NS_LOG=NrA3RsrpHandoverAlgorithm=level_logic` to watch the neighbour-vs-serving RSRP decision.
- **Key args:** `--simTime` (s, def 90), `--numUes` (def 1 — see note), `--altitudeKm` (def 600), `--hystDb` (A3 hysteresis, dB, def 2), `--tttMs` (A3 time-to-trigger, ms, def 512), `--neighbourBehindKm` (neighbour start offset behind serving, km, def 600).
- **Note:** keep `--numUes=1` for a clean run. `numUes>1` drives several UEs through the handover at once and hits a separate, structural limitation of the vendored 5G-LENA v3.3 X2 handover data-forwarding path (it pushes a PDCP control PDU through a build that only supports DATA PDUs); the A3/X2 decision itself fires correctly regardless.

### nrtv-p2p-example

Two nodes over a point-to-point link: an NRTV video server streams to a client; a `ClientRxTracePlot` records the Rx traffic.

```sh
./ns3 run "nrtv-p2p-example --time=10 --protocol=UDP"
```

- **Outputs:** an NRTV client trace gnuplot file `NRTV-<protocol>-client-trace.plt` (e.g. `NRTV-UDP-client-trace.plt`) in the working directory — render it with `gnuplot NRTV-UDP-client-trace.plt`. This example does **not** print metrics to stdout.
- **Key args:** `--time` (simulation time, s), `--protocol` (`TCP` or `UDP`, upper case), `--verbose` (enable trace logging).

### nrtv-variables-plot

Draws samples from the NRTV traffic-model random-variable distributions and plots histograms.

```sh
./ns3 run "nrtv-variables-plot --numOfSamples=100000"
```

- **Outputs:** gnuplot `.plt` files in the working directory — `nrtv-num-of-frames.plt`, `nrtv-slice-size.plt`, `nrtv-slice-encoding-delay.plt`, `nrtv-idle-time.plt` (render with `gnuplot *.plt`).
- **Key args:** `--numOfSamples` (number of samples drawn per distribution; default 100000).

### ntn-cbr-leo-link

A minimal CBR data plane over a single point-to-point LEO link — the lightweight, radio-physics-free path for latency / loss studies. The link carries a TR 38.821-derived one-way propagation delay and a receive-side error model, and `FlowMonitor` reports the measured throughput / delay / jitter end to end.

```sh
./ns3 run "ntn-cbr-leo-link --simTime=10 --intervalMs=5"
```

- **Outputs:** a measured `FlowMonitor` summary on stdout (throughput, mean delay, jitter).
- **Key args:** `--simTime` (s, def 10), `--intervalMs` (CBR inter-packet interval, ms, def 5), `--pktSize` (payload bytes, def 1024), `--errorRate` (per-packet loss on the rx device, def 0.01), `--delayMs` (one-way prop delay, ms, def 12.885 ≈ LEO-600 RTD 25.77 ms), `--dataRateMbps` (link rate, def 50).

### three-gpp-http-example

The 3GPP HTTP traffic model (browsing sessions of a main object plus embedded objects with reading-time gaps), retained for NTN web-browsing studies over a satellite bent-pipe / regenerative link. **Source-only:** `examples/three-gpp-http-example.cc` (arg `--SimulationTime`, default 300 s) is shipped as a reference program but is **not** registered as an `ns3 run` target in `examples/CMakeLists.txt`; the HTTP model is exercised instead by the `three-gpp-http-client-server-test` system suite:

```sh
./test.py -s three-gpp-http-client-server-test
```

## Build, run & test

The module builds as part of the ns3-ntn-toolkit tree:

```sh
./ns3 configure --enable-examples --enable-tests
./ns3 build
./test.py -s ntn-oran-application
./test.py -s ntn-oran-ai-flow-monitor
./test.py -s cbr-test
./test.py -s nrtv
```

The module ships four test suites: `ntn-oran-application` (unit — payload-header packet round-trip, in-band measurement of a known link delay, sequence-gap loss against a real `RateErrorModel`, C&C telemetry round-trip with real mobility + battery state), `ntn-oran-ai-flow-monitor` (unit — KPM series vs. sink ground truth, a mid-run error burst showing up as KPM loss + anomaly event, multi-slice flow classification, XML/CSV/Influx exporter round-trips), `cbr-test` (unit), and `nrtv` (system). See [INSTALL](../../INSTALL.md) for full toolkit setup.

## License & author

GPL-2.0-only. Muhammad Uzair, Independent Researcher.

## Scope & limitations (toolkit boundaries)

**A1** — the measured channel carries the TR 38.811 *large-scale* terms only (no NTN-TDL/Rician fast fading). **A5** — the vendored mmwave PHY runs FR2 numerology (60 kHz SCS) at an S-band carrier and models the gNB as a terrestrial array, not an FR1-NTN waveform or a satellite reflector beam; the link-level AMC/MCS/LDPC-BLER chain on top is real. See the toolkit-wide [`SCOPE_AND_LIMITATIONS.md`](../../SCOPE_AND_LIMITATIONS.md) for the authoritative statement of what is and is not modelled.
