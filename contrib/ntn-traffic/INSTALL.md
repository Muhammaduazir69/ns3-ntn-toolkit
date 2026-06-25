# Install & run — ntn-traffic

`ntn-traffic` is an ns-3.43 contributed module — the traffic and measurement
backbone of the toolkit. It provides **`NtnRealStackHelper`** (a real mmwave NR
NTN cell: SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC under SGP4 satellite
mobility), the **ORAN-NTN application suite** (`NtnOranApplication`,
`NtnOranSink`, `NtnOranPayloadHeader`, `NtnCommandAndControlApp`), the
**`NtnOranAiFlowMonitor`** AI-native KPM layer, and the classic 3GPP
**HTTP / NRTV / CBR** traffic models. It is a **required dependency for most
other toolkit modules' examples** (they compose its real data plane).

The recommended way to use it is inside the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) tree
(branch `ntn-integration-v2`), where every dependency below is already present.
`ntn-traffic` ships **bundled inside that tree** — there is no standalone
repository to clone. On a vanilla ns-3.43 tree, add the sibling modules in
section 2.

---

## 1. System requirements

| Component | Version |
|---|---|
| OS | Linux (Ubuntu 22.04+ / Fedora 39+ recommended) |
| C++ compiler | gcc ≥ 11 or clang ≥ 14 |
| CMake | ≥ 3.24 |
| Python | ≥ 3.10 |
| ns-3 | **3.43** |
| Disk | ~6 GB after build (incl. SNS3 TLE data) |

---

## 2. Dependencies

### 2a. mmWave NR PHY + `magister-stats` (REQUIRED for the library)

The library links `mmwave` (and its bundled `lte`) for `NtnRealStackHelper`,
`magister-stats` for the application-stats collectors, plus the standard
`network`, `mobility`, `propagation`, `applications`, `flow-monitor`,
`internet`, and `point-to-point` modules. On a vanilla ns-3.43 tree:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

> `magister-stats` ships with the SNS3 `satellite` tree (section 2c).

### 2b. Toolkit modules `ntn-cho` + `ntn-constellation` (REQUIRED for the examples)

The real-radio examples (`ntn-oran-qos-flows`, `ntn-tr38821-calibration`,
`ntn-real-stack-smoke`, `nrtv-p2p-example`, `nrtv-variables-plot`) link
`ntn-cho` (`NtnTr38811MobilityHelper` TR 38.811 UE mobility) and
`ntn-constellation` (`Sgp4MobilityModel`, `WalkerConstellation`). These are
**bundled in the toolkit** (no per-module repositories), so inside
`ns3-ntn-toolkit` they are already in `contrib/`. The `ntn-cbr-leo-link` and
`three-gpp-http-example` examples do **not** need them.

### 2c. SNS3 `satellite` (REQUIRED, transitively)

The toolkit siblings pull in the SNS3 `satellite` module, which also supplies
`magister-stats`:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

---

## 3. Install the module

`ntn-traffic` is **bundled-only** — it has no standalone GitHub repository.
Clone the toolkit and it is already in `contrib/ntn-traffic`:

```bash
git clone -b ntn-integration-v2 https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
# contrib/ntn-traffic is already present
```

GitLab mirror: `https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit`.
Or skip the build entirely with the prebuilt image
`uzairdocker69/ns3-ntn-toolkit:2.2.1` (or `:latest`).

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build ntn-traffic
./ns3 show profile | grep ntn-traffic   # expect: ... ntn-traffic ...
```

---

## 5. Run the examples

### 5a. ntn-oran-qos-flows — flagship ORAN-NTN, four 3GPP QoS flows

```bash
./ns3 run "ntn-oran-qos-flows --simSeconds=40"
```
One real LEO cell (`NtnRealStackHelper`) carries four `NtnOranApplication`
flows side by side, each with its own 5QI / S-NSSAI in real packet bytes
(UE0 5QI 1 voice, UE1 5QI 2 eMBB video, UE2 5QI 82 URLLC, UE3 5QI 9 mMTC),
plus satellite C&C telemetry (5QI 69). UEs move under TR 38.811 mobility; the
KPM layer runs via `EnableAiFlowMonitor("ntn-oran-qos-flows")`.
Outputs: live per-flow delay table + anomaly events and end-of-run per-flow
measured KPIs on stdout; `ntn-oran-qos-flows_kpm_series.csv` / `.lp` in the
working dir; `oran_flow_monitor.xml` and `sim_health.csv` in `--outputDir`
(default `ntn-oran-qos-flows-output`).
Args: `simSeconds`, `leoAltKm` (default 550), `freqGHz` (default 2),
`satEirpDbm`, `outputDir`.

### 5b. ntn-tr38821-calibration — measured radio vs. TR 38.821 link budget

```bash
./ns3 run "ntn-tr38821-calibration --simSeconds=120"
```
Calibrates the measured PHY SINR against the TR 38.821 Sec. 6.1.3 closed-form
CNR (Set-1 LEO-600, S-band DL, handheld UE) at the live slant range every 2 s.
Two gates (constant offset across the pass; SINR decay matching FSPL delta);
prints a per-sample table and a `CALIBRATED`/`FAIL` verdict, exit code
non-zero on failure (doubles as a CI check). `sim_health.csv` in `--outputDir`.
Args: `simSeconds`, `outputDir`.

### 5c. ntn-real-stack-smoke — minimal NtnRealStackHelper validation

```bash
./ns3 run "ntn-real-stack-smoke --simTime=10 --numUes=4 --altKm=600"
```
A single LEO gNB (SGP4 Walker satellite) over TR 38.811-mobility UEs, eMBB
streaming over the real radio. One-line measured summary and `sim_health.csv`
in `--outputDir` (default `ntn-real-stack-smoke-out`).
Args: `simTime`, `numUes`, `altKm`, `satEirpDbm`, `freqGhz`, `bwMhz`,
`outputDir`.

### 5d. ntn-cbr-leo-link — CBR over a LEO P2P link with a real error model

```bash
./ns3 run "ntn-cbr-leo-link --simTime=10 --intervalMs=5 --errorRate=0.01"
```
Drives the module's own `CbrApplication` / `CbrHelper` over a
satellite-representative point-to-point link with a `RateErrorModel` on the rx
NetDevice (loss from a real error model, not a formula); default ~12.9 ms
one-way delay ≈ LEO-600 RTD. KPIs (sent/rx bytes, loss, throughput/delay/jitter
from FlowMonitor) are measured. Needs only `ntn-traffic` + standard modules.
Args: `simTime`, `intervalMs`, `pktSize`, `errorRate`, `delayMs`,
`dataRateMbps`.

### 5e. nrtv-p2p-example — NRTV video stream over P2P

```bash
./ns3 run "nrtv-p2p-example --time=10 --protocol=UDP"
```
An NRTV video server streams to a client; a `ClientRxTracePlot` records the Rx
traffic. Writes a gnuplot file `NRTV-<protocol>-client-trace.plt` (render with
`gnuplot NRTV-UDP-client-trace.plt`); no stdout metrics.
Args: `time` (s), `protocol` (`TCP` or `UDP`, upper case), `verbose`.

### 5f. nrtv-variables-plot — NRTV random-variable histograms

```bash
./ns3 run "nrtv-variables-plot --numOfSamples=100000"
```
Draws samples from the NRTV traffic-model distributions and writes gnuplot
`.plt` files in the working dir (`nrtv-num-of-frames.plt`, `nrtv-slice-size.plt`,
`nrtv-slice-encoding-delay.plt`, `nrtv-idle-time.plt`; render with `gnuplot *.plt`).
Args: `numOfSamples` (default 100000).

> `examples/three-gpp-http-example.cc` (arg `--SimulationTime`) is shipped as a
> source file but is **not** registered as an `ns3 run` target in
> `examples/CMakeLists.txt`; the HTTP model is instead exercised by the
> `three-gpp-http-client-server-test` system suite (section 6).

---

## 6. Run the unit tests

The module ships **five** registered test suites:

```bash
./test.py --suite=ntn-oran-application          # unit
./test.py --suite=ntn-oran-ai-flow-monitor      # unit
./test.py --suite=ntn-real-stack-helper         # unit
./test.py --suite=cbr-test                      # unit
./test.py --suite=nrtv                          # system
./test.py --suite=three-gpp-http-client-server-test   # system (HTTP model)
```
`ntn-oran-application` covers payload-header round-trip, in-band measurement of
a known link delay, sequence-gap loss against a real `RateErrorModel`, and C&C
telemetry round-trip with real mobility + battery state.
`ntn-oran-ai-flow-monitor` covers KPM series vs. sink ground truth, a mid-run
error burst surfacing as KPM loss + an anomaly event, multi-slice flow
classification, and the XML/CSV/Influx exporter round-trips.
`ntn-real-stack-helper` exercises the real mmwave NR NTN cell end to end.

---

## 7. Common issues

**`ntn-traffic` not registered after configure** — `mmwave` (and `lte`) and
`magister-stats` are missing; the library links them (section 2a/2c) and will
not register without them.

**Real-radio examples missing after configure** — `ntn-oran-qos-flows`,
`ntn-tr38821-calibration`, `ntn-real-stack-smoke`, and the two NRTV examples
need `ntn-cho` and `ntn-constellation` in `contrib/` (section 2b). The library
and the `ntn-cbr-leo-link` example build without them.

**No metrics printed by an NRTV example** — `nrtv-p2p-example` and
`nrtv-variables-plot` emit gnuplot `.plt` files, not stdout numbers; render
them with `gnuplot`.

**HARQ / RLC timers misbehaving over the slant** — `NtnRealStackHelper` keeps
HARQ off by default (terrestrial HARQ timers break over LEO delay); opt into
the NTN-stretched profile with `SetNtnHarqProfile(true)`.

---

## 8. Uninstall

```bash
rm -rf contrib/ntn-traffic
./ns3 configure --enable-examples
./ns3 build
```
