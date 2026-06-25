# Install & run — oran-ntn

`oran-ntn` is an ns-3.43 contributed module that brings the O-RAN
disaggregated RAN control architecture to non-terrestrial networks: a
multi-tier RIC (real-time / Near-RT / on-board **Space-RIC**), E2AP-style
termination over SCTP/TCP, A1 policy ingest, KPM/RC/CCC service models, and
xApps with a conflict manager. The recommended way to run it is inside the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) tree
(branch `ntn-integration-v2`), where every dependency below is already
present. It also builds on a vanilla ns-3.43 tree, provided you add the
sibling toolkit modules listed in section 2 — the library links `satellite`,
`mmwave`, and `ntn-traffic`; the examples additionally pull in `ntn-cho`,
`ntn-constellation`, `ntn-slice`, and `ntn-observability`.

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

Two optional system libraries are auto-detected at configure time:

| Library | Enables | Fallback when absent |
|---|---|---|
| `libsctp` (`libsctp-dev`) | E2AP transport over real SCTP in `flexric-bridge/` | runtime fallback to TCP |
| ONNX Runtime (`onnxruntime`) | real inference in `OranNtnOnnxXapp` | registered heuristic xApp |
| SQLite3 (`libsqlite3-dev`) | NIST-pattern E2 data repository | in-memory store |

---

## 2. Dependencies

### 2a. SNS3 `satellite` (REQUIRED — library links it)

The O-RAN/NTN bridge and SGP4 geometry use the SNS3 `satellite` module:

```bash
cd contrib/
git clone https://github.com/sns3/sns3-satellite.git satellite
cd ..
```

> Size note: SNS3 + bundled TLE data is ~3.7 GB.

### 2b. mmWave NR PHY (REQUIRED — library links it)

The measured data plane runs real mmwave NR NTN cells, so `contrib/mmwave`
(and its bundled `lte` dependency) must be present. The toolkit uses the
**NYU/UNIPD** ns-3-mmwave module — **not** the CTTC NR module:

```bash
cd contrib/
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git mmwave
cd ..
```

### 2c. `ntn-traffic` (REQUIRED — library links it)

`oran-ntn` links `ntn-traffic`, whose `NtnRealStackHelper` stands up the
actual mmwave NR NTN stack (SpectrumPhy + MAC + HARQ/AMC + RLC/PDCP + RRC +
EPC) the RICs measure KPIs from. It is bundled in the toolkit under
`contrib/ntn-traffic/`; on a vanilla tree, copy it from the toolkit into
`contrib/ntn-traffic`.

### 2d. Example-only siblings (REQUIRED for the examples)

The examples additionally link `ntn-cho`, `ntn-constellation`, `ntn-slice`,
and `ntn-observability`. All four are bundled in the toolkit under
`contrib/`; on a vanilla tree, copy them from the toolkit. The `oran-ntn`
library itself builds without them — the examples do not.

### 2e. `ns3-ai-ntn` (OPTIONAL — learning-based xApps / gym envs)

The gym handover / beam-hop / slice / steering environments and
`OranNtnFederatedLearning` bridge to Python via the toolkit's `ns3-ai-ntn`
fork. The E2/KPM/RC control loop and all heuristic xApps build and run
**without** it. When present in `contrib/`, the build links it automatically.

---

## 3. Install the module

```bash
cd contrib/
git clone -b oran-ntn-v2 https://github.com/Muhammaduazir69/oran-ntn.git oran-ntn
cd ..
```

---

## 4. Configure & build

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build oran-ntn
./ns3 show profile | grep oran-ntn   # expect: ... oran-ntn ...
```

---

## 5. Run the examples

### 5a. oran-ntn-full-scenario — constellation-scale multi-tier RIC (flagship)

```bash
./ns3 run "oran-ntn-full-scenario --duration=120 --numPlanes=6 --satsPerPlane=11 --numUes=50 --enableSpaceRic=true --outputDir=/tmp/oran"
```
Anchored cells measured on a real mmwave NR NTN link; the remaining cells use
a TR 38.821-style budget over the same live SGP4 geometry (per-row provenance
in `kpm_feed.csv`). Args include `duration`, `numPlanes`, `satsPerPlane`,
`altitude`, `inclination`, `numTnGnbs`, `numUes`, `numRealCells`,
`realUesPerCell`, `kpmInterval`, `minElev`, `serviceElev`, `stickyServing`,
`satEirpDbm`, `freqGhz`, `bwMhz`, `conflictStrategy`, `enableSpaceRic`,
`enableFL`, `outputDir`.

### 5b. oran-ntn-ric-controlled-traffic — closed RIC control loop

```bash
./ns3 run "oran-ntn-ric-controlled-traffic --simSeconds=120 --xapp=mmimo --sinrThreshDb=5 --outputDir=/tmp/ric"
```
E2-KPM from the measured mmwave PHY drives an mMIMO precoder xApp, whose beam
decision is applied back onto the real link. Args: `simSeconds`, `leoAltKm`,
`freqGHz`, `satEirpDbm`, `numTx`, `sinrThreshDb`, `xapp`, `outputDir`.

### 5c. oran-ntn-real-stack-scenario — real-stack RIC flagship

```bash
./ns3 run "oran-ntn-real-stack-scenario --duration=120 --numUes=10 --numSats=4 --outputDir=/tmp/realstack"
```
KPM from MEASURED mmwave PHY SINR drives the RIC. Args: `duration`, `numUes`,
`numSats`, `altitude`, `satEirpDbm`, `freqGhz`, `bwMhz`, `minElev`,
`conflictStrategy`, `netSim`, `czml`, `outputDir`.

### 5d. ntn-e2e-full-stack — RIC + slice + observability composition

```bash
./ns3 run "ntn-e2e-full-stack --duration=120 --numUes=10 --outputDir=/tmp/e2e"
```
One shared mmwave cell feeds the RIC, the slice manager, and observability.
Args: `duration`, `numUes`, `altitude`, `satEirpDbm`, `outputDir`.

### 5e. oran-ntn-e2-termination — two-process E2 over real SCTP

```bash
# Terminal 1 (RIC side):
./ns3 run "oran-ntn-e2-termination --role=ric --proto=sctp --port=36421 --duration=20"
# Terminal 2 (E2 agent side):
./ns3 run "oran-ntn-e2-termination --role=agent --proto=sctp --host=127.0.0.1 --port=36421 --indications=10"
```
Standalone E2 end-to-end driver exercising the `flexric-bridge/` transport.
Args: `role` (ric|agent), `proto` (sctp|tcp), `host`, `port`, `duration`,
`indications`. Falls back to TCP if the kernel SCTP module is unavailable.

### Other built examples

`oran-ntn-ric-placement-ab`, `oran-ntn-cross-domain-slice`,
`oran-ntn-payload-options-ab`, `ntn-platform-latency-validation`, and
`oran-ntn-emergency-communication` (args: `simSeconds`, `disasterAt`,
`outputDir`) cover the WS3–WS6 A/B and flagship studies.

---

## 6. Run the unit tests

`oran-ntn` registers **three** test suites:

```bash
./test.py --suite=oran-ntn                # core E2 / A1 / KPM / RIC / xApp framework
./test.py --suite=oran-ntn-multi-tier-ric # RT / Near-RT / Space-RIC + placement
./test.py --suite=oran-ntn-ws4            # payload options / FH split / platform spec
```

The aggregate KPIs in any associated manuscript come from sweeping
`oran-ntn-full-scenario` / `oran-ntn-real-stack-scenario` over seeds and
xApp settings and aggregating the CSV output; they are not asserted by the
unit suites.

---

## 7. Common issues

**`oran-ntn` not registered after configure** — a required library dependency
is missing. The library needs `contrib/satellite/`, `contrib/mmwave/`, and
`contrib/ntn-traffic/`; it will not register without all three.

**Examples missing after configure** — the examples additionally need
`ntn-cho`, `ntn-constellation`, `ntn-slice`, and `ntn-observability` in
`contrib/` (step 2d); the library builds without them, the examples do not.

**E2 SCTP example falls back to TCP** — `libsctp` was not found at configure
time, or the kernel SCTP module is not loaded. Install `libsctp-dev` and
`modprobe sctp`, or run `oran-ntn-e2-termination` with `--proto=tcp`.

**`OranNtnOnnxXapp` runs the heuristic instead of an ONNX model** — ONNX
Runtime was not found at configure time (expected and harmless); install it
to enable real inference.

---

## 8. Uninstall

```bash
rm -rf contrib/oran-ntn
./ns3 configure --enable-examples
./ns3 build
```
