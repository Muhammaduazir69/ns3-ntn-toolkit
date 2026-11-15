<p align="center">
  <img src="docs/logo_lockup.png" alt="ns3-ntn-toolkit logo" width="780"/>
</p>

<h1 align="center">ns3-ntn-toolkit</h1>

<p align="center">
  <strong>A pre-integrated ns-3.43 simulation platform for 6G non-terrestrial networks:
  LEO satellite constellations, 3GPP NR-NTN protocol stacks, O-RAN control loops,
  terahertz links and reinforcement-learning agents, in one tree that clones, builds and runs.</strong>
</p>

<p align="center">
  <a href="https://youtu.be/JNStiBst68I"><img src="https://img.youtube.com/vi/JNStiBst68I/maxresdefault.jpg" alt="NTN Control Center: end-to-end simulation of non-terrestrial networks in ns-3 (video)" width="760"/></a><br/>
  <a href="https://youtu.be/JNStiBst68I"><img src="https://img.shields.io/badge/YouTube-Watch%20the%205--minute%20demo-red?logo=youtube" alt="Watch on YouTube"/></a><br/>
  <sub>Watch the NTN Control Center run, visualize and export every module of the toolkit, end to end (5 min).</sub>
</p>


<p align="center">
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">GitHub</a>
  &nbsp;·&nbsp;
  <a href="https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit">GitLab</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/">Documentation</a>
  &nbsp;·&nbsp;
  <a href="https://hub.docker.com/r/uzairdocker69/ns3-ntn-toolkit">Docker Hub</a>
  &nbsp;·&nbsp;
  <a href="INSTALL.md">Install guide</a>
  &nbsp;·&nbsp;
  <a href="SCOPE_AND_LIMITATIONS.md">What it does not model</a>
</p>

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg" alt="ns-3.43"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg" alt="GPL-2.0"/></a>
  <img src="https://img.shields.io/badge/3GPP-Rel--17%20%2F%2018%20%2F%2019%20NTN-orange.svg" alt="3GPP Rel-17/18/19 NTN"/>
  <img src="https://img.shields.io/badge/O--RAN-E2%20%C2%B7%20A1%20%C2%B7%20KPM%20%C2%B7%20RC-purple.svg" alt="O-RAN E2 A1 KPM RC"/>
  <img src="https://img.shields.io/badge/THz-100%20GHz%20to%201%20THz-success.svg" alt="THz 100 GHz to 1 THz"/>
  <img src="https://img.shields.io/badge/Sionna%20RT-ray%20tracing-red.svg" alt="Sionna RT"/>
  <img src="https://img.shields.io/badge/RL-Gymnasium%201.0-yellow.svg" alt="Gymnasium 1.0"/>
  <img src="https://img.shields.io/badge/modules-14%20custom-informational.svg" alt="14 custom modules"/>
  <img src="https://img.shields.io/badge/examples-93-informational.svg" alt="93 examples"/>
  <a href="https://doi.org/10.1109/OJCOMS.2026.3745010"><img src="https://img.shields.io/badge/DOI-10.1109%2FOJCOMS.2026.3745010-00629B.svg" alt="DOI 10.1109/OJCOMS.2026.3745010"/></a>
</p>

<p align="center">
  <img src="docs/architecture.png" alt="ns3-ntn-toolkit architecture: orbital and mobility, protocol L1 to L3, physical layer, control and learning, and operations layers over vendored SNS3 satellite and mmWave NR, on an unmodified ns-3.43 simulation core" width="960"/>
</p>

<p align="center"><sub>Architecture as published in the accompanying manuscript. Amber arrows follow one run left to right: orbit state, beam trigger, THz xApps, KPI export.</sub></p>

---

<p align="center">
  <img src="docs/paper1_fig1.png" alt="Fig. 1 of the article: the system the platform models, with space, aerial, ground and measurement layers; a LEO Walker shell with optical inter-satellite links and an on-board Space RIC; HAPS and UAV tiers; a feeder gateway, gNB, near-RT RIC with E2 and A1 interfaces, and served households, handsets and vehicles; and a measurement rail from instrumented packets to controllers" width="960"/>
</p>

<p align="center"><sub><b>Fig. 1 of the article.</b> The system the platform models. Height is altitude and link color is carrier. E2 reaches a satellite over the feeder bearer. The rail underneath shows the measurement path: packets leave an application, cross a modeled radio, and are read back at the sink, where every KPI is computed.</sub></p>

## What this is

Simulating a 6G non-terrestrial network usually means assembling four unrelated
codebases and hoping their coordinate frames, time bases and units agree. An
orbital propagator that speaks TLEs. A cellular stack that speaks slots and RNTIs.
A channel model written for a terrestrial street canyon. A control framework that
expects a terrestrial gNB to be sitting still.

This toolkit is that assembly, done once and kept honest. It is a fork of
**ns-3.43** carrying **14 custom modules** and three vendored upstreams, wired so
that a satellite's propagated position drives a real NR spectrum PHY, a real handover
decision rides a real Xn interface with a real propagation delay, and every KPI a
scenario prints came off a packet that actually crossed the air interface.

The distinguishing property is not breadth. It is that the **decision plane and
the measurement plane are the same plane**. A handover trigger that fires moves a
terminal onto a different cell whose SINR is then measured, rather than
incrementing a counter beside an unrelated number.

**Who it is for.** Researchers working on LEO satellite communications, NR-NTN
mobility management, satellite O-RAN and RIC placement, sub-terahertz and
terahertz links, space-air-ground integrated networks, and reinforcement learning
for satellite radio resource management, who would rather spend their time on the
contribution than on the integration.

---

## Try it in 60 seconds

### Docker, no build required

The whole toolkit, prebuilt: ns-3.43, all 14 modules, the SNS3 `satellite` stack,
the mmWave and 5G-LENA `nr` NR stacks, the Python tooling and the digital-twin
server.

```bash
docker pull uzairdocker69/ns3-ntn-toolkit:latest

# A LEO conditional-handover pass with a real NR cell under satellite mobility
docker run --rm uzairdocker69/ns3-ntn-toolkit:latest \
  ./ns3 run "ntn-cho-real-stack --trigger=d2 --simSeconds=60"

# An interactive shell inside the built tree
docker run --rm -it uzairdocker69/ns3-ntn-toolkit:latest bash
```

### From source

```bash
git clone --branch ntn-integration-v2 \
  https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git
cd ns3-ntn-toolkit
./ns3 configure --enable-examples --enable-tests
./ns3 build
./ns3 run ntn-real-stack-smoke
```

Full prerequisites, the SNS3 `satellite` dependency, GPU setup for Sionna RT and
the troubleshooting table are in **[INSTALL.md](INSTALL.md)**.

---

## The 14 modules

Each module is also published as a standalone repository on GitHub and GitLab, so
it can be dropped into an existing ns-3 tree on its own. `ntn-traffic` and
`ntn-fapi` live only in this repository.

| Module | Layer | What it gives you | Repository |
|---|---|---|---|
| **[`ntn-constellation`](contrib/ntn-constellation)** | Orbital and mobility | SGP4 from TLEs, Kepler + J2 for generated Walker shells, ISL contact-graph routing, constellation presets | [ntn-constellation](https://github.com/Muhammaduazir69/ntn-constellation) |
| **[`ntn-sagin`](contrib/ntn-sagin)** | Orbital and mobility | ground, UAV, HAPS and LEO layers, TR 36.777 air-to-ground channel, store and forward across contact gaps | [ntn-sagin](https://github.com/Muhammaduazir69/ntn-sagin) |
| **[`ntn-v2x`](contrib/ntn-v2x)** | Orbital and mobility | NR sidelink PC5 Mode 2, SAE J2735 basic safety messages, SUMO traces, a relay decision that gates the flow | [ntn-v2x](https://github.com/Muhammaduazir69/ntn-v2x) |
| **[`ntn-traffic`](contrib/ntn-traffic)** | Protocol, L1 to L3 | `NtnRealStackHelper`: the measured NR NTN data plane (PHY, HARQ, RLC, PDCP, RRC, EPC) every module builds on | [toolkit only](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/tree/ntn-integration-v2/contrib/ntn-traffic) |
| **[`ntn-cho`](contrib/ntn-cho)** | Protocol, L1 to L3 | conditional handover with time-to-exit and the TS 38.331 NTN trigger set (A3, D1, D2, T1) | [ntn-cho-framework](https://github.com/Muhammaduazir69/ntn-cho-framework) |
| **[`ntn-rrc`](contrib/ntn-rrc)** | Protocol, L1 to L3 | SIB19 with `cellSpecificKoffset` consumed by the scheduler, payload-aware timing advance, DRX that gates a flow | [ntn-rrc](https://github.com/Muhammaduazir69/ntn-rrc) |
| **[`ntn-fapi`](contrib/ntn-fapi)** | Protocol, L1 to L3 | SCF-222 FAPI MAC-PHY adapter on a live NR SAP with per-UE HARQ state | [toolkit only](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/tree/ntn-integration-v2/contrib/ntn-fapi) |
| **[`ntn-slice`](contrib/ntn-slice)** | Protocol, L1 to L3 | eMBB, URLLC and mMTC with per-5QI bearers and SLA percentiles from a delay histogram | [ntn-slice](https://github.com/Muhammaduazir69/ntn-slice) |
| **[`thz-ntn`](contrib/thz-ntn)** | Physical | 100 GHz to 1 THz on ITU-R P.676-13, P.618-13, P.838-3 and P.840, pointing error, RIS | [ns3-thz-ntn](https://github.com/Muhammaduazir69/ns3-thz-ntn) |
| **[`ntn-sionna`](contrib/ntn-sionna)** | Physical | NVIDIA Sionna RT ray tracing with per-query provenance and a TR 38.811 calibrator | [ntn-sionna](https://github.com/Muhammaduazir69/ntn-sionna) |
| **[`oran-ntn`](contrib/oran-ntn)** | Control and learning | Space O-RAN: E2 and A1 loops on a measured radio, RIC placement from live slant geometry, A1 intent leases | [oran-ntn](https://github.com/Muhammaduazir69/oran-ntn) |
| **[`ns3-ai-ntn`](contrib/ns3-ai-ntn)** | Control and learning | ns3-ai fork: Gymnasium 1.0 over versioned shared memory, AI-RAN inference contract | [ns3-ai](https://github.com/Muhammaduazir69/ns3-ai) |
| **[`ntn-observability`](contrib/ntn-observability)** | Operations | one scene recorder feeding InfluxDB, Grafana, NetSimulyzer and Cesium, with a reproducibility manifest | [ntn-observability](https://github.com/Muhammaduazir69/ntn-observability) |
| **[`ntn-digital-twin`](contrib/ntn-digital-twin)** | Operations | FastAPI constellation twin whose handover schedule ns-3 actuates; one shared A3 guard | [ntn-digital-twin](https://github.com/Muhammaduazir69/ntn-digital-twin) |
| **[`satellite (SNS3)`](contrib/satellite)** | Vendored upstream | SGP4 mobility, DVB-S2 and RCS2 link results, antenna patterns, TLE corpus | [sns3-satellite](https://github.com/sns3/sns3-satellite) |

---

## Results at a glance

Every number below is measured in a stored run and traced, in the module README,
to its data path, run count and the script that draws the figure.

| Module | At a glance | Details |
|---|---|---|
| `oran-ntn` | An A1 intent lease cuts the time a cell falls short of its intent from 5.7 % to 3.3 % at one gateway; on-board near-RT control avoids the 19 % deadline misses ground-only control has with one or two gateways (240 runs, 10 seeds per point). | [results](contrib/oran-ntn/README.md#results) |
| `ns3-ai-ntn` | Gym transfers cost about 18x fewer CPU cycles than ns3-gym (upstream benchmark, 6 seeds); `OranNtnGymHandover` steps on a measured radio. | [results](contrib/ns3-ai-ntn/README.md#results) |
| `ntn-observability` | A 60 s LEO pass read back from the exported line protocol: SINR about 25 dB, 4.7 Mb/s, elevation 89 to 51 degrees. | [results](contrib/ntn-observability/README.md#results) |
| `ntn-digital-twin` | A deterministic 66-satellite twin run exports 5 handovers in 45 min for ns-3 to actuate. | [results](contrib/ntn-digital-twin/README.md#results) |
| `ntn-cho` | TTE-aware CHO: 134.6 handovers per run against 463.3 for A3, with 0 % ping-pong (10 seeds). | [results](contrib/ntn-cho/README.md#results) |
| other modules | Each module README carries its measured results with source paths and run counts. | [ntn-constellation](contrib/ntn-constellation/README.md#results), [ntn-sagin](contrib/ntn-sagin/README.md#results), [ntn-v2x](contrib/ntn-v2x/README.md#results), [ntn-traffic](contrib/ntn-traffic/README.md#results), [ntn-rrc](contrib/ntn-rrc/README.md#results), [ntn-fapi](contrib/ntn-fapi/README.md#results), [ntn-slice](contrib/ntn-slice/README.md#results), [thz-ntn](contrib/thz-ntn/README.md#results), [ntn-sionna](contrib/ntn-sionna/README.md#results) |


### Conditional handover and xApp routing in detail

Two campaigns shipped with the tree, both reproducible from the committed CSVs
under `papers/sim_runs/`.

**Conditional handover over a LEO shell.** Ten seeds, a 780 km Walker shell at
86.4 degrees inclination, four algorithms on the same geometry and the same
traffic. The comparison is not about who hands over most successfully; it is
about how much churn each one buys that success with.

| Algorithm | Handovers per run | Success | Ping-pong |
|---|---:|---:|---:|
| A3 RSRP (baseline) | 463.3 ± 66.8 | 69.16% | 50.23% |
| Time-based | 361.7 ± 42.4 | 63.14% | 0.00% |
| Location-based | 199.5 ± 56.0 | 98.69% | 57.07% |
| **TTE-aware** | **134.6 ± 16.3** | **83.14%** | **0.00%** |

Location-based wins on success rate and pays for it: it hands over on geometry
alone, so more than half of its handovers come straight back. TTE-aware reaches
83% success on a third of the A3 baseline's handover count with no ping-pong at
all, because the decision is conditioned on how long the target will still be
serviceable rather than on how good it looks right now. The intervals are 95%
confidence over the ten seeds.

**O-RAN xApp routing.** One 600 s run of `oran-ntn-full-scenario` with five
active xApps and 71 E2 nodes: 7,738 near-real-time decisions producing **71,967**
recorded actions and 0 conflicts. The two counts differ by an order of magnitude
because one decision fans out to many actions, and Doppler compensation
dominates: 4,272 of the decisions, 68,489 of the actions. Reporting only the
larger number would flatter the decision engine, so the metrics file carries
both, per xApp. In this scenario the RC path records actions; the examples that
actuate the radio are listed in the
[oran-ntn README](contrib/oran-ntn/README.md#examples).

---

## What makes the numbers trustworthy

Simulation platforms are easy to overclaim and hard to check. Three mechanisms in
this tree exist specifically to make the claims checkable by someone who did not
write them.

**Provenance on every metric.** Every scenario built on `NtnRealStackHelper`
writes a health record (`sim_health.csv`) labelling each row with how its value
was obtained: measured in band, modeled from a closed form, or configured. A
number that came from an equation cannot be printed as if it came from a packet.
Measured across the 93 buildable examples, 72 write one; the other 21 do not
stand up a radio at all (calculators, trace tools, the THz sweeps) and so have
nothing to attest.

**Gates that can fail.** `tools/check_ntn_standards.py` runs 17 gates covering the
TR 38.821 Set-1 link budget, orbital geometry, the published latency bands and all
six NTN handover trigger classes, each asserted on its own firing count rather
than on a handover the radio would have performed anyway. `tools/check_doc_claims.py` fails the build when
a README makes a capability claim the code contradicts, or quotes a number the
committed data does not carry. `tools/check_dashboard_producers.py` walks each
dashboard panel back through the metric schema to the code that emits it.

**A stated boundary.** [SCOPE_AND_LIMITATIONS.md](SCOPE_AND_LIMITATIONS.md) is the
authoritative list of what the toolkit does not model, written so that a reviewer
can rely on an explicit scope rather than an inferred one. Among other things: the
absence of Rel-19 AI/ML lifecycle management, the multi-tap NTN-TDL that is not
implemented, and where propagation delay rides the transport leg rather than the
air interface.

---

## Reproducing a result

Every scenario that stands up a radio writes a CSV and a health record to
`--outputDir`. Not every example takes `--outputDir`; a few hardcode their
output directory, so check `--PrintHelp` before scripting a sweep.

```bash
# 1. A TR 38.821 Set-1 LEO-600 link-budget calibration
./ns3 run "ntn-tr38821-calibration --outputDir=out/"

# 2. The full standards gate set
python3 tools/check_ntn_standards.py

# 3. All 23 module test suites
./test.py -s ntn-cho -s oran-ntn -s thz-ntn -s ntn-constellation
```

The Docker image is the reproducible path, because the SNS3 `satellite` tree is a
compile-time dependency that this repository does not vendor. See
[INSTALL.md](INSTALL.md) for how to obtain it for a source build.

---

## Documentation

| Where | What is there |
|---|---|
| **[Documentation site](https://muhammaduazir69.github.io/ns3-ntn-toolkit/)** | Getting started, architecture, per-module pages, papers, citation |
| **[INSTALL.md](INSTALL.md)** | Prerequisites, source build, Docker, GPU setup, troubleshooting |
| **[SCOPE_AND_LIMITATIONS.md](SCOPE_AND_LIMITATIONS.md)** | Architectural boundaries, stated explicitly |
| **[CHANGELOG.md](CHANGELOG.md)** | Release history for the toolkit and each module |
| **[CONTRIBUTING.md](CONTRIBUTING.md)** | How to add a module or a scenario |
| `contrib/<module>/README.md` | Per-module reference, examples and install notes |

---

## Standards and references implemented

**3GPP.** TR 38.811 (NTN channel model, aperture, shadow-fading sigma tables,
Rician K-factor, NTN-TDL), TR 38.821 (NTN solutions, Set-1 LEO-600 and GEO
reference parameters, handover interruption budget), TS 38.101-5 (NTN FR1 bands
n255 and n256, channel bandwidths), TS 38.133, TS 38.211, TS 38.213 (K_offset,
timing advance), TS 38.214 (CQI and MCS tables), TS 38.300, TS 38.321, TS 38.331
(SIB19, CondEvent A3/A4/D1/D2/T1, conditional reconfiguration), TS 38.413,
TS 38.423 (Xn), TS 38.885 (V2X sidelink), TS 28.552 (performance measurements),
TR 36.777 (aerial vehicles), TR 37.885, TR 38.901.

**ITU-R.** P.676-13 (gaseous attenuation), P.618-13 (Earth-space propagation),
P.838-3 (rain), P.840 (cloud and fog), P.681-11 (land mobile satellite),
P.835-6 (reference atmospheres).

**O-RAN Alliance.** E2AP, E2SM-KPM, E2SM-RC, A1 policy, WG2 and WG3 architecture,
non-real-time and near-real-time RIC.

**Others.** ETSI EN 302 307-1 (DVB-S2 MODCOD), SAE J2735 (basic safety message),
Small Cell Forum FAPI 222.10.02.

---

## Keywords

6G, non-terrestrial network, NTN simulator, ns-3, ns-3.43, LEO satellite
constellation, satellite communication, satellite network simulation, 3GPP
Release 17, Release 18, Release 19, NR-NTN, 5G NR, 5G-LENA, mmWave, SGP4,
Walker-Delta, two-line element, orbital propagation, conditional handover, CHO,
time-to-exit, TTE, handover trigger, CondEventD2, SIB19, K_offset, timing advance,
Doppler, inter-satellite link, ISL, feeder link, regenerative payload, transparent
payload, on-board processing, link budget, EIRP, beam hopping, O-RAN, RAN
intelligent controller, RIC, xApp, rApp, E2 interface, E2SM-KPM, E2SM-RC, A1
policy, FlexRIC, Space O-RAN, network slicing, 5QI, S-NSSAI, URLLC, eMBB, mMTC,
terahertz communication, THz, sub-THz, molecular absorption, ITU-R P.676,
atmospheric attenuation, rain attenuation, scintillation, reconfigurable
intelligent surface, RIS, Sionna RT, ray tracing, GPU channel modeling, digital
twin, network digital twin, reinforcement learning, deep reinforcement learning,
Gymnasium, ns3-ai, multi-agent reinforcement learning, radio resource management,
space-air-ground integrated network, SAGIN, HAPS, UAV communication, V2X,
vehicle-to-everything, NR sidelink, PC5, SAE J2735, SUMO, FAPI, Small Cell Forum,
NetSimulyzer, Cesium, InfluxDB, Grafana, reproducible research, open-source
simulator.

---

## Citing this work

If the toolkit contributes to a publication, please cite the article that
describes it. Machine-readable metadata is in [CITATION.cff](CITATION.cff); the
[citation page](https://muhammaduazir69.github.io/ns3-ntn-toolkit/cite/) also has
an entry for the software release (v2.6.1 is the version the article describes).

> M. Uzair, B. G. Aldahlan, A. A. Albar, H. Althumali and A. Alghammas,
> "ns3-ntn-toolkit: An integrated ns-3 platform for 6G non-terrestrial networks,"
> *IEEE Open Journal of the Communications Society*, vol. 7, pp. 13440-13467,
> 2026, doi: [10.1109/OJCOMS.2026.3745010](https://doi.org/10.1109/OJCOMS.2026.3745010).

```bibtex
@article{uzair2026ns3ntntoolkit,
  author  = {Uzair, Muhammad and Aldahlan, Bassma G. and Albar, Ali A. and
             Althumali, Huda and Alghammas, Adnan},
  title   = {ns3-ntn-toolkit: An Integrated ns-3 Platform for 6G
             Non-Terrestrial Networks},
  journal = {IEEE Open Journal of the Communications Society},
  volume  = {7},
  pages   = {13440--13467},
  year    = {2026},
  doi     = {10.1109/OJCOMS.2026.3745010}
}
```

---

## Contact

**Muhammad Uzair**, Independent Researcher.
Email [muhammaduzairr69@gmail.com](mailto:muhammaduzairr69@gmail.com),
ORCID [0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680).
Questions and bug reports are welcome as
[GitHub issues](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/issues).

## License

GPL-2.0-only, matching ns-3. Vendored upstreams keep their own licenses: the SNS3
`satellite` module, the `mmwave` module, 5G-LENA `nr`, `netsimulyzer` and the
`ns3-ai` fork are each governed by the license in their own subtree.
