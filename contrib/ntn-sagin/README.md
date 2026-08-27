# ntn-sagin

> Space-Air-Ground Integrated Network for ns-3.43 — LEO space layer, an air layer of aircraft / HAPS / UAV swarms, and a ground layer of maritime, high-speed-train and ADS-B terminals, stitched together by a multi-layer / slice-aware router and carried over a real mmwave NR NTN cell. Part of **ns3-ntn-toolkit** — [toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) / [INSTALL](INSTALL.md).

<p align="center">
  <a href="https://www.nsnam.org"><img src="https://img.shields.io/badge/ns--3-3.43-blue.svg"/></a>
  <a href="https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html"><img src="https://img.shields.io/badge/license-GPL--2.0-green.svg"/></a>
  <img src="https://img.shields.io/badge/3GPP-TR%2036.777%20v15.0.0-orange.svg"/>
  <img src="https://img.shields.io/badge/scenarios-UMa--AV%20%E2%80%A2%20RMa--AV%20%E2%80%A2%20UMi--AV-purple.svg"/>
  <img src="https://img.shields.io/badge/unit_tests-ntn--sagin-success.svg"/>
</p>

---

<p align="center">
  <img src="docs/ntn_sagin_demo.gif" alt="module live demo" width="900"/>
</p>

## Overview

Most ns-3 NTN work stops at "satellite + ground". A realistic 6G access network spans three
layers — **space** (LEO/MEO/GEO), **air** (HAPS, UAVs, commercial flights) and **ground** — and a
single session frequently traverses two or three of them. `ntn-sagin` models all three:

```
ground UE  ──►  UAV-relay  ──►  HAPS (20 km)  ──►  LEO (550 km)
              ▲ TR 36.777 A2G channel ▲                ▲ SGP4 pass + measured NR radio
```

- **Space.** LEO passes driven by SGP4 (`Sgp4MobilityModel` from `ntn-constellation`, Walker
  geometry) and, for routed scenarios, contact-graph shortest-path ISL routing.
- **Air.** HAPS station-keeping (figure-8 lemniscate + waypoint trajectory), three UAV mobility
  patterns (waypoint / patrol / search-lawnmower), and constant-altitude aeronautical cruise.
- **Ground / mobile terminals.** Maritime **AIS** vessels, **OpenSky ADS-B** aircraft, and
  **high-speed-train** mobility, each with replayable trace feeds.
- **Channel.** 3GPP TR 36.777 v15.0.0 air-to-ground path loss and LOS probability for the
  UMa-AV / RMa-AV / UMi-AV reference scenarios — available both as the `A2gChannelTr36777`
  calculator and as a real `PropagationLossModel` (`SaginA2gPropagationLossModel`) that
  attenuates actual packets on a real mmwave NR spectrum channel.
- **Path / layer selection.** A greedy max-elevation `MultiLayerRouter` (Ground → UAV → HAPS → LEO)
  and a QFI → S-NSSAI slice-aware `SaginSliceRouter` that biases the layer choice by latency budget.
  Both are path-computation helpers (inherit `Object`, return `SaginHop` vectors); they install **no
  ns-3 forwarding table** — examples carry the chosen path over static/global routing or per-leg delay.
- **Measured KPIs.** Radio examples ride a real mmwave NR NTN cell (`NtnRealStackHelper`:
  SpectrumPhy + MAC + HARQ + RLC/PDCP + RRC + EPC), so SINR / TBLER / throughput are measured
  off the PHY trace. Data-plane examples carry `NtnOranApplication` traffic whose in-band
  `NtnOranPayloadHeader` lets the `NtnOranSink` measure one-way delay, jitter, loss and goodput
  per flow — no closed-form KPI formulas, no `OnOffApplication`.

## What's new (June 2026)

See the [CHANGELOG](CHANGELOG.md) for the full history.

- **New flagship example `ntn-sagin-remote-coverage`** — a multi-MNO shared LEO cell for
  remote/rural coverage. Two MNOs serve four village households; with `--sharing=1` MNO-B's UEs
  ride MNO-A's real cell on their own S-NSSAI, and the per-MNO delivered volume — measured
  in-band by the KPM monitor — yields the cost-sharing split. With `--sharing=0` MNO-B's
  households simply have no service.
- **Measured radio everywhere.** The radio examples (HAPS relay, UAV swarm, aeronautical,
  flight-E2, maritime, HST, ADS-B) now build a real mmwave NR NTN cell via `NtnRealStackHelper`;
  SINR / TBLER / throughput come off the PHY trace, not closed-form link budgets. The TR 36.777
  A2G channel was re-homed as `SaginA2gPropagationLossModel` so it attenuates real packets
  (see `sagin-a2g-real-stack`).
- **`NtnOranApplication` traffic suite.** The data-plane examples (multihop, slice, SGP4-routed)
  replaced `OnOffApplication` with `NtnOranApplication` sources and `NtnOranSink` sinks: every
  packet carries an in-band payload header (flow identity, 5QI, S-NSSAI, timestamps), so per-slice
  one-way delay / jitter / loss are measured from the packets themselves — e.g. the mMTC slice
  steered onto the GEO leg measures ≈119 ms one-way delay, exactly the 35 786 km physical
  propagation time.

## Models, helpers & key classes

Derived from `model/*.h` and `helper/*.h`.

| Class | Header | Role |
|---|---|---|
| `HapsMobilityModel` | `haps-mobility-model.h` | Lemniscate-of-Bernoulli (figure-8) station-keeping with tanh-clamped ±50 m vertical envelope |
| `HapsTrajectoryMobilityModel` | `haps-trajectory-mobility-model.h` | Waypoint/segment-driven HAPS trajectory playback |
| *(trace)* | `haps-trajectory-trace.h` | Loads / replays HAPS trajectory traces |
| `UavWaypointMobilityModel` | `uav-mobility-models.h` | RNG-driven random-waypoint UAV |
| `UavPatrolMobilityModel` | `uav-mobility-models.h` | Linear A↔B patrol at constant speed |
| `UavSearchPatternMobilityModel` | `uav-mobility-models.h` | Lawnmower search scan with configurable lane spacing |
| `AeronauticalMobilityModel` | `aeronautical-scenario.h` | Constant-altitude / constant-ground-speed cruise |
| `AisMobilityModel` | `ais-mobility-model.h` | Maritime vessel mobility (AIS-style) |
| *(trace)* | `ais-maritime-trace.h` | AIS message trace feed for vessel terminals |
| `OpenSkyMobilityModel` | `opensky-mobility-model.h` | ADS-B aircraft mobility from OpenSky-style feeds |
| *(trace)* | `opensky-adsb-trace.h` | OpenSky ADS-B trace loader |
| `HstMobilityModel` | `hst-mobility-model.h` | High-speed-train mobility along a track |
| *(trace)* | `hst-trace.h` | HST position/speed trace feed |
| `A2gChannelTr36777` | `a2g-channel-tr36777.h` | TR 36.777 v15.0.0 path loss (LOS + NLOS floor) and LOS probability for UMa-AV / RMa-AV / UMi-AV |
| `SaginA2gPropagationLossModel` | `sagin-a2g-propagation-loss-model.h` | The TR 36.777 A2G model as a real ns-3 `PropagationLossModel`; chained onto a real mmwave NR spectrum channel it attenuates actual packets (returns the excess over free space to avoid double-counting) |
| `MultiLayerRouter` | `multi-layer-router.h` | Greedy max-elevation Ground→UAV→HAPS→LEO **path computation** (returns a `SaginHop` vector; installs no forwarding table); pluggable `SaginScoreCallback` for RL/external scoring. **Optionally congestion-aware** (`SetLinkLoadSource` + `SetCongestionAware`, audit SAGIN-2): candidates carry link capacity and TxQueue occupancy, a filling link is penalised in the score, and one at or above the cutoff is rejected the same way a below-horizon hop is. **Default OFF**, and a link whose load is unknown is scored on elevation rather than assumed idle. Without it the router cannot see a saturated ISL at all and will steer every slice onto the same satellite. |
| `SaginSliceRouter` | `sagin-slice-router.h` | 5G QFI → S-NSSAI → `SliceProfile` mapping; computes a layer/hop choice (returns a `SaginHop` vector; installs no forwarding table) biased by latency budget + GEO allowance |
| `SaginHelper` | `sagin-helper.h` | Factory façade: builds a ready-to-route 4-layer scene in one call |

> ## Store-and-forward across contact gaps (audit SAGIN-2)
>
> `SaginCustodyQueue` (`model/sagin-custody-queue.h`) holds traffic while the next hop is out
> of contact and drains it, in arrival order, when the contact opens. Before it, a grep for
> queue, congestion, buffer, store-and-forward, DTN or bundle across this module and
> `ntn-constellation` returned nothing relevant — so a router that found no path simply dropped,
> and **disconnected-operation scenarios, the main reason contact-graph routing exists in the
> space community, could not be run at all.**
>
> It reports what the store-and-forward bought: `GetForwardedImmediately()` against
> `GetForwardedFromCustody()`, plus `GetExpired()`, `GetDroppedForSpace()` and
> `GetMaxCustodyDelay()` — the latency cost traded against the loss avoided. Overflow evicts the
> **oldest** item (closest to expiry anyway) and counts it; a forwarder that refuses a packet
> keeps custody rather than having it counted as delivered.
>
> **Scope:** this is a custody queue, not a DTN stack. No RFC 5050 bundle format, no custody
> signalling, no fragmentation, and it does not claim them.

> ## Regenerative payload: routing label only (audit SAGIN-3)
>
> `RegenMode{bent_pipe, regen_du, regen_cu, regen_full}` in `contact-graph-router.h` is used
> **purely as a transit filter** in `ShortestPath`. A repo-wide grep for `SetRegenMode`,
> `regen_du` or `regen_full` outside that file and its own unit test returns nothing — no
> `ntn-sagin` or `ntn-v2x` code ever sets it. Greps for `Xn` / `XnAP` across both modules return
> zero hits, and in `sagin-sgp4-routed-traffic` the satellite nodes get only
> `InternetStackHelper` + P2P devices: they are plain IP routers, not gNBs.
>
> **SAGIN-3 UPDATE — the Xn procedure now exists.** `NtnXnapHeader` and `NtnXnHandover` (in
> `ntn-constellation`) implement the TS 38.300 §9.2.3 sequence between two regenerative
> satellites: HANDOVER REQUEST → HANDOVER REQUEST ACKNOWLEDGE → SN STATUS TRANSFER → UE CONTEXT
> RELEASE, carrying the paired NG-RAN node UE XnAP IDs, the target NR CGI and the PDCP SN/HFN
> that SN Status Transfer exists to move. Every leg crosses the ISL, so the procedure costs four
> one-way traversals — 20.01 ms at 1500 km of separation, measured. `RegenMode` now selects
> BEHAVIOUR and not just Dijkstra transit: a bent-pipe payload has no on-board gNB, so it
> refuses to originate an Xn handover and answers HANDOVER PREPARATION FAILURE to any it
> receives rather than going silent.
>
> **Still not modelled:** PHY/MAC/PDCP/RRC re-origination on the satellite as part of *this*
> path (`NtnRealStackHelper` does build a real `NrGnbNetDevice` on the satellite under
> `PayloadOption::FullGnb`, but the two are not yet wired together), and the wire format is an
> ns-3 Header with toolkit-internal message-type values, **not** ASN.1 APER with TS 38.423
> procedure codes — an asn1c peer would not decode it. Previously: There is no PHY/MAC/PDCP/RRC re-origination, no
> on-board scheduling, and no inter-satellite mobility signalling (Xn Handover Request, SN Status
> Transfer, path switch). **A regenerative-payload handover study cannot be run in this toolkit
> today**, and the enum should not be read as evidence that it can. Treat the ISL path as
> **transparent payload only**.
>
> Closing it is staged work: instantiate a real `NrGnbNetDevice` on the satellite for
> `regen_du`/`regen_full`, carry the ISL as a transport bearer between them, add a minimal XnAP
> message set over that ISL, and have `SetRegenMode` select which stack is built rather than only
> filtering the Dijkstra transit set. None of that is done.

## Examples

Twelve example programs ship under `examples/`. Build any of them with `./ns3 build <NAME>`; the
binary lands at `build/contrib/ntn-sagin/examples/ns3.43-<NAME>-default`.

They fall into two families:

- **Real-cell examples** (`ntn-sagin-remote-coverage`, `sagin-haps-leo-relay`, `sagin-uav-swarm`,
  `sagin-a2g-real-stack`, `sagin-aeronautical`, `sagin-flight-leo-e2`,
  `sagin-maritime-leo-traffic`, `sagin-hst-leo-traffic`, `sagin-adsb-flight-leo-traffic`) build a
  real mmwave NR NTN cell with `NtnRealStackHelper`; measured SINR / TBLER / throughput are printed
  to the console and an honest `sim_health.csv` is written to `--outputDir`.
- **Data-plane examples** (`sagin-multihop-traffic`, `sagin-slice-traffic`,
  `sagin-sgp4-routed-traffic`) build a point-to-point IP data plane with geometry-driven error
  models and carry `NtnOranApplication` traffic; the `NtnOranSink` prints measured in-band KPIs
  (one-way delay, jitter, loss, goodput) to the console.

---

### ntn-sagin-remote-coverage  *(flagship)*

Remote/rural coverage with **multi-MNO infrastructure sharing** on a real LEO cell. A remote
village has four households: two subscribe to MNO-A, two to MNO-B; only MNO-A has a satellite
overhead (SGP4 Walker LEO at 550 km). With `--sharing=1` the cell is shared — MNO-B's UEs ride
MNO-A's real cell on their own S-NSSAI (SST 1 / SD 0xB) — and the per-MNO delivered volume,
measured in-band, yields the cost-sharing split. With `--sharing=0` MNO-B's households have no
service. Run both and compare.

```bash
./ns3 run "ntn-sagin-remote-coverage --sharing=0"
./ns3 run "ntn-sagin-remote-coverage --sharing=1"
```

**Outputs:** console report — per-MNO households served, delivered Mbps, measured cost-sharing
split, village coverage and cell SINR / throughput / one-way delay; `sim_health.csv` and
`kpm_series.csv` in `--outputDir`.
**Key args:** `--simSeconds`, `--sharing`, `--outputDir`.

### sagin-haps-leo-relay

Ground-UE → UAV-relay → HAPS → LEO multi-layer route. The `MultiLayerRouter` computes the layered
path every second and logs per-hop elevation/range, while the access hop (ground UE ↔ UAV relay)
is a real mmwave NR cell carrying the TR 36.777 A2G channel — access SINR/TBLER/throughput are
measured off the PHY trace.

> **The route is a geometry trace, not an actuated path (audit SAGIN-6).** `Route()` is called
> every second and its result goes to the CSV and nowhere else. The real stack is a single
> UAV-to-ground access cell whose behaviour does not depend on which relay layer was chosen, so
> the route columns cannot be cross-checked against delivered traffic and no measured KPI here
> validates the routing decision. Earlier prose said the router "selects the path", which reads
> as actuation. Read these columns as *what was visible, where, second by second*.
>
> For a route that is actuated, use **`sagin-multihop-traffic`**: it gates a P2P leg per candidate
> LEO (`ApplyRoute` → `SetGatedLink` → `RecomputeRoutingTables`), so its route column can be
> validated against per-leg `MacRx` bytes. `sagin-aeronautical` has the same geometry-only shape
> as this one.

```bash
./ns3 run "sagin-haps-leo-relay --simTime=30 --csv=sagin.csv"
```

**Outputs:** `<outputDir>/<csv>` with columns
`time_s,n_hops,uav_el_deg,uav_range_km,haps_el_deg,haps_range_km,leo_el_deg,leo_range_km`;
`sim_health.csv` in `--outputDir`.
**Key args:** `--simTime`, `--numUes`, `--uavAlt`, `--gnbTxDbm`, `--csv`, `--outputDir`.

### sagin-uav-swarm

8-UAV swarm (2 random-waypoint, 4 patrol, 2 search-pattern) served by a ground gNB over a real
mmwave NR cell carrying the TR 36.777 RMa-AV A2G channel. Each UAV is a UE; per-UAV measured SINR
is printed each second as the swarm manoeuvres.

```bash
./ns3 run "sagin-uav-swarm --simTime=30 --fcGHz=2.0"
```

**Outputs:** per-UAV measured-SINR table on the console; `sim_health.csv` in `--outputDir`.
**Key args:** `--simTime`, `--fcGHz`, `--gnbTxDbm`, `--outputDir`.

### sagin-a2g-real-stack

Channel-plugin recipe: the TR 36.777 A2G model re-homed as `SaginA2gPropagationLossModel` and
chained onto a real mmwave NR air interface. A UAV/HAPS aerial platform is the gNB, ground nodes
are UEs; the LOS-vs-NLOS branch shows up as a real measured SINR / throughput gap.

```bash
./ns3 run "sagin-a2g-real-stack --duration=12 --link=NLOS --scenario=UMa_AV"
```

**Outputs:** measured SINR/throughput summary on the console; `sim_health.csv` in `--outputDir`.
**Key args:** `--duration`, `--numUes`, `--uavAlt`, `--gnbTxDbm`, `--freqGhz`,
`--link` (LOS | NLOS), `--scenario` (UMa_AV | RMa_AV | UMi_AV), `--outputDir`.

### sagin-aeronautical

Passenger broadband on a commercial flight: a great-circle cruise leg at 11 km / 250 m/s served by
a passing LEO over a real mmwave NR NTN cell. The `MultiLayerRouter` logs the per-hop geometry;
the radio KPIs are measured off the PHY trace.

```bash
./ns3 run "sagin-aeronautical --simTime=30 --cruise=11000 --speed=250"
```

**Outputs:** `<outputDir>/<csv>` with columns
`time_s,ac_x_m,ac_alt_m,leo_x_m,leo_z_m,leo_el_deg,slant_km`; `sim_health.csv` in `--outputDir`.
**Key args:** `--simTime`, `--numUes`, `--cruise`, `--speed`, `--leoAltKm`, `--satEirpDbm`,
`--csv`, `--outputDir`.

### sagin-flight-leo-e2

Commercial long-haul flight served by a passing LEO over a real mmwave NR NTN cell, with OAI-style
O-RAN E2-KPM indications emitted from the satellite gNB to a Near-RT RIC. Each reporting period the
KPM report carries the measured DL SINR / TBLER / throughput plus real elevation / Doppler / delay
geometry; a mid-pass elevation descent drops the measured SINR below the in-service threshold so
the handover CSV shows a real RELEASE event.

```bash
./ns3 run "sagin-flight-leo-e2 --simSeconds=30 --reportMs=500"
```

**Outputs:** `sagin-flight-leo-kpm.csv` (KPM indications), `sagin-flight-leo-handover.csv`
(ACQUIRE/RELEASE events) and `sim_health.csv`, all in `--outputDir`.
**Key args:** `--simSeconds`, `--reportMs`, `--minElevDeg`, `--cruiseAltM`, `--cruiseSpeed`,
`--satAltM`, `--satSpeed`, `--satEirpDbm`, `--outputDir`.

### sagin-multihop-traffic

Real end-to-end multi-hop forwarding Ground→UAV→HAPS→LEO: three point-to-point hops joined by an
IPv4 stack with global routing, each hop with its own geometry-driven error model (TR 36.777 A2G,
free space, free space + min-elevation gate). An `NtnOranApplication` UDP flow is forwarded
hop-by-hop; the end-to-end PDR / delay / jitter / goodput are measured at the `NtnOranSink` from
the in-band payload header and track the live geometry.

```bash
./ns3 run "sagin-multihop-traffic --simSeconds=60 --dataRateMbps=20 --eirpDbm=43"
```

**Outputs:** per-second hop/path-loss/goodput table and a measured end-to-end summary
(txPackets, rxPackets, PDR, mean delay, jitter, goodput) on the console.
**Key args:** `--simSeconds`, `--freqGHz`, `--dataRateMbps`, `--packetBytes`, `--uavAltM`,
`--hapsAltKm`, `--leoAltKm`, `--satSpeed`, `--linkCapacityMbps`, `--eirpDbm`.

### sagin-slice-traffic

Cross-layer network-slice steering via `SaginSliceRouter` with real per-slice traffic. Three
concurrent flows (URLLC QFI 1 / 5QI 82, eMBB QFI 7 / 5QI 2, mMTC QFI 50 / 5QI 9) are steered onto
HAPS / LEO / GEO legs by latency budget; each leg's channel delay is the physical one-way
propagation time for that layer's altitude. Per-slice delay / jitter / loss are measured by
`NtnOranSink` from the in-band header — the mMTC slice on the GEO leg measures ≈119 ms one-way
delay (35 786 km / c), while the tight-URLLC slice is forced down to the HAPS layer.

```bash
./ns3 run "sagin-slice-traffic --simSeconds=60 --dataRateMbps=10"
```

**Outputs:** slice→layer decision table plus per-slice measured results (rx bytes, one-way delay,
jitter, loss, goodput) on the console.
**Key args:** `--simSeconds`, `--dataRateMbps`, `--packetBytes`, `--hapsAltKm`, `--leoAltKm`,
`--geoAltKm`.

### sagin-maritime-leo-traffic

Maritime AIS vessel terminal (`AisMobilityModel`) served by a passing LEO over a real mmwave NR
NTN cell. DL SINR/TBLER/throughput are measured off the PHY trace as the vessel sails and the
satellite crosses overhead.

```bash
./ns3 run "sagin-maritime-leo-traffic --simSeconds=30 --vesselSpeedKn=20"
```

**Outputs:** per-second link probe and measured summary on the console; `sim_health.csv` in
`--outputDir`.
**Key args:** `--simSeconds`, `--leoAltKm`, `--satSpeed`, `--freqGHz`, `--satEirpDbm`,
`--vesselSpeedKn`, `--outputDir`.

### sagin-hst-leo-traffic

High-speed-train terminal (`HstMobilityModel`, TR 38.901-style track) served by a passing LEO over
a real mmwave NR NTN cell, with measured DL KPIs as the train races along the track.

```bash
./ns3 run "sagin-hst-leo-traffic --simSeconds=30 --trainSpeedKmh=500"
```

**Outputs:** per-second link probe and measured summary on the console; `sim_health.csv` in
`--outputDir`.
**Key args:** `--simSeconds`, `--leoAltKm`, `--satSpeed`, `--freqGHz`, `--satEirpDbm`,
`--trainSpeedKmh`, `--outputDir`.

### sagin-adsb-flight-leo-traffic

In-flight-connectivity terminal aboard a commercial aircraft replayed from an ADS-B trace
(`OpenSkyMobilityModel`), served by a passing LEO over a real mmwave NR NTN cell with measured DL
KPIs.

```bash
./ns3 run "sagin-adsb-flight-leo-traffic --simSeconds=30 --cruiseAltM=11000"
```

**Outputs:** per-second link probe and measured summary on the console; `sim_health.csv` in
`--outputDir`.
**Key args:** `--simSeconds`, `--leoAltKm`, `--satSpeed`, `--freqGHz`, `--satEirpDbm`,
`--cruiseAltM`, `--cruiseSpeedMps`, `--outputDir`.

### sagin-sgp4-routed-traffic

SGP4-driven SAGIN with graph shortest-path ISL routing. Composes `ntn-constellation`
(`Sgp4MobilityModel` + `ContactGraphScheduler` + `ContactGraphRouter`) with a real ns-3 IP data
plane whose space links follow the live contact graph: contacts bring IPv4 interfaces up/down,
channel delays follow the real slant range, and global routing re-routes the live
`NtnOranApplication` flow as satellites hand over. The measured in-band goodput / one-way delay
tracks the actual end-to-end connectivity, cross-checked each second against the router's Dijkstra
path.

```bash
./ns3 run "sagin-sgp4-routed-traffic --simSeconds=120 --altKm=550 --inclDeg=53 --minElevDeg=10"
```

**Outputs:** per-second contact-graph/path/goodput trace and a measured summary (GSL/ISL
transitions, delivered packets, in-band one-way delay / jitter / loss) on the console.
**Key args:** `--simSeconds`, `--altKm`, `--inclDeg`, `--leadSeconds`, `--minElevDeg`,
`--dataRateMbps`, `--packetBytes`, `--spaceCapMbps`, `--eirpDbm`, `--minSnrDb`.

### ntn-sagin-orphan-mobility-showcase

Gives the toolkit's finished-but-unused SAGIN classes a real home: a HAPS flying a
programmatic loiter racetrack (`sagin::HapsTrajectoryMobilityModel`, interpolated to
`Simulator::Now()`), a real ground → HAPS → LEO packet plane, and a 3D scene. Doppler and
relay geometry are computed from the live positions over the run.

```bash
./ns3 run "ntn-sagin-orphan-mobility-showcase --duration=120"
```

**Outputs:** per-step relay-geometry / Doppler trace and a measured summary on the console;
the HAPS trajectory is written to `orphan-haps-trajectory.csv`; optional 3D scene files when
enabled — NetSimulyzer JSON (`--netSim`) and Cesium CZML (`--czml`).
**Key args:** `--duration` (sim duration, s; default 120), `--netSim` (NetSimulyzer 3D JSON
output path; empty = off), `--czml` (Cesium CZML 3D output path; empty = off).

## Build, run & test

```bash
# Configure (from the ns-3-dev root) and build the module + its examples.
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Build / run a single example.
./ns3 build ntn-sagin-remote-coverage
./ns3 run "ntn-sagin-remote-coverage --sharing=1"

# Run the unit-test suite for this module (32 test cases).
./test.py -s ntn-sagin
```

For module setup (dependencies, enabling modules, environment), see [INSTALL.md](INSTALL.md). For the full toolkit, see [ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).

## License & author

GPL-2.0-only — see [LICENSE](LICENSE).

**Muhammad Uzair**, Independent Researcher.

```bibtex
@misc{uzair2026ntnsagin,
  author = {Uzair, Muhammad},
  title  = {ntn-sagin: Space-Air-Ground Integrated Network for 6G NTN Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ntn-sagin}
}
```

## Acknowledgements

3GPP RAN1 (TR 36.777 work item) · ns-3 mobility module · OpenSky Network ADS-B · NIST NetSimulyzer for downstream visualisation.
