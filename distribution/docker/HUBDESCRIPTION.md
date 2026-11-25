<p align="center">
  <img src="https://raw.githubusercontent.com/Muhammaduazir69/ns3-ntn-toolkit/main/branding/out/lockup_1600x480.png" alt="ns3-ntn-toolkit" width="780"/>
</p>

# ns3-ntn-toolkit

**Pre-integrated ns-3.49 simulation platform for 6G non-terrestrial network research.** 14 integrated modules covering LEO constellations, 3GPP Rel-17/18/19 NR-NTN procedures, O-RAN + FlexRIC, a measured NTN/O-RAN application layer, sub-THz physics, NVIDIA Sionna RT, SAGIN, V2X, slicing, AI/ML and a live digital twin: all building in a single `./ns3 build`.

The C++ kernel, all 14 bundled modules, the SNS3 satellite stack, mmWave 5G NR, NetSimulyzer JSON sinks, InfluxDB observability bindings, and the FastAPI / Cesium-Live digital-twin server are baked in. Open the container and run any of the shipped reference scenarios immediately: no system-package wrangling, no missing protobuf, no Python-version drift.

---

## Quick start

```bash
# Pull the latest stable image
docker pull uzairdocker69/ns3-ntn-toolkit:latest

# Run an interactive shell with the standard ports exposed
#   8090 → FastAPI digital-twin predict API
#   3000 → Grafana (if you bring up the observability stack)
docker run --rm -it -p 8090:8090 -p 3000:3000 \
  uzairdocker69/ns3-ntn-toolkit:latest bash

# Run a reference scenario directly (44-satellite Walker shell, 5G-LENA NR cell, ISL routes)
docker run --rm uzairdocker69/ns3-ntn-toolkit:latest \
  ./ns3 run "ntn-constellation-walker-traffic --simSeconds=10"

# Pin to a specific release
docker pull uzairdocker69/ns3-ntn-toolkit:2.7.1
```

## Tags

| Tag      | Notes |
|----------|-------|
| `latest` | ns-3.49 + 5G-LENA v5.2 (November 2026). Same as `ns3.49` |
| `ns3.49` | The toolkit on ns-3.49 and 5G-LENA v5.2: upstream channel speed-ups and fixes, a corrected UE receive beam (a multi-element UE no longer loses array gain as the satellite moves), the mmWave SVD precoder fix, v5.2's link filter where exact, inter-UE spatial consistency, an opt-in Flent benchmark (`--NtnRealStackFlentTest=rrul`), Ubuntu 24.04 base |
| `2.7.1`  | Last release on ns-3.43: 2.7.0 plus the in-depth test campaign (690 test cases in 41 suites, every module) and the fixes it found: TCP timestamps on data segments (RFC 7323), TR 36.814 HTTP page model, CBR/HTTP robustness, slice-quota validation, NR PRB accounting, O-RAN gym and KPM dataset fields |
| `2.7.0`  | 3GPP Rel-17 NTN radio procedures in 5G-LENA (paired FDD, UL power control, PHR, BSR triggers, HARQ for long RTT), TR 38.821 Set-1 default link budget, exact SGP4 propagation delay, ITU-R P.676/P.840/P.834 atmosphere, BPv7 + SABR delay-tolerant routing, E2AP over TCP. Portable build (no `-march=native`) |
| `2.1.1`  | 2026-06-12 architecture-audit hardening: Rel-18 CondEventD2, E2 loop realism, KPM flow monitor across 40+ examples |
| `2.1.0`  | AI-Native ORAN-NTN release: real measured NR NTN data plane, NTN/O-RAN application layer, E2SM-KPM flow monitor, multi-tier RIC, TR 38.821 calibration |
| `2.0.x`  | Previous release line (May 2026 realism audit) |

The `2.7.1` image (`linux/amd64`, compressed size on the Tags page) is compiled with the `release` profile (-O3, no host-specific CPU flags), so it runs on any x86-64 machine.

## What's new in 2.7.1

- **In-depth tests for every module**: 690 test cases in 41 suites, each driving a running simulation with normal and failure paths and checking against published 3GPP, ITU, IETF and O-RAN reference values. All pass, and every example runs healthy with default and custom parameters.
- **Fixes the tests found**: TCP data segments now advance the timestamp echo point (RFC 7323), so retransmission timeouts no longer grow on long links; the HTTP page model follows TR 36.814 Table A.2.1.3-1; a failed HTTP connection is retried instead of stopping the run; CBR applications count only accepted packets and handle a zero interval; reproducible TDL fading streams.
- **Slicing and RAN accounting**: slice quotas refuse invalid values (NaN, negative, sum above 1); the PRB count follows the PHY, so measured downlink PRB usage can reach 100 %.
- **O-RAN**: the gym handover example reports the Python agent's steps and reward and feeds the PHY-KPM extractor in measured mode; the KPM dataset leaves CQI empty when none was reported instead of writing the out-of-range code 0.

## What's new in 2.7.0

- **NR NTN radio procedures in 5G-LENA** (21 patches over nr v3.3.1, shipped in `contrib/nr/ntn-patches`): RAR window offset by the UE-gNB RTT, HARQ feedback disabling and HARQ process pools sized for LEO round trips, NTN DRX HARQ-RTT timers, TS 38.213 uplink power control with power headroom reports and a PHR-aware PUSCH PRB limit, TS 38.321 Regular / Periodic / Padding BSR triggers, real paired FDD on bands n255 and n256 (uplink and downlink scheduled independently), frequency reuse 3 among satellite cells per TR 38.821, an SSB beam sweep with best-beam cell quality (TS 38.213 4.1, TS 38.215), a UE-directed beam on the handover target, and a robust MCS on the target until the first CSI report.
- **TR 38.821 Set-1 S-band handheld link budget is the default** in every real-stack example; `--NtnRealStackLinkBudget=Legacy` reproduces earlier releases bit for bit.
- **Orbits and propagation**: exact per-packet SGP4 light-time delay with the Sagnac term, event-driven contact transitions, ITU-R P.676 Annex 1 ray-traced slant path, P.840 cloud and P.834 apparent elevation, Ka-band DVB-S2 ground-link error model.
- **Conditional handover**: Rel-17 T1 and D1 conditions encoded in their 3GPP units.
- **Delay-tolerant SAGIN**: BPv7 bundles with per-bundle expiry, Bundle Age block and CRC-16, SABR contact-graph routing with priorities.
- **O-RAN**: per-hop E2 latency, RRU.PrbDl and uplink KPM names (TS 28.552), A1 ENFORCED / NOT_ENFORCED feedback, optional E2AP over TCP.
- **THz**: packet companion runs over a DVB-S2 ACM link for the analytic THz examples.
- **Per-UE and per-cell KPI datasets** (`ue_kpi.csv`, `cell_kpi.csv`) from the real-stack examples.

## What's new in 2.1.1: architecture-audit hardening

- **Rel-18 CondEventD2** conditional-handover trigger (moving ephemeris reference locations) joins the Rel-17 A4/T1/D1 CondEvents and the TR 38.821-studied elevation/timing-advance mechanisms; standards gate now 13/13.
- **E2 control-loop realism**: feeder-link delay on the RIC-to-node return path, opt-in control-loop-aligned indication dispatch, Unix-epoch timestamps for external RICs; the SCTP substitution is documented per module.
- **KPM everywhere**: one-call `EnableAiFlowMonitor()` wires the TS 28.552 / E2SM-KPM flow monitor in 40+ examples (auto-exported CSV + InfluxDB line protocol).
- **Mobility and physics fixes**: thz-ntn constellation geometry corrected to real SGP4 orbits, the remaining static-placeholder satellites/UEs/vehicles replaced with SGP4 / TR 38.811 / SUMO-FCD mobility, contact-gate hysteresis added.
- The `oran-ntn-full-scenario` flagship now runs on real SGP4 geometry with per-row KPM provenance (`phy-trace` vs `geometry-budget`), and **demonstrates on-orbit Space-RIC autonomy**: a regional feeder-link outage strands UEs on the receding satellite and the on-board RIC autonomously hands them over as the serving link crosses the service elevation (`space_ric_metrics.csv` reports hundreds of real geometry-driven decisions).
- **CSV-output accuracy**: measured wall-clock for flow-only scenarios, propagated-state satellite altitude, and live per-window throughput / SINR columns that previously read end-of-run aggregates mid-run.

## What's new in 2.1.0: AI-Native ORAN-NTN

- Every example runs on a **real mmWave NR NTN cell** (SpectrumPhy/MAC/RLC/PDCP/RRC/EPC) under SGP4 satellite mobility; every KPI is **measured in-band**: synthetic traffic generators and closed-form KPI shortcuts are gone.
- **`ntn-traffic` application layer**: `NtnOranApplication` 5QI traffic profiles with a 24-byte wire payload header (5QI / S-NSSAI / sequence / timestamp), measuring sink (one-way delay, RFC 3550 jitter, loss), command-and-control telemetry app.
- **`NtnOranAiFlowMonitor`**: TS 28.552 / E2SM-KPM-named KPI series, AI feature windows, anomaly events, CSV/XML/InfluxDB/E2 export.
- **Multi-tier RIC** in `oran-ntn`: on-board RT-RIC (<10 ms enforced), RIC placement with E2 latency from live slant geometry, NWDAF + transport-path controller + cross-domain SMO, optional ONNX Runtime xApp inference.
- **Regenerative payload architecture**: transparent / RU / RU+DU / full-gNB payload options, fronthaul split model (Opt 2, 7.2a, 7.2b, 8), platform latency classes, measured-trigger role switching.
- **Standards campaign**: TR 38.821 Set-1 LEO-600 S-band link-budget calibration, all six 3GPP NTN handover trigger classes (A3 / D1 / T1 / Rel-18 D2 / elevation / timing-advance), each asserted on its own firing count, orbital-theory test suite; repo gates 36/36 fidelity + 17/17 standards checks PASS.

## What's inside

| Capability | Specifics |
|---|---|
| ns-3 base | **3.49** (patched LTE for dual connectivity) + 5G-LENA nr v5.2 with the NTN patches (`contrib/nr/ntn-patches`) |
| Custom modules contributed | **14** (see the list below) |
| Data plane | real mmWave NR NTN cell under SGP4 mobility; all KPIs measured in-band |
| Combined unit + integration tests | **100+**, all passing |
| 3GPP NTN procedures | TS 38.213 TA · TS 38.331 SIB19 + UE Location Report · TS 38.321 NTN-DRX · TR 36.777 A2G · 5 NTN HO trigger classes |
| Standards calibration | TR 38.821 Set-1 LEO-600 S-band link budget · orbital-theory tests |
| 3GPP slicing | TS 23.501 + TS 22.261 default profiles (eMBB / URLLC / mMTC / V2X), S-NSSAI carried in-band |
| O-RAN xApps shipped | 16 (13 in `oran-ntn` + 3 NTN-aware in `flexric-bridge`) + optional ONNX Runtime inference |
| O-RAN RIC tiers | on-board RT-RIC (<10 ms enforced) · gateway / cloud placement, E2 latency from live geometry |
| O-RAN E2 wire | in-simulator E2AP/KPM/RC over a PER-style codec, plus a TCP/JSON stub for the same xApp logic; the live FlexRIC SCTP path is scaffolded and not demonstrated |
| Regenerative payloads | transparent / RU / RU+DU / full-gNB · FH splits (Opt 2, 7.2a, 7.2b, 8) · role switching |
| RL bridge | Gymnasium 1.0 over patched ns3-ai (Py 3.13 + NumPy 2 ready); SB3 PPO + PyG GAT |
| Channel models | TR 38.811 closed-form (default) · NVIDIA Sionna RT GPU ray-tracing (opt-in) |
| Vehicular | SUMO TraCI v20+ live + FCD-trace replay |
| Constellation feeds | live CelesTrak + Space-Track + 5 named presets (Starlink / OneWeb / Kuiper / Telesat / Iridium) |
| Observability | InfluxDB 2.7 (UDP + file) + Grafana 10.4 (4 dashboards) + NetSimulyzer 1.0 JSON |
| Live digital twin | FastAPI prediction API; CesiumJS Live mode |

### Bundled modules

`ntn-constellation` · `ntn-rrc` · `ntn-observability` · `ns3-ai-ntn` · `ntn-sagin` · `ntn-slice` · `ntn-v2x` · `ntn-sionna` · `ntn-digital-twin` · `ntn-cho` · `oran-ntn` (with `flexric-bridge`) · `thz-ntn` · `ntn-traffic` · `ntn-fapi`

Each module ships its own numerical verification harness: see the per-module READMEs linked from the [main README](https://github.com/Muhammaduazir69/ns3-ntn-toolkit#bundled-modules).

## Verification highlights

A few of the reproducible numbers shipped with the image:

- `ntn-cho`: HOs **135 ± 12** vs A3 baseline 463 ± 48 (10 seeds × 600 s × 66-sat Walker-Star); ping-pong 57 % → 0 %; Wilcoxon p < 0.005
- `oran-ntn`: 5 live xApps over 600 s → **85 074 actions, 0 reported conflicts**
- `ntn-traffic`: TR 38.821 Set-1 LEO-600 calibration: constant array-gain offset (σ < 1 dB), FSPL slope within 0.2 dB of theory
- toolkit gates: `tools/check_protocol_fidelity.py` **36/36** · `tools/check_ntn_standards.py` **17/17**
- `ntn-sionna`: 30-step LEO pass: max \|Δ path-loss\| = **0.002 dB** vs TR 38.811
- `ntn-digital-twin`: `/predict/handover` p99 = **29.9 ms** (16× under the 500 ms gate)
- `ntn-slice`: URLLC p99 = **47 ms** (mode-skip ON) vs 295 ms (forced GEO): 6.3× improvement

## Image provenance

- Built from <https://github.com/Muhammaduazir69/ns3-ntn-toolkit> (release line `2.7.0`, branch `ntn-integration-v2`) with `distribution/docker/build-release.sh`, which stages committed source only
- The image contains the published source tree plus the SNS3 satellite dependency: no papers, run outputs, or private documents

## Links

- **Source:** <https://github.com/Muhammaduazir69/ns3-ntn-toolkit>
- **GitLab mirror:** <https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit>
- **PyPI metapackage:** <https://pypi.org/project/ns3-ntn-toolkit/>
- **Live constellation demo:** <https://huggingface.co/spaces/Muhammaduazir69/ns3-ntn-toolkit-demo>
- **Issues & feature requests:** <https://github.com/Muhammaduazir69/ns3-ntn-toolkit/issues>

## Maintainer & cite

Muhammad Uzair, Independent Researcher · ORCID [0009-0002-4104-2680](https://orcid.org/0009-0002-4104-2680) · [muhammaduzairr69@gmail.com](mailto:muhammaduzairr69@gmail.com)

```bibtex
@software{ns3_ntn_toolkit_2026,
  author = {Uzair, Muhammad},
  title  = {{ns3-ntn-toolkit}: An Integrated ns-3.43 Platform for 6G Non-Terrestrial Network Simulation},
  year   = {2026},
  url    = {https://github.com/Muhammaduazir69/ns3-ntn-toolkit}
}
```

## License

GPL-2.0-only: see [LICENSE](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/main/LICENSE). Each bundled module retains its own license file (all GPL-2.0-compatible). NVIDIA Sionna (Apache-2.0) and EURECOM FlexRIC (BSD-3) are integrated as separate processes via UDP / SCTP: no source files vendored under GPL terms.
