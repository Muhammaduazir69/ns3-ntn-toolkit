<p align="center">
  <img src="https://raw.githubusercontent.com/Muhammaduazir69/ns3-ntn-toolkit/main/branding/out/lockup_1600x480.png" alt="ns3-ntn-toolkit" width="780"/>
</p>

# ns3-ntn-toolkit

**Pre-integrated ns-3.43 simulation platform for 6G non-terrestrial network research.** 15 integrated modules covering LEO constellations, 3GPP Rel-17/18/19 NR-NTN procedures, O-RAN + FlexRIC, a measured NTN/O-RAN application layer, sub-THz physics, NVIDIA Sionna RT, SAGIN, V2X, slicing, AI/ML and a live digital twin — all building in a single `./ns3 build`.

The C++ kernel, all 15 bundled modules, the SNS3 satellite stack, mmWave 5G NR, NetSimulyzer JSON sinks, InfluxDB observability bindings, and the FastAPI / Cesium-Live digital-twin server are baked in. Open the container and run any of the shipped reference scenarios immediately — no system-package wrangling, no missing protobuf, no Python-version drift.

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

# Run the integrated multi-module reference scenario directly
docker run --rm uzairdocker69/ns3-ntn-toolkit:latest \
  ./ns3 run "ntn-tn-integrated-analysis --algorithm=tte-aware --simTime=10"

# Pin to a specific release
docker pull uzairdocker69/ns3-ntn-toolkit:2.1.1
```

## Tags

| Tag      | Notes |
|----------|-------|
| `latest` | Tracks the most recent tagged release (currently `2.1.1`) |
| `2.1.1`  | **Current stable** — 2026-06-12 architecture-audit hardening on top of the AI-Native ORAN-NTN release: Rel-18 CondEventD2 handover trigger, E2 loop realism (return-path feeder delay + control-loop-aligned dispatch), KPM flow monitor wired across 40+ examples, mobility fixes, 13/13 standards gates |
| `2.1.0`  | AI-Native ORAN-NTN release: real measured NR NTN data plane, NTN/O-RAN application layer, E2SM-KPM flow monitor, multi-tier RIC, TR 38.821 calibration |
| `2.0.x`  | Previous release line (May 2026 realism audit) |

The `2.1.1` image is **~7.5 GB extracted** (`linux/amd64`); most of that is the SNS3 satellite module's antenna-pattern and constellation data.

## What's new in 2.1.1 — architecture-audit hardening

- **Rel-18 CondEventD2** conditional-handover trigger (moving ephemeris reference locations) joins the Rel-17 A4/T1/D1 CondEvents and the TR 38.821-studied elevation/timing-advance mechanisms; standards gate now 13/13.
- **E2 control-loop realism**: feeder-link delay on the RIC-to-node return path, opt-in control-loop-aligned indication dispatch, Unix-epoch timestamps for external RICs; the SCTP substitution is documented per module.
- **KPM everywhere**: one-call `EnableAiFlowMonitor()` wires the TS 28.552 / E2SM-KPM flow monitor in 40+ examples (auto-exported CSV + InfluxDB line protocol).
- **Mobility and physics fixes**: thz-ntn constellation geometry corrected to real SGP4 orbits, the remaining static-placeholder satellites/UEs/vehicles replaced with SGP4 / TR 38.811 / SUMO-FCD mobility, contact-gate hysteresis added.
- The `oran-ntn-full-scenario` flagship now runs on real SGP4 geometry with per-row KPM provenance (`phy-trace` vs `geometry-budget`), and **demonstrates on-orbit Space-RIC autonomy**: a regional feeder-link outage strands UEs on the receding satellite and the on-board RIC autonomously hands them over as the serving link crosses the service elevation (`space_ric_metrics.csv` reports hundreds of real geometry-driven decisions).
- **CSV-output accuracy**: measured wall-clock for flow-only scenarios, propagated-state satellite altitude, and live per-window throughput / SINR columns that previously read end-of-run aggregates mid-run.

## What's new in 2.1.0 — AI-Native ORAN-NTN

- Every example runs on a **real mmWave NR NTN cell** (SpectrumPhy/MAC/RLC/PDCP/RRC/EPC) under SGP4 satellite mobility; every KPI is **measured in-band** — synthetic traffic generators and closed-form KPI shortcuts are gone.
- **`ntn-traffic` application layer**: `NtnOranApplication` 5QI traffic profiles with a 24-byte wire payload header (5QI / S-NSSAI / sequence / timestamp), measuring sink (one-way delay, RFC 3550 jitter, loss), command-and-control telemetry app.
- **`NtnOranAiFlowMonitor`**: TS 28.552 / E2SM-KPM-named KPI series, AI feature windows, anomaly events, CSV/XML/InfluxDB/E2 export.
- **Multi-tier RIC** in `oran-ntn`: on-board RT-RIC (<10 ms enforced), RIC placement with E2 latency from live slant geometry, NWDAF + transport-path controller + cross-domain SMO, optional ONNX Runtime xApp inference.
- **Regenerative payload architecture**: transparent / RU / RU+DU / full-gNB payload options, fronthaul split model (Opt 2, 7.2a, 7.2b, 8), platform latency classes, measured-trigger role switching.
- **Standards campaign**: TR 38.821 Set-1 LEO-600 S-band link-budget calibration, all five 3GPP NTN handover trigger classes (A3 / D1 / T1 / elevation / timing-advance), orbital-theory test suite; repo gates 36/36 fidelity + 12/12 standards checks PASS.

## What's inside

| Capability | Specifics |
|---|---|
| ns-3 base | **3.43** (patched LTE for dual connectivity) |
| Custom modules contributed | **15** (see the list below) |
| Data plane | real mmWave NR NTN cell under SGP4 mobility; all KPIs measured in-band |
| Combined unit + integration tests | **100+**, all passing |
| 3GPP NTN procedures | TS 38.213 TA · TS 38.331 SIB19 + UE Location Report · TS 38.321 NTN-DRX · TR 36.777 A2G · 5 NTN HO trigger classes |
| Standards calibration | TR 38.821 Set-1 LEO-600 S-band link budget · orbital-theory tests |
| 3GPP slicing | TS 23.501 + TS 22.261 default profiles (eMBB / URLLC / mMTC / V2X), S-NSSAI carried in-band |
| O-RAN xApps shipped | 16 (13 in `oran-ntn` + 3 NTN-aware in `flexric-bridge`) + optional ONNX Runtime inference |
| O-RAN RIC tiers | on-board RT-RIC (<10 ms enforced) · gateway / cloud placement, E2 latency from live geometry |
| O-RAN E2 wire | live FlexRIC SCTP/E2AP via Docker; CI-friendly TCP/JSON stub for the same xApp logic |
| Regenerative payloads | transparent / RU / RU+DU / full-gNB · FH splits (Opt 2, 7.2a, 7.2b, 8) · role switching |
| RL bridge | Gymnasium 1.0 over patched ns3-ai (Py 3.13 + NumPy 2 ready); SB3 PPO + PyG GAT |
| Channel models | TR 38.811 closed-form (default) · NVIDIA Sionna RT GPU ray-tracing (opt-in) |
| Vehicular | SUMO TraCI v20+ live + FCD-trace replay |
| Constellation feeds | live CelesTrak + Space-Track + 5 named presets (Starlink / OneWeb / Kuiper / Telesat / Iridium) |
| Observability | InfluxDB 2.7 (UDP + file) + Grafana 10.4 (4 dashboards) + NetSimulyzer 1.0 JSON |
| Live digital twin | FastAPI prediction API; CesiumJS Live mode |

### Bundled modules

`ntn-constellation` · `ntn-rrc` · `ntn-observability` · `ns3-ai` (fork) · `ntn-sagin` · `ntn-slice` · `ntn-v2x` · `flexric-bridge` · `ntn-sionna` · `ntn-digital-twin` · `ntn-cho` · `oran-ntn` · `thz-ntn` · `ntn-traffic` · `ntn-fapi`

Each module ships its own numerical verification harness — see the per-module READMEs linked from the [main README](https://github.com/Muhammaduazir69/ns3-ntn-toolkit#bundled-modules).

## Verification highlights

A few of the reproducible numbers shipped with the image:

- `ntn-cho`: HOs **135 ± 12** vs A3 baseline 463 ± 48 (10 seeds × 600 s × 66-sat Walker-Star); ping-pong 57 % → 0 %; Wilcoxon p < 0.005
- `oran-ntn`: 5 live xApps over 600 s → **85 074 actions, 0 reported conflicts**
- `ntn-traffic`: TR 38.821 Set-1 LEO-600 calibration — constant array-gain offset (σ < 1 dB), FSPL slope within 0.2 dB of theory
- toolkit gates: `tools/check_protocol_fidelity.py` **36/36** · `tools/check_ntn_standards.py` **12/12**
- `ntn-sionna`: 30-step LEO pass — max \|Δ path-loss\| = **0.002 dB** vs TR 38.811
- `ntn-digital-twin`: `/predict/handover` p99 = **29.9 ms** (16× under the 500 ms gate)
- `ntn-slice`: URLLC p99 = **47 ms** (mode-skip ON) vs 295 ms (forced GEO) — 6.3× improvement

## Image provenance

- Built from <https://github.com/Muhammaduazir69/ns3-ntn-toolkit> (release line `2.1.1`, branch `ntn-integration-v2`) using `distribution/docker/Dockerfile`
- The image contains the published source tree plus the SNS3 satellite dependency — no papers, run outputs, or private documents

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

GPL-2.0-only — see [LICENSE](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/main/LICENSE). Each bundled module retains its own license file (all GPL-2.0-compatible). NVIDIA Sionna (Apache-2.0) and EURECOM FlexRIC (BSD-3) are integrated as separate processes via UDP / SCTP — no source files vendored under GPL terms.
