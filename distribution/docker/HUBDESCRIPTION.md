<p align="center">
  <img src="https://raw.githubusercontent.com/Muhammaduazir69/ns3-ntn-toolkit/main/branding/out/lockup_1600x480.png" alt="ns3-ntn-toolkit" width="780"/>
</p>

# ns3-ntn-toolkit

**Pre-integrated ns-3.43 simulation platform for 6G non-terrestrial network research.** 13 integrated modules covering LEO constellations, 3GPP Rel-17/18/19 NR-NTN procedures, O-RAN + FlexRIC, sub-THz physics, NVIDIA Sionna RT, SAGIN, V2X, slicing, AI/ML and a live digital twin — all building in a single `./ns3 build`.

The C++ kernel, all 13 contrib modules, the SNS3 satellite stack, mmWave 5G NR, NetSimulyzer JSON sinks, InfluxDB observability bindings, and the FastAPI / Cesium-Live digital-twin server are baked in. Open the container and run any of the shipped reference scenarios immediately — no system-package wrangling, no missing protobuf, no Python-version drift.

---

## Quick start

```bash
# Pull the latest stable image
docker pull uzairdocker69/ns3-ntn-toolkit:latest

# Run an interactive shell with the standard ports exposed
#   8000 → FastAPI digital-twin predict API
#   3000 → Grafana (if you bring up the observability stack)
docker run --rm -it -p 8000:8000 -p 3000:3000 \
  uzairdocker69/ns3-ntn-toolkit:latest

# Run the integrated multi-module reference scenario directly
docker run --rm uzairdocker69/ns3-ntn-toolkit:latest \
  ./ns3 run "ntn-tn-integrated-analysis --algorithm=tte-aware --simTime=10"

# Pin to a specific release
docker pull uzairdocker69/ns3-ntn-toolkit:2.0.0
```

## Tags

| Tag      | Points to | Notes |
|----------|-----------|-------|
| `latest` | `2.0.0`   | Current stable; tracks the most recent tagged release |
| `2.0.0`  | first tagged release | Frozen `sha256:8b29bc9dacbf…c102`; use this for reproducibility |

Both tags resolve to the same `linux/amd64` image — **~3.4 GB compressed on the registry, ~5.2 GB extracted**.

## What's inside

| Capability | Specifics |
|---|---|
| ns-3 base | **3.43** (patched LTE for dual connectivity) |
| Custom modules contributed | **13** (see the list below) |
| Combined unit + integration tests | **80+** across 13 repos, all passing |
| 3GPP NTN procedures | TS 38.213 TA · TS 38.331 SIB19 + UE Location Report · TS 38.321 NTN-DRX · TR 36.777 A2G |
| 3GPP slicing | TS 23.501 + TS 22.261 default profiles (eMBB / URLLC / mMTC / V2X) |
| O-RAN xApps shipped | 16 (13 in `oran-ntn` + 3 NTN-aware in `flexric-bridge`) |
| O-RAN E2 wire | live FlexRIC SCTP/E2AP via Docker; CI-friendly TCP/JSON stub for the same xApp logic |
| RL bridge | Gymnasium 1.0 over patched ns3-ai (Py 3.13 + NumPy 2 ready); SB3 PPO + PyG GAT |
| Channel models | TR 38.811 closed-form (default) · NVIDIA Sionna RT GPU ray-tracing (opt-in) |
| Vehicular | SUMO TraCI v20+ live + FCD-trace replay |
| Constellation feeds | live CelesTrak + Space-Track + 5 named presets (Starlink / OneWeb / Kuiper / Telesat / Iridium) |
| Observability | InfluxDB 2.7 (UDP + file) + Grafana 10.4 (4 dashboards) + NetSimulyzer 1.0 JSON |
| Live digital twin | FastAPI prediction API at p99 ≤ 30 ms; CesiumJS Live mode |

### Bundled modules

`ntn-constellation` · `ntn-rrc` · `ntn-observability` · `ns3-ai` (fork) · `ntn-sagin` · `ntn-slice` · `ntn-v2x` · `flexric-bridge` · `ntn-sionna` · `ntn-digital-twin` · `ntn-cho` · `oran-ntn` · `thz-ntn`

Each module ships its own animated demo and numerical verification harness — see the per-module repositories linked from the [main README](https://github.com/Muhammaduazir69/ns3-ntn-toolkit#bundled-modules).

## Verification highlights

A few of the reproducible numbers shipped with the image:

- `ntn-cho`: HOs **135 ± 12** vs A3 baseline 463 ± 48 (10 seeds × 600 s × 66-sat Walker-Star); ping-pong 57 % → 0 %; Wilcoxon p < 0.005
- `oran-ntn`: 5 live xApps over 600 s → **85 074 actions, 0 reported conflicts**
- `ntn-sionna`: 30-step LEO pass — max \|Δ path-loss\| = **0.002 dB** vs TR 38.811
- `ntn-digital-twin`: `/predict/handover` p99 = **29.9 ms** (16× under the 500 ms gate)
- `ntn-slice`: URLLC p99 = **47 ms** (mode-skip ON) vs 295 ms (forced GEO) — 6.3× improvement

## Image provenance

- Built from <https://github.com/Muhammaduazir69/ns3-ntn-toolkit> on tag `v2.0.0` using `distribution/docker/Dockerfile`
- Image config digest: `sha256:3b1feb0fc64a6904499fa9f9d16513956e8cca9962bcca585f9a580b39e1c626`
- Manifest digest: `sha256:8b29bc9dacbf1296820808f86eb5e4e24863e227979dd960f31a13240c19c102`

## Links

- **Source:** <https://github.com/Muhammaduazir69/ns3-ntn-toolkit>
- **GitLab mirror:** <https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit>
- **PyPI metapackage:** <https://pypi.org/project/ns3-ntn-toolkit/>
- **Live constellation demo:** <https://huggingface.co/spaces/Muhammaduazir69/ns3-ntn-toolkit-demo>
- **Documentation site:** <https://ns3-ntn-toolkit.dev/> (coming soon)
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

GPL-2.0-only — see [LICENSE](https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/main/ns-3-dev/LICENSE). Each bundled module retains its own license file (all GPL-2.0-compatible). NVIDIA Sionna (Apache-2.0) and EURECOM FlexRIC (BSD-3) are integrated as separate processes via UDP / SCTP — no source files vendored under GPL terms.
