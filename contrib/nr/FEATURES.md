@page features 5G-LENA Features

<!--
Copyright (c) 2025 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)

SPDX-License-Identifier: GPL-2.0-only
-->

# 5G-LENA Features Summary

*(Centre Tecnològic de Telecomunicacions de Catalunya – CTTC)*

---

## 📘 Overview

This document summarizes the main features currently **supported**, **partially supported**, or **planned** in the
**[5G-LENA](https://5g-lena.cttc.es/) (ns-3 nr)** simulator.

It is meant to be a compact, “at-a-glance” view, complementary to the detailed technical documentation:

- [NR Module Manual](https://cttc-lena.gitlab.io/nr/manual/nr-module.html)
- [Doxygen Reference (ns-3-dev)](https://cttc-lena.gitlab.io/nr/html/index.html)
- [CTTC-LENA GitLab Repository](https://gitlab.com/cttc-lena/nr)

---

## Table of Contents

- [Features per layer:](#features-per-layer)
  - [PHY](#phy)
  - [MAC](#mac)
  - [RLC / PDCP / RRC / Core](#rlc--pdcp--rrc--core)
  - [Application Layer - Traffic Models](#application-layer---traffic-models)
- [Calibration & Testing:](#calibration--testing)
  - [Calibration & Deployment Models](#calibration--deployment-models)
  - [CI & Testing Framework](#ci--testing-framework)
- [5G-LENA Extensions (Sidelink and V2X, O-RAN, NR-U)](#5g-lena-extensions-sidelink-and-v2x-o-ran-nr-u):
  - [Sidelink and NR-V2X](#sidelink-and-nr-v2x-extension)
  - [Fronthaul / O-RAN integration](#fronthaul--o-ran-integration)
  - [NR-U (Unlicensed NR)](#nr-u-unlicensed-nr)
- [Planned Roadmap](#-planned-roadmap-2025--2028)
---

## Features per layer:

### PHY

| Feature                                                                                                          | Status      | Notes                                                                                        |
|------------------------------------------------------------------------------------------------------------------|-------------|----------------------------------------------------------------------------------------------|
| [OFDMA (DL/UL)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#resource-allocation-model-ofdma-and-tdma)   | ✅ Supported | 3GPP-compliant slot/symbol allocation; variable TTI and single analog beam capability        |
| [TDMA (DL/UL)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#resource-allocation-model-ofdma-and-tdma)    | ✅ Supported | Configurable symbol granularity; variable TTI                                                |
| [Numerologies (μ = 0–4)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#frame-structure-model)             | ✅ Supported | Per-BWP numerology configuration (sub-carrier spacing and symbol duration)                   |
| [TDD and FDD](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#duplexing-schemes)                            | ✅ Supported | Per-cell configurable TDD pattern; automatic UL carrier advertisement and UE UL BWP derivation in FDD (**NR-v5.1**) |
| [Time-multiplex of shared/control](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#tdd-model)               | ✅ Supported | PDCCH/PDSCH in same slot; PUCCH/PUSCH in same slot                                           |
| [LDPC coding](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#data-phy-error-model)                         | ✅ Supported | 3GPP TS 38.212 base graphs 1 & 2                                                             |
| [Code block segmentation](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#data-phy-error-model)             | ✅ Supported | 3GPP TS 38.212                                                                               |
| [MCS tables](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#adaptive-modulation-and-coding-model)          | ✅ Supported | Per BWP; 3GPP TS 38.214 MCS Tables 1 & 2 (Up to 256-QAM)                                     |
| [Sounding Reference Signal](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#srs-transmission-and-reception) | ✅ Supported | UE-specific scheduled SRS (configurable offset and periodicity)                              |
| [CSI-RS & CSI-IM](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#csi-rs-and-csi-im)                        | ✅ Supported | channel estimation and interference measurement (**NR-v4.0**)                                |
| [SU-MIMO](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#mimo)                                             | ✅ Supported | 3GPP-compliant; Up to rank-4 and 32 antenna ports (**NR-v3.0**)                              |
| [Sub-band CSI feedback](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#cqi-feedback)                       | ✅ Supported | Sub-band CQI and PMI (**NR-v4.0**)                                                           |
| [Uplink power control](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#uplink-power-control)                | ✅ Supported | 3GPP compliant; PUSCH/PUCCH/SRS                                                              |
| [Attach to max-RSRP gNB](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#architecture)                      | ✅ Supported | UE attaches to gNB with highest RSRP (**NR-v4.0**)                                           |
| **🟦 Antenna models**                                                                                            |             |                                                                                              |
| [Phased array models](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#beamforming-model)                    | ✅ Supported | 3GPP UPAs; dual-pol; multi-port; multi-panel UE; spatial channel models                      |
| [ns-3 legacy antenna models](https://www.nsnam.org/docs/models/html/antenna.html#antennamodel)                   | ✅ Supported | Isotropic/Cosine/Parabolic/3GPP; non-spatial channel models; (**NR-v4.0**)                   |
| **🟨 Beamforming & Precoding**                                                                                   |             |                                                                                              |
| [Analog ideal beamforming](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#beamforming-model)               | ✅ Supported | Kronecker, Direct path, Cell scan, etc.                                                      |
| [Analog realistic beamforming](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#beamforming-model)           | ✅ Supported | Realistic BF (SRS-based, imperfect CSI)                                                      |
| [Digital Precoding](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#mimo)                                   | ✅ Supported | 3GPP Type-I; codebook/ideal; closed-loop; based on CSI feedback (PMI, RI, CQI) (**NR-v3.0**) |
| [PMI/RI search method](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#mimo)                                | ✅ Supported | 3GPP Codebook, Ideal, Maleki, Sasaoka, Fast                                                  |
| [Beam management (sweep/track)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#beamforming-model)          | ⚙️ Partial  | Simplified model; no SSB blocks                                                              |
| **🟩 Supported channel models**                                                                                  |             |                                                                                              |
| [3GPP TR 38.901](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)                         | ✅ Supported | UMi/UMa/RMa/InH/InF/V2V/NTN (**NR-v2.1**), O2I penetration loss; shadowing; fast fading      |
| [Spatial consistency](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)                    | ✅ Supported | 3GPP TR 38.901  Procedure A; temporal consistency                                            |
| [NYUSIM](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)                                 | ✅ Supported | NYU mmWave/THz channel; based on real NYU measurements (**NR-v4.0**)                         |
| [Fluctuating Two-Ray (FTR)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)              | ✅ Supported | FTR model built on top of 3GPP pathloss/channel for fast abstraction; (**NR-v4.0**)          |
| [Sionna Ray Tracing](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)                     | ✅ Supported | Sionna-based channel model for the precise simulation of radio wave propagation              |
| [Transmit filters](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)                     | ✅ Supported | Skip the channel of gNB-gNB and UE-UE links, or between NR and non-NR devices (**NR-v5.2**)   |
| [Legacy ns-3 non-spatial/Friis channel](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-channel-helper)  | ✅ Supported | ns-3 legacy non-spatial channel models; Ideal for large-scale simulations (**NR-v4.0**)      |

---

### MAC

| Feature                                                                                                                           | Status      | Notes                                                                                       |
|-----------------------------------------------------------------------------------------------------------------------------------|-------------|---------------------------------------------------------------------------------------------|
| **🟦 SCHEDULERS**                                                                                                                 |             |                                                                                             |
| [PF / RR / MR](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#scheduler)                                                    | ✅ Supported | Temporal fairness                                                                           |
| [QoS-aware scheduler](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#qos-schedulers)                                        | ✅ Supported | 5QI-aware; LC byte assignment (**NR-v2.5**)                                                 |
| [Random TDMA / OFDMA schedulers](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#resource-allocation-model-ofdma-and-tdma)   | ✅ Supported | Stress interference testing (**NR-v4.1**)                                                   |
| [AI Reinforcement-Learning scheduler](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#rl-based-scheduler)                    | ✅ Supported | Via ns3-gym (**NR-v4.0**); per-UE observations; DDQN, A2C, DDPG and PPO agents (**NR-v5.2**) |
| [Sub-band CQI-aware scheduling](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#sub-band-scheduling)                         | ✅ Supported | Optional sub-band CQI aware scheduling(**NR-v4.0**)                                         |
| **🟩 CORE MAC FEATURES**                                                                                                          |             |                                                                                             |
| [HARQ](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#harq)                                                                 | ✅ Supported | IR and CC methods; configurable max number of ReTx (**NR-v5.2**); retx reshaping and data in the RBGs left free (**NR-v5.2**) |
| [Multiple HARQ processes per UE](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#harq)                                       | ✅ Supported | Configurable; defaults to 16                                                                |
| [3GPP-compliant processing times](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#timing-relations)                          | ✅ Supported | NR K0/K1/K2 and N0/N1/N2 processing times                                                   |
| [Adaptive Modulation and Coding (AMC)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#adaptive-modulation-and-coding-model) | ✅ Supported | Error-model based and Shannon-based AMC                                                     |
| UL grant-based access                                                                                                             | ✅ Supported | UL grant-based access scheme with scheduling request (SR)                                   |
| UL BSR                                                                                                                            | ✅ Supported | 3GPP-compliant UL buffer status reporting; padding BSR (**NR-v5.2**) |
| [BSR / SR / CQI / RI / PMI processing](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#scheduler)                            | ✅ Supported | BSR multiplexed with PUSCH; SR/CQI/RI/PMI in PUCCH                                          |
| **🟨 ADVANCED FEATURES**                                                                                                          |             |                                                                                             |
| [Notching mask](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#ufa-aka-notching)                                            | ✅ Supported | UFA masks per-BWP and per-cell; configurable notched RBGs                                   |
| [CC / BWP managers](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#bwp-manager)                                             | ✅ Supported | Multi-carrier; FDM of numerologies; CC/BWP routing                                          |
| [Carrier Aggregation (CA)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#bwp-manager)                                      | ✅ Supported | Multiple CC/BWPs with flexible mapping                                                      |
| [RACH](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#mac-layer)                                                            | ✅ Supported | Contention-based for initial access; RA preamble, RAR, MSG3; target RACH configuration applied on handover; staggered initial attachment (**NR-v5.1**) |
| [Fronthaul Control](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#fronthaul-control)                                       | ✅ Supported | 7.2x split; Limited-capacity FH link; Dropping, Postponing, Optimize MCS/RBs; (**NR-v3.3**) |

---

### RLC / PDCP / RRC / Core

| Feature                                                                                | Status       | Notes                                                                                                                                                          |
|----------------------------------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [RLC AM / UM / TM](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#rlc-layer)     | ✅ Supported | LTE-based; TS 38.322 conformance rework, cross-epoch PDU discard, TM TxDrop trace (**NR-v5.1**)                                                                |
| [PDCP](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#pdcp-layer)                | ✅ Supported | Basic header compression                                                                                                                                       |
| [5QI handling](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#qos-schedulers)    | ✅ Supported | QoS per flow; PDCP discard timer; RLC reordering window timer                                                                                                  |
| [SDAP](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#sdap-layer)                | ⚙️ Ongoing   | Maybe will be contributed soon                                                                                                                                 |
| [RRC](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#rrc-layer)                  | ✅ Supported | Ideal and real RRC; RLF and handover (**NR-v5.0**); TR 36.839 handover failure model (**NR-v5.1**) |
| [Multi-flow per UE](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#architecture) | ✅ Supported | Independent bearers                                                                                                                                            |
| [EPC/5GC integration](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#epc-model)  | ✅ Supported | Via LTE-EPC model; RAN-only helper to connect external cores; GTP-U PDU Session Container (**NR-v5.2**)                                                         |
| Multiple core networks and roaming                                                     | ✅ Supported | Local breakout and home-routed roaming; S1-AP UE Context Release; external 5G core through nr-bridge (**NR-v5.2**)                                             |
| Unstructured PDU sessions (non-IP)                                                     | ✅ Supported | 3GPP TS 23.501 5.6.1, one unstructured non-IP protocol per session (**NR-v5.2**)                                                                               |

---

### Application layer - Traffic Models

| Feature                                                                                                                            | Status       | Notes                                                                 |
|------------------------------------------------------------------------------------------------------------------------------------|--------------|-----------------------------------------------------------------------|
| [NGMN apps (FTP, video, gaming, VoIP)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#ngmn-mixed-and-3gpp-xr-traffic-models) | ✅ Supported | NGMN-based traffic generators; NGMN mixed traffic model (**NR-v2.4**); per-frame VoIP, SeqTsSize header (**NR-v5.2**) |
| [3GPP XR (VR/AR/CG)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#ngmn-mixed-and-3gpp-xr-traffic-models)                   | ✅ Supported | 3GPP TR 38.838 XR traffic profiles; multi-flow XR models              |
| [3GPP FTP (Model 1)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#ngmn-mixed-and-3gpp-xr-traffic-models)                   | ✅ Supported | 3GPP FTP Model 1; TR 36.814 (**NR-v1.2**)                             |
| [ns-3 3GPP HTTP](https://www.nsnam.org/docs/models/html/applications.html#)                                                        | ✅ Supported | ns-3 3GPP HTTP model                                                  |

---

# Calibration & Testing

## Calibration & Deployment Models

| Feature                                                                                                                  | Status      | Notes                                                                                     |
|--------------------------------------------------------------------------------------------------------------------------|-------------|-------------------------------------------------------------------------------------------|
| [Calibration examples](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#cttc-nr-3gpp-calibration)                    | ✅ Supported | Indoor/Outdoor (R1-1709828/RP-180524) and R1-1707360 (3GPP SU-MIMO) (**NR-v4.1**)         |
| [Hexagonal wrap-around](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#cttc-nr-3gpp-calibration)                   | ✅ Supported | Wrap-around for up to 5 rings, 37-site/111-cell topology for RMa/UMa/UMi calibration; ring-1-only picocell placement (TR 36.839 dense-A) (**NR-v5.1**) |
| [Wraparound model](https://www.nsnam.org/docs/models/html/spectrum.html#wraparound-models)                               | ✅ Supported | Simulates outer interference to the edge devices without having to simulate outer devices |
| [FastFadingConstantPositionMobilityModel](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#cttc-nr-3gpp-calibration) | ✅ Supported | Calibration for static users with temporal fading                                         |
| RandomDirectionDisc2dMobilityModel                                                                                       | ✅ Supported | Random-direction mobility bounded by a disc; disc UE placement in the hexagonal scenario helper (**NR-v5.1**) |
| [REM maps (DL/UL)](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#nr-rem-helper)                                   | ✅ Supported | DL/UL REM maps, SIR/SNR/SINR for topology inspection                                      |

---

## CI & Testing Framework

| Feature                                                                                                      | Status    | Notes                                                               | Version     | Date         |
|--------------------------------------------------------------------------------------------------------------|-----------|---------------------------------------------------------------------|-------------|--------------|
| CI AI schedulers                                                                                         | ✅ Checked | Weekly job running every RL agent against the AI schedulers                 | **NR-v5.2** | Oct 11, 2026 |
| CI Valgrind memcheck                                                                                     | ✅ Checked | Weekly valgrind memcheck job                                                | **NR-v5.1** | Jul 29, 2026 |
| CI MemorySanitizer                                                                                       | ✅ Checked | Weekly MemorySanitizer job                                                  | **NR-v5.1** | Jul 22, 2026 |
| CI Lizard complexity                                                                                     | ✅ Checked | Fails MRs that raise max cyclomatic complexity or add duplicate code blocks | **NR-v5.0** | Jun 22, 2026 |
| CI Check dead URLs                                                                                           | ✅ Checked | Detects dead URLs in source and documentation                       | **NR-v4.2** | Nov 28, 2025 |
| [CI Calibration Regression testing](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#validation)         | ✅ Checked | **Calibration regression testing** with 3GPP calibration campaign   | **NR-v4.1** | Jul 7, 2025  |
| CI Mac OS                                                                                                    | ✅ Checked | Test NR on MAC OS                                                   | **NR-v4.1** | Jul 7, 2025  |
| CI Deprecated emacs line                                                                                     | ✅ Checked | Checks that the file does not contain deprecated emacs lines        | **NR-v3.2** | Sep 25, 2024 |
| CI Eigen                                                                                                     | ✅ Checked | Check whether the module works correctly without Eigen              | **NR-v3.2** | Sep 25, 2024 |
| CI LLVM's libc++                                                                                             | ✅ Checked | Detect issues caused by different C++ standard libraries            | **NR-v3.2** | Sep 25, 2024 |
| CI Memory issues                                                                                             | ✅ Checked | Early detection of memory bugs; prevents crashing and bogus results | **NR-v3.1** | Jul 19, 2024 |
| CI Commit message                                                                                            | ✅ Checked | Check that the commit message follows 5G-LENA guidelines            | **NR-v3.1** | Jul 19, 2024 |
| CI CMake format                                                                                              | ✅ Checked | Checks CMake format                                                 | **NR-v3.0** | Feb 16, 2024 |
| CI Spell-check                                                                                               | ✅ Checked | Detects spelling errors                                             | **NR-v3.0** | Feb 16, 2024 |
| CI Ubuntu rolling                                                                                            | ✅ Checked | Early detection of issues with the libraries using Ubuntu rolling   | **NR-v3.0** | Feb 16, 2024 |
| CI python-format                                                                                             | ✅ Checked | Check using black and isort formatter settings; python formatting   | **NR-v3.0** | Feb 16, 2024 |
| CI Clang-tidy                                                                                                | ✅ Checked | Detects clang-tidy issues                                           | **NR-v3.0** | Feb 16, 2024 |
| CI Clang-format                                                                                              | ✅ Checked | Detects clang-format issues                                         | **NR-v2.4** | Apr 5, 2023  |
| [CI Reuse](https://reuse.software/)                                                                          | ✅ Checked | Checks open-source licensing using REUSE                            | **NR-v2.4** | Apr 5, 2023  |
| [CI Regression and reproducibility testing](https://cttc-lena.gitlab.io/nr/manual/nr-module.html#validation) | ✅ Checked | **KPI regression and reproducibility** testing                      | **NR-v2.4** | Apr 5, 2023  |
| CI unit and system tests                                                                                     | ✅ Checked | Tests using Clang/GCC in debug/optimized/release mode               | **NR-v0.2** | Feb 1, 2019  |

---

# 5G-LENA Extensions (Sidelink and V2X, O-RAN, NR-U)

The following features are NOT yet available in the nr master.

## Sidelink and NR-V2X extension

The sidelink and NR-V2X are **under active development** and are available in a separate branch of 5G-LENA. The latest
release of the sidelink and NR V2X extension is compatible with 5G-LENA Release v3.1 and ns-3 Release 42 (available
since July 30, 2024). A new release will be available soon.

Sidelink and NR V2X extension documentation can be
found [here](https://5g-lena.cttc.es/static/archive/NR_V2X_V0.1_doc.pdf) Section
2.16. For the installation follow the [instructions](https://gitlab.com/cttc-lena/nr/-/blob/nr-v2x-dev/README.md).

| Feature                          | Status           | Notes                                  |
|----------------------------------|------------------|----------------------------------------|
| Broadcast                        | ✅ Supported     | Mode 4-like broadcast                 |
| Out-of-coverage                  | ✅ Supported     | V2V communication, No gNB required    |
| PSCCH/PSSCH                      | ✅ Supported     | Time multiplexing of PSCCH and PSSCH  |
| Resource allocation              | ✅ Supported     | Mode 2: UE-selected, sensing-based    |
| Semi-persistent scheduling (SPS) | ✅ Supported     | Sensing-based and random SPS          |
| Blind retransmissions            | ✅ Supported     | No feedback                           |
| HARQ feedback                    | ❌ Not supported | Ongoing                               |
| Sidelink control information     | ✅ Supported     | SCI update                            |
| 3GPP TR 38.885 compliant         | ✅ Supported     | TR 38.885 scenario and channel models |

---

## Fronthaul / O-RAN integration

Ongoing and **under active development**. Not yet publicly available.

| Feature     | Status     | Notes                 |
|-------------|------------|-----------------------|
| near-RT RIC | ⚙️ Ongoing | Integration & Testing |
| xApps       | ⚙️ Ongoing | Integration & Testing |
| E2          | ⚙️ Ongoing | Integration & Testing |

---

## NR-U (Unlicensed NR)

NR-U implements NR operation in unlicensed spectrum, i.e., energy detection, multiple channel access managers, including
duty-cycling as well as Listen-Before-Talk (LBT)-based procedures. Openly available as a separate
module [here](https://gitlab.com/cttc-lena/nr-u). NR-U documentation can be
found [here](https://cttc-lena.gitlab.io/nr-u/) and the installation instructions in NR-U
README.md.

The current NR-U code is compatible with 5G-LENA v1.2 and ns-3.35 (since July 2021). **Not under active development**,
but it may get updated. (If interested in contributing, contact us.)

| Feature                 | Status          | Notes                                                                        |
|-------------------------|-----------------|------------------------------------------------------------------------------|
| LBT Cat 2/3/4           | ✅ Supported     | ETSI-compliant LBT; LBT after MAC; ED omnidirectional                       |
| Channel Access Managers | ✅ Supported     | Per-BWP and per-node channel access; Modes: AlwaysOn, OnOff duty cycle, LBT |
| Wi-Fi coexistence       | ⚙️ Experimental | Coexistence with ns-3 Wi-Fi                                                  |
| Directional LBT         | ⚙️ Experimental | LBT with directional beams; null-space projected LBT and precoding           |


---

# 🧭 Planned Roadmap (2025 → 2028)

| Area          | Possible Enhancements                                                  |
|---------------|------------------------------------------------------------------------|
| **Channel**   | Sionna-Ray Tracing channel; hybrid spatially-consistent channels       |
| **Antenna**   | Improved polarization; near-field blocking models for handheld devices |
| **PHY**       | Precoding Type-II and MU-MIMO; SSB blocks transmission and scheduling  |
| **PHY**       | NTN maturation; TN-NTN integration                                     |
| **MAC**       | SL HARQ; LTM; MU-MIMO schedulers; O-RAN xApp hooks; network slicing    |
| **RLC**       | RLC implementations for high-throughput and ultra-low latency          |
| **SDAP**      | Add 5G SDAP layer (ongoing)                                            |
| **RRC**       | Handover (ongoing); Dual Connectivity; advanced mobility procedures    |
| **O-RAN**     | O-RAN xApp hooks; network slicing (ongoing)                            |
| **NR-U**      | Directional LBT                                                        |
| **V2X**       | HARQ feedback; multicast/unicast                                       |
| **Framework** | SDAP layer; 5GC; Network Digital Twin; O-RAN integration               |
| **6G-LENA**   | NTN; ISAC; FFS; RIS; AI                                                |

---

Legend

✅ = Supported ⚙️ = Partial / Ongoing/ Experimental ❌ = Not yet supported

© 2025 Centre Tecnològic de Telecomunicacions de Catalunya (CTTC) – OpenSim Research Unit
