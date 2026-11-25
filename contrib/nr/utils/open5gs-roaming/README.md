<!--
Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)

SPDX-License-Identifier: GPL-2.0-only
-->

# Roaming between ns-3 and Open5GS

Test setup for the `nr-external-core-roaming` example: a UE, subscriber of a core network of
ns-3 (PLMN 002/02), moves from its home gNB to a gNB of Open5GS (PLMN 001/01) and back.

- `core` container: Open5GS (visited network) and
  [nr-bridge](https://github.com/Gabrielcarvfer/PacketRusher/tree/nr-bridge), which runs the
  NGAP/NAS procedures of the gNBs and UEs of ns-3 and is the SEPP (AUSF, UDM) of their home
  network. The UPF reaches the internet through a NAT (N6).
- `mongo` container: database of Open5GS.
- ns-3 runs from the host build in the network namespace of `core`; the N3 interface of its
  visited gNB is an `EmuFdNetDevice` on the `n3-ns3` veth (10.210.1.2), whose peer is the UPF
  (10.210.1.1). A node of ns-3 in the DN of Open5GS has an `EmuFdNetDevice` on the `n6-ns3`
  veth (10.210.2.2), routed by the UPF host (10.210.2.1).

While at home the UE reaches the remote host of ns-3; while roaming it reaches instead the
gateway of the PDU sessions of Open5GS (10.46.0.1), the internet (8.8.8.8), and the node of ns-3
in the DN, through the UPF.

## Usage

```bash
./ns3 configure --enable-examples   # with emulation support (fd-net-device)
./ns3 build nr-external-core-roaming
contrib/nr/utils/open5gs-roaming/run.py [--keep] [-- example arguments]
```

It uses podman-compose (rootless podman, with `podman unshare nsenter`) or docker compose (with
`sudo nsenter`); with podman and no compose command, it installs podman-compose in
`~/.cache/ns-3/open5gs-roaming-venv`. `--packetrusher-src` (or `PACKETRUSHER_SRC`) selects a local PacketRusher
checkout instead of the `nr-bridge` branch on GitHub. The script sets up the core container,
runs the example, prints the traffic counters of N6, saves the log of the core in `core.log`,
and removes the containers (unless `--keep`). The example prints `PASS` when each destination
answered only while its network served the UE.
