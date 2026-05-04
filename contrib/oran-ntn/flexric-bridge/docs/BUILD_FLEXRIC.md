# Building FlexRIC for the ns3-ntn-toolkit

The W8 deliverable links the toolkit to a real EURECOM FlexRIC Near-RT
RIC. FlexRIC and the asn1c toolchain are large external dependencies, so
this module ships the **stub-mode** path (CI-friendly Python loopback)
plus everything you need to bring up the **live-mode** stack.

## What you need

| Tool / package | Why |
|---|---|
| Docker (≥ 24) + `docker compose` | Builds & runs FlexRIC in an isolated env |
| `libsctp-dev` (host) for non-Docker dev | RIC speaks SCTP on 36421/36422 |
| `wireshark` (or `tshark`) | Dissect the live E2AP exchange |
| ~3 GB free disk | FlexRIC + asn1c + Ubuntu base layers |

If you do not have Docker, FlexRIC also builds natively on Ubuntu 22.04 —
follow the [official FlexRIC README](https://gitlab.eurecom.fr/mosaic5g/flexric)
for the host-build path.

## Quick start (Docker)

```bash
cd contrib/oran-ntn/flexric-bridge
docker compose -f docker/docker-compose.yml up --build
```

This brings up five containers:

| Container | Role |
|---|---|
| `ntn-flexric-ric` | EURECOM Near-RT RIC (`nearRT-RIC`) on SCTP 36421/36422 |
| `ntn-flexric-agent` | This toolkit's `ntn-e2-agent` — bridges ns-3 to the RIC |
| `ntn-flexric-cho` | `ntn-cho-xapp` — TTE-aware Conditional Handover |
| `ntn-flexric-beam` | `ntn-beam-xapp` — multi-beam selection |
| `ntn-flexric-slice` | `ntn-slice-xapp` — eMBB/URLLC/mMTC orchestrator |
| `ntn-flexric-tcpdump` | captures the live wire to `./captures/e2ap.pcap` |

Validation gate 3 — wireshark dissection — uses the captured pcap:

```bash
wireshark -d sctp.port==36421,e2ap captures/e2ap.pcap
```

## Step by step

1. **Pull the FlexRIC source.** The Dockerfile already does this. To pin
   a known-good commit:

   ```bash
   docker compose -f docker/docker-compose.yml build \
       --build-arg FLEXRIC_REF=br-flexric-ntn-2026.05
   ```

2. **Override the RIC config.** The default `ric.conf` accepts every E2
   node and admits all RAN functions. Drop a custom file at
   `contrib/oran-ntn/flexric-bridge/docker/ric.conf` and add a bind mount
   to `services.ric` in `docker-compose.yml` to override.

3. **Wire ns-3 to the RIC.** The bridge's `ntn-e2-agent` listens on
   container port 36421 and forwards KPM measurements pushed via
   `submit_kpm_measurement(...)`. From an ns-3 example:

   ```python
   from ntn_flexric_bridge.e2_agent_ns3 import E2AgentNs3
   from ntn_flexric_bridge import e2sm_kpm_ntn
   agent = E2AgentNs3(gnb_id=1)
   agent.run_stub_server(port=36421)   # call from a thread
   # ...per simulation tick...
   agent.submit_kpm_measurement(
       e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=44714),
       e2sm_kpm_ntn.KpmIndicationMessage(measurements=[...]),
   )
   ```

4. **Capture the wire trace.** The `tcpdump` container starts capturing
   automatically; the resulting pcap lands in `./captures/e2ap.pcap` on
   the host. Wireshark dissects E2AP natively (≥ 4.0); for older
   versions install the `e2ap` Lua dissector that ships with FlexRIC.

## Stub mode (no FlexRIC)

For CI or local dev without Docker, run the bridge against itself:

```bash
ntn-e2-agent --port 36421 &           # the RIC stub
ntn-cho-xapp --host 127.0.0.1 --port 36421 --seconds 30
```

The wire is JSON-framed (see `e2ap_codec.encode_message`), but every
message has the same procedure-code / RIC-request-id structure as real
E2AP — so the same xApp code works against both modes.

## Troubleshooting

* `connection refused` from xApps → `nearRT-RIC` is not yet listening;
  wait ~3 s after `docker compose up` or add a `healthcheck` in the
  compose file.
* `SCTP: Operation not permitted` → drop `cap_add: NET_ADMIN` and rely
  on the host's SCTP module instead of in-container kernel state.
* xApp Setup hangs → the RIC is rejecting the advertised RAN function
  IDs. Confirm `e2sm_kpm_ntn.RAN_FUNCTION_ID == 147` and
  `e2sm_rc_ntn.RAN_FUNCTION_ID == 148` in the running xApp; FlexRIC
  logs the rejection reason on its stdout.

## Reference

* O-RAN.WG3.E2AP-v01.01 (the wire procedures we encode)
* O-RAN.WG3.E2SM-KPM-v3.00 (KPM service model — extended for NTN)
* O-RAN.WG3.E2SM-RC-v01.03 (RC service model — extended for NTN)
* [EURECOM FlexRIC](https://gitlab.eurecom.fr/mosaic5g/flexric)
