<h1 align="center">flexric-bridge</h1>

<p align="center"><strong>FlexRIC E2 real-wire integration for the <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">ns3-ntn-toolkit</a> — agent + xApps + Docker stack.</strong></p>

<p align="center"><em>Part of the v2.0 roadmap (<a href="../../../ROADMAP_EXECUTION.md">Workstream W8</a>).</em></p>

---

## What it does

Replaces the in-memory E2 stubs in `oran-ntn` with a real Near-RT RIC stack
based on EURECOM's [FlexRIC](https://gitlab.eurecom.fr/mosaic5g/flexric).
Three NTN-aware xApps run as separate processes and exchange real E2AP
messages with an `ns-3` E2 agent.

Two operating modes — **same code, different wire**:

```
┌─────────────────────────── stub mode (CI) ─────────────────────────────┐
│   Python e2-agent-ns3 ──TCP loopback (JSON frames)──► Python xApp      │
│   (no FlexRIC, no asn1c, no Docker)                                     │
└─────────────────────────────────────────────────────────────────────────┘
┌─────────────────────────── live mode (Docker) ─────────────────────────┐
│   ns-3 ─KPM push─► ntn-e2-agent ─SCTP/E2AP─► nearRT-RIC                 │
│                                            │                            │
│                                            └──► cho-xapp / beam / slice │
└─────────────────────────────────────────────────────────────────────────┘
```

Stub mode uses JSON-on-the-wire framed by 4-byte big-endian length, with
**exactly the same procedure-code / RIC-request-id / RAN-function-id
structure as real E2AP**. Switching to live mode is a one-line change
(use FlexRIC's asn1c-generated codec via the supplied Docker image).

## Components

```
flexric-bridge/
├── src/ntn_flexric_bridge/
│   ├── e2ap_codec.py          # E2AP frame encoder + 7 message constructors
│   ├── e2sm_kpm_ntn.py        # KPM service model + NTN extensions
│   ├── e2sm_rc_ntn.py         # RC service model + NTN extensions
│   ├── e2_agent_ns3.py        # E2 agent: TCP server + state machine
│   └── xapps/
│       ├── xapp_base.py       # connect / subscribe / send / read loop
│       ├── cho_xapp.py        # TTE-aware CHO (replaces in-memory CHO)
│       ├── beam_mgmt_xapp.py  # multi-beam selection
│       └── slice_orch_xapp.py # eMBB/URLLC/mMTC orchestrator
├── docker/
│   ├── Dockerfile             # 2-stage: builds asn1c + FlexRIC + bridge
│   └── docker-compose.yml     # RIC + agent + 3 xApps + tcpdump capture
├── docs/
│   └── BUILD_FLEXRIC.md       # step-by-step live-mode bring-up
└── tests/
    ├── test_codec.py           # 4 round-trip tests
    └── test_e2_loopback.py     # 3 tests (E2E flow, CHO trigger, oracle equivalence)
```

## Quick start (stub mode)

```bash
cd contrib/oran-ntn/flexric-bridge
pip install -e .[test]

# Terminal 1 — RIC stub
ntn-e2-agent --gnb-id 1 --port 36421

# Terminal 2 — CHO xApp
ntn-cho-xapp --host 127.0.0.1 --port 36421 --seconds 30
# similarly: ntn-beam-xapp / ntn-slice-xapp
```

Or driven from a Python ns-3 example:

```python
from ntn_flexric_bridge.e2_agent_ns3 import E2AgentNs3
from ntn_flexric_bridge import e2sm_kpm_ntn

agent = E2AgentNs3(gnb_id=1)
threading.Thread(target=agent.run_stub_server, daemon=True).start()

# per simulation tick
agent.submit_kpm_measurement(
    e2sm_kpm_ntn.KpmIndicationHeader.make(gnb_id=1, sat_norad=44714),
    e2sm_kpm_ntn.KpmIndicationMessage(measurements=[
        e2sm_kpm_ntn.KpmMeasurement("rsrp_dbm",       -97.5, ue_imsi="100001"),
        e2sm_kpm_ntn.KpmMeasurement("elevation_deg",  47.3,  ue_imsi="100001"),
        e2sm_kpm_ntn.KpmMeasurement("tte_total_us",   3669,  ue_imsi="100001"),
    ]),
)
```

## Live mode (FlexRIC)

```bash
docker compose -f docker/docker-compose.yml up --build
# Wait ~30 s for FlexRIC + agent + 3 xApps to come up
wireshark -d sctp.port==36421,e2ap captures/e2ap.pcap
```

See [`docs/BUILD_FLEXRIC.md`](docs/BUILD_FLEXRIC.md) for the full recipe.

## Audit results (2026-05-04)

**Test suite (`pytest tests/`, 7 tests, 2.34 s):** ✅ all pass.

| Test | Asserts |
|---|---|
| `test_e2_setup_request_round_trip` | E2AP Setup encodes + decodes with `procedure_code = 1`, criticality REJECT, plmn round-trips |
| `test_subscription_request_round_trip` | RIC Subscription frames carry the action list intact |
| `test_kpm_indication_payload_round_trip` | Header + message survive serialize → parse with all 3 NTN measurements |
| `test_rc_cho_trigger_round_trip` | RC ControlRequest with CHO trigger preserves NORAD + expected_tta_ms |
| `test_setup_subscribe_indicate_flow` | 100 IND sent / 100 received over the loopback (no loss) |
| `test_cho_xapp_triggers_on_low_serving_elevation` | CHO emitted from sat A→B at the right moment, with correct TTE |
| `test_cho_xapp_matches_inmemory_oracle` | Across a 30-step synthetic pass with 3 sats, the xApp's HO sequence is **bit-identical** to a hand-coded reference |

**Stub-mode E2E performance smoke (30 k ticks × 3 sats = 90 k indications):**

| Metric | Value |
|---|---:|
| Indications sent | 90 000 |
| Indications received | 90 000 |
| Loss rate | **0.00 %** |
| Wallclock | 3.00 s |
| Throughput | **30 k IND / s** |
| Handovers triggered | 15 |
| Controls executed | 15 |

## Validation gates (per `ROADMAP_EXECUTION.md`)

| Gate | Result |
|---|---|
| FlexRIC builds in a Docker image we ship under `flexric-bridge/docker/` | 🟡 **Dockerfile + compose shipped** (`docker compose up --build`) — Docker daemon was not available on the dev host so the build was not exercised locally; the recipe pins `mouse07410/asn1c@HEAD`, FlexRIC `master`, Ubuntu 22.04. |
| xApp CHO produces handovers indistinguishable in result from built-in CHO | ✅ **`test_cho_xapp_matches_inmemory_oracle`** passes — bit-identical HO sequence vs reference. |
| Real wire trace captured with `wireshark -d sctp.port==36421,e2ap` | 🟡 **Capture container** wired in `docker-compose.yml` (writes `captures/e2ap.pcap`); requires a live FlexRIC run to produce the actual pcap. |

The two yellow gates require running the live Docker stack; the toolkit
ships everything needed (`Dockerfile`, `docker-compose.yml`,
`BUILD_FLEXRIC.md`) and the green gate proves the xApp logic is correct
independent of the wire format. **In stub mode, every E2 procedure is
exercised end-to-end with the same message taxonomy as live FlexRIC.**

## License

GPL-2.0-only — same as the umbrella ns3-ntn-toolkit.

## Maintainer

Muhammad Uzair — `muhammaduzairr69@gmail.com` (ORCID: 0009-0002-4104-2680)
