"""W2 + W3 end-to-end pipeline test.

Two layers:

1. **File-mode pipeline** (always runs): spawns the C++ demo, asserts that
   the resulting InfluxDB line-protocol file and NetSimulyzer JSON both
   contain the canonical schema names (ntn_radio / ntn_timing_advance /
   ntn_sat_pos / ntn_drx) with the expected number of points.

2. **Docker-mode pipeline** (only if docker + InfluxDB are running): spawns
   the C++ demo in UDP mode, queries the InfluxDB v2 HTTP API, and asserts
   that the same measurements appear server-side. Skipped automatically with
   a clear note if InfluxDB is unreachable.

Run from the W1 venv (sgp4 etc are not needed here, but `requests` is):

    cd contrib/ntn-constellation
    .venv/bin/python ../ntn-observability/tests-py/test_e2e_pipeline.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

NS3_ROOT = Path(__file__).resolve().parents[3]
DEMO_BIN = (
    NS3_ROOT
    / "build"
    / "contrib"
    / "ntn-observability"
    / "examples"
    / "ns3.43-ntn-observability-demo-default"
)
LP_OUT = Path("/tmp/ntn-observability-e2e.lp")
JSON_OUT = Path("/tmp/ntn-observability-e2e.json")
SIM_SECONDS = 30
RUN_ID = "pytest"

# Tags / measurements / fields the dashboards in dashboards/*.json depend on.
EXPECTED_MEASUREMENTS = {
    "ntn_radio",
    "ntn_timing_advance",
    "ntn_sat_pos",
    "ntn_drx",
    "ntn_sib19",
    "ntn_ue_report",
}
EXPECTED_FIELDS_SUBSTRING = {
    "rsrp_dbm",
    "ta_total_us",
    "sat_x_m",
    "lat_deg",
}


def assert_(cond: bool, msg: str) -> None:
    if not cond:
        print(f"[FAIL] {msg}")
        sys.exit(1)


def run_demo_file_mode() -> None:
    if not DEMO_BIN.exists():
        sys.exit(f"error: build the demo first ({DEMO_BIN})")
    LP_OUT.unlink(missing_ok=True)
    JSON_OUT.unlink(missing_ok=True)
    cmd = [
        str(DEMO_BIN),
        f"--simTime={SIM_SECONDS}",
        f"--runId={RUN_ID}",
        f"--influxFile={LP_OUT}",
        f"--netSim={JSON_OUT}",
    ]
    print("[demo] " + " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=str(NS3_ROOT))


def check_lp_file() -> None:
    body = LP_OUT.read_text(encoding="utf-8")
    n_lines = body.count("\n")
    print(f"[lp] {n_lines} lines, {len(body)} bytes")
    assert_(n_lines > 50, f"too few lp points ({n_lines}); expected > 50 in {SIM_SECONDS}s")

    seen_meas = set()
    seen_fields = set()
    for ln in body.splitlines():
        if not ln:
            continue
        head = ln.split(" ", 1)[0]
        # head = measurement[,tag=val,...]
        meas = head.split(",", 1)[0]
        seen_meas.add(meas)
        # Tokens between space-separated section 2 are field=val pairs.
        try:
            fields_section = ln.split(" ", 2)[1]
            for kv in fields_section.split(","):
                if "=" in kv:
                    seen_fields.add(kv.split("=", 1)[0])
        except IndexError:
            pass

    missing_meas = EXPECTED_MEASUREMENTS - seen_meas
    missing_fields = EXPECTED_FIELDS_SUBSTRING - seen_fields
    print(f"[lp] measurements seen: {sorted(seen_meas)}")
    assert_(not missing_meas, f"missing measurements: {missing_meas}")
    assert_(not missing_fields, f"missing fields: {missing_fields}")

    # Tag presence
    assert_(f"run_id={RUN_ID}" in body, "run_id tag missing")
    print("[lp] PASS")


def check_netsimulyzer_json() -> None:
    body = JSON_OUT.read_text(encoding="utf-8")
    doc = json.loads(body)
    print(f"[json] {len(body)} bytes; nodes={len(doc['nodes'])} series={len(doc['series'])} "
          f"events={len(doc['events'])}")
    assert_("configuration" in doc, "configuration block missing")
    assert_(len(doc["nodes"]) >= 2, "expected at least sat + ue nodes")
    assert_(len(doc["series"]) >= 2, "expected at least 2 series (TA, RSRP)")
    types_seen = {ev["type"] for ev in doc["events"]}
    assert_("NodeMove" in types_seen, "no NodeMove events")
    assert_("SeriesSample" in types_seen, "no SeriesSample events")
    assert_("LogMessage" in types_seen, "no LogMessage events")
    print("[json] PASS")


def docker_pipeline_optional() -> None:
    try:
        import requests
    except ImportError:
        print("[docker] skipped — requests not installed")
        return
    influx_url = os.environ.get("INFLUX_URL", "http://127.0.0.1:8086")
    token = os.environ.get("INFLUX_TOKEN", "ntn-toolkit-token")
    org = os.environ.get("INFLUX_ORG", "ntn")
    try:
        r = requests.get(f"{influx_url}/health", timeout=2)
        if r.status_code != 200:
            print(f"[docker] skipped — InfluxDB /health returned {r.status_code}")
            return
    except Exception as exc:  # noqa: BLE001
        print(f"[docker] skipped — InfluxDB unreachable at {influx_url} ({exc})")
        return

    print(f"[docker] InfluxDB up at {influx_url}; running UDP-mode demo")
    udp_run_id = f"{RUN_ID}-udp"
    cmd = [
        str(DEMO_BIN),
        f"--simTime=10",
        f"--runId={udp_run_id}",
        "--udpHost=127.0.0.1",
        "--udpPort=8089",
        f"--netSim=/tmp/ntn-observability-udp.json",
    ]
    subprocess.run(cmd, check=True, cwd=str(NS3_ROOT))

    # InfluxDB UDP listener takes ~1 s to flush. Poll up to 10 s.
    import time
    flux = (
        f'from(bucket: "ntn") |> range(start: -1h) '
        f'|> filter(fn: (r) => r.run_id == "{udp_run_id}") |> count()'
    )
    headers = {"Authorization": f"Token {token}", "Content-Type": "application/vnd.flux"}
    found = 0
    for _ in range(10):
        time.sleep(1)
        r = requests.post(
            f"{influx_url}/api/v2/query?org={org}", headers=headers, data=flux, timeout=5,
        )
        if r.status_code == 200 and udp_run_id in r.text:
            found = sum(int(line.split(",")[-1].strip())
                        for line in r.text.splitlines()
                        if udp_run_id in line and line.split(",")[-1].strip().isdigit())
            if found > 0:
                break
    assert_(found > 0, "no UDP-mode points landed in InfluxDB")
    print(f"[docker] PASS — {found} points landed in InfluxDB via UDP")


def main() -> int:
    run_demo_file_mode()
    check_lp_file()
    check_netsimulyzer_json()
    docker_pipeline_optional()
    print("[OK] full W3 pipeline validated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
