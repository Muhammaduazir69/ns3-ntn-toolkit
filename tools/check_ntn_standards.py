#!/usr/bin/env python3
# Copyright (c) 2026 Muhammad Uzair
# SPDX-License-Identifier: GPL-2.0-only
"""WS5 standards-validation gate (AI-Native ORAN-NTN adoption).

Aggregates every standards check into one PASS/FAIL:
  1. unit/e2e test suites (orbital theory, ORAN apps, KPM monitor, multi-tier
     RIC, WS4 payload/split/role-switch) via ./test.py;
  2. TR 38.821 Set-1 LEO-600 CNR calibration on the measured radio
     (offset constant across the pass + elevation slope matches FSPL);
  3. paper-Table-3 per-platform latency bands with real packets;
  4. the five 3GPP NTN handover trigger classes all execute a handover on
     the measured radio (a3 / d1 / t1 / elevation / ta).

Run from the ns-3 root:  python3 tools/check_ntn_standards.py
"""

import re
import subprocess
import sys

NS3_ROOT = "."

TEST_SUITES = [
    "ntn-standards-validation",
    "ntn-oran-application",
    "ntn-oran-ai-flow-monitor",
    "oran-ntn-multi-tier-ric",
    "oran-ntn-ws4",
]

failures = []


def run(cmd, timeout=900):
    return subprocess.run(cmd, shell=True, cwd=NS3_ROOT, capture_output=True,
                          text=True, timeout=timeout)


def check(name, ok, detail=""):
    print(f"[standards] {'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        failures.append(name)


def main():
    # 1. test suites
    for suite in TEST_SUITES:
        r = run(f"./test.py -s {suite}")
        ok = "FAIL" not in r.stdout and "CRASH" not in r.stdout and r.returncode == 0
        check(f"test-suite {suite}", ok)

    # 2. TR 38.821 calibration (exit code carries the verdict)
    r = run('./ns3 run "ntn-tr38821-calibration --simSeconds=120"')
    m = re.search(r"offset_std=([\d.]+) dB", r.stdout)
    check("tr38821-calibration", r.returncode == 0 and "CALIBRATED" in r.stdout,
          f"offset_std={m.group(1)} dB" if m else "")

    # 3. Table-3 latency bands
    r = run("./ns3 run ntn-platform-latency-validation")
    check("table3-latency-bands", r.returncode == 0 and "ALL CLASSES PASS" in r.stdout)

    # 4. NTN trigger classes: Rel-17 CondEvents A4-combined a3/d1/t1,
    #    Rel-18 CondEventD2 (moving references), plus the TR 38.821-studied
    #    elevation and timing-advance mechanisms.
    for trig in ["a3", "d1", "t1", "d2", "elevation", "ta"]:
        r = run(f'./ns3 run "ntn-cho-handover-traffic --simSeconds=60 --numUes=2 '
                f'--trigger={trig}"')
        m = re.search(r"handovers=(\d+)", r.stdout)
        handovers = int(m.group(1)) if m else 0
        check(f"ho-trigger-class {trig}", r.returncode == 0 and handovers >= 1,
              f"handovers={handovers}")

    print()
    if failures:
        print(f"[standards] FAIL — {len(failures)} check(s): {', '.join(failures)}")
        return 1
    print("[standards] PASS — toolkit validated against 3GPP TR 38.821 link "
          "budget, orbital theory, paper Table 3, and all five NTN HO classes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
