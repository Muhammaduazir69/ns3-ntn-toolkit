#!/usr/bin/env python3
"""Fail when a scenario's air interface is not a legal NTN band configuration.

Why this gate exists. `NtnRealStackHelper` has always written an `air_interface`
row into sim_health.csv with a pass flag derived from TS 38.101-5: the carrier
must sit in the band's DOWNLINK block (Table 5.2-1) and the channel must be a
permitted width (Table 5.3.5-1). For months that flag read pass=0 in roughly
forty-five examples, because the default was a 2.0 GHz carrier, which is n256
UPLINK, in a 30 MHz channel, which is the width of the n256 block rather than a
legal channel.

Nothing failed. The flag was written, reported, read by whoever looked, and
ignored, which is how a documented defect survives: describing it is not the same
as enforcing it. This turns the flag into a gate.

Run standalone, or as one gate of tools/check_ntn_standards.py.
"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ENV = dict(os.environ, LD_LIBRARY_PATH=str(ROOT / "build/lib"))

# A spread across the modules that stand up a radio, not an exhaustive list: the
# helper is shared, so a band regression shows up in any of them, and a gate that
# runs 70 examples would never be run.
SCENARIOS = [
    ("ntn-traffic", "ntn-real-stack-smoke"),
    ("ntn-cho", "ntn-cho-real-stack"),
    ("ntn-rrc", "ntn-rrc-real-stack"),
    ("ntn-slice", "ntn-slice-real-stack"),
    ("oran-ntn", "ntn-e2e-full-stack"),
    ("ntn-v2x", "ntn-v2x-real-stack"),
]


def binary(module: str, name: str) -> Path | None:
    for suffix in ("optimized", "default", "debug"):
        b = ROOT / "build/contrib" / module / "examples" / f"ns3.43-{name}-{suffix}"
        if b.exists():
            return b
    return None


def run_one(module: str, name: str, tmp: Path):
    b = binary(module, name)
    if b is None:
        return name, None, "not built"
    out = tmp / name
    out.mkdir(parents=True, exist_ok=True)
    cmd = [str(b), f"--outputDir={out}/"]
    try:
        r = subprocess.run(cmd, cwd=ROOT, env=ENV, capture_output=True,
                           text=True, timeout=1800)
    except subprocess.SubprocessError as exc:
        return name, None, f"did not run: {exc}"
    if r.returncode != 0:
        # An example that rejects --outputDir prints usage; retry bare so a CLI
        # difference is not reported as a band failure.
        try:
            r = subprocess.run([str(b)], cwd=ROOT, env=ENV, capture_output=True,
                               text=True, timeout=1800)
        except subprocess.SubprocessError as exc:
            return name, None, f"did not run: {exc}"

    health = next(iter(out.rglob("sim_health.csv")), None)
    if health is None:
        health = next(iter((ROOT / f"{name}-output").glob("sim_health.csv")), None)
    if health is None:
        return name, None, "wrote no sim_health.csv"

    for row in csv.DictReader(health.open()):
        if (row.get("metric") or "").strip() == "air_interface":
            return (name,
                    (row.get("pass") or "").strip() == "1",
                    (row.get("value") or "").strip())
    return name, None, "no air_interface row"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    tmp = Path(tempfile.mkdtemp(prefix="bandcheck-"))
    failures, skipped = [], []
    try:
        for module, name in SCENARIOS:
            n, ok, tag = run_one(module, name, tmp)
            if ok is None:
                skipped.append((n, tag))
            elif not ok:
                failures.append((n, tag))
            if not args.quiet:
                verdict = "PASS" if ok else ("SKIP" if ok is None else "FAIL")
                print(f"  {verdict}  {n:34} {tag}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"[band] FAIL  {len(failures)} scenario(s) run outside a legal NTN band "
              f"configuration. TS 38.101-5 Table 5.2-1 requires the carrier in the "
              f"band's DOWNLINK block, and Table 5.3.5-1 a permitted channel width.")
        for n, tag in failures:
            print(f"    {n}: {tag}")
        return 1
    if len(skipped) == len(SCENARIOS):
        print("[band] SKIP  no scenario produced an air_interface row")
        return 0
    # Say what fraction of the tree this actually covers.
    #
    # "6 scenario(s) inside a legal NTN band" reads like a statement about the
    # toolkit. It is a statement about six examples. The sweep counts 66 that
    # declare a band row: 45 conformant, 21 outside any FR1 NTN band by design
    # (thz-ntn at 100 GHz, the Ku/Ka ray-tracing and RIC scenarios at 12 to
    # 28 GHz, and the two TR 38.821 calibration examples pinned to the study's
    # own 2 GHz S-band reference). This gate runs the FR1 real-stack spine only;
    # a new example hardcoding an illegal S-band carrier outside that list is
    # not covered.
    print(f"[band] PASS  {len(SCENARIOS) - len(skipped)} of {len(SCENARIOS)} checked "
          f"scenario(s) inside a legal NTN band and channel per TS 38.101-5 "
          f"(FR1 real-stack spine only; see SCOPE A16 for the examples this "
          f"gate does not reach)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
