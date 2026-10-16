#!/usr/bin/env python3
# Copy of papers/paper3_tnsm_oran_ntn/sim/ from the author's tree; paths below are absolute to that tree.
"""Campaign driver for Paper 3 (TNSM): oran-ntn-feeder-mgmt.

Every run writes summary.csv, outages.csv, switches.csv, loop_hist.csv and
checks.csv into runs/<group>/<config>/seedNN/. A run is skipped when its
summary.csv already exists, so the driver can be restarted.

Usage: run_campaign.py <group> [--jobs N]
groups: hb, eps, main, ablation, sens
"""
import argparse
import itertools
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = "/home/uzair/6g_ntn_ns3/ns-3-dev"
BIN = os.path.join(NS3, "build/contrib/oran-ntn/examples/ns3.43-oran-ntn-feeder-mgmt-optimized")
RUNS = os.path.join(HERE, "runs")
SEEDS = range(1, 11)
SIM_S = 3600

# Heartbeat setting used by `reactive` and `pal` in the main grid; chosen from
# the `hb` group (see analyze.py, tuning section).
HB_MAIN = {"hbPeriodS": 0.1, "hbMiss": 3}


def jobs_for(group):
    out = []
    if group == "hb":
        for strat, hb in itertools.product(["reactive", "pal"], [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]):
            for gw in [1]:
                cfg = dict(strategy=strat, shell=550, numGw=gw, hbPeriodS=hb, hbMiss=3)
                out.append((f"{strat}_s550_gw{gw}_hb{hb}", cfg))
    elif group == "eps":
        for strat, eps in itertools.product(["pal", "reactive-ttl"], [0.05, 0.2, 0.5, 0.9, 0.99]):
            for gw in [1, 2]:
                cfg = dict(strategy=strat, shell=550, numGw=gw, leaseEps=eps, **HB_MAIN)
                out.append((f"{strat}_s550_gw{gw}_eps{eps}", cfg))
    elif group == "main":
        for strat, shell, gw in itertools.product(
            ["ground", "onboard", "reactive", "reactive-ttl", "pal"], [550, 1200], [1, 2, 3, 4]
        ):
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "ablation":
        for strat, shell, gw in itertools.product(["pal-nopre", "pal-gw"], [550, 1200], [1, 2]):
            cfg = dict(strategy=strat, shell=shell, numGw=gw, **HB_MAIN)
            out.append((f"{strat}_s{shell}_gw{gw}", cfg))
    elif group == "sens":
        # Ephemeris error and unpredicted-outage rate, closest baseline vs pal.
        for strat, sig in itertools.product(["reactive", "pal"], [0.0, 2.0, 5.0]):
            cfg = dict(strategy=strat, shell=550, numGw=1, ephSigmaS=sig, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_eph{sig}", cfg))
        for strat, rain in itertools.product(["reactive", "pal"], [0.0, 6.0]):
            cfg = dict(strategy=strat, shell=550, numGw=1, rainPerHour=rain, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_rain{rain}", cfg))
        for strat in ["onboard", "reactive", "pal"]:
            cfg = dict(strategy=strat, shell=550, numGw=1, procScaleOnboard=5.0, **HB_MAIN)
            out.append((f"{strat}_s550_gw1_proc5", cfg))
    else:
        sys.exit(f"unknown group {group}")
    return out


def run_one(group, name, cfg, seed):
    d = os.path.join(RUNS, group, name, f"seed{seed:02d}")
    if os.path.exists(os.path.join(d, "summary.csv")):
        return "skip"
    os.makedirs(d, exist_ok=True)
    args = [BIN, f"--simSeconds={SIM_S}", f"--run={seed}", f"--outputDir={d}"]
    args += [f"--{k}={v}" for k, v in cfg.items()]
    with open(os.path.join(d, "cmd.txt"), "w") as f:
        f.write(" ".join(args) + "\n")
    r = subprocess.run(args, capture_output=True, text=True, cwd=NS3)
    with open(os.path.join(d, "stdout.txt"), "w") as f:
        f.write(r.stdout[-4000:])
        f.write(r.stderr[-4000:])
    return "ok" if r.returncode == 0 else f"FAIL {r.returncode}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("group")
    ap.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args()
    todo = [(a.group, n, c, s) for n, c in jobs_for(a.group) for s in SEEDS]
    print(f"{a.group}: {len(todo)} runs, {a.jobs} parallel", flush=True)
    with ThreadPoolExecutor(a.jobs) as ex:
        for i, res in enumerate(ex.map(lambda t: run_one(*t), todo)):
            if res.startswith("FAIL") or i % 20 == 0:
                print(i, todo[i][1], todo[i][3], res, flush=True)
    print("done", flush=True)


if __name__ == "__main__":
    main()
