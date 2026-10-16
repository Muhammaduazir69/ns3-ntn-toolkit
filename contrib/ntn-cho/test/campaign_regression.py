#!/usr/bin/env python3
"""Regression checks for ntn-cho-tte-campaign (Phase-1 fixes, Oct 2026).

Each check runs the campaign for a short scenario and tests one property that a
fix guarantees. Every check was confirmed to FAIL with its fix reverted (see
papers/paper2_taes_tte_cho/research/module_deep/FIX_LOG.md).

    python3 campaign_regression.py [--bin PATH] [--out DIR] [--only F1,F8,...]
"""
import argparse
import csv
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = os.path.abspath(os.path.join(HERE, "../../.."))
BIN = os.path.join(NS3, "build/contrib/ntn-cho/examples/ns3.43-ntn-cho-tte-campaign-optimized")
TLE = os.path.abspath(os.path.join(NS3, "../papers/paper2_taes_tte_cho/sim/tle/tle_error_samples.csv"))


def run(out, *args):
    if os.path.exists(out):
        shutil.rmtree(out)
    os.makedirs(out)
    cmd = [ARGS.bin] + list(args) + [f"--outputDir={out}"]
    res = subprocess.run(cmd, cwd=NS3, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} failed:\n{res.stdout}\n{res.stderr}")
    return res.stdout


def rows(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def check_f1(d):
    """F1: every handover interruption is booked as outage."""
    o = os.path.join(d, "f1")
    run(o, "--policy=a3", "--a3OffsetDb=3", "--a3TttMs=160", "--channel=los", "--numUes=30",
        "--simTime=60", "--rngRun=1")
    ints = {}
    for e in rows(os.path.join(o, "events.csv")):
        if e["kind"] == "ho":
            ints[e["ue"]] = ints.get(e["ue"], 0.0) + float(e["interruption_ms"]) / 1000.0
    bad = 0
    total = sum(ints.values())
    for u in rows(os.path.join(o, "ue_summary.csv")):
        if abs(float(u["outage_s"]) - ints.get(u["ue"], 0.0)) > 1e-3:
            bad += 1
    return total > 0 and bad == 0, f"interruptions {total:.3f} s; terminals off by > 1 ms: {bad}"


def check_f3(d):
    """F3: decisions at every 100 ms measurement, not every 200 ms."""
    o = os.path.join(d, "f3")
    run(o, "--policy=a3", "--a3OffsetDb=3", "--a3TttMs=160", "--channel=los", "--numUes=30",
        "--simTime=60", "--rngRun=1")
    t = [round(float(e["t_s"]) * 10) for e in rows(os.path.join(o, "events.csv")) if e["kind"] == "ho"]
    odd = sum(1 for x in t if x % 2 == 1)
    return len(t) > 0 and odd > 0, f"handovers {len(t)}, on odd 100 ms ticks: {odd}"


def check_f4(d):
    """F4: the '+net' fallback target meets the quality floor Q = -3 dB."""
    o = os.path.join(d, "f4")
    run(o, "--policy=d2", "--d2ServElevDeg=35", "--d2CandElevDeg=45", "--measFallback=1",
        "--channel=suburban", "--numUes=100", "--simTime=120", "--rngRun=2")
    ho = [e for e in rows(os.path.join(o, "events.csv")) if e["kind"] == "ho"]
    low = [e for e in ho if float(e["target_l3_db"]) < -3.0 - 1e-9]
    return len(ho) > 0 and not low, f"handovers {len(ho)}, targets below Q: {len(low)}"


def check_f5(d):
    """F5: CondEventT1 is re-armed from the current prediction."""
    o = os.path.join(d, "f5")
    run(o, "--policy=t1", "--channel=los", "--numUes=30", "--simTime=60", "--rngRun=1")
    s = rows(os.path.join(o, "summary.csv"))[0]
    return float(s["t1_rearms"]) > 0, f"t1_rearms {s['t1_rearms']}"


def check_f8(d):
    """F8: identical channel and mobility realizations across policies."""
    probe = os.path.join(d, "f8probe")
    run(probe, "--policy=a3", "--channel=suburban", "--numUes=60", "--simTime=1", "--rngRun=3")
    veh = [u["ue"] for u in rows(os.path.join(probe, "ue_summary.csv")) if "veh" in u["class"].lower()]
    ue = veh[0] if veh else "0"
    dumps = []
    for name, pol in [("a", ["--policy=a3", "--a3OffsetDb=6", "--a3TttMs=640"]), ("b", ["--policy=t1"])]:
        o = os.path.join(d, "f8" + name)
        run(o, *pol, "--channel=suburban", "--numUes=60", "--simTime=120", "--rngRun=3",
            f"--dumpLinksUe={ue}")
        dumps.append({(r["t_s"], r["sat"]): (r["los"], r["sf_db"], r["l1_db"])
                      for r in rows(os.path.join(o, "link_dump.csv"))})
    common = set(dumps[0]) & set(dumps[1])
    diff = sum(1 for k in common if dumps[0][k] != dumps[1][k])
    return len(common) > 100 and diff == 0, f"terminal {ue}: {len(common)} common link samples, differing: {diff}"


def check_f12(d):
    """F12: a static terminal's shadowing decorrelates as the satellite moves."""
    o = os.path.join(d, "f12")
    run(o, "--policy=a3", "--channel=los", "--ueProfile=static", "--numUes=20", "--simTime=180",
        "--rngRun=1", "--dumpLinksUe=0")
    sf = {}
    for r in rows(os.path.join(o, "link_dump.csv")):
        sf.setdefault(r["sat"], set()).add(r["sf_db"])
    changed = sum(1 for v in sf.values() if len(v) >= 2)
    return changed > 0, f"links {len(sf)}, with a new shadowing realization during the run: {changed}"


def check_f16(d):
    """F16: accuracy samples are drawn over all terminals."""
    o = os.path.join(d, "f16")
    run(o, "--mode=tte-accuracy", "--ueProfile=static", "--numUes=60", "--accuracySamples=400",
        "--rngRun=1")
    ues = {r["ue"] for r in rows(os.path.join(o, "tte_accuracy.csv"))}
    return len(ues) >= 57, f"distinct terminals among 400 samples: {len(ues)} of 60"


def check_n2(d):
    """N2: with prediction-based preparation, planned targets are the longest-lived in the sky."""
    o = os.path.join(d, "n2")
    run(o, "--policy=tte-cho", "--rankByUsable=1", "--prepareByTte=1", "--channel=los",
        "--ueProfile=static", "--numUes=40", "--simTime=300", "--rngRun=1")
    ho = [e for e in rows(os.path.join(o, "events.csv")) if e["kind"] == "ho"]
    best = sum(1 for e in ho if e["target_rank_all"] == "1")
    share = best / len(ho) if ho else 0.0
    return len(ho) > 10 and share >= 0.95, f"handovers {len(ho)}, target = longest-lived covering satellite: {share:.1%}"


def check_n4(d):
    """N4: file-driven ephemeris offsets are applied exactly."""
    o = os.path.join(d, "n4")
    out = run(o, "--policy=tte-cho", "--channel=los", "--numUes=10", "--simTime=5", "--rngRun=1",
              f"--ephErrorFile={TLE}", "--ephAgeMinH=6", "--ephAgeMaxH=12",
              "--assumedSigmaAlongM=4392")
    line = [l for l in out.splitlines() if l.startswith("ephfile-check")]
    dev = float(line[0].split("=")[-1].split()[0]) if line else 1e9
    import bisect
    sample = []
    with open(TLE) as fh:
        for r in csv.DictReader(fh):
            if float(r["dt_min"]) == 0 and 6 <= float(r["age_h"]) < 12:
                sample.append(float(r["err_a_m"]))
    sample.sort()

    def known(x):
        i = bisect.bisect_left(sample, x - 1e-4)
        return i < len(sample) and abs(sample[i] - x) <= 1e-4 * max(1.0, abs(x)) + 1e-4

    offs = rows(os.path.join(o, "eph_offsets.csv"))
    hit = sum(1 for r in offs if known(float(r["err_a_m"])))
    return dev < 1e-3 and hit == len(offs), f"max deviation {dev:.2e} m; offsets from the 6-12 h rows: {hit}/{len(offs)}"


# TR 38.811 Table 6.6.1-1 LOS probabilities (%), urban and dense urban (the
# values of ns-3 channel-condition-model.cc UrbanLOSProb / DenseUrbanLOSProb).
PLOS_TABLES = {"urban": {10: 24.6, 20: 38.6, 30: 49.3, 40: 61.3, 50: 72.6, 60: 80.5, 70: 91.9, 80: 96.8, 90: 99.2},
               "denseurban": {10: 28.2, 20: 33.1, 30: 39.8, 40: 46.8, 50: 53.7, 60: 61.2, 70: 73.8, 80: 82.0,
                              90: 98.1}}


def check_env(d):
    """Urban and dense-urban channels draw the LOS state from their own TR 38.811 tables:
    the pooled share of LOS draws in every elevation bin with >= 200 draws lies within
    5 points of the table."""
    msgs, ok = [], True
    for env, table in PLOS_TABLES.items():
        o = os.path.join(d, f"env_{env}")
        run(o, "--policy=a3", "--a3OffsetDb=6", "--a3TttMs=640", f"--channel={env}", "--numUes=100",
            "--simTime=60", "--rngRun=1", "--dumpLosDraws=1")
        cnt, los = {}, {}
        for r in rows(os.path.join(o, "los_draws.csv")):
            b = int(r["elev_bin"])
            cnt[b] = cnt.get(b, 0) + 1
            los[b] = los.get(b, 0) + int(r["los"])
        worst, used = 0.0, 0
        for b, n in cnt.items():
            if n < 200:
                continue
            used += 1
            worst = max(worst, abs(100.0 * los[b] / n - table[b]))
        ok = ok and used >= 3 and worst <= 5.0
        msgs.append(f"{env}: {used} bins, max |share - table| {worst:.1f} pts")
    return ok, "; ".join(msgs)


CHECKS = {"F1": check_f1, "F3": check_f3, "F4": check_f4, "F5": check_f5, "F8": check_f8,
          "F12": check_f12, "F16": check_f16, "N2": check_n2, "N4": check_n4,
          "ENV": check_env}

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=BIN)
    ap.add_argument("--out", default="/tmp/ntn-cho-campaign-regression")
    ap.add_argument("--only", default="")
    ARGS = ap.parse_args()
    names = [n for n in CHECKS if not ARGS.only or n in ARGS.only.split(",")]
    fails = 0
    for n in names:
        try:
            ok, msg = CHECKS[n](ARGS.out)
        except Exception as exc:  # a run that fails is a failed check
            ok, msg = False, str(exc)[:300]
        fails += 0 if ok else 1
        print(f"{'PASS' if ok else 'FAIL'} {n}: {msg}", flush=True)
    sys.exit(1 if fails else 0)
