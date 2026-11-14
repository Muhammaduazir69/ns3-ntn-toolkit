#!/usr/bin/env python3
"""Campaign of the street-canyon relay study (Nov 2026), one binary (the ntn-v2x-canyon-relay example), resumable, one jsonl
line per run with the binary hash. Studies run in priority order, primary first:

  primary  T = 1 s, shared uplink, no grant delay (a variant charges one), 100 ms handover, TR 37.885 urban sidelink, SUMO stop-and-go, cross-street
           gaps: 15 policies, three canyons, twenty starts.
  period   the main rivals at other T (0.1 s from an idealized and a realistic uplink, 0.5, 2, 5 s).
  cap      uplink capacity; ho: handover interruption; setup: relay link latency; mob: constant speed against stop-and-go
           and higher density; chan: sidelink range and channel; geo: azimuth, latitude, mask; real: Manhattan facades.

    python3 run_ns3_canyon5.py <jobs> [study ...]
"""
import csv, hashlib, json, os, subprocess, sys, tempfile, threading, shutil, time
from concurrent.futures import ThreadPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = os.environ.get("NS3_DIR", os.path.abspath(os.path.join(HERE, "..", "..", "..", "..")))
EXE = os.environ.get("P8_BIN", os.path.join(HERE, "bin", "p8-canyon-relay"))
OUT = os.path.join(HERE, "out", "ns3_canyon5.jsonl")
CANYONS = {"suburban": dict(width=30, h50=10), "urban": dict(width=20, h50=18), "dense": dict(width=15, h50=30)}
SIM, WARM = 300.0, 10.0
EPOCH0, SPACING = 1792800000.0, 4320.0      # twenty starts spread over one day
EXE2 = EXE   # the study was first run on four builds of this example; the source here is the last one
SHA = hashlib.sha256(open(EXE, "rb").read()).hexdigest()
SHA2 = hashlib.sha256(open(EXE2, "rb").read()).hexdigest()
EXE3 = EXE
SHA3 = hashlib.sha256(open(EXE3, "rb").read()).hexdigest()
ALLP = ("direct", "multisat", "relay", "blind", "blindfar", "blindbest", "blindlb", "randpersist", "random",
        "roundrobin", "rand2", "rand3", "hybrid", "anyfwd", "anyp")
RIVALS = ("multisat", "relay", "blindlb", "randpersist", "random", "rand2", "anyfwd", "anyp")
BASE = dict(periodMs=1000, ulKbps=300, grantRtt=1, hoMs=100, slUrban=1, alignResel=1, nbAgeS=1.5,
            gapEveryM=240, gapLenM=18, mob="sumo")
HELD = range(10, 20)   # starts not used in any earlier development


def study(name):
    J = []
    def add(c, n, p, rep, **kw):
        J.append(dict(study=name, canyon=c, n=n, policy=p, rep=rep, **{**BASE, **kw}))
    if name == "primary":
        for rep in list(HELD) + list(range(10)):       # held-out starts first
            for c in ("urban", "dense"):
                for p in ALLP:
                    add(c, 60, p, rep)
        for rep in HELD:
            for p in ("direct", "multisat", "relay", "blindlb", "random"):
                add("suburban", 60, p, rep)
    elif name == "period":
        for T in (5000, 2000, 500):
            for rep in HELD:
                for c in ("urban", "dense"):
                    for p in ("multisat", "relay", "blindlb", "randpersist", "random"):
                        add(c, 60, p, rep, periodMs=T, nbAgeS=max(1.0, 1.5 * T / 1000))
        for rep in HELD:           # 10 Hz: realistic uplink with capacity scaled to the load, then the earlier idealized setting
            for p in ("multisat", "relay", "blindlb", "random"):
                add("urban", 60, p, rep, periodMs=100, nbAgeS=1.0, ulKbps=3000)
                add("dense", 60, p, rep, periodMs=100, nbAgeS=1.0, ulKbps=3000)
                add("urban", 60, p, rep, periodMs=100, nbAgeS=1.0, ulKbps=0, grantRtt=0, hoMs=0, slUrban=0, gapEveryM=0, gapLenM=0, mob="const", alignResel=0)
    elif name == "cap":
        for rep in HELD:
            for p in ("multisat", "relay", "random", "anyfwd", "anyp"):
                for C in (0, 100):
                    add("urban", 60, p, rep, ulKbps=C)
                    add("dense", 60, p, rep, ulKbps=C)
                add("urban", 60, p, rep, ulKbps=30)
    elif name == "cap2":       # the uplink budget is assumed, so the comparison is shown over a range of budgets
        for rep in HELD:
            for c in ("urban", "dense"):
                for p in ("multisat", "relay", "random", "rand3", "anyfwd", "anyp"):
                    add(c, 60, p, rep, ulKbps=1000)
                for p in ("relay", "anyfwd"):
                    add(c, 60, p, rep, ulKbps=600)
                for C in (0, 100):
                    add(c, 60, "rand3", rep, ulKbps=C)
            add("urban", 60, "rand3", rep, ulKbps=30)
    elif name == "mob2":       # one factor at a time: constant speed with the cross streets kept; density with capacity scaled
        for rep in HELD:
            for c in ("urban", "dense"):
                for p in ("multisat", "relay", "blindlb", "randpersist", "random", "anyfwd"):
                    add(c, 60, p, rep, mob="const")
                add(c, 60, "randpersist", rep, mob="const", gapEveryM=0, gapLenM=0)
            for p in ("multisat", "relay", "blindlb", "random", "anyfwd"):
                add("urban", 240, p, rep, ulKbps=1200)
    elif name == "r2kappa":    # thinning between its extremes, across budgets
        for rep in HELD:
            for c in ("urban", "dense"):
                for K in (2, 3, 5, 8):
                    for C in (100, 300, 600, 0):
                        add(c, 60, "anyp", rep, kappa=K, ulKbps=C)
                add(c, 60, "anyp", rep, kappa=1, ulKbps=600)
    elif name == "r2gap":      # the crossing of any decoder and announced
        for rep in HELD:
            for c in ("urban", "dense"):
                for C in (350, 400, 450, 500):
                    for p in ("relay", "anyfwd"):
                        add(c, 60, p, rep, ulKbps=C)
    elif name == "r2k":        # more names per message: fresh beyond three, announced beyond one, uniform among announcers
        for rep in HELD:
            for c in ("urban", "dense"):
                for C in (100, 300, 0):
                    for K in (5, 8):
                        add(c, 60, "randk", rep, relayK=K, ulKbps=C)
                    for K in (2, 3):
                        add(c, 60, "relayk", rep, relayK=K, ulKbps=C)
                    if C != 300:
                        add(c, 60, "relayrand", rep, relayK=1, ulKbps=C)
                for K in (3, 5):
                    add(c, 60, "anypn", rep, kappa=K)
    elif name == "r2prio":     # the same rules with a vehicle's own messages served before forwards
        for rep in HELD:
            for c in ("urban", "dense"):
                for C in (100, 300):
                    add(c, 60, "relay", rep, ulKbps=C, ulPrio=1)
                    add(c, 60, "rand3", rep, ulKbps=C, ulPrio=1)
                    add(c, 60, "anyfwd", rep, ulKbps=C, ulPrio=1)
                    for K in (3, 5, 8):
                        add(c, 60, "anyp", rep, kappa=K, ulKbps=C, ulPrio=1)
    elif name == "r2dev":      # the rules the headline may name, on the other ten starts of the primary comparison
        for rep in list(range(10)) + list(HELD):
            for c in ("urban", "dense"):
                add(c, 60, "relayrand", rep, relayK=1)
                if rep < 10:
                    for K in (2, 3, 5, 8):
                        add(c, 60, "anyp", rep, kappa=K, ulKbps=300)
    elif name == "r2speed":    # constant speed at the mean speed of the SUMO traffic
        for rep in HELD:
            for c in ("urban", "dense"):
                for p in ("random", "randpersist"):
                    add(c, 60, p, rep, mob="const", vMin=3.0, vMax=9.2)
    elif name == "r2more":     # the announced family under the other queue, and on the other ten starts of the primary comparison
        for rep in HELD:
            for c in ("urban", "dense"):
                for C in (100, 300):
                    add(c, 60, "relayrand", rep, relayK=1, ulKbps=C, ulPrio=1)
                    add(c, 60, "relayrand", rep, relayK=2, ulKbps=C, ulPrio=1)
                    add(c, 60, "relayk", rep, relayK=2, ulKbps=C, ulPrio=1)
                for C in (100, 300, 0):
                    add(c, 60, "relayrand", rep, relayK=2, ulKbps=C)
        for rep in range(10):
            for c in ("urban", "dense"):
                add(c, 60, "relayk", rep, relayK=2)
                add(c, 60, "relayk", rep, relayK=3)
                add(c, 60, "relayrand", rep, relayK=2)
        for rep in HELD:               # the same with more vehicles, because the best rule may move with the density
            for p, kw in (("relayrand", dict(relayK=1)), ("relayrand", dict(relayK=2)), ("relayk", dict(relayK=2)), ("anyp", dict(kappa=3))):
                add("urban", 240, p, rep, **kw)
                add("urban", 240, p, rep, ulKbps=1200, **kw)
    elif name == "r3grant":    # the scheduling round trip actually charged (grantActive marks these rows)
        for rep in HELD:
            for c in ("urban", "dense"):
                for p, kw in (("multisat", {}), ("relay", {}), ("relayrand", dict(relayK=1)), ("relayk", dict(relayK=2)), ("rand3", {}), ("anyp", dict(kappa=3)), ("anyp", dict(kappa=5)), ("anyfwd", {})):
                    add(c, 60, p, rep, grantActive=1, **kw)
                for p in ("relay", "anyfwd"):
                    add(c, 60, p, rep, grantActive=1, ulKbps=600)
    elif name == "ho":
        for H in (0, 1000):
            for rep in HELD:
                for p in ("multisat", "relay", "random", "anyfwd"):
                    add("urban", 60, p, rep, hoMs=H)
    elif name == "setup":
        for S in (100, 500):
            for rep in HELD:
                for p in ("relay", "random", "randpersist", "blindlb"):
                    add("urban", 60, p, rep, relaySetupMs=S)
    elif name == "mob":
        for rep in HELD:
            for p in ("multisat", "relay", "blindlb", "random", "anyfwd"):
                add("urban", 60, p, rep, mob="const", gapEveryM=0, gapLenM=0)
                add("dense", 60, p, rep, mob="const", gapEveryM=0, gapLenM=0)
                add("urban", 240, p, rep)
    elif name == "chan":
        for rep in HELD:
            for p in ("relay", "blindlb", "random", "anyfwd"):
                add("urban", 60, p, rep, slUrban=0)
                add("urban", 60, p, rep, slNlosP=1.0)
                add("urban", 60, p, rep, maxRelayM=100)
    elif name == "geo":
        for rep in HELD:
            for p in ("multisat", "relay", "blindlb", "random"):
                for kw in (dict(streetAzDeg=119), dict(streetAzDeg=74), dict(lat=25.77, lon=-80.19), dict(lat=52.5, lon=13.4), dict(mask=10), dict(mask=30)):
                    add("urban", 60, p, rep, **kw)
    elif name == "real":
        ST = json.load(open(os.path.join(HERE, "realcity", "stretches_ns3.json")))
        for s, info in ST.items():
            for rep in HELD:
                for p in ("multisat", "relay", "blindlb", "random", "anyfwd"):
                    add("real", 60, p, rep, facadeFile=os.path.join(HERE, "realcity", os.path.basename(info["file"])), street=s, width=info["W"], streetAzDeg=info["az"], gapEveryM=0, gapLenM=0, length=info["L"], bldLen=info["Lb"], h50=info["h50"], hSigma=info["s"])
    else:
        raise SystemExit("unknown study " + name)
    return J


def aoi(gen, rx, t0, t1, thresholds=(1.0, 5.0)):
    order = np.argsort(rx)
    gen, rx = gen[order], rx[order]
    before = rx <= t0
    cur = gen[before].max() if before.any() else 0.0
    segs, last = [], t0
    for g, r in zip(gen, rx):
        if r <= t0 or g <= cur:
            continue
        if r > t1:
            break
        segs.append((last - cur, r - cur))
        last, cur = r, g
    segs.append((last - cur, t1 - cur))
    span = t1 - t0
    mean = sum((b * b - a * a) / 2 for a, b in segs) / span
    above = [sum(max(0.0, b - max(a, x)) for a, b in segs) / span for x in thresholds]
    return mean, above


def lag1(src_ids, seqs, n, warm_seq):
    """Pooled over vehicles: share of messages lost, and share lost given that the previous one was lost."""
    top = int(seqs.max()) - 1 if len(seqs) else 0
    lost = cons = prev_lost = tot = 0
    for k in range(n):
        got = np.zeros(top + 1, bool)
        s = seqs[(src_ids == k) & (seqs <= top)]
        got[s] = True
        f = ~got[warm_seq:]
        tot += len(f); lost += int(f.sum())
        prev_lost += int(f[:-1].sum()); cons += int((f[:-1] & f[1:]).sum())
    return (lost / tot if tot else float("nan")), (cons / prev_lost if prev_lost else float("nan"))


def key(j):
    return json.dumps({k: v for k, v in sorted(j.items()) if k != "study"}, sort_keys=True)


def run(j):
    c, n, rep = j["canyon"], j["n"], j["rep"]
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False, dir=os.path.join(HERE, "out")) as tf:
        out = tf.name
    a = dict(nVeh=n, policy=j["policy"], simSeconds=SIM, out=out, RngRun=rep + 1, epochUnix=EPOCH0 + SPACING * rep)
    if c in CANYONS:
        a.update(CANYONS[c])
    for k, v in j.items():
        if k not in ("study", "canyon", "n", "policy", "rep", "mob", "street", "grantActive"):
            a[k] = v
    if j["mob"] == "sumo":
        a["mobTrace"] = os.path.join(HERE, "traces", f"n{n}_r{rep}.csv")
    t0 = time.time()
    r2 = j["study"].startswith("r2")
    r3 = j["study"].startswith("r3")
    if not r3:
        # --grantRtt took no effect in the builds these studies were first run on (the slant range it reads stayed 0),
        # so they are reproduced with 0; the r3 studies charge it. run_ns3_canyon6.py does the same.
        a["grantRtt"] = 0
    cmd = [EXE3 if r3 else (EXE2 if r2 else EXE)] + [f"--{k}={v}" for k, v in a.items()]
    r = subprocess.run(cmd, cwd=NS3, capture_output=True, text=True, timeout=14400)
    base = dict(j, sha256=(SHA3 if r3 else (SHA2 if r2 else SHA))[:16], wall_s=round(time.time() - t0, 1))
    if r.returncode != 0:
        if os.path.exists(out):
            os.unlink(out)
        return dict(base, error=f"rc={r.returncode} " + r.stderr[-300:])
    summ = dict(kv.split("=", 1) for kv in r.stdout.split() if "=" in kv)
    D = np.genfromtxt(out, delimiter=",", skip_header=1).reshape(-1, 5)
    os.unlink(out)
    src, seq, gen, rx, via = D[:, 0].astype(int), D[:, 1].astype(int), D[:, 2], D[:, 3], D[:, 4].astype(int)
    A = [aoi(gen[src == k], rx[src == k], WARM, SIM) for k in range(n)]
    ages = np.array([x for x, _ in A])
    # unique messages delivered, duplicates removed
    uniq = len(np.unique(src * (1 << 20) + seq))
    gen_n = int(summ["generated"])
    T = j["periodMs"] / 1000.0
    pf, pff = lag1(src, seq, n, int(WARM / T))
    return dict(base, generated=gen_n, delivered=uniq, rx_rows=len(D), delivered_share=uniq / gen_n,
                relay_share=float(np.mean(via == 1)) if len(via) else 0.0,
                delay_ms=float(1e3 * np.mean(rx - gen)) if len(rx) else float("nan"),
                aoi_mean_s=float(np.mean(ages)), aoi_p90_s=float(np.percentile(ages, 90)),
                stale_t2=float(np.mean([s[0] for _, s in A])),
                relay_asked=int(summ["relay_asked"]), relayed=int(summ["relayed"]), sent_direct=int(summ["sent_direct"]),
                blocked_gen=int(summ["blocked_gen"]), ul_sent=int(summ["ul_sent"]), ul_drop_q=int(summ["ul_drop_q"]),
                ul_drop_los=int(summ["ul_drop_los"]), ul_bytes=int(summ["ul_bytes"]), ul_wait_ms=float(summ["ul_wait_ms"]),
                far_fallback=int(summ["far_fallback"]), p_fail=pf, p_fail_given_fail=pff,
                **{k: float(v) for k, v in summ.items() if k.startswith(("prr_", "prrnohd_"))}, error="")


def main(par, names):
    done = set()
    if os.path.exists(OUT):
        for l in open(OUT):
            r = json.loads(l)
            if not r.get("error"):
                if r.get("sha256") not in (SHA[:16], SHA2[:16], SHA3[:16]):
                    raise SystemExit("row from another binary in " + OUT)
                d = {k: r[k] for k in r if k in KEYS(r)}
                done.add(key(d))
    todo = []
    for nm in names:
        todo += [j for j in study(nm) if key(j) not in done]
    # duplicates across studies run once
    seen = set(); uniq = []
    for j in todo:
        if key(j) not in seen:
            seen.add(key(j)); uniq.append(j)
    print(len(uniq), "runs to do", flush=True)
    lock = threading.Lock()
    stopfile = os.path.join(HERE, "out", "STOP5")

    def work(j):
        if os.path.exists(stopfile):
            return
        fr = shutil.disk_usage("/").free / 1e9
        if fr < 2.0:
            print("LOW DISK, stopping", fr, flush=True)
            open(stopfile, "w").write("low disk\n")
            return
        r = run(j)
        with lock:
            with open(OUT, "a") as f:
                f.write(json.dumps(r) + "\n")
        print(j["study"], j["canyon"], j["n"], j["policy"], j["rep"], "ERR" if r.get("error") else f'{r["wall_s"]}', flush=True)

    with ThreadPoolExecutor(par) as ex:
        list(ex.map(work, uniq))
    print("done", flush=True)


RESULT = {"sha256", "wall_s", "error", "generated", "delivered", "rx_rows", "delivered_share", "relay_share", "delay_ms",
          "aoi_mean_s", "aoi_p90_s", "stale_t2", "relay_asked", "relayed", "sent_direct", "blocked_gen", "ul_sent", "ul_drop_q",
          "ul_drop_los", "ul_bytes", "ul_wait_ms", "far_fallback", "p_fail", "p_fail_given_fail", "study"}


def KEYS(r):
    return {k for k in r if k not in RESULT and not k.startswith(("prr_", "prrnohd_"))}


if __name__ == "__main__":
    main(int(sys.argv[1]), sys.argv[2:] or ["primary"])
