#!/usr/bin/env python3
"""Further runs (10 Nov 2026) on the final build (names carried in the tag, relayfar, relaylb).

In this build --grantRtt takes effect. The primary comparison was run on builds 1 and 2, where the
grant round trip was inert, so every run here passes --grantRtt=0 and records grantRtt=1 like the primary rows; r4pos
checks that this reproduces the stored primary rows exactly.

  r4pos  any satellite, nearest announcer, two random announcers on all 20 starts, both canyons, with per-message
         rows kept long enough to compute the server-side position error from the SUMO traces (out/ns3_canyon6_pos.jsonl)
  r4ann  farthest announcer and nearest announcer a building length away, 20 starts (into ns3_canyon5.jsonl)
  r4cap  nearest and two random announcers at 150, 200, 250 kb/s, starts 10-19 (into ns3_canyon5.jsonl)

    python3 run_ns3_canyon6.py <jobs> study [study ...]
"""
import hashlib, json, os, shutil, subprocess, sys, tempfile, threading, time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import run_ns3_canyon5 as R5

HERE = R5.HERE
EXE4 = R5.EXE
SHA4 = hashlib.sha256(open(EXE4, "rb").read()).hexdigest()
OUT_MAIN = R5.OUT
OUT_POS = os.path.join(HERE, "out", "ns3_canyon6_pos.jsonl")
BASE = R5.BASE
ALL = list(range(10, 20)) + list(range(10))


def study(name):
    J = []
    def add(c, p, rep, **kw):
        J.append(dict(study=name, canyon=c, n=60, policy=p, rep=rep, **{**BASE, **kw}))
    if name == "r4pos":
        for rep in ALL:
            for c in ("urban", "dense"):
                add(c, "multisat", rep); add(c, "relay", rep); add(c, "relayrand", rep, relayK=2)
    elif name == "r4posall":   # the remaining rules of the primary comparison, for the position error at the server
        FAM = [("direct", {}), ("blind", {}), ("blindfar", {}), ("blindbest", {}), ("blindlb", {}), ("randpersist", {}),
               ("random", {}), ("roundrobin", {}), ("rand2", {}), ("rand3", {}), ("hybrid", {}), ("anyfwd", {}),
               ("anyp", dict(kappa=1)), ("anyp", dict(kappa=3)), ("anyp", dict(kappa=5)), ("relayrand", dict(relayK=1)),
               ("relayk", dict(relayK=2)), ("relayk", dict(relayK=3)), ("relayfar", {}), ("relaylb", {})]
        for rep in ALL:
            for c in ("urban", "dense"):
                for p, kw in FAM:
                    add(c, p, rep, **kw)
    elif name == "r4ann":
        for rep in ALL:
            for c in ("urban", "dense"):
                add(c, "relayfar", rep); add(c, "relaylb", rep)
    elif name == "r4cap":
        for rep in R5.HELD:
            for c in ("urban", "dense"):
                for C in (150, 200, 250):
                    add(c, "relay", rep, ulKbps=C); add(c, "relayrand", rep, relayK=2, ulKbps=C)
    else:
        raise SystemExit("unknown study " + name)
    return J


def load_traj(n, rep):
    lines = open(os.path.join(HERE, "traces", f"n{n}_r{rep}.csv")).read().split("\n")
    dt = float(lines[0])
    S = [np.array([float(x) for x in l.split(",")[2:]]) for l in lines[1:] if l.strip()]
    return dt, S


def pos_err(gen, rx, s, dt, t0, t1, step=0.05):
    """Time average over [t0, t1] of the distance the vehicle drove since the generation time of the newest message the
    server holds (distance along its path, from the SUMO trace, linear between samples). Before the first reception after
    t0 with nothing received earlier, the time is excluded (reported separately as cold time)."""
    order = np.argsort(rx); gen, rx = gen[order], rx[order]
    t = np.arange(t0, t1, step)
    cm = np.maximum.accumulate(gen) if len(gen) else gen
    idx = np.searchsorted(rx, t, side="right") - 1          # last reception at or before t
    newest = np.where(idx >= 0, cm[np.clip(idx, 0, None)] if len(cm) else np.nan, np.nan)
    ok = np.isfinite(newest)
    sp = lambda x: np.interp(x / dt, np.arange(len(s)), s)
    err = np.abs(sp(t[ok]) - sp(newest[ok]))
    return (float(err.mean()) if ok.any() else float("nan")), float(1 - ok.mean()), err


def run(j):
    c, n, rep = j["canyon"], j["n"], j["rep"]
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False, dir=os.path.join(HERE, "out")) as tf:
        out = tf.name
    a = dict(nVeh=n, policy=j["policy"], simSeconds=R5.SIM, out=out, RngRun=rep + 1, epochUnix=R5.EPOCH0 + R5.SPACING * rep)
    a.update(R5.CANYONS[c])
    for k, v in j.items():
        if k not in ("study", "canyon", "n", "policy", "rep", "mob", "street", "grantActive"):
            a[k] = v
    a["grantRtt"] = 0            # inert in the builds of the primary comparison; see the docstring
    a["mobTrace"] = os.path.join(HERE, "traces", f"n{n}_r{rep}.csv")
    t0 = time.time()
    r = subprocess.run(["nice", "-n", "19", EXE4] + [f"--{k}={v}" for k, v in a.items()], cwd=R5.NS3, capture_output=True, text=True, timeout=14400)
    base = dict(j, sha256=SHA4[:16], wall_s=round(time.time() - t0, 1))
    if r.returncode != 0:
        if os.path.exists(out):
            os.unlink(out)
        return dict(base, error=f"rc={r.returncode} " + r.stderr[-300:])
    summ = dict(kv.split("=", 1) for kv in r.stdout.split() if "=" in kv)
    D = np.genfromtxt(out, delimiter=",", skip_header=1).reshape(-1, 5)
    os.unlink(out)
    src, seq, gen, rx, via = D[:, 0].astype(int), D[:, 1].astype(int), D[:, 2], D[:, 3], D[:, 4].astype(int)
    A = [R5.aoi(gen[src == k], rx[src == k], R5.WARM, R5.SIM) for k in range(n)]
    ages = np.array([x for x, _ in A])
    uniq = len(np.unique(src * (1 << 20) + seq))
    gen_n = int(summ["generated"])
    T = j["periodMs"] / 1000.0
    pf, pff = R5.lag1(src, seq, n, int(R5.WARM / T))
    cold = int(sum(1 for k in range(n) if not np.any(rx[src == k] <= R5.WARM)))
    res = dict(base, generated=gen_n, delivered=uniq, rx_rows=len(D), delivered_share=uniq / gen_n,
               relay_share=float(np.mean(via == 1)) if len(via) else 0.0,
               delay_ms=float(1e3 * np.mean(rx - gen)) if len(rx) else float("nan"),
               aoi_mean_s=float(np.mean(ages)), aoi_p90_s=float(np.percentile(ages, 90)),
               stale_t2=float(np.mean([s[0] for _, s in A])),
               relay_asked=int(summ["relay_asked"]), relayed=int(summ["relayed"]), sent_direct=int(summ["sent_direct"]),
               blocked_gen=int(summ["blocked_gen"]), ul_sent=int(summ["ul_sent"]), ul_drop_q=int(summ["ul_drop_q"]),
               ul_drop_los=int(summ["ul_drop_los"]), ul_bytes=int(summ["ul_bytes"]), ul_wait_ms=float(summ["ul_wait_ms"]),
               far_fallback=int(summ["far_fallback"]), p_fail=pf, p_fail_given_fail=pff,
               **{k: float(v) for k, v in summ.items() if k.startswith(("prr_", "prrnohd_"))}, error="")
    if j["study"].startswith("r4pos"):
        dtr, S = R5_traj(n, rep)
        errs, colds, allerr = [], [], []
        for k in range(n):
            m, cf, e = pos_err(gen[src == k], rx[src == k], S[k], dtr, R5.WARM, R5.SIM)
            errs.append(m); colds.append(cf); allerr.append(e)
        allerr = np.concatenate(allerr)
        res.update(cold=cold, pos_err_m=float(np.nanmean(errs)), pos_p95_m=float(np.percentile(allerr, 95)),
                   pos_gt20=float(np.mean(allerr > 20.0)), cold_time=float(np.mean(colds)),
                   stale_t5=float(np.mean([s[1] for _, s in A])))
    return res


_TR = {}
def R5_traj(n, rep):
    if (n, rep) not in _TR:
        _TR[(n, rep)] = load_traj(n, rep)
    return _TR[(n, rep)]


JK = set(BASE) | {"study", "canyon", "n", "policy", "rep", "relayK", "kappa"}
def jkey(r):
    return json.dumps({k: r[k] for k in sorted(r) if k in JK}, sort_keys=True)


def main(par, names):
    todo = [j for nm in names for j in study(nm)]
    done = set()
    for f in (OUT_MAIN, OUT_POS):
        if os.path.exists(f):
            for l in open(f):
                r = json.loads(l)
                if not r.get("error") and r.get("sha256") == SHA4[:16]:
                    done.add(jkey(r))
    todo = [j for j in todo if jkey(j) not in done]
    print(len(todo), "runs to do", flush=True)
    lock = threading.Lock()
    stopfile = os.path.join(HERE, "out", "STOP6")

    def work(j):
        if os.path.exists(stopfile):
            return
        avail = int(open("/proc/meminfo").read().split("MemAvailable:")[1].split()[0]) / 1e6
        if avail < 2.0:
            print("LOW RAM, stopping", avail, flush=True); open(stopfile, "w").write("low ram\n"); return
        r = run(j)
        with lock:
            with open(OUT_POS if j["study"].startswith("r4pos") else OUT_MAIN, "a") as f:
                f.write(json.dumps(r) + "\n")
        print(j["study"], j["canyon"], j["policy"], j["rep"], j.get("ulKbps"), "ERR " + r["error"][:80] if r.get("error") else r["wall_s"], flush=True)

    with ThreadPoolExecutor(par) as ex:
        list(ex.map(work, todo))
    print("done", flush=True)


if __name__ == "__main__":
    main(int(sys.argv[1]), sys.argv[2:])
