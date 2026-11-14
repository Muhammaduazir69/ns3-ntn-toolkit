#!/usr/bin/env python3
"""ns-3 campaign of the route-break study, one binary (the ntn-constellation-route-lifetime example; its sha256 is in every row).
Resumable: a run whose key is already in out/p7_v8.jsonl with this binary's hash is skipped. Priority order, at most N parallel at
nice 19; stops starting runs under 2 GB of free disk; writes its pid to out/run_p7.pid.
Parts: main, check, front, trig, noise, extra (OneWeb S14), real, fresh (Starlink, Kuiper, new starts), trace, reach, bbm, tacq, pred, cross.
"""
import hashlib, json, os, re, shutil, subprocess, sys, threading
import numpy as np
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
NS3 = os.environ.get("NS3_DIR", os.path.abspath(os.path.join(HERE, "..", "..", "..", "..")))
BIN = os.environ.get("P7_BIN", os.path.join(HERE, "bin", "p7-route-lifetime"))
OUT = os.path.join(HERE, "out", "p7_v8.jsonl")
PID = os.path.join(HERE, "out", "run_p7.pid")
DUR = 2705.0
SHELLS = {"starlink": dict(planes=72, perPlane=22, phasing=39, inc=53.0, altKm=550, star=0),
          "kuiper": dict(planes=34, perPlane=34, phasing=1, inc=51.9, altKm=630, star=0),
          "oneweb": dict(planes=12, perPlane=49, phasing=1, inc=87.9, altKm=1200, star=1)}
PAIRS = {"lon-nyc": (51.51, -0.13, 40.71, -74.01), "syd-lax": (-33.87, 151.21, 34.05, -118.24)}
S6 = sorted(int(x) for x in np.random.default_rng(20261030).uniform(0, 10 * 86400, 6))
S14 = sorted(int(x) for x in np.random.default_rng(20261104).uniform(0, 10 * 86400, 14))
SH = sorted(int(x) for x in np.random.default_rng(20261105).uniform(0, 10 * 86400, 12))
EPOCH_REAL = 1790942400.0  # 2026-10-02 12:00 UTC, the reference of the element sets
EPOCHS = {"starlink16": 1792152000.0}  # 2026-10-16 12:00 UTC


def P(policy, **k):
    return dict(policy=policy, **k)


# t_a = 1 s policy sets
LIFE_EPS = (0.0, 0.01, 0.02, 0.03, 0.05, 0.07, 0.10, 0.15, 0.20)
E = lambda eps, t=None, grid=None: dict(policy="elevb", eps=eps, **({"trigger": t} if t else {}), **({"floorGrid": grid} if grid else {}))
KEY = [P("life", eps=0.05), P("life", eps=0.10), P("life", eps=0.0), P("track", eps=0.05), P("track", eps=0.10), E(0.05), E(0.10), E(0.05, 24.0), E(0.10, 24.0),
       P("asr", window=120.0), P("asrk", window=120.0), P("alpr"), P("isasr", gamma=1.0), P("ondemand")]
REST = [P("snap", period=1.0), P("elev"), P("asr", window=30.0), P("asr", window=300.0), P("mrv"), P("life", eps=0.05, hold=1),
        P("alpr", proactive=0), P("life", eps=0.02, hold=1), P("life", eps=0.10, hold=1)]
FRONT = [P("life", eps=e) for e in (0.01, 0.02, 0.03, 0.07, 0.15, 0.20)] + [P("track", eps=e) for e in (0.02, 0.07, 0.15)] + \
        [E(0.02), P("asr", window=60.0), P("asr", window=180.0), P("asrk", window=60.0), P("asrk", window=180.0), P("isasr", gamma=0.3), P("isasr", gamma=3.0)] + \
        [P("life", eps=e, hold=1) for e in (0.03, 0.07, 0.15)] + [P("elev", trigger=24.0)]
TRIG = [E(e, t) for t in (23.3, 23.5) for e in (0.02, 0.05, 0.10)] + [E(0.02, 24.0), E(0.05, 27.0), E(0.05, 30.0)] + \
       [E(e, t, 1.0) for t in (23.5, 24.0, 25.0) for e in (0.02, 0.05, 0.10)] + [P("track", eps=0.05, trigger=24.0), P("track", eps=0.10, trigger=24.0)]
NOISE = [P("track", eps=e, elevNoiseDeg=sg, **f) for e in (0.05, 0.10) for sg in (0.01, 0.05)
         for f in (dict(trackArc=30.0, trackSamples=3), dict(trackArc=60.0, trackSamples=30))] + [dict(E(0.05, 24.0), elevNoiseDeg=0.05)]
EXTRA = [P("life", eps=e) for e in LIFE_EPS] + [P("life", eps=0.05, hold=1), P("life", eps=0.10, hold=1), P("track", eps=0.05), P("track", eps=0.10),
         E(0.05), E(0.10), E(0.02, 24.0), E(0.05, 24.0), E(0.10, 24.0), P("asr", window=60.0), P("asr", window=120.0), P("asr", window=180.0), P("asrk", window=120.0),
         P("alpr"), P("isasr", gamma=0.3), P("isasr", gamma=1.0), P("isasr", gamma=3.0), P("ondemand")]
FRESH = [P("life", eps=e) for e in LIFE_EPS] + [E(0.02, 24.0), E(0.05, 24.0), E(0.10, 24.0), P("track", eps=0.05), P("track", eps=0.10), P("asr", window=120.0), P("asr", window=180.0),
         P("alpr"), P("isasr", gamma=1.0), P("isasr", gamma=3.0), P("ondemand")]
REALC = [P("life", eps=e) for e in (0.0, 0.02, 0.03, 0.07, 0.15)] + [P("life", eps=0.05, hold=1), P("life", eps=0.10, hold=1), E(0.02, 24.0), E(0.05), E(0.10, 24.0),
         P("asr", window=60.0), P("asr", window=180.0), P("isasr", gamma=1.0), P("alpr"),
         P("track", eps=0.05, elevNoiseDeg=0.05, trackArc=60.0, trackSamples=30), P("track", eps=0.10, elevNoiseDeg=0.05, trackArc=60.0, trackSamples=30)]
REALD = [P("life", eps=0.05), P("life", eps=0.10), P("track", eps=0.05), P("track", eps=0.10), E(0.05, 24.0), P("asr", window=120.0), P("ondemand")]
STRESS = [P("snap", period=1.0), P("ondemand"), E(0.05, 24.0), P("track", eps=0.05), P("life", eps=0.05)]
BBM_ALL = [P("ondemand"), E(0.05, 24.0), P("track", eps=0.05), P("asr", window=120.0), P("asr", window=180.0), P("life", eps=0.05), P("life", eps=0.10)]
SF = sorted(int(x) for x in np.random.default_rng(20261108).uniform(0, 10 * 86400, 6))
SR = sorted(int(x) for x in np.random.default_rng(20261106).uniform(0, 6 * 3600, 12))


def jobs():
    J = []
    cells = [(s, p) for s in SHELLS for p in PAIRS]
    def add(pri, part, sh, pr, st, po, **extra):
        J.append((pri, part, sh, pr, st, dict(po, tAcq=extra.pop("tAcq", 1.0), **extra)))
    for st in S6:
        for sh, pr in cells:
            for po in KEY:
                add(0, "main", sh, pr, st, po)
            for po in FRONT:
                add(1, "front", sh, pr, st, po, noData=1)
            for po in TRIG:
                add(2, "trig", sh, pr, st, po, noData=1)
            for po in REST:
                add(3, "main", sh, pr, st, po)
    for st in S14:
        for pr in PAIRS:
            for po in EXTRA:
                add(2, "extra", "oneweb", pr, st, po, noData=1)
    for st in S6:
        for sh, pr in cells:
            for po in NOISE:
                add(3, "noise", sh, pr, st, po, noData=1)
    for st in S14:
        for pr in PAIRS:
            for po in NOISE:
                add(3, "noise", "oneweb", pr, st, po, noData=1)
    for st in S6:
        for sh, pr in cells:
            add(5, "noise", sh, pr, st, P("track", eps=0.05, elevNoiseDeg=0.05, trackArc=60.0, trackSamples=30))
    for st in SR:
        for sh in ("real_starlink", "real_oneweb"):
            for pr in PAIRS:
                for po in REALD:
                    add(4, "real", sh, pr, st, po, **({} if st in SR[:6] else {"noData": 1}))
                for po in REALC:
                    add(4, "real", sh, pr, st, po, noData=1)
    for st in SF:
        for sh in ("starlink", "kuiper"):
            for pr in PAIRS:
                for po in FRESH:
                    add(5, "fresh", sh, pr, st, po, noData=1)
    for st in SH:
        for sh, pr in cells:
            for po in (P("ondemand"), P("snap", period=1.0)):
                add(3, "trace", sh, pr, st, po, noData=1)
    for st in S6:
        for sh, pr in cells:
            add(3, "reach", sh, pr, st, P("life", eps=0.05), noData=1)
    for st in S6[:2]:
        for sh, pr in cells:
            for po in (P("life", eps=0.05), P("track", eps=0.05), E(0.05), P("asr", window=120.0), P("isasr", gamma=1.0)):
                add(1, "check", sh, pr, st, po, noData=1)
    for st in S6:
        for sh, pr in cells:
            for po in BBM_ALL:
                add(4, "bbm", sh, pr, st, po, bbm=1)
    for st in S6:
        for ta in (0.2, 3.0):
            for po in STRESS:
                add(5, "bbm", "starlink", "lon-nyc", st, po, tAcq=ta, bbm=1)
        for po in (P("snap", period=1.0),):
            add(5, "bbm", "starlink", "lon-nyc", st, po, bbm=1)
        for ta in (0.0, 0.2, 3.0):
            for po in STRESS:
                add(5, "tacq", "starlink", "lon-nyc", st, po, tAcq=ta)
        for sh in ("starlink", "oneweb"):
            for gd in (0.5, 2.0):
                for sg in (5.0, 20.0):
                    add(5, "pred", sh, "lon-nyc", st, P("life", eps=0.05, guard=gd, predSigmaKm=sg))
            for sg in (5.0, 20.0):
                add(5, "pred", sh, "lon-nyc", st, P("asr", window=120.0, predSigmaKm=sg))
                add(5, "pred", sh, "lon-nyc", st, P("asrk", window=120.0, predSigmaKm=sg))
        for po in (P("ondemand"), P("elev"), E(0.05, 24.0), P("track", eps=0.05), P("asr", window=120.0), P("asrk", window=120.0), P("alpr"), P("life", eps=0.05)):
            add(5, "cross", "oneweb", "lon-nyc", st, po, crossLatDeg=70.0)
    J.sort(key=lambda j: j[0])
    return J


def key(part, sh, pr, st, cfg):
    return json.dumps([part, sh, pr, st, sorted(cfg.items())])


def run(j, sha):
    pri, part, sh, pr, st, po = j
    la, loa, lb, lob = PAIRS[pr]
    a = dict(latA=la, lonA=loa, latB=lb, lonB=lob, duration=DUR, tick=0.1, guard=0.5, startOffset=st, islRefresh=5.0)
    if sh.startswith("real_"):
        nm = sh[5:]
        a.update(tle=os.path.join(HERE, "data", f"real_{nm}.tle"), edges=os.path.join(HERE, "data", f"real_{nm}.edges"),
                 epochUnix=EPOCHS.get(nm, EPOCH_REAL), planes=SHELLS[nm]["planes"], perPlane=SHELLS[nm]["perPlane"], phasing=1,
                 inc=SHELLS[nm]["inc"], altKm=SHELLS[nm]["altKm"], star=SHELLS[nm]["star"])
        a["RngRun"] = 1 + st % 1000003
    else:
        a.update(SHELLS[sh])
        a["RngRun"] = 1 + st % 1000003
    a.update(po)
    if part == "reach":
        d = os.path.join(HERE, "out", "reach8"); os.makedirs(d, exist_ok=True)
        a["reachOut"] = os.path.join(d, f"{sh}_{pr}_{st}.csv")
    cmd = ["nice", "-n", "19", BIN] + [f"--{k}={v}" for k, v in a.items()]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=21600, cwd=NS3)
        t = re.sub(r"\bnan\b", "NaN", r.stdout.strip().splitlines()[-1])
        res = json.loads(t)
    except Exception as e:
        res = dict(error=f"{type(e).__name__}: {str(e)[:200]}")
    res.update(part=part, shell=sh, pair=pr, start=st, cfg=sorted(po.items()), bin_sha=sha)
    return res


def main(par, parts=None):
    open(PID, "w").write(str(os.getpid()))
    sha = hashlib.sha256(open(BIN, "rb").read()).hexdigest()
    done = set()
    if os.path.exists(OUT):
        for l in open(OUT):
            r = json.loads(l)
            if "error" not in r and r.get("bin_sha") == sha:
                done.add(json.dumps([r["part"], r["shell"], r["pair"], r["start"], r["cfg"]]))
    todo = [j for j in jobs() if key(j[1], j[2], j[3], j[4], j[5]) not in done and (not parts or j[1] in parts)]
    print(len(todo), "runs to do; sha", sha[:12], flush=True)
    lock = threading.Lock()

    def work(j):
        if shutil.disk_usage(HERE).free < 2e9:
            print("disk under 2 GB, stopping", flush=True)
            return
        r = run(j, sha)
        with lock:
            with open(OUT, "a") as fh:
                fh.write(json.dumps(r) + "\n")
        print(j[1], j[2], j[3], j[4], "ERR " + r["error"] if "error" in r else f"ok cpu={r.get('cpu_s')}", flush=True)

    with ThreadPoolExecutor(par) as ex:
        list(ex.map(work, todo))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 4, set(sys.argv[2].split(",")) if len(sys.argv) > 2 else None)
