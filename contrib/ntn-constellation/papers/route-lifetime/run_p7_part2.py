#!/usr/bin/env python3
"""Second set of runs on the same binary as run_p7.py (sha in every row), into out/p7_v9.jsonl. Resumable, nice 19, at most N
jobs, stops under 2 GB free disk, pid in out/run_p7_part2.pid. Run run_p7.py first.
  repro   a sample of stored run_p7.py rows of every kind, rerun to show they reproduce (new flags off)
  isasr   every ISASR configuration, both the variant with free ISL links and Algorithm 3 (isasrIslAct=1), gamma 0.1 to 1000
  relev   the lattice elevation-rank sweep on the four real cells, 12 starts each
"""
import hashlib, json, os, random, shutil, sys, threading
from concurrent.futures import ThreadPoolExecutor
import run_p7 as R7

HERE = R7.HERE
BIN = R7.BIN
OUT = os.path.join(HERE, "out", "p7_v9.jsonl")
PID = os.path.join(HERE, "out", "run_p7_part2.pid")
R7.BIN = BIN
P, E = R7.P, R7.E
CELLS = [(s, p) for s in R7.SHELLS for p in R7.PAIRS]
REALC = [(s, p) for s in ("real_starlink", "real_oneweb") for p in R7.PAIRS]


R7.SHELLS["kuiper2"] = dict(planes=36, perPlane=36, phasing=1, inc=42.0, altKm=610, star=0)
R7.SHELLS["starlink70"] = dict(planes=36, perPlane=20, phasing=1, inc=70.0, altKm=570, star=0)
R7.SHELLS["starlink16"] = dict(R7.SHELLS["starlink"])
R7.PAIRS["mad-nyc"] = (40.42, -3.70, 40.71, -74.01)
SHO = sorted(int(x) for x in R7.np.random.default_rng(20261110).uniform(0, 10 * 86400, 18))
SHO_REAL = sorted(int(x) for x in R7.np.random.default_rng(20261111).uniform(0, 6 * 3600, 18))
SOW = sorted(int(x) for x in R7.np.random.default_rng(20261112).uniform(0, 10 * 86400, 40))
HELD = [("kuiper2", "mad-nyc"), ("kuiper2", "syd-lax"), ("starlink70", "mad-nyc"), ("starlink70", "syd-lax"),
        ("real_starlink16", "lon-nyc"), ("real_starlink16", "syd-lax")]
BENCH = [P("life", eps=e) for e in (0.0, 0.02, 0.05, 0.10, 0.15)] + [P("elev", trigger=24.0), E(0.02, 24.0, 1.0), E(0.05, 24.0), E(0.10, 24.0),
         P("asr", window=60.0), P("asr", window=120.0), P("asr", window=180.0), P("isasr", gamma=1.0, isasrIslAct=1),
         P("isasr", gamma=1000.0, isasrIslAct=1), P("isasr", gamma=1.0), P("track", eps=0.05), P("track", eps=0.10)]


def jobs():
    J = []
    def add(part, sh, pr, st, po, **ex):
        J.append((part, sh, pr, st, dict(po, tAcq=ex.pop("tAcq", 1.0), **ex)))
    # repro: 48 stored rows, fixed seed, spread over parts and policies
    rows = [json.loads(l) for l in open(os.path.join(HERE, "out", "p7_v8.jsonl"))]
    rng = random.Random(9)
    byk = {}
    for r in rows:
        byk.setdefault((r["part"], dict(r["cfg"])["policy"]), []).append(r)
    for k in sorted(byk):
        for r in rng.sample(byk[k], min(2, len(byk[k]))):
            J.append(("repro:" + r["part"], r["shell"], r["pair"], r["start"], dict(r["cfg"])))
    gam = (0.3, 1.0, 3.0, 1000.0)
    P0 = ("lon-nyc", "syd-lax")
    starts = [(c, st) for c in CELLS for st in R7.S6] + [(("oneweb", p), st) for p in P0 for st in R7.S14] + \
             [(c, st) for c in REALC for st in R7.SR] + [((s, p), st) for s in ("starlink", "kuiper") for p in P0 for st in R7.SF]
    for (sh, pr), st in starts:
        for gm in gam:
            add("isasr", sh, pr, st, P("isasr", gamma=gm, isasrIslAct=1), noData=1)
            if gm != 1000.0:
                add("isasr", sh, pr, st, P("isasr", gamma=gm), noData=1)
    for (sh, pr) in CELLS:
        for st in R7.S6:
            add("isasr", sh, pr, st, P("isasr", gamma=1.0, isasrIslAct=1))
    # held-out constellations and epoch: never used while the code or the analysis was developed
    for (sh, pr) in HELD:
        ss = SHO_REAL if sh.startswith("real_") else SHO
        for st in ss[6:]:
            for po in (P("ondemand"), P("snap", period=1.0)):
                add("htrace", sh, pr, st, po, noData=1)
        for st in ss[:6]:
            for po in (P("ondemand"), P("snap", period=1.0)):
                add("hpacket", sh, pr, st, po)
            for po in BENCH:
                add("hbench", sh, pr, st, po, noData=1)
    # the gamma grid edge, one point below 0.3, on the real cells and the development lattices
    for (sh, pr) in REALC:
        for st in R7.SR:
            add("isasr", sh, pr, st, P("isasr", gamma=0.1, isasrIslAct=1), noData=1)
    for (sh, pr) in CELLS:
        for st in R7.S6:
            add("isasr", sh, pr, st, P("isasr", gamma=0.1, isasrIslAct=1), noData=1)
    for pr in ("lon-nyc", "syd-lax"):
        for st in SOW:
            for po in (P("ondemand"), P("snap", period=1.0)):
                add("owtrace", "oneweb", pr, st, po, noData=1)
    sets = [P("elev", trigger=24.0), P("elev")] + [E(e, t, g) for t in (23.3, 23.5, 24.0, 25.0) for e in (0.02, 0.05, 0.10) for g in (1.0, None)]
    have = [E(0.05), E(0.02, 24.0), E(0.10, 24.0), E(0.05, 24.0)]
    for (sh, pr) in REALC:
        for st in R7.SR:
            for po in sets:
                if po.get("trigger") == 25.0 and "floorGrid" not in po and po.get("eps") == 0.05:
                    continue
                if po in have:
                    continue
                add("relev", sh, pr, st, po, noData=1)
    return J


def main(par):
    open(PID, "w").write(str(os.getpid()))
    sha = hashlib.sha256(open(BIN, "rb").read()).hexdigest()
    done = set()
    if os.path.exists(OUT):
        for l in open(OUT):
            try:
                r = json.loads(l)
            except Exception:
                continue
            if "error" not in r and r.get("bin_sha") == sha:
                done.add(json.dumps([r["part"], r["shell"], r["pair"], r["start"], r["cfg"]]))
    todo = [j for j in jobs() if json.dumps([j[0], j[1], j[2], j[3], sorted(j[4].items())]) not in done]
    print(len(todo), "runs to do; sha", sha[:12], flush=True)
    lock = threading.Lock()

    def work(j):
        if shutil.disk_usage(HERE).free < 2e9:
            print("disk under 2 GB, stopping", flush=True)
            return
        part, sh, pr, st, po = j
        r = R7.run((0, part.split(":")[-1], sh, pr, st, po), sha)
        r["part"] = part
        with lock:
            with open(OUT, "a") as fh:
                fh.write(json.dumps(r) + "\n")
        print(part, sh, pr, st, "ERR " + r["error"] if "error" in r else "ok", flush=True)

    with ThreadPoolExecutor(par) as ex:
        list(ex.map(work, todo))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 3)
