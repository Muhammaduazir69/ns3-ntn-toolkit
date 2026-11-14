#!/usr/bin/env python3
"""Real element sets for the routing comparison: pick one shell from a CelesTrak file, cluster it into planes
by right ascension, and write the satellites (3-line TLE) and a +Grid candidate list for the ns-3 example.

Starlink: the 53.16 deg shell within 5 km of its median altitude, as in real_shell_check.py.
OneWeb:   the near-polar shell near 1200 km.
In-plane links join neighbors in argument of latitude. Cross-plane terminals are matched, shortest range first at the
reference instant, one toward higher node and one toward lower node, so every satellite has at most four links.
Writes data/real_<name>.tle, data/real_<name>.edges and out/real_shells.json."""
import json, os, sys
import numpy as np
from sgp4.api import Satrec, jday
import isl_core as C

HERE = os.path.dirname(os.path.abspath(__file__))
REF = (2026, 10, 2, 12, 0, 0)
REFS = {"starlink16": (2026, 10, 16, 12, 0, 0)}


def load(fn, inc_lo, inc_hi, h_lo, h_hi, band):
    L = [l.rstrip() for l in open(fn) if l.strip()]
    rec = []
    for k in range(0, len(L) - 2, 3):
        s = Satrec.twoline2rv(L[k + 1], L[k + 2])
        inc = np.degrees(s.inclo); n = s.no_kozai / 60
        h = ((C.MU / n ** 2) ** (1 / 3) - C.RE) / 1e3
        if inc_lo < inc < inc_hi and h_lo < h < h_hi:
            rec.append((L[k], L[k + 1], L[k + 2], s, h))
    med = np.median([x[4] for x in rec])
    return [x for x in rec if abs(x[4] - med) <= band], med


def elems(s, jd, fr):
    e, r, v = s.sgp4(jd, fr)
    if e != 0:
        return None
    r, v = np.array(r) * 1e3, np.array(v) * 1e3
    h = np.cross(r, v); O = np.arctan2(h[0], -h[1])
    nv = np.array([np.cos(O), np.sin(O), 0])
    u = np.arctan2(r @ np.cross(h / np.linalg.norm(h), nv), r @ nv)
    return O % (2 * np.pi), u % (2 * np.pi), r


def planes_of(O, gap_deg):
    order = np.argsort(O)
    Od = np.degrees(O[order])
    # start at the largest gap so no plane straddles 0
    d = np.diff(np.r_[Od, Od[0] + 360])
    s = int(np.argmax(d)) + 1
    order = np.roll(order, -s)
    Od = np.degrees(O[order]) ; Od = np.where(Od < Od[0], Od + 360, Od)
    groups, cur = [], [order[0]]
    for a, b, ob in zip(Od[:-1], Od[1:], order[1:]):
        if b - a > gap_deg:
            groups.append(cur); cur = []
        cur.append(ob)
    groups.append(cur)
    return groups


def build(name, fn, inc, hband, band, gap, star, xband, xmax):
    rec, med = load(fn, *inc, *hband, band)
    jd, fr = jday(*REFS.get(name, REF))
    E = [elems(x[3], jd, fr) for x in rec]
    rec = [x for x, e in zip(rec, E) if e is not None]; E = [e for e in E if e is not None]
    O = np.array([e[0] for e in E]); U = np.array([e[1] for e in E]); R = np.array([e[2] for e in E])
    G = [g for g in planes_of(O, gap) if len(g) >= 3]
    keep = [i for g in G for i in g]
    idx = {o: n for n, o in enumerate(keep)}
    edges, cross = [], 0
    for g in G:
        g2 = sorted(g, key=lambda i: U[i])
        for a, b in zip(g2, g2[1:] + g2[:1]):
            edges.append((idx[a], idx[b], 0))
    # Cross-plane terminals: each satellite has one toward higher node and one toward lower node. All candidate pairs whose
    # nodes differ by the band below and whose range is within the limit, shortest first, each terminal used once.
    keepa = np.array(keep)
    Ok, Rk = O[keepa], R[keepa]
    cand = []
    for a in range(len(keepa)):
        dO = np.degrees((Ok - Ok[a] + np.pi) % (2 * np.pi) - np.pi)
        dist = np.linalg.norm(Rk - Rk[a], axis=1)
        for b in np.where((dO > xband[0]) & (dO < xband[1]) & (dist < xmax))[0]:
            cand.append((dist[b], a, int(b)))
    cand.sort()
    east, west = set(), set()
    for dd, a, b in cand:
        if a not in east and b not in west:
            east.add(a); west.add(b)
            edges.append((a, b, 1)); cross += 1
    with open(os.path.join(HERE, "data", f"real_{name}.tle"), "w") as f:
        for i in keep:
            f.write(f"{rec[i][0]}\n{rec[i][1]}\n{rec[i][2]}\n")
    with open(os.path.join(HERE, "data", f"real_{name}.edges"), "w") as f:
        for a, b, c in edges:
            f.write(f"{a} {b} {c}\n")
    rng = [np.linalg.norm(R[keep[a]] - R[keep[b]]) / 1e3 for a, b, _ in edges]
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    ok = [(a, b) for (a, b, _), d in zip(edges, rng) if d < 5000]
    A, Bb = zip(*ok)
    ncomp, lab = connected_components(coo_matrix((np.ones(len(A)), (A, Bb)), shape=(len(keep), len(keep))), directed=False)
    largest = int(np.bincount(lab).max())
    return dict(sats=len(keep), planes=len(G), per_plane_min=min(map(len, G)), per_plane_max=max(map(len, G)),
                alt_km=float(med), inc_mean=float(np.degrees(np.mean([rec[i][3].inclo for i in keep]))), edges=len(edges),
                cross=cross, components=int(ncomp), largest_component=largest, max_range_km=float(max(rng)), median_range_km=float(np.median(rng)))


if __name__ == "__main__":
    os.chdir(HERE)
    if len(sys.argv) > 1 and sys.argv[1] == "starlink16":
        r = build("starlink16", "data/starlink_20261016.tle", (53.10, 53.25), (455, 480), 5.0, 1.0, False, (1.0, 15.0), 3000e3)
        d = json.load(open("out/real_shells.json")); d["starlink16"] = r
        json.dump(d, open("out/real_shells.json", "w"), indent=1); print(r); sys.exit(0)
    res = {"starlink": build("starlink", "data/starlink_20261002.tle", (53.10, 53.25), (455, 480), 5.0, 1.0, False, (1.0, 15.0), 3000e3),
           "oneweb": build("oneweb", "data/oneweb_20261002.tle", (87.0, 89.0), (1150, 1250), 50.0, 5.0, True, (5.0, 25.0), 3000e3)}
    json.dump(res, open("out/real_shells.json", "w"), indent=1)
    print(json.dumps(res, indent=1))
