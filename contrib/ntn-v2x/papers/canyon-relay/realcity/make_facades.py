#!/usr/bin/env python3
"""Facade profile of each stretch from the NYC footprints: finds the free corridor of the street, then, every meter along
it and per wall, the first building within 12 m behind the wall line (a setback tolerance); no building means a gap.
Writes facade_<name>.csv (side,x0,x1,height m), stretches_ns3.json and facade_stats.json (law fit, gaps, correlation)."""
import gzip, json, math, os
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.strtree import STRtree
HERE = os.path.dirname(os.path.abspath(__file__))
ST = json.load(open(f"{HERE}/stretches.json"))
FT = 0.3048
res, stats = {}, {}
for name, (la, lo, az, L, Wnom) in ST.items():
    B = json.load(gzip.open(f"{HERE}/nyc_{name}.json.gz", "rt"))
    A = math.radians(az); ca, sa = math.cos(A), math.sin(A)
    kx = 111320.0 * math.cos(math.radians(la)); ky = 111132.0
    def loc(p):
        e, n = (p[0] - lo) * kx, (p[1] - la) * ky
        return (e * sa + n * ca, e * ca - n * sa)      # along the azimuth, then to its right
    polys, H = [], []
    for h, ps in B:
        for r in ps:
            if len(r) >= 4:
                pg = Polygon([loc(q) for q in r]).buffer(0)
                if not pg.is_empty:
                    polys.append(pg); H.append(h * FT)
    H = np.array(H); tree = STRtree(polys)
    # occupancy against lateral offset
    ys = np.arange(-120, 120, 1.0)
    occ = []
    for y in ys:
        ln = LineString([(0, y), (L, y)])
        idx = tree.query(ln)
        occ.append(sum(polys[i].intersection(ln).length for i in idx) / L)
    occ = np.array(occ)
    free = occ < 0.08
    # corridor: the free run closest to the nominal width and to the origin
    runs = []; i = 0
    while i < len(ys):
        if free[i]:
            j = i
            while j + 1 < len(ys) and free[j + 1]:
                j += 1
            runs.append((ys[i], ys[j] + 1)); i = j + 1
        else:
            i += 1
    cand = [r for r in runs if r[1] - r[0] >= 0.6 * Wnom]
    y0, y1 = min(cand, key=lambda r: abs((r[0] + r[1]) / 2))
    W = y1 - y0
    prof = {}
    for side, yw, sgn in ((0, y0, -1), (1, y1, +1)):     # side 0: the wall at y = 0 of the model
        h = np.zeros(int(L))
        for x in range(int(L)):
            seg = LineString([(x + 0.5, yw + sgn * 2.0 * -1), (x + 0.5, yw + sgn * 12.0)])
            idx = tree.query(seg)
            best, hb = 1e9, 0.0
            for i in idx:
                g = polys[i].intersection(seg)
                if not g.is_empty:
                    d = abs(g.bounds[1] - yw) if sgn > 0 else abs(g.bounds[3] - yw)
                    if d < best:
                        best, hb = d, H[i]
            h[x] = hb
        prof[side] = h
    # run-length encoding into segments
    rows, seg_h = [], []
    for side in (0, 1):
        h = prof[side]; s = 0
        for x in range(1, int(L) + 1):
            if x == int(L) or abs(h[x] - h[s]) > 0.01:
                rows.append((side, s, x, round(float(h[s]), 1)))
                if h[s] > 0:
                    seg_h.append((side, s, x, float(h[s])))
                s = x
    with open(f"{HERE}/facade_{name}.csv", "w") as f:
        for r in rows:
            f.write("%d,%d,%d,%.1f\n" % r)
    lens = np.array([b - a for _, a, b, _ in seg_h]); hs = np.array([h for *_, h in seg_h])
    lz = np.log(hs)
    gapshare = float(np.mean(np.r_[prof[0], prof[1]] == 0))
    c1 = []
    for side in (0, 1):
        z = [np.log(h) for sd, a, b, h in seg_h if sd == side]
        c1.append(np.corrcoef(z[:-1], z[1:])[0, 1])
    # sampled every 30 m the two walls, for cross-wall correlation
    pa, pb = prof[0][15::30], prof[1][15::30]
    m = (pa > 0) & (pb > 0)
    res[name] = dict(file=f"{HERE}/facade_{name}.csv", W=round(float(W), 1), az=az, L=L, Lb=round(float(lens.mean()), 1),
                     h50=round(float(np.exp(np.median(lz))), 1), s=round(float(lz.std()), 3))
    stats[name] = dict(n_segments=len(seg_h), gap_share=round(gapshare, 3), lag1_corr=[round(float(c), 3) for c in c1],
                       cross_wall_corr=round(float(np.corrcoef(np.log(pa[m]), np.log(pb[m]))[0, 1]), 3),
                       meters_law_h50=res[name]["h50"], p10=round(float(np.percentile(hs, 10)), 1), p90=round(float(np.percentile(hs, 90)), 1), start=[la, lo], corridor=[float(y0), float(y1)], occ_at_center=round(float(occ[len(ys)//2]), 3))
    print(name, res[name], stats[name], flush=True)
json.dump(res, open(f"{HERE}/stretches_ns3.json", "w"), indent=1)
json.dump(stats, open(f"{HERE}/facade_stats.json", "w"), indent=1)
