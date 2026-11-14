#!/usr/bin/env python3
"""NYC Open Data building footprints (dataset 5zhs-2jue, field height_roof in feet) around Manhattan street stretches.
Stores only (rounded coordinates, height) per stretch in nyc_<name>.json.gz. Resumable."""
import gzip, json, math, os, sys, urllib.parse, urllib.request
# name: (start lat, start lon, azimuth deg, length m, nominal building-to-building width m)
ST = {
 "eighth":   (40.7500, -73.9950, 29.0, 1200.0, 30.5),   # Eighth Avenue, Midtown
 "second":   (40.7395, -73.9800, 29.0, 1200.0, 30.5),   # Second Avenue, Gramercy to Midtown East
 "first":    (40.7230, -73.9885, 29.0, 1200.0, 30.5),   # First Avenue, East Village
 "amsterdam":(40.7780, -73.9820, 29.0, 1200.0, 30.5),   # Amsterdam Avenue, Upper West Side
 "w45":      (40.7610, -73.9920, 119.0, 1000.0, 18.3),  # West 45th Street, Midtown (a narrow cross street)
}
json.dump(ST, open("stretches.json", "w"), indent=1)
for name, (la, lo, az, L, W) in ST.items():
    fn = f"nyc_{name}.json.gz"
    if os.path.exists(fn):
        continue
    A = math.radians(az); m = 150.0
    lat2 = la + L * math.cos(A) / 111132.0; lon2 = lo + L * math.sin(A) / (111320.0 * math.cos(math.radians(la)))
    n, s = max(la, lat2) + m / 111132.0, min(la, lat2) - m / 111132.0
    dl = m / (111320.0 * math.cos(math.radians(la)))
    w, e = min(lo, lon2) - dl, max(lo, lon2) + dl
    out = []; off = 0
    while True:
        q = {"$select": "the_geom,height_roof,bin", "$where": f"within_box(the_geom,{n},{w},{s},{e})", "$limit": 2000, "$offset": off, "$order": "bin"}
        u = "https://data.cityofnewyork.us/resource/5zhs-2jue.json?" + urllib.parse.urlencode(q)
        d = json.load(urllib.request.urlopen(u, timeout=300))
        for r in d:
            if "height_roof" in r and "the_geom" in r:
                out.append([float(r["height_roof"]), [[[round(x, 6), round(y, 6)] for x, y in p[0]] for p in r["the_geom"]["coordinates"]]])
        print(name, off, len(d), flush=True)
        if len(d) < 2000:
            break
        off += 2000
    json.dump(out, gzip.open(fn, "wt"))
    print(name, "buildings", len(out), flush=True)
