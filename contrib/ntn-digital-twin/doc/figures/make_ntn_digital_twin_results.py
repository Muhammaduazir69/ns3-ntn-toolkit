#!/usr/bin/env python3
"""Data paths are relative to the toolkit workspace root (the folder that holds ns-3-dev/); pass a path on the command line to override.
Results figure for ntn-digital-twin, from the artifacts of a stored twin run.

Run: deterministic Walker source (66 satellites, 6 planes x 11, 53 deg), observer at
33.68 N 73.05 E, 2700 s horizon, 10 s step; artifacts written 6 Oct 2026 by the
twin (twin-orbits.tle, twin-pred.txt, twin.czml).

(a) the handover schedule the twin exported for ns-3 (twin-pred.txt): serving
    satellite against shared-epoch time, with the confidence of each change;
(b) sub-satellite tracks of the 66 satellites read back from the CZML the loop
    wrote (5 min window at loop time, 30 s samples).

    python3 make_ntn_digital_twin_results.py [ARTIFACT_DIR]
"""
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT = "./ns-3-dev/toolkit-audit-2026-10-06/runs/A6-ai-twin-security/twin-walker"
HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    names = [l.strip() for i, l in enumerate(open(os.path.join(d, "twin-orbits.tle"))) if i % 3 == 0]
    meta, preds = {}, []
    for line in open(os.path.join(d, "twin-pred.txt")):
        if line.startswith("#"):
            for tok in line[1:].split():
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    meta[k] = v
            continue
        t, ue, g, c = line.strip().split(",")
        preds.append((float(t), int(g), float(c)))
    horizon = float(meta["horizon_s"])

    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["Arial", "Liberation Sans", "DejaVu Sans"],
                         "font.size": 10, "axes.linewidth": 0.8, "svg.fonttype": "none"})
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 4.1), gridspec_kw={"width_ratios": [1.0, 1.25]})

    ax = axs[0]
    order = sorted({g for _, g, _ in preds})
    ypos = {g: i for i, g in enumerate(order)}
    for i, (t, g, c) in enumerate(preds):
        t_end = preds[i + 1][0] if i + 1 < len(preds) else horizon
        ax.plot([t / 60, t_end / 60], [ypos[g], ypos[g]], color="#0072B2", lw=5, solid_capstyle="butt")
        ax.plot(t / 60, ypos[g], "v", color="#D55E00", ms=8, zorder=5)
        ax.text(t / 60, ypos[g] + 0.22, "conf %.2f" % c, ha="center", va="bottom", fontsize=8.5)
        if i + 1 < len(preds):
            ax.plot([t_end / 60] * 2, [ypos[g], ypos[preds[i + 1][1]]], color="#0072B2", lw=0.8, ls=":")
    ax.set_yticks(range(len(order)))
    ax.set_yticklabels(["sat %d (%s)" % (g, names[g - 1].replace("TWIN ", "")) for g in order], fontsize=9)
    ax.set_ylim(-0.6, len(order) - 0.2)
    ax.set_xlim(0, horizon / 60)
    ax.set_xlabel("Time from shared epoch (min)")
    ax.set_title("(a) Exported handover schedule (%d changes)" % len(preds), fontsize=10.5, loc="left")
    ax.grid(True, axis="x", color="#dddddd", lw=0.6)

    ax = axs[1]
    czml = json.load(open(os.path.join(d, "twin.czml")))
    for pkt in czml:
        if not pkt.get("id", "").startswith("sat-"):
            continue
        cart = pkt["position"]["cartesian"]
        lat, lon = [], []
        for k in range(0, len(cart), 4):
            x, y, z = cart[k + 1:k + 4]
            lo = math.degrees(math.atan2(y, x))
            if lon and abs(lo - lon[-1]) > 180:  # break the track at the antimeridian
                lat.append(float("nan"))
                lon.append(float("nan"))
            lat.append(math.degrees(math.atan2(z, math.hypot(x, y))))
            lon.append(lo)
        ax.plot(lon, lat, "-", color="#999999", lw=1.0)
        ax.plot(lon[-1], lat[-1], "o", color="#0072B2", ms=3.2)
    olat, olon = float(meta["observer_lat_deg"]), float(meta["observer_lon_deg"])
    ax.plot(olon, olat, "*", color="#D55E00", ms=13, mec="black", mew=0.5, label="Observer (UE)")
    ax.plot([], [], "o-", color="#0072B2", ms=3.2, mfc="#0072B2", lw=0, label="Satellite, end of window")
    ax.plot([], [], "-", color="#999999", lw=1.0, label="5 min sub-satellite track")
    ax.set_xlim(-180, 180)
    ax.set_ylim(-60, 60)
    ax.set_xticks(range(-180, 181, 60))
    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("(b) Walker shell read back from the twin's CZML", fontsize=10.5, loc="left")
    ax.grid(True, color="#e3e3e3", lw=0.6)
    ax.legend(frameon=True, framealpha=0.9, edgecolor="none", fontsize=8.6, loc="lower left")
    for a in axs:
        a.spines["top"].set_visible(False)
        a.spines["right"].set_visible(False)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn_digital_twin_results." + ext), dpi=200)
    print("satellites=%d predictions=%d horizon=%.0f s" % (len(names), len(preds), horizon))


if __name__ == "__main__":
    main()
