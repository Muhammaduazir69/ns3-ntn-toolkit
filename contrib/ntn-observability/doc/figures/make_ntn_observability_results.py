#!/usr/bin/env python3
"""Data paths are relative to the toolkit workspace root (the folder that holds ns-3-dev/); pass a path on the command line to override.
Results figure for ntn-observability, drawn from an exported line-protocol file.

The figure is read back from what the sink wrote, not from the simulator's
memory, so it doubles as a check that the export carries the measured series.
Default input: the stored 60 s run of ntn-observability-traffic
(550 km SGP4 pass, 20 GHz, 85 dBm EIRP, one run, seed 1), sweep of 25 Jun 2026.

    python3 make_ntn_observability_results.py [LP_FILE]
"""
import os
import statistics as st
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT = "./sweep-results-2026-06-25/ntn-observability/traffic-heavy.lp"
HERE = os.path.dirname(os.path.abspath(__file__))


def parse(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("ntn_downlink"):
            continue
        head, fields, ts = line.rsplit(" ", 2)
        d = {}
        for kv in fields.split(","):
            k, v = kv.split("=", 1)
            d[k] = float(v.rstrip("i"))
        d["t"] = int(ts) / 1e9
        rows.append(d)
    t0 = rows[0]["t"] - 1.0  # first point is written at t = 1 s
    for r in rows:
        r["t"] -= t0
    return rows


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    rows = parse(path)
    meas = [r for r in rows if "sinr_db" in r]
    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["Arial", "Liberation Sans", "DejaVu Sans"],
                         "font.size": 10, "axes.linewidth": 0.8, "svg.fonttype": "none"})
    fig, axs = plt.subplots(1, 3, figsize=(12.0, 3.7))

    ax = axs[0]
    ax.plot([r["t"] for r in rows], [r["elevation_deg"] for r in rows], "-", color="#0072B2", lw=1.8,
            label="Elevation")
    ax.set_ylabel("Elevation (deg)", color="#0072B2")
    ax2 = ax.twinx()
    ax2.plot([r["t"] for r in rows], [r["slant_range_km"] for r in rows], "--", color="#E69F00", lw=1.8,
             label="Slant range")
    ax2.set_ylabel("Slant range (km)", color="#E69F00")
    ax.set_title("(a) SGP4 geometry (field values)", fontsize=10.5, loc="left")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, frameon=False, fontsize=9, loc="center left")

    ax = axs[1]
    ax.plot([r["t"] for r in meas], [r["sinr_db"] for r in meas], "-o", color="#009E73", ms=3, lw=1.3)
    ax.set_ylabel("SINR (dB)")
    ax.set_ylim(20, 28)
    ax3 = ax.twinx()
    ax3.bar([r["t"] for r in meas], [r["tbler"] for r in meas], 0.6, color="#CC79A7", alpha=0.6)
    ax3.set_ylabel("TBLER", color="#CC79A7")
    ax3.set_ylim(0, 0.6)
    ax.set_zorder(ax3.get_zorder() + 1)
    ax.patch.set_visible(False)
    ax.set_title("(b) Measured PHY: SINR and TBLER", fontsize=10.5, loc="left")

    ax = axs[2]
    ax.plot([r["t"] for r in meas], [r["goodput_mbps"] for r in meas], "-", color="#000000", lw=1.6)
    ax.set_ylabel("Goodput (Mb/s)")
    ax.set_ylim(4.0, 5.2)
    ax.set_title("(c) Sink goodput", fontsize=10.5, loc="left")
    for a in axs:
        a.set_xlabel("Simulation time (s)")
        a.grid(True, color="#dddddd", lw=0.6)
        a.spines["top"].set_visible(False)
    axs[2].spines["right"].set_visible(False)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn_observability_results." + ext), dpi=200)
    print("points=%d measured=%d mean_sinr=%.2f dB mean_tbler=%.4f mean_goodput=%.3f Mb/s elev %.1f->%.1f deg"
          % (len(rows), len(meas), st.mean(r["sinr_db"] for r in meas), st.mean(r["tbler"] for r in meas),
             st.mean(r["goodput_mbps"] for r in meas), rows[0]["elevation_deg"], rows[-1]["elevation_deg"]))


if __name__ == "__main__":
    main()
