#!/usr/bin/env python3
"""Data paths are relative to the toolkit workspace root (the folder that holds ns-3-dev/); pass a path on the command line to override.
Results figure for oran-ntn: A1 intents and control placement under feeder outages.

Reads the stored campaign of `oran-ntn-feeder-mgmt` (main group: 550 km
Walker-Delta shell, 1 to 4 gateways, 10 seeds per configuration, 1 h simulated
per run) and plots, per strategy, with 95 % t-intervals over seeds:

  (a) share of served time the cell falls short of its A1 intent,
  (b) mean age of the intent an on-board controller enforces (stale intents),
  (c) near-RT deadline misses (control placement).

It also writes oran_ntn_results_summary.csv beside the figure so every number
quoted in the README can be traced.

    python3 make_oran_ntn_results.py [RUNS_DIR]
RUNS_DIR defaults to papers/paper3_tnsm_oran_ntn/sim/runs/main in the
workspace that holds the campaign.
"""
import csv
import glob
import math
import os
import statistics as st
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT = "./papers/paper3_tnsm_oran_ntn/sim/runs/main"
# strategy key -> (label, color, marker, dash); Okabe-Ito colors, markers and dashes so print works
STRATS = [
    ("ground", "Ground only", "#000000", "s", (0, (1, 0))),
    ("onboard", "On-board only", "#E69F00", "^", (0, (5, 2))),
    ("reactive", "Reactive hybrid", "#56B4E9", "v", (0, (3, 1, 1, 1))),
    ("onboard-ttl", "On-board + lease", "#009E73", "D", (0, (1, 1))),
    ("reactive-ttl", "Reactive + lease", "#CC79A7", "o", (0, (6, 2, 1, 2))),
    ("pal", "PAL (predictive lease)", "#0072B2", "P", (0, (1, 0))),
]
T95 = 2.262  # t(0.975, 9) for 10 seeds


def load(runs, key, gw):
    rows = []
    for f in sorted(glob.glob(os.path.join(runs, "%s_s550_gw%d" % (key, gw), "seed*", "summary.csv"))):
        with open(f) as fh:
            rows.append(next(csv.DictReader(fh)))
    return rows


def mci(v):
    m = st.mean(v)
    return m, (T95 * st.stdev(v) / math.sqrt(len(v)) if len(v) > 1 else 0.0)


def main():
    runs = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    here = os.path.dirname(os.path.abspath(__file__))
    plt.rcParams.update({"font.family": "sans-serif",
                         "font.sans-serif": ["Arial", "Liberation Sans", "DejaVu Sans"],
                         "font.size": 10, "axes.linewidth": 0.8, "svg.fonttype": "none"})
    gws = [1, 2, 3, 4]
    metrics = {
        "viol": lambda r: 100.0 * float(r["sla_viol_frac"]),
        "stale": lambda r: float(r["stale_mean_s"]),
        "nfmiss": lambda r: 100.0 * (1.0 - float(r["nf_hit_ratio"])),
    }
    table = []
    data = {}
    for key, *_ in STRATS:
        for gw in gws:
            rows = load(runs, key, gw)
            if not rows:
                continue
            rec = {"strategy": key, "gateways": gw, "seeds": len(rows)}
            for m, fn in metrics.items():
                mean, ci = mci([fn(r) for r in rows])
                rec[m + "_mean"], rec[m + "_ci95"] = mean, ci
                data[(key, gw, m)] = (mean, ci)
            rec["binary_sha"] = rows[0]["binary_sha"]
            table.append(rec)
    if not table:
        sys.exit("no runs found under " + runs)

    fig, axs = plt.subplots(1, 3, figsize=(12.0, 3.9))
    panels = [("viol", "(a) Time short of the A1 intent (%)", False),
              ("stale", "(b) Mean age of enforced intent (s)", True),
              ("nfmiss", "(c) Near-RT deadline misses (%)", True)]
    for ax, (m, title, logy) in zip(axs, panels):
        for i, (key, label, col, mk, dash) in enumerate(STRATS):
            pts = [(gw + (i - 2.5) * 0.045,) + data[(key, gw, m)] for gw in gws if (key, gw, m) in data]
            if m == "stale" and key == "ground":
                continue  # ground only enforces the RIC's own copy; its age is about 1e-6 s
            if not pts:
                continue
            x, y, e = zip(*pts)
            # on log axes a symmetric 95 % interval can reach below zero; clip the lower whisker
            lo = [min(ei, 0.9 * yi) for yi, ei in zip(y, e)] if logy else e
            ax.errorbar(x, y, yerr=[lo, e], color=col, marker=mk, ls=dash, lw=1.4, ms=5.5, capsize=2.5,
                        label=label, mfc="white" if key in ("onboard", "reactive") else col)
        ax.set_title(title, fontsize=10.5, loc="left")
        ax.set_xlabel("Gateways")
        ax.set_xticks(gws)
        if logy:
            ax.set_yscale("log")
        ax.grid(True, which="major", color="#d0d0d0", lw=0.6)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
    axs[1].text(0.97, 0.95, "ground only: about 1e-6 s\n(not drawn)", transform=axs[1].transAxes,
                ha="right", va="top", fontsize=8.5, color="#333333")
    axs[2].text(0.97, 0.42, "on-board only and on-board\n+ lease overlap (0.002 %)", transform=axs[2].transAxes,
                ha="right", va="center", fontsize=8.5, color="#333333")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=6, frameon=False, fontsize=9.2, bbox_to_anchor=(0.5, 1.02))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(here, "oran_ntn_results." + ext), dpi=200)

    with open(os.path.join(here, "oran_ntn_results_summary.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(table[0].keys()))
        w.writeheader()
        for rec in table:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in rec.items()})
    print("wrote oran_ntn_results.png/.svg and oran_ntn_results_summary.csv (%d configs)" % len(table))


if __name__ == "__main__":
    main()
