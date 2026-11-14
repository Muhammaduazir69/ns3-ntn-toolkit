#!/usr/bin/env python3
"""Results figures for the ntn-cho README, built only from stored simulator outputs.

1. ntn_cho_ten_seed_kpis.png: ntn-cho-full-constellation, four policies x ten seeds (40 runs, April 2026),
   66 satellites (6 x 11) at 780 km, 86.4 deg, 30 UEs, 600 s, suburban. Source: kpi_summary.txt of each run in
   papers/sim_runs/ntn-cho/<policy>-seed<k>/ (toolkit workspace). Mean and Student-t 95 % half-width.
2. ntn_cho_rate.png: page render of Fig. "handover rate" of the exit-rule study (papers/paper2_taes_tte_cho,
   figures_v8/fig_rate.pdf), built from sim/runs_nov and sim/analysis_rate.
Usage: python3 make_results.py [WORKSPACE]   (WORKSPACE defaults to the toolkit workspace root)
"""
import os
import re
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
RUNS = os.path.join(WS, "papers", "sim_runs", "ntn-cho")
T9 = 2.262157  # Student t, 9 degrees of freedom, two-sided 95 %
POL = [("a3", "A3 RSRP"), ("time", "Time-based"), ("location", "Location (D1)"), ("tte-aware", "TTE-aware")]
COL = {"a3": "#808080", "time": "#91ace0", "location": "#ee822f", "tte-aware": "#3a71ae"}

plt.rcParams.update({"font.family": "sans-serif",
                     "font.sans-serif": ["Arial", "Liberation Sans", "Nimbus Sans", "DejaVu Sans"],
                     "font.size": 9, "axes.linewidth": 0.8, "axes.grid": True, "grid.color": "#dddddd",
                     "grid.linewidth": 0.6, "axes.axisbelow": True, "svg.fonttype": "none"})


def read(path):
    s = open(path).read()
    g = lambda k: float(re.search(k + r":\s*([0-9.]+)", s).group(1))
    return g("Total Handovers"), g("HO Success Rate"), g("Ping-Pong Rate")


stats = {}
for p, _ in POL:
    v = np.array([read(os.path.join(RUNS, "%s-seed%d" % (p, k), "kpi_summary.txt")) for k in range(1, 11)])
    m, h = v.mean(0), T9 * v.std(0, ddof=1) / np.sqrt(len(v))
    stats[p] = (m, h)
    print("%-10s n=%d  HOs %.1f +- %.1f  success %.2f +- %.2f %%  ping-pong %.2f +- %.2f %%"
          % (p, len(v), m[0], h[0], m[1], h[1], m[2], h[2]))

fig, axs = plt.subplots(1, 3, figsize=(9.0, 2.9))
titles = ("(a) handovers per run", "(b) handover success (%)", "(c) ping-pong (%)")
for i, ax in enumerate(axs):
    x = np.arange(len(POL))
    for j, (p, lab) in enumerate(POL):
        m, h = stats[p]
        ax.bar(j, m[i], 0.62, color=COL[p], yerr=h[i], capsize=3, error_kw=dict(lw=0.9, ecolor="#1a1a1a"))
        ax.text(j, m[i] + h[i] + (0.02 * ax.get_ylim()[1] if i == 0 else 1.2), "%.1f" % m[i], ha="center",
                va="bottom", fontsize=7.6)
    ax.set_xticks(x)
    ax.set_xticklabels([l for _, l in POL], rotation=20, ha="right", fontsize=8)
    ax.set_title(titles[i], fontsize=9, weight="bold")
    ax.grid(axis="x", visible=False)
axs[0].set_ylim(0, 560)
axs[1].set_ylim(0, 112)
axs[2].set_ylim(0, 70)
fig.tight_layout()
for ext, kw in (("png", {"dpi": 200}), ("svg", {})):
    fig.savefig(os.path.join(HERE, "ntn_cho_ten_seed_kpis." + ext), facecolor="white", **kw)

pdf = os.path.join(WS, "papers", "paper2_taes_tte_cho", "figures_v8", "fig_rate.pdf")
if os.path.exists(pdf):
    subprocess.run(["pdftoppm", "-png", "-r", "200", "-singlefile", pdf, os.path.join(HERE, "ntn_cho_rate")],
                   check=True)
