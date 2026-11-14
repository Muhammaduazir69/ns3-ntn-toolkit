#!/usr/bin/env python3
"""Results figure for the ntn-traffic README: the measured radio of NtnRealStackHelper against
3GPP TR 38.821 closed forms, from the ten-seed campaign stored on disk.

Data (one row per sample, ten seeds each, produced by the two shipped calibration examples):
  r1_calibration_samples.csv  ntn-tr38821-calibration --simSeconds=600 --radio=mmwave --RngRun=1..10
  r1_arraygain_samples.csv    ntn-tr38821-array-gain-calibration, --boresightMode=point, RngRun 1..10
Usage: python3 make_results.py [DATA_DIR]
  DATA_DIR defaults to papers/paper1_ojcoms_toolkit/revision_experiments/r1_seeds in the toolkit workspace.
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    HERE, "..", "..", "..", "..", "..", "papers", "paper1_ojcoms_toolkit", "revision_experiments", "r1_seeds")
BLUE, ORANGE, GREY, INK = "#3a71ae", "#ee822f", "#808080", "#1a1a1a"

plt.rcParams.update({"font.family": "sans-serif",
                     "font.sans-serif": ["Arial", "Liberation Sans", "Nimbus Sans", "DejaVu Sans"],
                     "font.size": 9, "axes.linewidth": 0.8, "axes.grid": True, "grid.color": "#dddddd",
                     "grid.linewidth": 0.6, "legend.frameon": False, "svg.fonttype": "none"})


def rows(name):
    with open(os.path.join(DATA, name)) as f:
        return list(csv.DictReader(f))


cal = [r for r in rows("r1_calibration_samples.csv") if r["window"] == "cell_edge"]
ag = [r for r in rows("r1_arraygain_samples.csv") if r["boresight_regime"] == "point"]

fig, (a, b) = plt.subplots(1, 2, figsize=(9.0, 3.4))
# (a) measured CNR against the TR 38.821 closed form over the pass (cell-edge window, >= 10 deg)
el = np.array([float(r["elevation_deg"]) for r in cal])
me = np.array([float(r["measured_cnr_db"]) for r in cal])
tr = np.array([float(r["tr_cnr_db"]) for r in cal])
off = float(np.mean(me - tr))
a.scatter(el, me, s=5, color=BLUE, alpha=0.35, lw=0, label="measured SINR, 10 seeds")
o = np.argsort(el)
a.plot(el[o], tr[o], color=INK, lw=1.4, label="TR 38.821 CNR (closed form)")
a.plot(el[o], tr[o] + off, color=ORANGE, lw=1.4, ls="--", label="closed form + mean offset %.2f dB" % off)
a.set_xlabel("elevation (deg)")
a.set_ylabel("dB")
a.set_title("(a) LEO-600 S-band pass, elevation $\\geq$ 10$^\\circ$", fontsize=9, weight="bold")
a.legend(fontsize=7.5, loc="lower right")

# (b) paired array-gain residual against the analytic TR 38.811 pattern (regime B minus regime A)
by = {}
for r in ag:
    by.setdefault((r["seed"], r["t_s"]), {})[r["regime"]] = r
th, pd_, an = [], [], []
for k, d in by.items():
    if "A_steered" in d and "B_fixed" in d:
        rb, ra = d["B_fixed"], d["A_steered"]
        th.append(float(rb["theta_offboresight_deg"]))
        pd_.append(float(rb["residual_db"]) - float(ra["residual_db"]))
        an.append(float(rb["analytic_pattern_db"]))
th, pd_, an = map(np.array, (th, pd_, an))
b.scatter(th, pd_, s=5, color=BLUE, alpha=0.35, lw=0, label="measured, paired (B minus A), 10 seeds")
o = np.argsort(th)
b.plot(th[o], an[o], color=INK, lw=1.4, label="analytic Airy pattern")
b.set_xlabel("off-boresight angle (deg)")
b.set_ylabel("pattern gain (dB)")
b.set_ylim(-45, 8)
b.set_title("(b) fixed boresight, beam pattern", fontsize=9, weight="bold")
b.legend(fontsize=7.5, loc="lower left")
fig.tight_layout()
for ext, kw in (("png", {"dpi": 200}), ("svg", {})):
    fig.savefig(os.path.join(HERE, "ntn_traffic_calibration." + ext), facecolor="white", **kw)
print("samples: calibration %d, array gain pairs %d, mean offset %.3f dB" % (len(cal), len(th), off))
