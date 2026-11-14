#!/usr/bin/env python3
"""System design figure of the ntn-slice module (slice definition, MAC-enforced quotas, monitors).

House style after Deng et al., IEEE OJ-COMS 2026. Writes ntn-slice-architecture.png (200 dpi) and .svg.

    nice -n 19 python3 make_architecture.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- palette (Deng et al. 2026, sampled) -------------------------------------------
ROLES = {
    "smo": ("#3a71ae", "#3a71ae", "white"),
    "nonrt": ("#75bd42", "#75bd42", "white"),
    "nearrt": ("#ee822f", "#ee822f", "white"),
    "rt": ("#3d7199", "#3d7199", "white"),
    "ai": ("#69c8bd", "#69c8bd", "white"),
    "nf": ("#91ace0", "#91ace0", "#1a1a1a"),
    "du": ("#b6c7ea", "#8fa9df", "#1a1a1a"),
    "green": ("#acd78e", "#8cc56a", "#1a1a1a"),
    "card": ("#fef1c7", "#f3d98a", "#1a1a1a"),
    "pale": ("#dae3f2", "#b9c8e3", "#1a1a1a"),
}
LINES = {
    "flow": ("#7f8fb3", "-", 2.2, "-|>"),
    "bcast": ("#e54c5e", "-", 1.6, "-|>"),
    "ctrl": ("#35b44a", "-", 1.6, "-|>"),
    "xmod": ("#2e54a1", (0, (4, 2)), 1.4, "-|>"),
}
INK = "#1a1a1a"


def setup():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Nimbus Sans", "Liberation Sans", "DejaVu Sans"],
        "font.weight": "bold", "svg.fonttype": "none",
    })


def box(ax, x, y, w, h, title, role="nf", sub=None, fs=9.0, dashed=False):
    fc, ec, tc = ROLES[role]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=%g" % (0.10 * min(w, h)),
                                fc=fc, ec="#2e54a1" if dashed else ec, lw=1.3 if dashed else 0.9,
                                ls=(0, (4, 2)) if dashed else "-", zorder=5))
    ty = y + h * (0.70 if sub else 0.5)
    ax.text(x + w / 2, ty, title, ha="center", va="center", color=tc, fontsize=fs, weight="bold", zorder=6)
    if sub:
        ax.text(x + w / 2, y + h * 0.34, sub, ha="center", va="center", color=tc, fontsize=fs - 1.6,
                weight="normal", zorder=6, linespacing=1.2)


def panel(ax, x, y, w, h, title, fc="white"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="square,pad=0", fc=fc, ec="#808080", lw=0.9, zorder=1))
    ax.text(x + 0.08, y + h - 0.08, title, ha="left", va="top", fontsize=9.6, weight="bold", color=INK, zorder=2)


def line(ax, pts, kind="flow", label=None, at=0, dx=0.05, dy=0.05, ha="left", fs=8.0):
    col, ls, lw, arr = LINES[kind]
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=lw, ls=ls, zorder=3, solid_capstyle="butt")
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle=arr, mutation_scale=11, color=col, lw=lw, ls=ls,
                                 shrinkA=0, shrinkB=0, zorder=3))
    if label:
        a, b = pts[at], pts[at + 1]
        ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, label, fontsize=fs, weight="bold", color=INK,
                ha=ha, va="bottom", zorder=7)


def legend(ax, x, y, items, fs=8.4):
    from matplotlib.textpath import TextPath
    for kind, text in items:
        col, ls, lw, _ = LINES[kind]
        ax.plot([x, x + 0.34], [y, y], color=col, lw=lw + 0.3, ls=ls)
        ax.text(x + 0.42, y, text, fontsize=fs, weight="bold", va="center", ha="left", color=INK)
        x += 0.42 + TextPath((0, 0), text, size=fs).get_extents().width / 72.0 * 1.12 + 0.30


def footer(ax, W):
    ax.text(W - 0.05, 0.02, "Muhammad Uzair, Independent Researcher", ha="right", va="bottom", fontsize=6.8,
            weight="normal", color="#606060")


# ---- figure --------------------------------------------------------------------------
def main():
    setup()
    W, H = 11.0, 7.2
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    fig.patch.set_facecolor("white")

    legend(ax, 0.25, 6.96, [("flow", "Slice mapping"), ("bcast", "Quota actuation (SetQuotas)"),
                            ("ctrl", "Measured counters"), ("xmod", "Observation only, not actuating")])
    box(ax, 0.2, 6.08, 10.6, 0.62, "NtnRealStackHelper (ntn-traffic)", role="smo",
        sub="5G-LENA NR cell (nr) or mmwave cell over an SGP4 Walker pass, TR 38.811 UEs, NtnOranApplication QoS flows",
        fs=10)

    panel(ax, 0.2, 2.25, 2.85, 3.6, "Slice definition")
    box(ax, 0.35, 4.45, 2.55, 0.85, "ntn-slice-types.h", role="green", sub="Snssai (SST + SD), SliceProfile\nTS 22.261 default profiles")
    box(ax, 0.35, 3.40, 2.55, 0.80, "NtnSliceSelector", role="green", sub="DSCP, port range, app label\nfirst match wins")
    box(ax, 0.35, 2.40, 2.55, 0.80, "NtnSliceHelper", role="green", sub="ThreeSliceDefault()\nShouldSkipGeo(), IsGeoSatellite()")

    panel(ax, 3.35, 2.25, 4.1, 3.6, "Enforced in the MAC (5G-LENA)")
    box(ax, 3.5, 4.45, 3.8, 0.85, "NtnSliceQuotaScheduler", role="nearrt",
        sub="OFDMA PF; per-slot RBG quota per slice\nwork conserving: unused RBGs are lent", fs=9.2)
    box(ax, 3.5, 3.40, 3.8, 0.80, "Quota controllers (examples)", role="nonrt",
        sub="static | predict (ephemeris) | reactive (measured)\ncontract | contract_eph | contract_trim", fs=8.8)
    box(ax, 3.5, 2.40, 1.8, 0.80, "TrimmedQuota()", role="du", sub="ntn-slice-contract-\ntrim.h", fs=8.4)
    box(ax, 5.5, 2.40, 1.8, 0.80, "Baselines", role="card", sub="pf | qos (5QI priority)\nbwp | policed | limiter", fs=8.2)

    panel(ax, 7.75, 2.25, 3.05, 3.6, "Observed and shadow")
    box(ax, 7.9, 4.45, 2.75, 0.85, "SliceOrchestratorXapp", role="rt", sub="shadow PRB allocator, logged\nStepWithShares() for RL", fs=8.8)
    box(ax, 7.9, 3.40, 2.75, 0.80, "SliceIsolationMonitor", role="rt", sub="per-slice p99 delay, loss\nBreachEvent records")
    box(ax, 7.9, 2.40, 2.75, 0.80, "Grafana dashboard", role="rt", sub="dashboards/ntn-slice.json\n(ntn-observability stack)")

    panel(ax, 0.2, 0.20, 10.6, 1.65, "Outputs")
    box(ax, 0.35, 0.32, 2.55, 0.85, "JSON line per run", role="pale", sub="per-window elevation, delay p99,\nrates, quota, bytes per RBG", fs=8.6)
    box(ax, 3.5, 0.32, 1.8, 0.85, "RBG accounting", role="pale", sub="TakeRbgCounts()\nTakeBytesPerRbg()", fs=8.6)
    box(ax, 5.5, 0.32, 1.8, 0.85, "Sinks", role="pale", sub="per-packet one-way\ndelay (in-band stamp)", fs=8.6)
    box(ax, 7.9, 0.32, 2.75, 0.85, "CSV files", role="pale", sub="slices.csv, ntn-three-slice.csv\nsim_health.csv", fs=8.6)

    line(ax, [(2.90, 3.80), (3.15, 3.80), (3.15, 4.70), (3.50, 4.70)], "flow")
    ax.text(3.18, 4.25, "RNTI to slice", fontsize=7.6, weight="bold", color=INK, ha="left", va="center", rotation=90)
    line(ax, [(1.62, 4.45), (1.62, 4.20)], "flow")
    line(ax, [(4.70, 4.20), (4.70, 4.45)], "bcast")
    line(ax, [(6.20, 4.45), (6.20, 4.20)], "ctrl")
    ax.text(6.27, 4.33, "backlog, bytes/RBG", fontsize=7.4, weight="bold", color=INK, ha="left", va="center")
    ax.text(4.63, 4.33, "SetQuotas", fontsize=7.4, weight="bold", color=INK, ha="right", va="center")
    line(ax, [(4.40, 3.20), (4.40, 3.40)], "flow")
    line(ax, [(6.40, 1.17), (6.40, 2.05), (7.60, 2.05), (7.60, 3.80), (7.90, 3.80)], "ctrl")
    line(ax, [(7.30, 4.88), (7.90, 4.88)], "xmod")
    line(ax, [(9.27, 2.40), (9.27, 1.17)], "flow")
    footer(ax, W)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn-slice-architecture." + ext), dpi=200, facecolor="white")


if __name__ == "__main__":
    main()
