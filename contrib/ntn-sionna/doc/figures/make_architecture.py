#!/usr/bin/env python3
"""System design figure of the ntn-sionna module (ns-3 bridge, transports, Sionna RT server).

House style after Deng et al., IEEE OJ-COMS 2026. Writes ntn-sionna-architecture.png (200 dpi) and .svg.

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

    legend(ax, 0.25, 6.96, [("bcast", "Query {tx, rx, freq_hz}"), ("ctrl", "Response {path_loss_db, n_paths}"),
                            ("flow", "Loss into the ns-3 channel")])
    box(ax, 0.2, 6.08, 10.6, 0.62, "Opt-in: the closed-form TR 38.811 channel stays the default", role="smo",
        sub="every run reports which path produced its numbers: ray traced, cached, replayed or FSPL fallback; "
            "RequireLiveTransport makes a missing server fatal", fs=10)

    def col(x0, w, title, items, ys=(4.80, 4.02, 3.24, 2.46)):
        panel(ax, x0, 2.25, w, 3.6, title)
        for (t, r, sub), y in zip(items, ys):
            box(ax, x0 + 0.15, y, w - 0.30, 0.66, t, role=r, sub=sub, fs=8.6)

    col(0.2, 3.55, "ns-3 side (C++, bridge/)", [
        ("NtnSionnaChannel", "nearrt", "PropagationLossModel; counts traced\nand fallback evaluations separately"),
        ("NtnSionnaCascadeChannel", "nf", "traced geometry + NtnAtmosphericLossChain\n(P.676 gas, P.618/P.838 rain, P.681 LMS)"),
        ("SionnaCirPropagationLossModel", "nf", "multipath taps, per-tap Doppler\n(CirDopplerSynthesizer); taps from caller"),
        ("NtnAtmosphericPropagationLossModel", "du", "ITU-R excess only, no double-counted\nFSPL; chains on any spectrum channel"),
    ])
    col(4.0, 3.2, "Transports", [
        ("SionnaUdpTransport", "rt", "socket to a live server\n(GPU on another machine)"),
        ("SionnaPybindTransport", "rt", "in-process, no socket"),
        ("SionnaCachingTransport", "du", "4-D LRU keyed on the geometry\nthat changes the answer"),
        ("SionnaReplayTransport", "du", "record once, replay headless\nSionnaBatchClient for batches"),
    ])
    col(7.45, 3.35, "Python side", [
        ("sionna-server.py", "nonrt", "Mitsuba scene resident on the GPU\nLOS-only or full multipath"),
        ("Sionna RT", "nonrt", "ray tracing, radio maps\n(NVIDIA, open source)"),
        ("tools/", "card", "osm_to_sionna_scene.py\nlidar_dem_ingest.py, probe_sionna_env.py"),
        ("test/sionna-stub-server.py", "card", "same wire protocol, closed-form\nFSPL, for tests without Sionna"),
    ])

    panel(ax, 0.2, 0.20, 10.6, 1.65, "Calibration and consumers")
    box(ax, 0.35, 0.32, 3.25, 0.85, "SionnaCalibrator", role="pale", sub="residual of traced loss against a\nreference model (TR 38.811), dB gate", fs=8.6)
    box(ax, 3.85, 0.32, 3.3, 0.85, "NtnRealStackHelper (ntn-traffic)", role="pale", sub="mmwave NR cell, SGP4 Walker mobility\nmeasured SINR, TBLER, goodput", fs=8.6)
    box(ax, 7.40, 0.32, 3.25, 0.85, "Outputs", role="pale", sub="per-step PL tables, *_kpm_series.csv\nsim_health.csv", fs=8.6)

    # query and response
    line(ax, [(3.60, 5.25), (4.15, 5.25)], "bcast")
    line(ax, [(6.95, 5.25), (7.60, 5.25)], "bcast")
    line(ax, [(7.60, 4.95), (6.95, 4.95)], "ctrl")
    line(ax, [(4.15, 4.95), (3.60, 4.95)], "ctrl")
    line(ax, [(9.12, 4.80), (9.12, 4.68)], "flow")
    line(ax, [(1.97, 4.80), (1.97, 4.68)], "flow")
    # into ns-3
    line(ax, [(2.60, 2.25), (2.60, 1.17)], "flow")
    line(ax, [(3.40, 2.25), (3.40, 1.95), (5.50, 1.95), (5.50, 1.17)], "flow")
    line(ax, [(7.15, 0.74), (7.40, 0.74)], "flow")
    footer(ax, W)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn-sionna-architecture." + ext), dpi=200, facecolor="white")


if __name__ == "__main__":
    main()
