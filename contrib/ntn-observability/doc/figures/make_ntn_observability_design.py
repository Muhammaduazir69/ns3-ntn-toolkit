#!/usr/bin/env python3
"""System design figure for the ntn-observability module.

Measured sources on the left, the recording core in the middle, the sinks and
the external tools (InfluxDB, Grafana, NetSimulyzer, Cesium) on the right, and
the schema, manifest and dashboard gate underneath.
House style: Deng et al., IEEE OJ-COMS 2026.

    python3 make_ntn_observability_design.py   # writes ntn_observability_design.png/.svg here
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

# ---- house palette (Deng et al. 2026, sampled) ---------------------------
SMO, NONRT, NEARRT, RT, NF = "#3a71ae", "#75bd42", "#ee822f", "#3d7199", "#91ace0"
DU_PALE, DU_EDGE, CREAM, PALE = "#b6c7ea", "#8fa9df", "#fef1c7", "#dae3f2"
GREEN_PALE, PANEL, NAVY, INK = "#acd78e", "#808080", "#2e54a1", "#1a1a1a"
L_ORAN, L_NEW, L_FH, FLOW, ATTACH = "#35b44a", "#e54c5e", "#4771c3", "#a6b4cc", "#ee822f"
ROLES = {
    "smo": (SMO, SMO, "white"), "nonrt": (NONRT, NONRT, "white"), "nearrt": (NEARRT, NEARRT, "white"),
    "rt": (RT, RT, "white"), "nf": (NF, NF, INK), "du": (DU_PALE, DU_EDGE, INK),
    "card": (CREAM, "#f3d98a", INK), "pale": (PALE, "#b9c8e3", INK), "green": (GREEN_PALE, "#8cc56a", INK),
}
LINES = {
    "a1": (L_NEW, "-", 1.6, "-|>"), "e2": (L_ORAN, "-", 1.6, "<|-|>"), "fh": (L_FH, "-", 1.8, "<|-|>"),
    "flow": (FLOW, "-", 4.0, "-|>"), "ctrl": (INK, (0, (4, 2)), 1.2, "-|>"), "attach": (ATTACH, "-", 1.4, "<|-|>"),
}


def setup():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Liberation Sans", "Nimbus Sans", "DejaVu Sans"],
        "font.weight": "bold", "svg.fonttype": "none", "pdf.fonttype": 42,
    })


def box(ax, x, y, w, h, title, role, sub=None, dashed=False, fs=10.0, sfs=8.2):
    fc, ec, tc = ROLES[role]
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=%g" % (0.10 * min(w, h)),
                                fc=fc, ec=NAVY if dashed else ec, lw=1.4 if dashed else 0.9,
                                ls=(0, (4, 2)) if dashed else "-", zorder=5))
    ty = y + h - 0.24 if sub else y + h / 2
    ax.text(x + w / 2, ty, title, ha="center", va="center", color=tc, fontsize=fs, weight="bold", zorder=6)
    if sub:
        ax.text(x + w / 2, (y + ty - 0.16) / 2, sub, ha="center", va="center", color=tc, fontsize=sfs,
                weight="normal", zorder=6, linespacing=1.25)


def panel(ax, x, y, w, h, title):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="square,pad=0", fc="white", ec=PANEL, lw=0.9, zorder=1))
    ax.text(x + 0.10, y + h - 0.08, title, ha="left", va="top", fontsize=10.5, weight="bold", color=INK, zorder=2)


def route(ax, pts, kind, label=None, at=0, dx=0.06, dy=0.0, ha="left", va="center", fs=8.6):
    col, ls, lw, arr = LINES[kind]
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=lw, ls=ls, zorder=3, solid_capstyle="butt")
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle=arr, mutation_scale=11, color=col, lw=lw, ls=ls,
                                 shrinkA=0, shrinkB=0, zorder=3))
    if label:
        a, b = pts[at], pts[at + 1]
        ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, label, fontsize=fs, weight="bold", color=INK,
                ha=ha, va=va, zorder=7)


def legend(ax, y, items):
    x = 0.25
    for kind, text, width in items:
        col, ls, lw, _ = LINES[kind]
        ax.plot([x, x + 0.42], [y, y], color=col, lw=lw + (0.3 if kind != "flow" else 0), ls=ls,
                solid_capstyle="butt")
        ax.text(x + 0.52, y, text, fontsize=9, weight="bold", va="center", ha="left", color=INK)
        x += 0.52 + width


def main():
    setup()
    LINES["e2"] = (L_ORAN, "-", 1.6, "-|>")
    LINES["fh"] = (L_FH, "-", 1.8, "-|>")
    W, H = 12.0, 7.0
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    legend(ax, 6.75, [("flow", "Measured KPIs, positions, events", 3.0), ("e2", "Export stream", 1.45),
                      ("fh", "External ingest / playback", 2.35), ("ctrl", "Checked by a gate", 1.7)])

    ty = [5.05, 3.95, 2.85, 1.75]
    h = 0.85
    panel(ax, 0.15, 1.55, 3.30, 4.95, "Measured sources")
    panel(ax, 3.65, 1.55, 3.30, 4.95, "Recording core")
    panel(ax, 7.15, 1.55, 4.70, 4.95, "Sinks and tools")
    xs, xc, xk, xt = 0.35, 3.85, 7.35, 9.80
    bw, kw, tw = 2.90, 2.05, 1.85

    box(ax, xs, ty[0], bw, h, "NR NTN radio", "pale", "PHY SINR, TBLER, sink goodput", fs=9.6)
    box(ax, xs, ty[1], bw, h, "SGP4 geometry", "green", "positions, elevation, slant", fs=9.6)
    box(ax, xs, ty[2], bw, h, "Control events", "pale", "handover, TA, SIB19, DRX", fs=9.6)
    box(ax, xs, ty[3], bw, h, "NtnOranAiFlowMonitor", "pale", "per-flow KPM, TS 28.552", fs=9.6)

    box(ax, xc, ty[0], bw, h, "NtnObservabilityHelper", "rt", "run-id tags, sink setup", fs=9.6)
    box(ax, xc, ty[2], bw, ty[1] + h - ty[2], "NtnSceneRecorder", "rt",
        "one tap for positions, KPIs\nand events; ECEF or local ENU\nconverted to ECEF", fs=9.6)
    box(ax, xc, ty[3], bw, h, "EnableAiFlowMonitor", "rt", "auto-export at end of run", fs=9.6)

    box(ax, xk, ty[0], kw, h, "NtnInfluxSink", "du", "line protocol, file\nor UDP, bounded buffer", fs=9.4, sfs=7.8)
    box(ax, xk, ty[1], kw, h, "NetSimulyzer JSON", "du", "1.0 schema", fs=9.4, sfs=7.8)
    box(ax, xk, ty[2], kw, h, "CZML", "du", "FIXED (ECEF) frame", fs=9.4, sfs=7.8)
    box(ax, xk, ty[3], kw, h, "kpm_series", "du", ".csv and .lp", fs=9.4, sfs=7.8)

    box(ax, xt, ty[0], tw, h, "InfluxDB 2.7", "nf", "+ Grafana 10.4; also\ningests .lp files", fs=9.4, sfs=7.8)
    box(ax, xt, ty[1], tw, h, "NetSimulyzer", "nf", "3D desktop playback", fs=9.4, sfs=7.8)
    box(ax, xt, ty[2], tw, h, "Cesium globe", "nf", "time-dynamic scene", fs=9.4, sfs=7.8)
    box(ax, xt, ty[3], tw, h, "Analysis", "nf", "pandas, plots", fs=9.4, sfs=7.8)

    def mid(i):
        return ty[i] + h / 2

    route(ax, [(xs + bw, mid(0)), (xc, mid(0))], "flow")
    route(ax, [(xs + bw, mid(1)), (xc, mid(1))], "flow")
    route(ax, [(xs + bw, mid(2)), (xc, mid(2))], "flow")
    route(ax, [(xs + bw, mid(3)), (xc, mid(3))], "flow")
    route(ax, [(xc + bw, mid(0)), (xk, mid(0))], "e2")
    route(ax, [(xc + bw, mid(1)), (xk, mid(1))], "e2")
    route(ax, [(xc + bw, mid(2)), (xk, mid(2))], "e2")
    route(ax, [(xc + bw, mid(3)), (xk, mid(3))], "e2")
    for i in range(4):
        route(ax, [(xk + kw, mid(i)), (xt, mid(i))], "fh")

    panel(ax, 0.15, 0.20, 11.70, 1.15, "Contracts that keep the exports honest")
    box(ax, 0.35, 0.33, 3.70, 0.70, "ntn-metric-schema.h", "card", "every measurement, tag and field named once",
        fs=9.4, sfs=7.8)
    box(ax, 4.20, 0.33, 3.70, 0.70, "NtnReproManifest", "card", "git SHA, ns-3 version, seed, argv as JSON",
        fs=9.4, sfs=7.8)
    box(ax, 8.05, 0.33, 3.60, 0.70, "check_dashboard_producers.py", "pale",
        "each Grafana panel traced to its producer", dashed=True, fs=9.4, sfs=7.8)
    route(ax, [(11.65, 0.68), (11.76, 0.68), (11.76, ty[0] + 0.30), (xt + tw, ty[0] + 0.30)], "ctrl")

    fig.text(0.995, 0.004, "Muhammad Uzair, Independent Researcher", ha="right", va="bottom", fontsize=7,
             weight="normal", color="#666666")
    here = os.path.dirname(os.path.abspath(__file__))
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(here, "ntn_observability_design." + ext), dpi=200)
    print("wrote ntn_observability_design.png/.svg")


if __name__ == "__main__":
    main()
