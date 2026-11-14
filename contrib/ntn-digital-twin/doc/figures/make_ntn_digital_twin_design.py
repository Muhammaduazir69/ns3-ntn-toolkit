#!/usr/bin/env python3
"""System design figure for the ntn-digital-twin module.

The twin sync loop (TLE or Walker source, SGP4 propagation, CZML and line
protocol), the FastAPI prediction service, the shared A3 guard, the
prediction file that ns-3 actuates, and the checks underneath.
House style: Deng et al., IEEE OJ-COMS 2026.

    python3 make_ntn_digital_twin_design.py   # writes ntn_digital_twin_design.png/.svg here
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
    LINES["a1"] = (L_NEW, (0, (4, 2)), 1.5, "-|>")
    W, H = 12.0, 7.1
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")
    legend(ax, 6.85, [("flow", "Ephemeris / state", 1.65), ("e2", "Prediction or export", 1.95),
                      ("ctrl", "Uses", 0.6), ("a1", "Not implemented (sim to twin)", 2.6)])

    ry, rh = [4.90, 3.65, 2.40], 0.95
    panel(ax, 0.15, 2.00, 2.75, 4.60, "Inputs")
    panel(ax, 3.10, 2.00, 4.55, 4.60, "Twin service (Python, systemd units)")
    panel(ax, 7.85, 2.00, 4.00, 4.60, "Consumers")

    box(ax, 0.35, ry[0], 2.35, rh, "CelesTrak TLEs", "green", "live feed,\n6 h on-disk cache", fs=9.6)
    box(ax, 0.35, ry[2], 2.35, rh, "Walker source", "green", "fixed elements,\nshared epoch with ns-3", fs=9.6)

    xa, xb, cw = 3.30, 5.55, 1.90
    box(ax, xa, ry[0], cw, rh, "twin_loop", "rt", "fetch, SGP4 propagate,\nemit; crash-tolerant", fs=9.6)
    box(ax, xb, ry[0], cw, rh, "FastAPI :8090", "smo", "/health, /constellation\n/state, /predict/handover",
        fs=9.6)
    box(ax, xa, ry[1], xb + cw - xa, rh, "a3_guard (one implementation)", "nearrt",
        "dB link-budget margin, hysteresis, time-to-trigger, minimum service time", fs=9.6)
    box(ax, xa, ry[2], cw, rh, "emit_predictions_file", "rt", "t_s, ueId, gnbId,\nconfidence", fs=9.2)
    box(ax, xb, ry[2], cw, rh, "CZML + line protocol", "rt", "ntn_sat_pos, the\nobservability schema",
        fs=9.2)

    xc, ccw = 8.05, 3.60
    box(ax, xc, ry[0], ccw, rh, "REST clients", "nf", "scripts, dashboards,\nany HTTP caller", fs=9.6)
    box(ax, xc, ry[1], ccw, rh, "Cesium viewer, InfluxDB, Grafana", "nf",
        "Live toggle polls /constellation/state", fs=9.4)
    box(ax, xc, ry[2], ccw, rh, "ns-3 (oran-ntn)", "du",
        "OranNtnTwinPredictionConsumer fires\nHANDOVER_TRIGGER at t_s on the real stack", fs=9.6)

    def mid(i):
        return ry[i] + rh / 2

    route(ax, [(2.70, mid(0)), (xa, mid(0))], "flow")
    route(ax, [(2.70, mid(2)), (3.00, mid(2)), (3.00, ry[0] + 0.18), (xa, ry[0] + 0.18)], "flow")
    ax.text(2.88, mid(1), "deterministic\nmode", fontsize=8, weight="normal", ha="right", va="center")
    route(ax, [(xa + 0.95, ry[0]), (xa + 0.95, ry[1] + rh)], "ctrl")
    route(ax, [(xb + 0.95, ry[0]), (xb + 0.95, ry[1] + rh)], "ctrl")
    route(ax, [(xa + 0.95, ry[1]), (xa + 0.95, ry[2] + rh)], "e2")
    route(ax, [(xa + cw, ry[0] + 0.25), (xa + cw + 0.10, ry[0] + 0.25), (xa + cw + 0.10, ry[1] + rh + 0.12),
               (xb + cw + 0.10, ry[1] + rh + 0.12), (xb + cw + 0.10, mid(2)), (xb + cw, mid(2))], "e2")
    route(ax, [(xb + cw, mid(0)), (xc, mid(0))], "e2")
    route(ax, [(xb + cw + 0.10, mid(1)), (xc, mid(1))], "e2")
    route(ax, [(xa + cw / 2, ry[2]), (xa + cw / 2, 2.12), (7.75, 2.12), (7.75, ry[2] + 0.25), (xc, ry[2] + 0.25)],
          "e2")
    ax.text(6.5, 2.16, "prediction file, 1-indexed cells", fontsize=8.2, weight="bold", ha="center", va="bottom")
    route(ax, [(xc + ccw - 0.5, ry[2]), (xc + ccw - 0.5, 1.82), (1.52, 1.82), (1.52, ry[2])], "a1")
    ax.text(6.9, 1.78, "ns-3 state is not fed back into the twin", fontsize=8.2, weight="bold", ha="center",
            va="top", color=L_NEW)

    panel(ax, 0.15, 0.20, 11.70, 1.33, "Checks")
    box(ax, 0.35, 0.30, 3.75, 0.80, "ntn-twin-agreement (C++)", "card",
        "replays the twin's TLEs with ns-3 SGP4 and\ncompares handover instants (TWIN-02)", fs=9.2, sfs=7.8)
    box(ax, 4.25, 0.30, 3.70, 0.80, "pytest, 28 tests", "card",
        "A3 guard, loop outages, API latency,\nprediction-file format", fs=9.2, sfs=7.8)
    box(ax, 8.10, 0.30, 3.55, 0.80, "Gate 9 (Python)", "pale",
        "twin vs Skyfield on the same orbits,\na geometry check, not a sim check", dashed=True, fs=9.2, sfs=7.8)

    fig.text(0.995, 0.004, "Muhammad Uzair, Independent Researcher", ha="right", va="bottom", fontsize=7,
             weight="normal", color="#666666")
    here = os.path.dirname(os.path.abspath(__file__))
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(here, "ntn_digital_twin_design." + ext), dpi=200)
    print("wrote ntn_digital_twin_design.png/.svg")


if __name__ == "__main__":
    main()
