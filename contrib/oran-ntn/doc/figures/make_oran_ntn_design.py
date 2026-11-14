#!/usr/bin/env python3
"""System design figure for the oran-ntn module.

Draws the ground segment (SMO / Non-RT RIC, Near-RT RIC, RIC placement),
the space segment (Space RIC, RT RIC, E2 node, payload options) and the
measured data plane underneath, with the A1, E2 and feeder interfaces.
House style: Deng et al., IEEE OJ-COMS 2026 (sampled palette, rounded flat
boxes, legend strip on top, orthogonal routing).

    python3 make_oran_ntn_design.py        # writes oran_ntn_design.png/.svg here
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
    W, H = 12.0, 7.6
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.axis("off")

    legend(ax, 7.33, [("a1", "A1 policy / intent", 1.65), ("e2", "E2 (KPM up, RC down)", 1.95),
                      ("fh", "Feeder link (live slant delay)", 2.55), ("flow", "Measured KPI flow", 1.75),
                      ("ctrl", "Actuation", 1.0)])

    # ---- ground segment ----------------------------------------------------
    panel(ax, 0.15, 2.95, 5.75, 4.05, "Ground segment")
    box(ax, 0.40, 5.45, 5.25, 1.10, "SMO / Non-RT RIC", "smo",
        "A1 policy manager + OSC schema registry, cross-domain SMO,\n"
        "NWDAF analytics, TPN controller (O1 is not modeled)")
    box(ax, 0.40, 3.95, 3.25, 1.10, "Near-RT RIC", "nearrt",
        "xApps, conflict manager, SDL,\nKPM / RC / CCC service models")
    box(ax, 3.95, 3.95, 1.70, 1.10, "RIC placement", "nf", "on-board, HAPS,\ngateway, cloud")
    box(ax, 0.40, 3.12, 3.25, 0.60, "Twin prediction consumer", "card", fs=9.2)
    box(ax, 3.95, 3.12, 1.70, 0.60, "FlexRIC bridge", "pale", dashed=True, fs=9.2)

    # ---- space segment -----------------------------------------------------
    panel(ax, 6.15, 2.95, 5.70, 4.05, "Space segment (SGP4 orbit)")
    box(ax, 6.40, 5.45, 2.65, 1.10, "Space RIC", "rt", "autonomy on feeder loss,\nA1 lease, then safe intent")
    box(ax, 9.30, 5.45, 2.30, 1.10, "RT RIC", "rt", "on the O-DU,\nloop < 10 ms enforced")
    box(ax, 6.40, 3.95, 2.65, 1.10, "E2 node", "du", "indications, RC actions,\nstore-and-forward buffer")
    box(ax, 9.30, 3.95, 2.30, 1.10, "Payload option", "du",
        "transparent, O-RU,\nO-RU+O-DU, full gNB", dashed=True)
    ax.text(10.45, 3.30, "role switch and FH split\nchecked on measured delay", ha="center", va="center",
            fontsize=8.2, weight="normal", color=INK)

    # ---- measured data plane ----------------------------------------------
    panel(ax, 0.15, 0.30, 11.70, 2.40, "Measured data plane: ntn-traffic NtnRealStackHelper")
    bw, by, bh = 1.95, 0.85, 1.30
    box(ax, 0.40, by, bw, bh, "Ground UEs", "green", "TR 38.811 classes\n(pedestrian, vehicle, ...)")
    box(ax, 2.73, by, bw, bh, "NR NTN cell", "pale", "SpectrumPhy, MAC, HARQ,\nRLC, PDCP, RRC, EPC")
    box(ax, 5.06, by, bw, bh, "KPM producer", "card", "TS 28.552 names,\nprovenance tag per row")
    box(ax, 7.39, by, bw, bh, "Actuators", "card", "TriggerHandover (RRC),\nbeam loss model")
    box(ax, 9.72, by, bw, bh, "Run outputs", "card", "kpm_feed, action_log,\nconflict_log, sim_health")

    # ---- interfaces ----------------------------------------------------------
    route(ax, [(2.02, 5.45), (2.02, 5.05)], "a1", "A1", dx=0.08)
    route(ax, [(5.65, 6.00), (6.40, 6.00)], "a1", "A1 + lease", dx=0.0, dy=0.17, ha="center")
    route(ax, [(3.65, 4.50), (3.95, 4.50)], "e2", "E2", dx=0.0, dy=0.20, ha="center")
    route(ax, [(5.65, 4.50), (6.40, 4.50)], "fh", "feeder", dx=0.0, dy=0.20, ha="center")
    route(ax, [(7.72, 5.45), (7.72, 5.05)], "e2", "E2 (local)", dx=0.08)
    route(ax, [(10.45, 5.45), (10.45, 5.05)], "e2", "PHY KPIs", dx=0.08)
    route(ax, [(2.02, 3.72), (2.02, 3.95)], "ctrl")
    # measured KPM up, RC action down
    route(ax, [(6.75, 2.15), (6.75, 3.95)], "flow")
    ax.text(6.65, 2.82, "KPM", ha="right", va="center", fontsize=8.6, weight="bold", color=INK, zorder=7)
    route(ax, [(8.47, 3.95), (8.47, 2.15)], "ctrl")
    ax.text(8.57, 2.82, "RC action", ha="left", va="center", fontsize=8.6, weight="bold", color=INK, zorder=7)
    route(ax, [(2.35, 1.50), (2.73, 1.50)], "attach")
    route(ax, [(4.68, 1.50), (5.06, 1.50)], "flow")
    route(ax, [(8.47, 0.85), (8.47, 0.55), (3.70, 0.55), (3.70, 0.85)], "ctrl", "handover / beam change lands on the radio",
          at=1, dx=0.0, dy=0.11, ha="center", fs=8.2)

    fig.text(0.995, 0.006, "Muhammad Uzair, Independent Researcher", ha="right", va="bottom", fontsize=7,
             weight="normal", color="#666666")
    here = os.path.dirname(os.path.abspath(__file__))
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(here, "oran_ntn_design." + ext), dpi=200)
    print("wrote oran_ntn_design.png/.svg")


if __name__ == "__main__":
    main()
