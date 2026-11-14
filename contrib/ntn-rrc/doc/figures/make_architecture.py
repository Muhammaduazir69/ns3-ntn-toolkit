#!/usr/bin/env python3
"""System design figure of the ntn-rrc module (classes, data flow, sibling modules).

House style: sans-bold rounded role boxes, sampled palette and a legend strip, after
Deng et al., IEEE OJ-COMS 2026. Writes ntn-rrc-architecture.png (200 dpi) and .svg
next to this script.

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

    legend(ax, 0.25, 6.96, [("flow", "Geometry and state"), ("bcast", "SIB19 broadcast (124-byte frame)"),
                            ("ctrl", "Measured feedback / gating"), ("xmod", "Cross-module hook")])

    box(ax, 0.2, 6.08, 10.6, 0.62, "NtnRrcHelper", role="smo",
        sub="SetPayloadMode, SetReferencePosition, InstallTimingAdvance, InstallSib19Broadcaster, "
            "InstallUeLocationReporter, InstallDrx", fs=10)

    # inputs
    panel(ax, 0.2, 2.25, 2.6, 3.6, "Orbit and terminal inputs")
    box(ax, 0.35, 4.45, 2.3, 0.85, "Satellite orbit", role="green", sub="Sgp4MobilityModel (Walker)\nor SNS3 TLE, data/iss-zarya.tle")
    box(ax, 0.35, 3.40, 2.3, 0.80, "UE mobility", role="green", sub="TR 38.811 classes (ntn-cho)\nGNSS fix every T_g")
    box(ax, 0.35, 2.40, 2.3, 0.80, "Gateway mobility", role="green", sub="feeder leg in the timing\nloop (transparent payload)")

    # network side
    panel(ax, 3.1, 2.25, 4.15, 3.6, "Network side (gNB)")
    box(ax, 3.25, 4.45, 1.75, 0.85, "NtnTimingAdvance", role="nearrt", sub="TA = 2(d_s + d_f)/c\ncommon + UE-specific, drift", fs=8.4)
    box(ax, 5.35, 4.45, 1.75, 0.85, "NtnSib19Broadcaster", role="nonrt", sub="T_C, T_D, T_V fit\nvalidity rule, K_offset", fs=8.4)
    box(ax, 5.35, 3.40, 1.75, 0.80, "Sib19Codec", role="du", sub="TS 38.331 field steps\nserialize / parse")
    box(ax, 3.25, 3.40, 1.75, 0.80, "NtnRachWindow", role="du", sub="RAR window the\nround trip demands")
    box(ax, 3.25, 2.40, 1.75, 0.80, "NtnMeasReportCodec", role="du", sub="TS 38.133 level\nindices, saturating", fs=8.2)
    box(ax, 5.35, 2.40, 1.75, 0.80, "Coefficient fits", role="card", sub="Taylor | economized | window\nslot | budget | fixed", fs=8.2)

    # UE side
    panel(ax, 7.55, 2.25, 3.25, 3.6, "UE side")
    box(ax, 7.7, 4.45, 2.95, 0.85, "NtnUlTimingPrecompensator", role="rt",
        sub="decode SIB19, propagate orbit (linear,\ngravity, Kepler, Kepler-J2), form TA(t)", fs=8.6)
    box(ax, 7.7, 3.40, 2.95, 0.80, "NtnDrxStateMachine", role="rt", sub="Active, OnDuration, Short/LongSleep\n+ AwaitingPass between passes")
    box(ax, 7.7, 2.40, 2.95, 0.80, "NtnUeLocationReporter", role="rt", sub="periodic / event / on demand\nECEF to WGS-84 (closed form)")

    # radio stack
    panel(ax, 0.2, 0.20, 10.6, 1.65, "Radio stack: NtnRealStackHelper (ntn-traffic), NR cell and EPC")
    box(ax, 0.35, 0.32, 2.3, 0.85, "CSV outputs", role="pale", sub="*-ta.csv, *-sib19.csv\ndrx_metrics.csv")
    box(ax, 3.25, 0.32, 1.75, 0.85, "PHY trace", role="pale", sub="measured DL SINR,\nTBLER")
    box(ax, 5.35, 0.32, 1.75, 0.85, "UL scheduler", role="pale", sub="K_offset consumed via\nSetKOffsetConsumption()", fs=8.6)
    box(ax, 7.7, 0.32, 2.95, 0.85, "Traffic and sinks", role="pale", sub="real UDP over the cell\nsim_health.csv")

    # flows
    line(ax, [(2.65, 4.88), (3.25, 4.88)], "flow")
    line(ax, [(2.65, 3.80), (2.95, 3.80), (2.95, 4.60), (3.25, 4.60)], "flow")
    line(ax, [(2.65, 2.80), (2.85, 2.80), (2.85, 4.72), (3.25, 4.72)], "flow")
    line(ax, [(5.00, 4.88), (5.35, 4.88)], "flow")
    line(ax, [(4.12, 4.45), (4.12, 4.20)], "flow")
    line(ax, [(6.40, 4.45), (6.40, 4.20)], "flow")
    line(ax, [(6.22, 3.20), (6.22, 3.40)], "flow")
    line(ax, [(7.10, 3.80), (7.33, 3.80), (7.33, 4.88), (7.70, 4.88)], "bcast")
    ax.text(7.36, 4.30, "SIB19", fontsize=7.8, weight="bold", color=INK, ha="left", va="center", rotation=90, zorder=7)
    # cross-module and feedback
    line(ax, [(5.70, 4.45), (5.70, 4.33), (5.17, 4.33), (5.17, 2.05), (5.75, 2.05), (5.75, 1.17)], "xmod",
         label="K_offset", at=4, dx=0.07, dy=0.0)
    line(ax, [(4.12, 1.17), (4.12, 2.40)], "ctrl", label="SINR", at=0, dx=0.07, dy=0.25)
    line(ax, [(7.70, 3.70), (7.47, 3.70), (7.47, 2.05), (8.6, 2.05), (8.6, 1.17)], "ctrl", label="DRX gates traffic",
         at=3, dx=0.08, dy=0.02)
    footer(ax, W)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn-rrc-architecture." + ext), dpi=200, facecolor="white")


if __name__ == "__main__":
    main()
