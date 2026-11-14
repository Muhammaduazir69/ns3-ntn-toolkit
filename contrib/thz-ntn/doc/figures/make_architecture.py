#!/usr/bin/env python3
"""System design figure of the thz-ntn module (ITU-R loss chain, antennas, ns-3 plug-ins).

House style after Deng et al., IEEE OJ-COMS 2026. Writes thz-ntn-architecture.png (200 dpi) and .svg.

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

    legend(ax, 0.25, 6.96, [("flow", "Loss terms and their consumers"), ("xmod", "Closed loop with oran-ntn (RIC-controlled RIS)")])
    box(ax, 0.2, 6.08, 10.6, 0.62, "ThzNtnHelper", role="smo",
        sub="builds the composite channel, ISL channels and mobility; SGP4 Walker geometry from ntn-constellation", fs=10)

    def col(x0, w, title, items):
        panel(ax, x0, 2.25, w, 3.6, title)
        ys = [4.80, 4.02, 3.24, 2.46]
        for (t, r, sub), y in zip(items, ys):
            box(ax, x0 + 0.15, y, w - 0.30, 0.66, t, role=r, sub=sub, fs=8.6)

    col(0.2, 3.55, "Atmosphere and weather (ITU-R)", [
        ("ThzNtnMolecularAbsorption", "nearrt", "P.676-13 Annex 1, 44 O2 + 35 H2O lines\nslant path over P.835-6 layers"),
        ("HitranLut", "du", "tabulated P.676-13 kernel (legacy file name)\ndata/hitran2024-lut-subthz.csv"),
        ("ThzNtnWeatherAttenuation", "nf", "rain P.838-3, cloud and fog P.840, snow"),
        ("ThzNtnScintillation", "nf", "P.618-13 amplitude and phase, AR(1)"),
    ])
    col(4.0, 3.5, "Antennas, beams and surfaces", [
        ("ThzNtnAntennaArray / Beamforming", "rt", "UPA, UCA, Cassegrain; DFT codebook\nwideband beam squint"),
        ("ThzNtnBeamTracking", "rt", "EKF or ephemeris-based pointing\nThzNtnPointingError"),
        ("ThzNtnRis, ThzNtnRisXapp", "rt", "N^2 gain, phase quantization\nservice model for O-RAN control"),
        ("Fading and RF chain", "du", "ThzNtnAlphaMuFading\nThzNtnHardwareImpairments"),
    ])
    col(7.75, 3.05, "Sensing and links", [
        ("ThzNtnIsac", "nonrt", "debris ranging, CRLB\nThzNtnIsacScheduler"),
        ("ThzNtnIslChannel", "nonrt", "vacuum ISL budget\nseparation, Doppler"),
        ("ThzNtnLinkBudget", "du", "SNR, Shannon capacity\nMCS selection (PHY/MAC)"),
        ("ThzNtnFreeSpaceLoss", "du", "FSPL, frequency and\ndistance scaling"),
    ])

    panel(ax, 0.2, 0.20, 10.6, 1.65, "Into ns-3")
    box(ax, 0.35, 0.32, 2.5, 0.85, "ThzNtnChannelModel", role="pale", sub="composite cascade of all\nterms, analytic examples", fs=8.8)
    box(ax, 3.05, 0.32, 2.6, 0.85, "ThzNtnPropagationLossModel", role="pale", sub="chained on the mmwave NR\nchannel (carrier capped at 100 GHz)", fs=8.5)
    box(ax, 5.85, 0.32, 2.45, 0.85, "ThzNtnLinkErrorModel", role="pale", sub="asks the loss chain per packet\nP2P traffic at 300 GHz", fs=8.6)
    box(ax, 8.5, 0.32, 2.15, 0.85, "Outputs", role="pale", sub="CSV sweeps, sim_health.csv\nKPM flow series", fs=8.6)

    line(ax, [(1.60, 2.25), (1.60, 1.17)], "flow")
    line(ax, [(3.40, 2.25), (3.40, 1.17)], "flow")
    line(ax, [(5.00, 2.25), (5.00, 1.17)], "flow")
    line(ax, [(9.58, 2.25), (9.58, 1.17)], "flow")
    line(ax, [(5.65, 0.62), (5.85, 0.62)], "flow")
    line(ax, [(7.35, 3.57), (7.62, 3.57), (7.62, 1.30), (5.45, 1.30), (5.45, 1.17)], "xmod")
    ax.text(6.55, 1.33, "E2 KPM, RIS on/off", fontsize=7.4, weight="bold", color=INK, ha="center", va="bottom")
    footer(ax, W)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "thz-ntn-architecture." + ext), dpi=200, facecolor="white")


if __name__ == "__main__":
    main()
