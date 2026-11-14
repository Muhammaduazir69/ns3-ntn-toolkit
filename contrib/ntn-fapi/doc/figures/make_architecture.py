#!/usr/bin/env python3
"""System design figure of the ntn-fapi module (SAP decorators, FAPI messages, outputs).

House style after Deng et al., IEEE OJ-COMS 2026. Writes ntn-fapi-architecture.png (200 dpi) and .svg.

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
    W, H = 11.0, 6.9
    fig = plt.figure(figsize=(W, H))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")
    fig.patch.set_facecolor("white")

    legend(ax, 0.25, 6.66, [("bcast", "P7 request, MAC to PHY"), ("ctrl", "P7 indication, PHY to MAC"),
                            ("flow", "Recorded FAPI struct"), ("xmod", "Provenance probe")])
    box(ax, 0.2, 5.78, 10.6, 0.62, "NtnRealStackHelper (ntn-traffic)", role="smo",
        sub="mmwave NR NTN cell over an SGP4 pass (ntn-constellation), TR 38.811 UEs (ntn-cho), EPC, UDP flows", fs=10)

    panel(ax, 0.2, 1.95, 2.7, 3.6, "MAC (mmwave, in tree)")
    box(ax, 0.35, 3.95, 2.4, 0.95, "MmWaveEnbMac", role="nf", sub="scheduler, DCI\nHARQ processes")
    box(ax, 0.35, 2.45, 2.4, 0.95, "Slot allocation", role="du", sub="SetSlotAllocInfo()\nreal DL and UL grants")

    panel(ax, 3.2, 1.95, 4.6, 3.6, "NtnFapiSapBridge (ntn-fapi)")
    box(ax, 3.35, 4.25, 4.3, 0.85, "FapiPhySapProviderDecorator", role="nearrt",
        sub="MAC to PHY, forwarded unchanged; records\nDL_TTI.request, UL_TTI.request, TX_DATA.request", fs=8.8)
    box(ax, 3.35, 3.22, 4.3, 0.85, "FapiPhySapUserDecorator", role="nonrt",
        sub="PHY to MAC, forwarded unchanged; records\nSLOT.indication from SlotIndication(SfnSf)", fs=8.8)
    box(ax, 3.35, 2.45, 2.05, 0.62, "OnPhyRx()", role="rt", sub="RX_DATA, CRC.indication", fs=8.6)
    box(ax, 5.60, 2.45, 2.05, 0.62, "HARQ state", role="du", sub="key: SFN, slot, RNTI", fs=8.6)

    panel(ax, 8.1, 1.95, 2.7, 3.6, "PHY (mmwave, in tree)")
    box(ax, 8.25, 3.95, 2.4, 0.95, "MmWaveEnbPhy", role="nf", sub="slot cadence\nTB transmission")
    box(ax, 8.25, 2.45, 2.4, 0.95, "RxPacketTraceUe", role="du", sub="measured SINR, TB\ndecode (m_corrupt)")

    panel(ax, 0.2, 0.20, 10.6, 1.45, "Struct library, outputs and checks")
    box(ax, 0.35, 0.32, 2.4, 0.85, "Struct library", role="pale", sub="fapi-common.h, fapi-messages.h\nfapi-pdu-types.h, fapi-helpers.h", fs=8.6)
    box(ax, 3.00, 0.32, 2.4, 0.85, "fapi_sap.csv", role="pale", sub="message counts, SAP latency\nDL_TTI.request to CRC.indication", fs=8.6)
    box(ax, 5.65, 0.32, 2.4, 0.85, "ProbeAirPropagation()", role="card", sub="labels latency\nmeasured-no-air-propagation", fs=8.6)
    box(ax, 8.30, 0.32, 2.35, 0.85, "SetNtnTimingAdvance()", role="card", sub="RACH.indication carries\nthe TA residual (T_c units)", fs=8.6)

    # MAC -> PHY through the provider decorator
    line(ax, [(2.75, 4.65), (3.35, 4.65)], "bcast")
    line(ax, [(7.65, 4.65), (8.25, 4.65)], "bcast")
    line(ax, [(1.55, 3.95), (1.55, 3.40)], "flow")
    # PHY -> MAC through the user decorator
    line(ax, [(8.25, 4.15), (7.95, 4.15), (7.95, 3.65), (7.65, 3.65)], "ctrl")
    line(ax, [(3.35, 3.65), (3.05, 3.65), (3.05, 4.15), (2.75, 4.15)], "ctrl")
    # PHY reception trace -> OnPhyRx
    line(ax, [(9.45, 2.45), (9.45, 2.15), (4.37, 2.15), (4.37, 2.45)], "ctrl")
    ax.text(6.1, 2.20, "per-TB decode outcome", fontsize=7.8, weight="bold", color=INK, ha="center", va="bottom")
    line(ax, [(5.40, 2.76), (5.60, 2.76)], "flow")
    # to outputs
    line(ax, [(4.20, 1.95), (4.20, 1.17)], "flow")
    line(ax, [(6.85, 1.17), (6.85, 1.95)], "xmod")
    line(ax, [(9.45, 1.17), (9.45, 1.80), (7.45, 1.80), (7.45, 1.95)], "flow")
    footer(ax, W)
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(HERE, "ntn-fapi-architecture." + ext), dpi=200, facecolor="white")


if __name__ == "__main__":
    main()
