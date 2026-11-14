#!/usr/bin/env python3
# House diagram style: sans-serif bold labels, flat rounded boxes, role colors sampled from
# Deng et al., IEEE OJ-COMS 2026, orthogonal links, one legend strip across the top.
# Self-contained so the figure can be regenerated from this folder alone:
#     python3 <this file>      ->  <name>.png (200 dpi) and <name>.svg next to it
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

HERE = os.path.dirname(os.path.abspath(__file__))
INK = "#1a1a1a"
PANEL_EDGE = "#808080"
ROLES = {  # role: (fill, edge, text)
    "engine": ("#3a71ae", "#3a71ae", "white"),     # SMO blue: the module's decision engines
    "model": ("#b6c7ea", "#8fa9df", INK),          # O-DU pale blue: module model classes
    "mobility": ("#acd78e", "#8cc56a", INK),       # pale green: mobility / geometry
    "channel": ("#f5b482", "#f0934b", INK),        # pale orange: channel / radio models
    "io": ("#fef1c7", "#e3c66f", INK),             # cream: files, traces, inputs and outputs
    "ns3": ("#dae3f2", "#b9c8e3", INK),            # pale blue-grey: ns-3 core
    "sib": ("#fce6d5", "#f0934b", INK),            # pale peach: sibling toolkit modules
    "accent": ("#75bd42", "#75bd42", "white"),     # green: measured outputs
    "warn": ("#9b78b8", "#9b78b8", "white"),       # purple: helpers / facades
}
LINES = {  # kind: (color, linestyle, width)
    "geo": ("#ee822f", "-", 1.6),       # ephemeris / positions
    "pkt": ("#4771c3", "-", 1.6),       # packets on a real data plane
    "ctl": ("#e54c5e", "-", 1.6),       # decisions / control
    "kpi": ("#35b44a", "-", 1.6),       # measured KPIs / traces
    "dep": ("#7f8fb3", (0, (4, 2)), 1.2),  # build dependency / composition
}
LEGEND_TEXT = {"geo": "Ephemeris / position", "pkt": "Packets (IP, NR)", "ctl": "Decision / control",
               "kpi": "Measured KPI / trace", "dep": "Uses (sibling module)"}


def setup():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Liberation Sans", "Nimbus Sans", "DejaVu Sans"],
        "font.size": 9, "font.weight": "bold", "pdf.fonttype": 42, "svg.fonttype": "none",
    })


def canvas(w, h):
    setup()
    fig = plt.figure(figsize=(w, h))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.axis("off")
    fig.patch.set_facecolor("white")
    return fig, ax


def box(ax, x, y, w, h, title, role="model", sub=None, fs=9.0, dashed=False, z=5):
    fc, ec, tc = ROLES[role]
    r = 0.10 * min(w, h)
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=%g" % r, fc=fc,
                                ec="#2e54a1" if dashed else ec, lw=1.3 if dashed else 0.9,
                                ls=(0, (4, 2)) if dashed else "-", zorder=z))
    if sub:
        ax.text(x + w / 2, y + h - 0.05, title, ha="center", va="top", color=tc, fontsize=fs, weight="bold",
                zorder=z + 1)
        ax.text(x + w / 2, y + (h - 0.20) / 2 + 0.05, sub, ha="center", va="center", color=tc, fontsize=fs - 1.6,
                weight="normal", zorder=z + 1, linespacing=1.2)
    else:
        ax.text(x + w / 2, y + h / 2, title, ha="center", va="center", color=tc, fontsize=fs, weight="bold",
                zorder=z + 1, linespacing=1.15)


def panel(ax, x, y, w, h, title, fs=9.6, fill="white"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.06", fc=fill, ec=PANEL_EDGE,
                                lw=0.9, zorder=1))
    ax.text(x + 0.10, y + h - 0.08, title, ha="left", va="top", fontsize=fs, weight="bold", color=INK, zorder=2)


def link(ax, pts, kind="pkt", label=None, at=0, dx=0.05, dy=0.04, ha="left", va="bottom", fs=7.6, both=False,
         head=True):
    col, ls, lw = LINES[kind]
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=lw, ls=ls, zorder=3, solid_capstyle="butt")
    style = ("<|-|>" if both else "-|>") if head else "-"
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle=style, mutation_scale=11, color=col, lw=lw, ls=ls,
                                 shrinkA=0, shrinkB=0, zorder=3))
    if label:
        a, b = pts[at], pts[at + 1]
        ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, label, fontsize=fs, weight="bold", color=INK,
                ha=ha, va=va, zorder=8,
                bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.9))


def legend(ax, x, y, kinds, fs=8.4, gap=0.32):
    from matplotlib.textpath import TextPath
    from matplotlib.font_manager import FontProperties
    fp = FontProperties(family="sans-serif", weight="bold")
    for k in kinds:
        col, ls, lw = LINES[k]
        ax.plot([x, x + 0.42], [y, y], color=col, lw=lw + 0.6, ls=ls)
        t = LEGEND_TEXT[k]
        ax.text(x + 0.50, y, t, fontsize=fs, weight="bold", va="center", ha="left", color=INK)
        wid = TextPath((0, 0), t, size=fs, prop=fp).get_extents().width / 72.0
        x += 0.50 + wid + gap
    return x


def footer(ax, w):
    ax.text(w - 0.08, 0.06, "Muhammad Uzair, Independent Researcher", ha="right", va="bottom", fontsize=6.8,
            weight="normal", color="#666666")


def save(fig, name):
    for ext, kw in (("png", {"dpi": 200}), ("svg", {})):
        fig.savefig(os.path.join(HERE, name + "." + ext), facecolor="white", **kw)
    plt.close(fig)
    print("wrote", name + ".png/.svg")


def main():
    W, H = 12.4, 7.0
    fig, ax = canvas(W, H)
    legend(ax, 0.25, 6.78, ["geo", "ctl", "pkt", "kpi"])
    bh = 0.85

    # mobility layers
    panel(ax, 0.15, 0.95, 2.75, 5.55, "Layers and mobility")
    lx, lw = 0.30, 2.45
    box(ax, lx, 5.15, lw, 0.80, "Space: Sgp4MobilityModel", "sib", sub="ntn-constellation, TLE / Walker", fs=8.4)
    box(ax, lx, 4.20, lw, 0.80, "Air: HAPS", "mobility", sub="figure-8 station keeping,\nloiter trajectory", fs=8.6)
    box(ax, lx, 3.25, lw, 0.80, "Air: UAV, aircraft", "mobility", sub="waypoint / patrol / search,\ncruise, OpenSky ADS-B", fs=8.6)
    box(ax, lx, 2.30, lw, 0.80, "Surface: AIS, HST", "mobility", sub="recorded vessel tracks,\nhigh-speed train", fs=8.6)
    box(ax, lx, 1.35, lw, 0.80, "Ground sites", "io", sub="buoys, gateways (data/*.csv),\nvillages, ground UEs", fs=8.6)

    # module
    panel(ax, 3.15, 0.95, 6.75, 5.55, "ntn-sagin  (namespace ns3::ntnsagin)")
    c1, c2, bw = 3.40, 6.75, 2.95
    r1, r2, r3 = 4.45, 2.95, 1.45
    box(ax, c1, r1, bw, bh, "A2gChannelTr36777", "channel", sub="TR 36.777 UMa/RMa/UMi-AV,\nLOS probability, valid to 300 m")
    box(ax, c2, r1, bw, bh, "SaginA2gPropagationLossModel", "channel", sub="excess over free space,\nchained onto the NR channel", fs=8.4)
    box(ax, c1, r2, bw, bh, "MultiLayerRouter", "engine", sub="max-elevation hop choice,\noptional congestion awareness")
    box(ax, c2, r2, bw, bh, "SaginSliceRouter", "engine", sub="QFI to S-NSSAI to layer,\nlatency budget, GEO allowance")
    box(ax, c1, r3, bw, bh, "SaginCustodyQueue", "engine", sub="store and forward over contact gaps;\nFCFS, LCFS, keep-newest, lifetime")
    box(ax, c2, r3, bw, bh, "SaginHelper", "warn", sub="builds a four-layer scene\n(ground, UAV, HAPS, LEO)")

    # data plane
    panel(ax, 10.15, 0.95, 2.10, 5.55, "Data plane")
    rx, rw = 10.25, 1.90
    box(ax, rx, r1, rw, bh, "NR NTN cell", "sib", sub="ntn-traffic\nNtnRealStackHelper")
    box(ax, rx, r2, rw, bh, "P2P + IPv4", "ns3", sub="gated links,\nrange-based delay")
    box(ax, rx, r3, rw, bh, "NtnOranSink", "sib", sub="in-band delay,\njitter, loss")

    box(ax, 3.15, 0.28, 6.75, 0.55, "Positions of every layer drive channel, routing and contact state each tick", "io",
        fs=8.6)
    box(ax, 10.15, 0.28, 2.10, 0.55, "Measured KPIs", "accent", fs=8.8)

    # positions bus from layers into module
    bus = 2.98
    for y in (5.55, 4.60, 3.65, 2.70, 1.75):
        ax.plot([lx + lw, bus], [y, y], color=LINES["geo"][0], lw=LINES["geo"][2], zorder=3)
    ax.plot([bus, bus], [1.75, 5.55], color=LINES["geo"][0], lw=LINES["geo"][2], zorder=3)
    link(ax, [(bus, r1 + bh / 2), (c1, r1 + bh / 2)], "geo")
    link(ax, [(bus, r2 + bh / 2), (c1, r2 + bh / 2)], "geo")
    link(ax, [(bus, r3 + bh / 2), (c1, r3 + bh / 2)], "geo")
    # channel chain
    link(ax, [(c1 + bw, r1 + bh / 2), (c2, r1 + bh / 2)], "geo")
    link(ax, [(c2 + bw, r1 + bh / 2), (rx, r1 + bh / 2)], "ctl", label="path loss", dx=-0.25, dy=0.08, fs=7.2)
    # routing decisions
    link(ax, [(c1 + bw, r2 + bh / 2), (c2, r2 + bh / 2)], "ctl")
    link(ax, [(c2 + bw, r2 + bh / 2), (rx, r2 + bh / 2)], "ctl", label="hop path", dx=-0.25, dy=0.08, fs=7.2)
    link(ax, [(c1 + bw / 2, r2), (c1 + bw / 2, r3 + bh)], "ctl")
    # custody to links
    link(ax, [(c1 + bw - 0.45, r3), (c1 + bw - 0.45, 1.13), (9.98, 1.13), (9.98, r2 + 0.22), (rx, r2 + 0.22)], "pkt",
         label="drained on contact", at=1, dx=-0.9, dy=0.03, fs=7.2)
    link(ax, [(rx + rw / 2, r1), (rx + rw / 2, r2 + bh)], "pkt")
    link(ax, [(rx + rw / 2, r2), (rx + rw / 2, r3 + bh)], "pkt")
    link(ax, [(rx + rw / 2, r3), (rx + rw / 2, 0.83)], "kpi")
    footer(ax, W)
    save(fig, "ntn_sagin_architecture")


if __name__ == "__main__":
    main()
