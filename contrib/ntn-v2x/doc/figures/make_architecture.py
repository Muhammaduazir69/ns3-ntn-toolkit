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
    c1, c2, bw = 3.40, 6.75, 2.95
    r1, r2, r3 = 4.45, 2.95, 1.45

    panel(ax, 0.15, 0.95, 2.75, 5.55, "Inputs")
    lx, lw = 0.30, 2.45
    box(ax, lx, 5.45, lw, 0.72, "Element sets", "io", sub="data/starlink_dtc_20261002.tle", fs=8.8)
    box(ax, lx, r1, lw, bh, "Vehicle traces", "io", sub="FCD CSV (traces/), or\nSUMO export as CSV")
    box(ax, lx, r3, lw, bh, "Street canyon", "io", sub="facade heights: lognormal\nlaw or NYC footprints")

    panel(ax, 3.15, 0.95, 6.75, 5.55, "ntn-v2x  (namespace ns3::ntnv2x)")
    box(ax, c1, r1, bw, bh, "SumoTraciBridge", "mobility", sub="offline FCD replay,\nposition and velocity")
    box(ax, c2, r1, bw, bh, "NtnV2xBsmHeader", "model", sub="SAE J2735 BSMcoreData,\n21-octet UPER core")
    box(ax, c1, r2, bw, bh, "NtnSlUeMac + NtnSlResourcePool", "engine", sub="NR PC5 Mode 2: sensing,\nselection window, reservation", fs=8.4)
    box(ax, c2, r2, bw, bh, "NtnSlChannel", "channel", sub="TR 37.885 highway LOS, NLOSv,\nhalf duplex, SINR decode")
    box(ax, c1, r3, bw, bh, "V2xLeoRelay", "engine", sub="direct or relay per vehicle,\nmin SNR, max V2V range")
    box(ax, c2, r3, bw, bh, "V2xLeoDirect", "model", sub="closed-form FSPL budget,\nreference only", dashed=True)

    panel(ax, 10.15, 0.95, 2.10, 5.55, "Satellite leg")
    rx, rw = 10.25, 1.90
    box(ax, rx, r1, rw, bh, "Sgp4Mobility", "sib", sub="ntn-constellation")
    box(ax, rx, r2, rw, bh, "Uplink", "sib", sub="NR NTN cell or\nP2P, range delay")
    box(ax, rx, r3, rw, bh, "Server / Sink", "sib", sub="NtnOranSink,\nAoI, delay, PDR")

    box(ax, 3.15, 0.28, 6.75, 0.55, "NtnSlChannel reports PRR against distance (TS 38.885 method)", "accent", fs=8.8)
    box(ax, 10.15, 0.28, 2.10, 0.55, "Uplink KPIs", "accent", fs=8.8)

    # inputs
    link(ax, [(lx + lw, 5.78), (rx + rw / 2, 5.78), (rx + rw / 2, r1 + bh)], "geo", label="SGP4 orbits", at=0,
         dx=1.2, dy=0.04)
    link(ax, [(lx + lw, r1 + bh / 2), (c1, r1 + bh / 2)], "geo")
    link(ax, [(lx + lw, r3 + bh / 2), (c1, r3 + bh / 2)], "geo", label="LOS", dx=-0.25, dy=0.07, fs=7.2)
    # module
    link(ax, [(c1 + bw, r1 + bh / 2), (c2, r1 + bh / 2)], "geo")
    link(ax, [(c2 + bw / 2, r1), (c2 + bw / 2, 4.05), (c1 + bw / 2, 4.05), (c1 + bw / 2, r2 + bh)], "pkt",
         label="BSM every period", at=1, dx=-0.62, dy=0.04, fs=7.2)
    link(ax, [(c1 + bw, r2 + bh / 2), (c2, r2 + bh / 2)], "pkt", both=True)
    link(ax, [(c1 + bw / 2, r2), (c1 + bw / 2, r3 + bh)], "ctl", label="decoded announcements", dx=0.08, dy=-0.06,
         fs=7.2, va="center")
    link(ax, [(c1 + bw - 0.45, r3), (c1 + bw - 0.45, 1.13), (9.98, 1.13), (9.98, r2 + 0.30), (rx, r2 + 0.30)],
         "ctl", label="relay or direct", at=1, dx=0.6, dy=0.03, fs=7.2)
    # satellite leg
    link(ax, [(rx + rw / 2, r1), (rx + rw / 2, r2 + bh)], "geo")
    link(ax, [(rx + rw / 2, r2), (rx + rw / 2, r3 + bh)], "pkt")
    link(ax, [(rx + rw / 2, r3), (rx + rw / 2, 0.83)], "kpi")
    footer(ax, W)
    save(fig, "ntn_v2x_architecture")


if __name__ == "__main__":
    main()
