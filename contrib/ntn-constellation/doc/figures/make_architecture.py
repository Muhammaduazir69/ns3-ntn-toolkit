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

    # inputs
    panel(ax, 0.15, 0.95, 2.45, 5.55, "Inputs")
    box(ax, 0.30, 5.10, 2.15, 0.95, "Element sets", "io", sub="TLE files (data/), or\nntn-fetch: CelesTrak")
    box(ax, 0.30, 4.10, 2.15, 0.85, "Walker config", "io", sub="planes, sats/plane,\nphasing F, h, i")
    box(ax, 0.30, 1.40, 2.15, 0.85, "TR 38.821 corpus", "io", sub="Set-1/Set-2 budgets,\nStarlink latency")

    # module
    panel(ax, 2.85, 0.95, 7.10, 5.55, "ntn-constellation  (namespace ns3::ntncon)")
    c1, c2, c3, bw, bh = 3.05, 5.35, 7.65, 2.10, 0.85
    r1, r2, r3 = 4.10, 2.75, 1.40
    box(ax, c1, r1, bw, bh, "WalkerConstellation", "mobility", sub="BuildDelta / BuildStar")
    box(ax, c2, r1, bw, bh, "Sgp4MobilityModel", "mobility", sub="Kepler + J2 by default;\nVallado SGP4 opt-in")
    box(ax, c3, r1, bw, bh, "NtnSatLinkErrorModel", "channel", sub="per-packet C/N0 to BLER\n(DVB-S2 MODCOD curve)")
    box(ax, c1, r2, bw, bh, "NtnVisibilityIndex", "model", sub="exact argmax elevation\n(branch and bound)")
    box(ax, c2, r2, bw, bh, "ContactGraphScheduler", "engine", sub="GSL / ISL visibility,\nlimb clearance, events")
    box(ax, c3, r2, bw, bh, "ContactGraphRouter", "engine", sub="BFS, Dijkstra, lifetime-\naware, regen-only filter")
    box(ax, c1, r3, bw, bh, "CalibrationHarness", "model", sub="Tr38821CorpusReader,\nresiduals vs reference")
    box(ax, c2, r3, bw, bh, "NtnXnHandover", "model", sub="XnAP sequence over the\nISL (TS 38.300 9.2.3)")

    # ns-3 side
    panel(ax, 10.20, 0.95, 2.05, 5.55, "ns-3 data plane")
    rx, rw = 10.30, 1.85
    box(ax, rx, r1, rw, bh, "P2P GSL / ISL", "ns3", sub="delay = range / c")
    box(ax, rx, r2, rw, bh, "Ipv4StaticRouting", "ns3", sub="host routes per path")
    box(ax, rx, r3, rw, bh, "NtnOranApp / Sink", "sib", sub="ntn-traffic: in-band\nseq + timestamp")

    # bottom strip
    box(ax, 2.85, 0.28, 7.10, 0.55, "Reused by ntn-cho, ntn-sagin, ntn-v2x, oran-ntn, ntn-digital-twin, ntn-rrc", "sib",
        fs=8.6)
    box(ax, 10.20, 0.28, 2.05, 0.55, "sim_health.csv", "accent", fs=8.8)

    ym1 = r1 + bh / 2
    # element sets -> Sgp4 (over the top of the Walker box)
    link(ax, [(2.45, 5.55), (6.40, 5.55), (6.40, r1 + bh)], "geo", label="SetTle()", at=0, dx=0.6, dy=0.04)
    link(ax, [(2.45, ym1 - 0.05), (c1, ym1 - 0.05)], "geo")
    link(ax, [(c1 + bw, ym1), (c2, ym1)], "geo")
    link(ax, [(c2 + bw, ym1), (c3, ym1)], "geo")
    link(ax, [(c3 + bw, ym1), (rx, ym1)], "ctl")
    # positions down
    link(ax, [(6.40, r1), (6.40, r2 + bh)], "geo", label="positions", dx=0.06, dy=-0.08, fs=7.2, va="center")
    link(ax, [(5.75, r1), (5.75, 3.80), (4.10, 3.80), (4.10, r2 + bh)], "geo")
    ym2 = r2 + bh / 2
    link(ax, [(c2 + bw, ym2), (c3, ym2)], "ctl")
    link(ax, [(c3 + bw, ym2), (rx, ym2)], "ctl")
    link(ax, [(2.45, r3 + bh / 2), (c1, r3 + bh / 2)], "geo")
    # data plane vertical
    link(ax, [(rx + rw / 2, r2 + bh), (rx + rw / 2, r1)], "pkt")
    link(ax, [(rx + rw / 2, r3 + bh), (rx + rw / 2, r2)], "pkt", both=True)
    link(ax, [(rx + rw / 2, r3), (rx + rw / 2, 0.83)], "kpi")
    footer(ax, W)
    save(fig, "ntn_constellation_architecture")


if __name__ == "__main__":
    main()
