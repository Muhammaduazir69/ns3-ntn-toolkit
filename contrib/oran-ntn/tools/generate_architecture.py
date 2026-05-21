#!/usr/bin/env python3
"""Generate the oran-ntn architecture PNG (clean, IEEE academic-blue, no overlaps).

Run from the module root:
    python3 tools/generate_architecture.py

Output:
    visualization/oran_ntn_architecture.png

Style: academic-blue palette (NAVY/BLUE/GOLD/STEEL/GRAY) per the project's
figure-style spec. Layout is pure absolute (x,y) cm placement with ≥0.4-cm
clearance around every label; no `above of=` chains, no overlapping arrows.
"""
from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# -----------------------------------------------------------------------------
# Palette
# -----------------------------------------------------------------------------
NAVY     = "#0E3A6B"
BLUE     = "#2E78B5"
STEEL    = "#5B8CC9"
GOLD     = "#D9A21B"
GRAY     = "#4A4A4A"
DARK     = "#1A1A1A"

TINT     = "#E8EFF7"   # very light blue
TINTMID  = "#C7D6E8"
TINTGOLD = "#F6EBC7"
TINTGRAY = "#EAEAEA"
WHITE    = "#FFFFFF"

# -----------------------------------------------------------------------------
# Canvas
# -----------------------------------------------------------------------------
W, H = 16.0, 12.0          # data-units (1 unit ≈ 1 cm at the chosen fig size)
FIG_W, FIG_H = 13.0, 9.7   # inches → ~2340×1746 px at dpi=180

OUT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "visualization",
    "oran_ntn_architecture.png",
)


def _box(ax, x, y, w, h, *, fill=WHITE, edge=NAVY, lw=0.9, alpha=1.0):
    p = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.06,rounding_size=0.18",
        facecolor=fill, edgecolor=edge, linewidth=lw, alpha=alpha,
    )
    ax.add_patch(p)


def _title_box(ax, x, y, w, h, *, title, subtitle=None,
               fill=NAVY, edge=NAVY, title_color=WHITE, sub_color=TINTMID,
               title_fs=11, sub_fs=8.5):
    _box(ax, x, y, w, h, fill=fill, edge=edge, lw=1.0)
    if subtitle:
        ax.text(x + w / 2, y + h * 0.62, title,
                ha="center", va="center",
                fontsize=title_fs, fontweight="bold", color=title_color)
        ax.text(x + w / 2, y + h * 0.28, subtitle,
                ha="center", va="center",
                fontsize=sub_fs, color=sub_color)
    else:
        ax.text(x + w / 2, y + h / 2, title,
                ha="center", va="center",
                fontsize=title_fs, fontweight="bold", color=title_color)


def _arrow(ax, x1, y1, x2, y2, *, color=NAVY, lw=1.0, style="-|>"):
    arr = FancyArrowPatch(
        (x1, y1), (x2, y2),
        arrowstyle=style, mutation_scale=12,
        color=color, lw=lw, shrinkA=0, shrinkB=0,
    )
    ax.add_patch(arr)


def _edge_label(ax, x, y, text, *, color=NAVY, fs=8):
    ax.text(x, y, text, ha="center", va="center",
            fontsize=fs, color=color, fontstyle="italic",
            bbox=dict(boxstyle="round,pad=0.20",
                      facecolor=WHITE, edgecolor="none", alpha=0.95))


def build_figure():
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H), facecolor=WHITE)
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")

    # -- Title strip -----------------------------------------------------------
    ax.text(W / 2, H - 0.45, "oran-ntn  —  Near-RT RIC + Space RIC for NTN",
            ha="center", va="center",
            fontsize=15, fontweight="bold", color=DARK)
    ax.text(W / 2, H - 0.90,
            "13 xApps  ·  E2SM-RC (28 actions)  ·  A1 policy engine (11 policies)  ·  Space RIC autonomous mode",
            ha="center", va="center",
            fontsize=9.5, color=GRAY, style="italic")

    # =========================================================================
    # Tier 1 — Non-RT RIC / SMO   (y ≈ 9.4 – 10.3)
    # =========================================================================
    t1_y, t1_h = 9.4, 0.95
    _title_box(ax, 2.0, t1_y, W - 4.0, t1_h,
               title="Non-RT RIC  /  SMO",
               subtitle="orbit-aware A1 policy authoring   ·   rApps   ·   FL participation   ·   O1 KPI catalog",
               fill=NAVY, title_fs=12, sub_fs=8.7)

    # =========================================================================
    # Tier 2 — Near-RT RIC band   (y ≈ 5.55 – 9.05)
    # Container, then 4 platform services, then 13 xApps in 3 rows.
    # =========================================================================
    t2_y, t2_h = 5.55, 3.45
    _box(ax, 0.6, t2_y, W - 1.2, t2_h, fill=TINT, edge=BLUE, lw=1.1)
    ax.text(W / 2, t2_y + t2_h - 0.30, "Near-RT RIC",
            ha="center", va="center",
            fontsize=12, fontweight="bold", color=NAVY)

    # A1 arrow between Tier 1 and Tier 2  (clear vertical channel)
    _arrow(ax, W / 2, t1_y - 0.06, W / 2, t2_y + t2_h - 0.06,
           color=NAVY, lw=1.2)
    _edge_label(ax, W / 2 + 0.55, (t1_y + t2_y + t2_h) / 2,
                "A1", color=NAVY, fs=9)

    # ---- 4 platform services (row 1, just under the band title) -------------
    svc_y, svc_h = t2_y + t2_h - 1.10, 0.55
    svc_w = 3.05
    svc_x0 = 0.9
    services = [
        ("E2 Termination", "E2AP / SCTP"),
        ("SDL", "shared data layer"),
        ("Conflict Manager", "5 strategies"),
        ("A1 Adapter", "policy ingest"),
    ]
    for i, (lbl, sub) in enumerate(services):
        x = svc_x0 + i * (svc_w + 0.15)
        _box(ax, x, svc_y, svc_w, svc_h, fill=BLUE, edge=NAVY, lw=0.9)
        ax.text(x + svc_w / 2, svc_y + svc_h * 0.66, lbl,
                ha="center", va="center", color=WHITE,
                fontsize=9, fontweight="bold")
        ax.text(x + svc_w / 2, svc_y + svc_h * 0.26, sub,
                ha="center", va="center", color=TINTMID,
                fontsize=7.5, style="italic")

    # ---- 13 xApps in 3 rows × (5 / 5 / 3) -----------------------------------
    xapps = [
        # row 1 — RAN-side handover + steering
        ("HO Predict",        "DQN",     "ran"),
        ("Beam Hop",          "PPO",     "ran"),
        ("Slice Manager",     "MAPPO",   "ran"),
        ("Doppler Comp.",     "Kalman",  "ran"),
        ("TN-NTN Steering",   "rule",    "ran"),
        # row 2 — QoS / efficiency
        ("Interference Mgmt", "ICIC",    "qos"),
        ("Energy Harvest",    "RL",      "qos"),
        ("Predictive Alloc.", "LSTM",    "qos"),
        ("Multi-Conn.",       "DC/MC",   "qos"),
        ("ISAC",              "joint",   "qos"),
        # row 3 — THz-specific
        ("THz Beam Mgmt",     "EKF",     "thz"),
        ("THz RIS",           "phase",   "thz"),
        ("THz Spectrum",      "sensing", "thz"),
    ]
    row_fills = {"ran": STEEL, "qos": GOLD, "thz": NAVY}
    row_textcolor = {"ran": WHITE, "qos": DARK, "thz": WHITE}
    row_subcolor = {"ran": TINT, "qos": GRAY, "thz": TINTMID}

    xa_w, xa_h, xa_gap = 1.65, 0.62, 0.13
    rows = [xapps[0:5], xapps[5:10], xapps[10:13]]
    row_y = [
        svc_y - 0.80,   # row 1
        svc_y - 1.50,   # row 2
        svc_y - 2.20,   # row 3
    ]
    for r, row in enumerate(rows):
        n = len(row)
        total = n * xa_w + (n - 1) * xa_gap
        x0 = (W - total) / 2.0
        for i, (lbl, sub, kind) in enumerate(row):
            x = x0 + i * (xa_w + xa_gap)
            _box(ax, x, row_y[r], xa_w, xa_h,
                 fill=row_fills[kind], edge=NAVY, lw=0.8)
            ax.text(x + xa_w / 2, row_y[r] + xa_h * 0.62, lbl,
                    ha="center", va="center",
                    fontsize=7.6, fontweight="bold",
                    color=row_textcolor[kind])
            ax.text(x + xa_w / 2, row_y[r] + xa_h * 0.22, sub,
                    ha="center", va="center",
                    fontsize=6.4, style="italic",
                    color=row_subcolor[kind])

    # =========================================================================
    # Tier 3 — E2 nodes   (y ≈ 3.1 – 5.05)
    # =========================================================================
    t3_y, t3_h = 3.10, 1.95
    ax.text(0.95, t3_y + t3_h + 0.20, "E2 nodes",
            ha="left", va="center",
            fontsize=10, fontweight="bold", color=NAVY)

    # E2 arrows from Tier 2 down to Tier 3 (clear vertical channels, labelled)
    _arrow(ax, 3.10, t2_y + 0.05, 3.10, t3_y + t3_h + 0.04,
           color=NAVY, lw=1.0)
    _edge_label(ax, 3.55, t2_y - 0.20, "E2 KPM / RC", color=NAVY)
    _arrow(ax, W / 2, t2_y + 0.05, W / 2, t3_y + t3_h + 0.04,
           color=NAVY, lw=1.0)
    _edge_label(ax, W / 2 + 0.95, t2_y - 0.20, "E2 KPM / RC", color=NAVY)
    _arrow(ax, W - 3.10, t2_y + 0.05, W - 3.10, t3_y + t3_h + 0.04,
           color=GOLD, lw=1.0)
    _edge_label(ax, W - 3.10 + 1.15, t2_y - 0.20, "Gym hooks", color=GOLD)

    node_w = 4.55
    node_specs = [
        # (x,                    fill,  title,                 sub-1,                       sub-2)
        (0.85,            STEEL,         "LEO satellite gNB",    "DVB-S2X · ModCod",        "Markov fading"),
        ((W - node_w)/2,  BLUE,          "Terrestrial gNB",      "mmWave NR PHY",           "dual connectivity"),
        (W - 0.85 - node_w, NAVY,        "ns3-ai integration",   "5 Gym envs · LibTorch",   "shared-mem IPC"),
    ]
    for (x, c, t, s1, s2) in node_specs:
        _box(ax, x, t3_y, node_w, t3_h, fill=c, edge=NAVY, lw=1.0)
        ax.text(x + node_w / 2, t3_y + t3_h * 0.72, t,
                ha="center", va="center",
                fontsize=10.5, fontweight="bold", color=WHITE)
        ax.text(x + node_w / 2, t3_y + t3_h * 0.42, s1,
                ha="center", va="center",
                fontsize=8.5, color=TINT)
        ax.text(x + node_w / 2, t3_y + t3_h * 0.18, s2,
                ha="center", va="center",
                fontsize=8.5, color=TINT, style="italic")

    # =========================================================================
    # Tier 4 — Satellite Bridge / PHY KPM Extractor   (y ≈ 1.45 – 2.85)
    # =========================================================================
    t4_y, t4_h = 1.45, 1.40
    _box(ax, 0.6, t4_y, W - 1.2, t4_h, fill=TINT, edge=BLUE, lw=1.0)
    ax.text(W / 2, t4_y + t4_h - 0.27,
            "Satellite Bridge   +   PHY KPM Extractor",
            ha="center", va="center",
            fontsize=10.5, fontweight="bold", color=NAVY)

    detail_w, detail_h, detail_gap = 2.30, 0.55, 0.18
    details = [
        ("SGP4 orbit",     "propagation"),
        ("Markov 3-state", "fading"),
        ("DVB-S2X",        "28 ModCods"),
        ("Inter-beam",     "interference"),
        ("ISL topology",   "intra / inter"),
        ("C/N₀ + link",    "budget"),
    ]
    total = len(details) * detail_w + (len(details) - 1) * detail_gap
    x0 = (W - total) / 2.0
    for i, (lbl, sub) in enumerate(details):
        x = x0 + i * (detail_w + detail_gap)
        _box(ax, x, t4_y + 0.15, detail_w, detail_h,
             fill=STEEL, edge=NAVY, lw=0.8)
        ax.text(x + detail_w / 2, t4_y + 0.15 + detail_h * 0.65, lbl,
                ha="center", va="center",
                fontsize=8, fontweight="bold", color=WHITE)
        ax.text(x + detail_w / 2, t4_y + 0.15 + detail_h * 0.25, sub,
                ha="center", va="center",
                fontsize=7, style="italic", color=TINT)

    # Down arrows from Tier 3 to Tier 4 (centred channels, no labels needed)
    for cx in (3.10, W / 2, W - 3.10):
        _arrow(ax, cx, t3_y - 0.04, cx, t4_y + t4_h + 0.04,
               color=BLUE, lw=0.9)

    # =========================================================================
    # Tier 5 — UE classes   (y ≈ 0.40 – 1.00)
    # =========================================================================
    t5_y, t5_h = 0.40, 0.62
    _box(ax, 2.0, t5_y, W - 4.0, t5_h, fill=TINTGRAY, edge=GRAY, lw=0.8)
    ax.text(2.4, t5_y + t5_h / 2, "User Equipment",
            ha="left", va="center",
            fontsize=9, fontweight="bold", color=DARK)
    classes = [("Static", "0 km/h"), ("Pedestrian", "3 km/h"),
               ("Vehicular", "60 km/h"), ("HST", "300 km/h"),
               ("Aerial", "120 m/s")]
    x0 = 5.0
    cls_w = (W - 4.0 - (x0 - 2.0)) / len(classes)
    for i, (name, sp) in enumerate(classes):
        cx = x0 + i * cls_w + cls_w / 2 - 0.6
        ax.text(cx, t5_y + t5_h * 0.62, name,
                ha="left", va="center",
                fontsize=8.5, color=DARK, fontweight="bold")
        ax.text(cx, t5_y + t5_h * 0.28, sp,
                ha="left", va="center",
                fontsize=7, color=GRAY, style="italic")

    # Short tick arrow from Tier 4 to Tier 5 (visual continuity)
    _arrow(ax, W / 2, t4_y - 0.04, W / 2, t5_y + t5_h + 0.04,
           color=GRAY, lw=0.8)

    # =========================================================================
    # Footer attribution
    # =========================================================================
    ax.text(W / 2, 0.10,
            "Muhammad Uzair  ·  Independent Researcher  ·  github.com/Muhammaduazir69/oran-ntn",
            ha="center", va="center",
            fontsize=7.5, color=GRAY, style="italic")

    return fig


def main():
    fig = build_figure()
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    fig.savefig(OUT_PATH, dpi=180, bbox_inches="tight",
                facecolor="white", pad_inches=0.15)
    plt.close(fig)
    sz = os.path.getsize(OUT_PATH) / 1024
    print(f"wrote {OUT_PATH}  ({sz:.0f} KB)")


if __name__ == "__main__":
    sys.exit(main())
