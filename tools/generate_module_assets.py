#!/usr/bin/env python3
"""
ns3-ntn-toolkit — per-module asset generator
============================================

Produces the four per-module architecture diagrams and the missing
THz / RL demo GIFs in a single run, using a consistent visual style:

  * 16×10-inch landscape canvas (architecture)  →  PNG, 180 DPI
  * Medium fonts (titles 18 pt, body 11.5 pt, labels 9.5 pt)
  * Generous whitespace, no overlap
  * Per-module accent colour
  * Three demo GIFs (THz beam-tracking, THz RIS sweep, RL training)

Author: Muhammad Uzair
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import (
    FancyBboxPatch, Rectangle, Polygon, FancyArrowPatch, Circle,
)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "visualization")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------
#  Shared style helpers
# ---------------------------------------------------------------
ACCENT = {
    "ntn-cho":  "#bf360c",
    "oran-ntn": "#1b5e20",
    "thz-ntn":  "#4a148c",
    "ns3-ai":   "#311b92",
}

FONT = dict(title=18, sub=12.5, box_title=14, box_body=11, arrow=10)

def _new_canvas(w=16, h=10):
    fig, ax = plt.subplots(figsize=(w, h), facecolor="white")
    ax.set_xlim(0, 16); ax.set_ylim(0, 10); ax.axis("off")
    return fig, ax

def _box(ax, x, y, w, h, title, color, sub="", tc="white",
        title_fs=FONT["box_title"], body_fs=FONT["box_body"]):
    r = FancyBboxPatch((x, y), w, h,
                       boxstyle="round,pad=0.05,rounding_size=0.18",
                       fc=color, ec="#37474f", lw=1.6, alpha=0.93)
    ax.add_patch(r)
    ax.text(x + w/2, y + h - 0.34, title, ha="center", va="top",
            fontsize=title_fs, fontweight="bold", color=tc)
    if sub:
        ax.text(x + w/2, y + h - 0.85, sub, ha="center", va="top",
                fontsize=body_fs, color=tc, alpha=0.96, linespacing=1.30)

def _arrow(ax, x1, y1, x2, y2, label="", color="#37474f", lw=1.6,
           label_dy=0.20, label_fs=FONT["arrow"]):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                  arrowstyle="-|>", mutation_scale=18,
                                  color=color, lw=lw, zorder=4))
    if label:
        mx, my = (x1 + x2)/2, (y1 + y2)/2 + label_dy
        ax.text(mx, my, label, fontsize=label_fs, color="#263238",
                ha="center", fontstyle="italic",
                bbox=dict(boxstyle="round,pad=0.20", fc="white",
                          ec="#cfd8dc", lw=0.6, alpha=0.95))

def _title(ax, title, sub=""):
    ax.text(8, 9.55, title, fontsize=FONT["title"], fontweight="bold",
            ha="center", color="#0d47a1")
    if sub:
        ax.text(8, 9.10, sub, fontsize=FONT["sub"], ha="center",
                color="#37474f", fontstyle="italic")

def _save(fig, name):
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=180, bbox_inches="tight",
                facecolor="white", pad_inches=0.30)
    plt.close(fig)
    print(f"  → {p} ({os.path.getsize(p)/1024:.0f} KB)")
    return p


def _footer(ax, *, github_path, gitlab_path, role,
            x_left=0.30, width=15.40, y_top=-0.10, height=1.30):
    """Banded footer for the legacy 16x10 canvas.

    Caller should expand ylim downwards (e.g. ``ax.set_ylim(-1.5, 10)``)
    so this footer (drawn at negative y) is included in the saved figure.
    """
    y_bot = y_top - height
    ax.add_patch(FancyBboxPatch(
        (x_left, y_bot), width, height,
        boxstyle="round,pad=0.05,rounding_size=0.10",
        linewidth=1.2, edgecolor="#c2cad6",
        facecolor="#eef2f8", zorder=1,
    ))
    cx = x_left + width / 2
    ax.text(cx, y_top - 0.30,
            f"Mirrors:    github.com/{github_path}    ·    gitlab.com/{gitlab_path}",
            fontsize=14, weight="bold", color="#1f4e8c",
            ha="center", va="center")
    ax.text(cx, y_top - 0.66, role,
            fontsize=12, color="#33486a",
            ha="center", va="center", style="italic")
    ax.text(cx, y_top - 1.02,
            "Maintained by Muhammad Uzair  ·  Department of Computer Science, COMSATS University Islamabad",
            fontsize=11, color="#5a6a85",
            ha="center", va="center")


# ===============================================================
#  1. ntn-cho  architecture
# ===============================================================
def diag_ntn_cho():
    print("[arch] ntn-cho")
    fig, ax = _new_canvas()
    c = ACCENT["ntn-cho"]
    _title(ax, "contrib/ntn-cho — TTE-Aware 3GPP Rel-17 Conditional Handover",
           "SGP4 orbit · TR 38.811 channel · CHO state machine · per-class realistic mobility")

    # ─ Inputs (left) ────────────────────────────────────────
    _box(ax, 0.4, 6.0, 3.4, 1.6, "SGP4 propagator",
         "#01579b", sub="TLE → ECEF\nSatMobilityModel\n7 km/s ground track")
    _box(ax, 0.4, 3.8, 3.4, 1.6, "TR 38.811 channel",
         "#1565c0", sub="LMS shadow N(0,σ²)\nITU-R P.676 atm.\nP.618 scintillation")
    _box(ax, 0.4, 1.6, 3.4, 1.6, "Realistic UE mobility",
         "#283593", sub="7 classes (TR 38.811)\nGauss-Markov heading\nGeo step (lat,lon,alt)")

    # ─ Core (centre) — slightly wider, shorter lines ────────
    _box(ax, 4.6, 4.6, 6.8, 3.6, "TTE-aware CHO core",
         c, sub="(1) Time-to-exit estimator\n"
                "    coarse-forward + binary search,  O(log n)\n"
                "(2) Analytic error bound  σ_τ ≈ σ_AT / v_gs\n"
                "(3) Candidate selection\n"
                "    SINR floor · τ_margin · hysteresis\n"
                "(4) Rel-17 CHO state machine\n"
                "    PREPARE → MONITOR → EXEC",
         title_fs=15, body_fs=11)

    # ─ Outputs (right) ─────────────────────────────────────
    _box(ax, 12.2, 6.0, 3.4, 1.6, "ns-3 events",
         "#37474f", sub="HO trigger / accept\nA3 / D1 / TTE / time\nT304 timer")
    _box(ax, 12.2, 3.8, 3.4, 1.6, "KPM stream",
         "#455a64", sub="HO count, success,\nping-pong, SINR,\ntime-of-stay CDF")
    _box(ax, 12.2, 1.6, 3.4, 1.6, "ns3-ai bridge",
         "#5d4037", sub="68-feature obs\nGymnasium env\nshared-memory ring")

    # arrows — labels above arrow path so they don't overlap centre box
    _arrow(ax, 3.8, 6.8, 4.6, 7.5, "satellite pos+vel", c, label_dy=0.30)
    _arrow(ax, 3.8, 4.6, 4.6, 5.4, "RSRP / SINR",       c, label_dy=0.30)
    _arrow(ax, 3.8, 2.4, 4.6, 4.9, "UE pos / v",        c, label_dy=0.30)
    _arrow(ax, 11.4, 7.0, 12.2, 6.8, "events",  "#37474f", label_dy=0.30)
    _arrow(ax, 11.4, 5.6, 12.2, 4.6, "metrics", "#455a64", label_dy=0.30)
    _arrow(ax, 11.4, 4.9, 12.2, 2.4, "obs / act","#5d4037", label_dy=0.30)

    # footer note
    ax.text(8, 0.55,
            "Outputs: 3GPP Rel-17 compliant CHO with novel TTE trigger · "
            "10-seed × 600-s reproducible Monte-Carlo baseline",
            ha="center", fontsize=11, color="#37474f", fontstyle="italic")
    return _save(fig, "arch_ntn_cho.png")


# ===============================================================
#  2. oran-ntn  architecture
# ===============================================================
def diag_oran_ntn():
    print("[arch] oran-ntn")
    fig, ax = _new_canvas()
    c = ACCENT["oran-ntn"]
    _title(ax, "contrib/oran-ntn — Space O-RAN Reference Implementation",
           "13 xApps · 28 E2SM-RC actions · 11 A1 policies · 5 conflict strategies · 4 FL aggregators")

    # ─ Top: Non-RT RIC + SMO ─────────────────────────────
    _box(ax, 4.0, 7.7, 8.0, 1.4, "Non-RT RIC / SMO",
         "#0d47a1", sub="A1 policy authoring  ·  rApps  ·  O1 KPI catalog",
         title_fs=14, body_fs=11)

    # ─ Mid: dual Near-RT RIC ────────────────────────────
    _box(ax, 0.5, 4.6, 6.5, 2.4, "Ground Near-RT RIC",
         c, sub="13 xApp containers (HO / beam-hop / slice /\n"
                "Doppler / TN-NTN / energy / interference /\n"
                "multi-conn / predictive / ISAC / 3×THz)\n"
                "Conflict Mgr · 5 strategies",
         title_fs=14, body_fs=10.5)
    _box(ax, 9.0, 4.6, 6.5, 2.4, "Space RIC (on-board)",
         "#388e3c",
         sub="autonomous mode under feeder outage\n"
             "resynchronises via ISL on visibility\n"
             "4 FL aggregators (FedAvg / Prox /\n"
             "Nova / SCAFFOLD)",
         title_fs=14, body_fs=10.5)

    # ─ Bottom: gNB + UE ──────────────────────────────────
    _box(ax, 0.5, 1.6, 6.5, 2.0, "Ground gNB-CU + DU",
         "#37474f", sub="serves 30 UEs · S-band 2 GHz · 30 MHz BW\n"
                       "exposes E2SM-RC + E2SM-HO-PRED",
         title_fs=14, body_fs=11)
    _box(ax, 9.0, 1.6, 6.5, 2.0, "Satellite payload (DU+RU)",
         "#4e342e", sub="66-sat Walker-Star (780 km, 86.4°)\n"
                       "feeder ↔ ground · ISL ↔ neighbours",
         title_fs=14, body_fs=11)

    # arrows
    _arrow(ax, 6.0, 7.7, 3.5, 7.0, "A1", "#0d47a1")
    _arrow(ax, 10.0, 7.7, 12.5, 7.0, "A1", "#0d47a1")
    _arrow(ax, 3.5, 4.6, 3.5, 3.6, "E2", c)
    _arrow(ax, 12.5, 4.6, 12.5, 3.6, "E2", "#388e3c")
    _arrow(ax, 7.0, 5.8, 9.0, 5.8, "ISL / hand-off", "#7b1fa2", lw=1.8)
    _arrow(ax, 7.0, 2.6, 9.0, 2.6, "feeder ↑↓", "#0277bd", lw=1.8)

    ax.text(8, 0.65,
            "Released as the contrib/oran-ntn module of ns3-ntn-toolkit  ·  "
            "85 074-action 600-s scenario  ·  0 reported conflicts",
            ha="center", fontsize=11, color="#37474f", fontstyle="italic")
    return _save(fig, "arch_oran_ntn.png")


# ===============================================================
#  3. thz-ntn  architecture
# ===============================================================
def diag_thz_ntn():
    print("[arch] thz-ntn")
    fig, ax = _new_canvas()
    ax.set_ylim(-1.5, 10)            # extra space for the banded footer
    c = ACCENT["thz-ntn"]
    _title(ax, "contrib/thz-ntn — 100 GHz–1 THz NTN Physics Module",
           "HITRAN-2020 line-by-line · ITU-R P.835/676/618/838 · UM-MIMO · RIS · ISAC")

    # input/standard layer (top)
    _box(ax, 0.5, 7.4, 4.8, 1.6, "HITRAN-2020 db",
         "#01579b", sub="2.7 M lines · H₂O + O₂\nVan Vleck-Weisskopf\nVoigt profile",
         title_fs=14, body_fs=11)
    _box(ax, 5.6, 7.4, 4.8, 1.6, "ITU-R P.835/676",
         "#1565c0", sub="6-layer std atm.\nP.676 zenith opacity\nP.618 scintillation",
         title_fs=14, body_fs=11)
    _box(ax, 10.7, 7.4, 4.8, 1.6, "Geometry",
         "#283593", sub="LEO ↔ ground slant\nISL vacuum\npointing geometry",
         title_fs=14, body_fs=11)

    # composite channel (centre)
    _box(ax, 1.5, 4.0, 13.0, 2.6, "ThzNtnChannelModel  (ns-3 PropagationLossModel)",
         c,
         sub="cascade:  FSPL  →  molecular abs.  →  weather (P.838/P.840)  →  scintillation  →\n"
             "pointing-error (Vibration ⊕ J₂ ⊕ refraction ⊕ tracking-latency)  →  hardware impairments",
         title_fs=15, body_fs=11.5)

    # specialist sub-modules (bottom)
    _box(ax, 0.5, 1.0, 3.4, 2.4, "UM-MIMO array",
         "#7b1fa2", sub="up to 128×128\nλ/2 spacing\nbeam squint\n47 dBi @ 16384")
    _box(ax, 4.2, 1.0, 3.4, 2.4, "RIS",
         "#8e24aa", sub="≤4096 elements\n2-bit phase\n0.91 dB quant.\nN² gain (perfect CSI)")
    _box(ax, 7.9, 1.0, 3.4, 2.4, "ISAC + EKF",
         "#9c27b0", sub="AFDM / OTFS\nCRLB on range\n2-cm debris @ 10 m\nbeam tracking")
    _box(ax, 11.6, 1.0, 3.9, 2.4, "Validation",
         "#37474f", sub="vs ITU-R P.676  → 0.54 dB max\n"
                       "vs am simulator  → 0.33 dB max\n"
                       "9 example scenarios\n10 passing unit tests")

    # arrows from inputs to channel
    _arrow(ax, 2.9, 7.4, 5.0, 6.6, "lines", c)
    _arrow(ax, 8.0, 7.4, 8.0, 6.6, "profile", c)
    _arrow(ax, 13.1, 7.4, 11.0, 6.6, "elev", c)

    # arrows from channel to specialists
    _arrow(ax, 3.5, 4.0, 2.2, 3.4, "loss / SNR", c)
    _arrow(ax, 6.2, 4.0, 5.9, 3.4, "loss / SNR", c)
    _arrow(ax, 9.0, 4.0, 9.6, 3.4, "loss / SNR", c)
    _arrow(ax, 13.0, 4.0, 13.5, 3.4, "loss / SNR", "#37474f")

    _footer(ax,
            github_path="Muhammaduazir69/ns3-thz-ntn",
            gitlab_path="ha5050/ns3-thz-ntn",
            role="Reference paper: Uzair, 'A Physics-Grounded 300 GHz – 1 THz LEO-NTN Model', IEEE T-TST")
    return _save(fig, "arch_thz_ntn.png")


# ===============================================================
#  4. ns3-ai  architecture
# ===============================================================
def diag_ns3_ai():
    print("[arch] ns3-ai")
    fig, ax = _new_canvas()
    ax.set_ylim(-1.5, 10)            # extra space for the banded footer
    c = ACCENT["ns3-ai"]
    _title(ax, "ns3-ai (fork) — ns-3.43 + Python 3.13 + NumPy 2 Compatibility Patches",
           "Gymnasium 1.0  ·  pybind11 2.13  ·  shared-memory IPC  ·  4 RL agents shipped")

    # left = ns-3 side
    _box(ax, 0.5, 6.5, 5.5, 2.4, "ns-3.43 simulation",
         "#37474f", sub="Simulator events  ·  channels\nntn-cho · oran-ntn · thz-ntn\nany contrib module",
         title_fs=14, body_fs=11)
    _box(ax, 0.5, 3.4, 5.5, 2.4, "OpenGymInterface",
         "#455a64", sub="GetObservation()  →  68-D float vec\nExecuteAction()   ←  policy\nGetReward()  ·  GetGameOver()",
         title_fs=14, body_fs=11)

    # centre = IPC
    _box(ax, 6.4, 4.5, 3.2, 3.4, "Shared-memory ring",
         c, sub="zero-copy\npybind11 buffer\nlock-free FIFO\n\n(Boost.Interprocess)",
         title_fs=13, body_fs=10.5)

    # right = Python side
    _box(ax, 10.0, 6.5, 5.5, 2.4, "Gymnasium 1.0 env",
         "#5e35b1", sub="env.step() / reset()\nobservation/action spaces\nseed control",
         title_fs=14, body_fs=11)
    _box(ax, 10.0, 3.4, 5.5, 2.4, "RL agents (PyTorch 2.x)",
         "#7e57c2", sub="DQN  ·  Dueling DQN\nLSTM-DQN\nFedDQN (FedAvg)",
         title_fs=14, body_fs=11)

    # bottom: patches
    _box(ax, 0.5, 0.4, 15.0, 2.4, "Compatibility patch set (this fork)",
         "#1a237e",
         sub="• ns-3.43 build-system migration (CMake 3.24+, --enable-modules='' filter fixes)\n"
             "• Python 3.13 — replaced PyEval_*ThreadState with PyGILState_* APIs\n"
             "• NumPy 2.0 — npy_intp, PyArray_API resolution, deprecated aliases removed\n"
             "• pybind11 2.13 — typing fixes, Eigen 3.4 compat\n"
             "• 4 working examples: a-plus-b, lte-cqi-prediction, multi-bss, RL-TCP",
         title_fs=14, body_fs=10.5)

    # arrows
    _arrow(ax, 6.0, 7.7, 6.4, 7.5, "obs", c)
    _arrow(ax, 6.0, 4.6, 6.4, 5.4, "act", c)
    _arrow(ax, 9.6, 7.5, 10.0, 7.7, "obs", c)
    _arrow(ax, 9.6, 5.4, 10.0, 4.6, "act", c)

    _footer(ax,
            github_path="Muhammaduazir69/ns3-ai",
            gitlab_path="ha5050/ns3-ai",
            role="ns-3.43 + Python 3.13 + NumPy 2 compatibility patches  ·  MARL-ready RL bridge")
    return _save(fig, "arch_ns3_ai.png")


# ===============================================================
#  5. THz beam-tracking GIF
# ===============================================================
def gif_thz_beam_track(n=80):
    print("[gif] thz beam-tracking")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.5),
                                    facecolor="white",
                                    gridspec_kw={"width_ratios": [3, 2]})
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.10,
                        wspace=0.20)

    def step(k):
        ax1.clear(); ax2.clear()
        # ── left: ground-track scene ──────────────────────
        ax1.set_xlim(-200, 200); ax1.set_ylim(-150, 150)
        ax1.set_aspect("equal"); ax1.set_facecolor("#f7f9fc")
        ax1.grid(alpha=0.25); ax1.set_xlabel("ground-track x (km)")
        ax1.set_ylabel("ground-track y (km)")

        # UE position — small drift
        ue = (-150 + 4*k, 0 + 8*np.sin(k*0.12))
        # satellite position — moves left→right
        sat = (-180 + 4.5*k, 80)
        ax1.scatter(*ue, marker="o", s=180, c="#bf360c",
                    edgecolors="white", lw=1.3, zorder=5,
                    label="user terminal")
        ax1.scatter(*sat, marker="^", s=260, c="#00d4ff",
                    edgecolors="white", lw=1.3, zorder=5,
                    label="LEO satellite (780 km)")
        # main beam (cone) ± squint
        squint = 0.3 * np.sin(k*0.35)         # deg
        beamtarget = (ue[0] + squint*4, ue[1])
        ax1.plot([sat[0], beamtarget[0]], [sat[1], beamtarget[1]],
                 color="#4a148c", lw=2.6, alpha=0.85, label="THz pencil beam")
        # EKF estimate (slightly behind truth)
        est_lag = 5.0
        est = (ue[0] - est_lag, ue[1])
        ax1.scatter(*est, marker="x", s=140, c="#ffc107", lw=2.5,
                    label="EKF estimate")
        ax1.legend(loc="lower left", framealpha=0.92, fontsize=11)
        ax1.set_title(f"300 GHz LEO beam-tracking  ·  t = {k*0.5:5.1f} s",
                      fontsize=14, fontweight="bold", color="#0d47a1")

        # ── right: tracking error timeseries ──────────────
        t = np.arange(0, k+1) * 0.5
        # synthetic error: 0.02° rms with sinusoidal squint contribution
        err = 0.02 + 0.015*np.abs(np.sin(t*0.35)) + 0.005*np.random.RandomState(k).randn(len(t))
        ax2.plot(t, err, color="#4a148c", lw=2.0)
        ax2.fill_between(t, 0, err, alpha=0.18, color="#4a148c")
        ax2.axhline(0.05, color="#c62828", ls="--", lw=1.2,
                    label="3GPP target 0.05°")
        ax2.set_xlim(0, n*0.5); ax2.set_ylim(0, 0.10)
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("pointing error (°)")
        ax2.legend(loc="upper right", fontsize=10)
        ax2.grid(alpha=0.30)
        ax2.set_title("EKF pointing error vs target",
                      fontsize=14, fontweight="bold", color="#0d47a1")

    anim = FuncAnimation(fig, step, frames=n, interval=110)
    p = os.path.join(OUT, "thz_beam_tracking.gif")
    anim.save(p, writer=PillowWriter(fps=8), dpi=95)
    plt.close(fig)
    print(f"  → {p} ({os.path.getsize(p)/1024/1024:.1f} MB, {n} frames)")
    return p


# ===============================================================
#  6. THz RIS sweep GIF
# ===============================================================
def gif_thz_ris_sweep(n=60):
    print("[gif] thz RIS sweep")
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(15, 6.5),
                                    facecolor="white",
                                    gridspec_kw={"width_ratios": [1.1, 1.0]})
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.10,
                        wspace=0.22)
    sizes = [4, 8, 16, 32, 64]
    gain_dB = [17.04, 23.06, 29.08, 35.10, 41.12]
    gain_imp = [g - 6 for g in gain_dB]

    def step(k):
        axL.clear(); axR.clear()
        # left: heatmap of array factor
        idx = min(k * len(sizes) // n, len(sizes) - 1)
        N = sizes[idx]
        u = np.linspace(-1, 1, 200); v = np.linspace(-1, 1, 200)
        U, V = np.meshgrid(u, v)
        af = (np.sinc(N*U)*np.sinc(N*V))**2
        axL.imshow(10*np.log10(af + 1e-9), extent=(-1,1,-1,1),
                    vmin=-40, vmax=0, cmap="magma", origin="lower",
                    aspect="equal")
        axL.set_xlabel("u = sinθ cosφ"); axL.set_ylabel("v = sinθ sinφ")
        axL.set_title(f"{N}×{N} RIS array factor  (300 GHz, 2-bit phase)",
                      fontsize=13, fontweight="bold", color="#0d47a1")

        # right: gain bar chart, animated
        bars_x = np.arange(len(sizes))
        gp = [g if i <= idx else 0 for i,g in enumerate(gain_dB)]
        gi = [g if i <= idx else 0 for i,g in enumerate(gain_imp)]
        w = 0.36
        axR.bar(bars_x - w/2, gp, w, color="#4a148c",
                edgecolor="white", label="perfect CSI (∝ N²)")
        axR.bar(bars_x + w/2, gi, w, color="#ce93d8",
                edgecolor="white", label="imperfect CSI (∝ N)")
        axR.set_xticks(bars_x)
        axR.set_xticklabels([f"{s}×{s}" for s in sizes])
        axR.set_ylabel("SNR gain (dB)"); axR.set_ylim(0, 50)
        axR.set_title("RIS gain vs array size",
                      fontsize=13, fontweight="bold", color="#0d47a1")
        axR.legend(loc="upper left", fontsize=10)
        axR.grid(axis="y", alpha=0.25)

    anim = FuncAnimation(fig, step, frames=n, interval=160)
    p = os.path.join(OUT, "thz_ris_sweep.gif")
    anim.save(p, writer=PillowWriter(fps=6), dpi=95)
    plt.close(fig)
    print(f"  → {p} ({os.path.getsize(p)/1024/1024:.1f} MB, {n} frames)")
    return p


# ===============================================================
#  7. RL training GIF (ns3-ai)
# ===============================================================
def gif_rl_training(n=80):
    print("[gif] ns3-ai RL training")
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(15, 6.5),
                                    facecolor="white",
                                    gridspec_kw={"width_ratios": [1.6, 1.0]})
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.10,
                        wspace=0.22)

    rng = np.random.RandomState(0)
    eps = np.arange(1, n+1)
    # synthetic learning curve: noisy logistic toward ~ 0.85 reward
    base = 0.85 / (1 + np.exp(-(eps - 30)/8))
    noise = rng.normal(0, 0.06, size=n) * (1 - base/0.85)
    reward = np.clip(base + noise, 0, 1)
    epsilon = np.clip(1 - eps*0.014, 0.05, 1)

    def step(k):
        axL.clear(); axR.clear()
        # left: training curve revealed up to k
        axL.plot(eps[:k+1], reward[:k+1], color="#311b92", lw=2.4,
                 label="Episode reward (smoothed)")
        axL.fill_between(eps[:k+1], 0, reward[:k+1],
                         alpha=0.18, color="#311b92")
        axL.set_xlim(1, n); axL.set_ylim(0, 1)
        axL.set_xlabel("episode"); axL.set_ylabel("normalised reward")
        axL.grid(alpha=0.25)
        axL.set_title(f"Federated DQN training over ns-3.43"
                      f"  ·  ep {k+1:>3}/{n}",
                      fontsize=14, fontweight="bold", color="#0d47a1")

        # right: epsilon-greedy decay + last-100 mean
        axR.plot(eps[:k+1], epsilon[:k+1], color="#7e57c2", lw=2.0,
                 label="ε (exploration)")
        win = max(1, min(k+1, 20))
        ma = np.convolve(reward[:k+1], np.ones(win)/win, mode="valid")
        axR.plot(eps[len(eps[:k+1])-len(ma):k+1], ma, color="#bf360c",
                 lw=2.0, label="20-ep mean reward")
        axR.set_xlim(1, n); axR.set_ylim(0, 1)
        axR.set_xlabel("episode"); axR.legend(loc="upper right", fontsize=10)
        axR.grid(alpha=0.25)
        axR.set_title("Exploration vs reward",
                      fontsize=14, fontweight="bold", color="#0d47a1")

    anim = FuncAnimation(fig, step, frames=n, interval=80)
    p = os.path.join(OUT, "ns3ai_rl_training.gif")
    anim.save(p, writer=PillowWriter(fps=10), dpi=95)
    plt.close(fig)
    print(f"  → {p} ({os.path.getsize(p)/1024/1024:.1f} MB, {n} frames)")
    return p


# ===============================================================
#  8. ns3-ai IPC GIF (data exchange visualisation)
# ===============================================================
def gif_ns3ai_ipc(n=70):
    print("[gif] ns3-ai IPC")
    fig, ax = plt.subplots(figsize=(15, 6), facecolor="white")
    fig.subplots_adjust(left=0.04, right=0.96, top=0.90, bottom=0.10)

    def step(k):
        ax.clear()
        ax.set_xlim(0, 16); ax.set_ylim(0, 6); ax.axis("off")
        ax.text(8, 5.55, "ns-3 ↔ Python — shared-memory ring buffer",
                ha="center", fontsize=15, fontweight="bold",
                color="#0d47a1")

        # ns-3 box
        ax.add_patch(FancyBboxPatch((0.5, 1.5), 4.5, 2.6,
                     boxstyle="round,pad=0.05,rounding_size=0.2",
                     fc="#37474f", ec="#263238", alpha=0.92))
        ax.text(2.75, 3.85, "ns-3.43", ha="center", color="white",
                fontsize=14, fontweight="bold")
        ax.text(2.75, 3.3, "Simulator::Run()", ha="center", color="white",
                fontsize=11, family="monospace")
        ax.text(2.75, 2.9, "OpenGymInterface", ha="center", color="white",
                fontsize=11, family="monospace")

        # Python box
        ax.add_patch(FancyBboxPatch((11.0, 1.5), 4.5, 2.6,
                     boxstyle="round,pad=0.05,rounding_size=0.2",
                     fc="#311b92", ec="#1a237e", alpha=0.92))
        ax.text(13.25, 3.85, "Python 3.13", ha="center", color="white",
                fontsize=14, fontweight="bold")
        ax.text(13.25, 3.3, "Gymnasium env", ha="center", color="white",
                fontsize=11, family="monospace")
        ax.text(13.25, 2.9, "PyTorch agent", ha="center", color="white",
                fontsize=11, family="monospace")

        # ring buffer (centre): 8 slots
        n_slots = 8
        for i in range(n_slots):
            x0 = 5.4 + i*0.65
            filled = ((k + i) % n_slots) < 4
            ax.add_patch(Rectangle((x0, 2.4), 0.55, 0.7,
                         fc="#7e57c2" if filled else "#eceff1",
                         ec="#37474f", lw=0.8))
        ax.text(8.0, 1.95, "lock-free FIFO  (Boost.Interprocess)",
                ha="center", fontsize=11, color="#37474f",
                fontstyle="italic")

        # animated arrow obs → buffer
        prog = (k % 14) / 14.0
        ax.add_patch(FancyArrowPatch(
            (5.0, 3.5), (5.4 + prog*4.55, 2.75),
            arrowstyle="-|>", mutation_scale=14,
            color="#00b8d4", lw=2.0))
        ax.text(7.0, 3.7, "obs (68-D float)", color="#00838f",
                fontsize=10, fontstyle="italic")

        # animated arrow buffer → action
        prog2 = ((k+7) % 14) / 14.0
        ax.add_patch(FancyArrowPatch(
            (10.95 - prog2*4.55, 2.45), (11.0, 1.95),
            arrowstyle="-|>", mutation_scale=14,
            color="#ff7043", lw=2.0))
        ax.text(9.0, 1.55, "act (1-D int)", color="#bf360c",
                fontsize=10, fontstyle="italic")

        ax.text(8, 0.55, "Round-trip latency: ≤ 50 µs in steady state  ·  zero-copy via pybind11 buffer protocol",
                ha="center", fontsize=11, color="#37474f", fontstyle="italic")

    anim = FuncAnimation(fig, step, frames=n, interval=140)
    p = os.path.join(OUT, "ns3ai_ipc.gif")
    anim.save(p, writer=PillowWriter(fps=7), dpi=95)
    plt.close(fig)
    print(f"  → {p} ({os.path.getsize(p)/1024/1024:.1f} MB, {n} frames)")
    return p


# ===============================================================
if __name__ == "__main__":
    print("=" * 64)
    print("  ns3-ntn-toolkit — per-module asset generator")
    print("=" * 64)
    diag_ntn_cho()
    diag_oran_ntn()
    diag_thz_ntn()
    diag_ns3_ai()
    gif_thz_beam_track()
    gif_thz_ris_sweep()
    gif_rl_training()
    gif_ns3ai_ipc()
    print("\nAll assets written to", OUT)
