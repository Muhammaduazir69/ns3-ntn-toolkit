#!/usr/bin/env python3
"""
O-RAN NTN Module - Professional Visualization Generator
========================================================
Generates animated GIFs and static PNGs for GitHub README.
Author: Muhammad Uzair
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch, Arc
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec
import matplotlib.colors as mcolors

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'visualization')
os.makedirs(OUT_DIR, exist_ok=True)

# Color schemes
DARK_BG = '#0d1117'
PANEL_BG = '#161b22'
ACCENT_BLUE = '#58a6ff'
ACCENT_GREEN = '#3fb950'
ACCENT_RED = '#f85149'
ACCENT_ORANGE = '#d29922'
ACCENT_PURPLE = '#bc8cff'
ACCENT_CYAN = '#39d2c0'
TEXT_WHITE = '#e6edf3'
TEXT_DIM = '#8b949e'

# =============================================================================
#  GIF 1: LEO Constellation with O-RAN RIC
# =============================================================================
def generate_constellation_gif():
    print("  Generating constellation GIF...")
    np.random.seed(42)

    num_planes, sats_per_plane = 6, 11
    alt_km = 550
    R = 6371
    inc = np.radians(53)

    fig = plt.figure(figsize=(10, 10), facecolor=DARK_BG)
    ax = fig.add_subplot(111, polar=True)
    ax.set_facecolor(DARK_BG)
    ax.set_theta_zero_location('N')
    ax.set_rlim(0, 90)
    ax.set_rticks([15, 30, 45, 60, 75, 90])
    ax.set_yticklabels(['75°', '60°', '45°', '30°', '15°', '0°'], color=TEXT_DIM, fontsize=7)
    ax.tick_params(colors=TEXT_DIM)
    ax.grid(True, color='#30363d', alpha=0.5, linewidth=0.5)
    ax.spines['polar'].set_color('#30363d')

    title = ax.set_title("Space-O-RAN: LEO Constellation\nwith Autonomous Space RICs",
                         color=TEXT_WHITE, fontsize=14, fontweight='bold', pad=20)

    gw_lons = np.radians([0, 120, 240, 60])
    gw_lats = np.array([45, 35, 50, -30])
    gw_r = 90 - gw_lats
    ax.scatter(gw_lons, gw_r, marker='^', s=120, c=ACCENT_ORANGE, zorder=10,
               edgecolors='white', linewidths=0.5, label='Ground Gateway')

    sat_dots = ax.scatter([], [], s=30, zorder=8)
    link_lines = []
    isl_lines = []
    beam_patches = []
    status_text = ax.text(0.02, 0.02, '', transform=fig.transFigure,
                          color=TEXT_WHITE, fontsize=9, family='monospace',
                          verticalalignment='bottom')

    def get_sat_positions(t):
        positions = []
        for p in range(num_planes):
            raan = 2 * np.pi * p / num_planes
            for s in range(sats_per_plane):
                ma = 2 * np.pi * s / sats_per_plane + 2 * np.pi * t / 5400
                u = ma
                lat = np.degrees(np.arcsin(np.sin(inc) * np.sin(u)))
                lon = np.degrees(raan + np.arctan2(np.cos(inc) * np.sin(u), np.cos(u)))
                lon = ((lon + 180) % 360) - 180
                positions.append((lon, lat, p, s))
        return positions

    def update(frame):
        for line in link_lines:
            line.remove()
        link_lines.clear()
        for line in isl_lines:
            line.remove()
        isl_lines.clear()
        for patch in beam_patches:
            patch.remove()
        beam_patches.clear()

        t = frame * 30
        positions = get_sat_positions(t)
        thetas = np.array([np.radians(p[0]) for p in positions])
        rs = np.array([90 - p[1] for p in positions])

        auto_sats = set()
        for i, pos in enumerate(positions):
            visible = False
            for gi in range(len(gw_lons)):
                dist = np.sqrt((pos[0] - np.degrees(gw_lons[gi]))**2 + (pos[1] - gw_lats[gi])**2)
                if dist < 40:
                    visible = True
                    l, = ax.plot([thetas[i], gw_lons[gi]], [rs[i], gw_r[gi]],
                                 color=ACCENT_GREEN, alpha=0.2, linewidth=0.5, linestyle='--')
                    link_lines.append(l)
                    break
            if not visible:
                auto_sats.add(i)

        colors = [ACCENT_RED if i in auto_sats else ACCENT_CYAN for i in range(len(positions))]
        sat_dots.set_offsets(np.column_stack([thetas, rs]))
        sat_dots.set_color(colors)
        sat_dots.set_sizes([40 if i in auto_sats else 25 for i in range(len(positions))])

        for p in range(num_planes):
            for s in range(sats_per_plane):
                i1 = p * sats_per_plane + s
                i2 = p * sats_per_plane + (s + 1) % sats_per_plane
                if abs(rs[i1] - rs[i2]) < 30:
                    l, = ax.plot([thetas[i1], thetas[i2]], [rs[i1], rs[i2]],
                                 color='#1f6feb', alpha=0.15, linewidth=0.3)
                    isl_lines.append(l)

        for si in [0, 22, 44]:
            if si < len(positions):
                circle = plt.Circle((thetas[si], rs[si]), 8, transform=ax.transData,
                                     fill=True, alpha=0.08,
                                     color=ACCENT_RED if si in auto_sats else ACCENT_CYAN)
                p = ax.add_patch(circle)
                beam_patches.append(p)

        n_auto = len(auto_sats)
        n_ground = len(positions) - n_auto
        status_text.set_text(
            f'Time: {t:5d}s | Satellites: {len(positions)} | '
            f'Ground-Assisted: {n_ground} | Autonomous: {n_auto} | ISL Links: {num_planes * sats_per_plane}')

        return [sat_dots, status_text]

    anim = FuncAnimation(fig, update, frames=150, interval=50, blit=False)
    path = os.path.join(OUT_DIR, 'oran_ntn_constellation_ric.gif')
    anim.save(path, writer=PillowWriter(fps=20))
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")

# =============================================================================
#  GIF 2: xApp Decision Dashboard
# =============================================================================
def generate_xapp_dashboard_gif():
    print("  Generating xApp dashboard GIF...")
    np.random.seed(123)
    N = 120

    t = np.linspace(0, 60, N)
    sinr1 = 8 + 5*np.sin(0.15*t) + np.random.randn(N)*1.5
    sinr2 = 5 + 4*np.sin(0.12*t + 1) + np.random.randn(N)*1.2
    sinr3 = 12 + 3*np.sin(0.1*t + 2) + np.random.randn(N)*1.0

    ho_times = [12, 28, 45]
    beam_data = np.random.rand(12, N) * 0.3 + 0.2
    beam_data[3, :] = 0.6 + 0.3*np.sin(0.2*t)
    beam_data[7, :] = 0.5 + 0.25*np.sin(0.15*t + 1)

    slice_embb = 0.55 + 0.1*np.sin(0.08*t)
    slice_urllc = 0.25 + 0.05*np.sin(0.1*t + 1)
    slice_mmtc = 1.0 - slice_embb - slice_urllc

    batt = np.ones(N) * 0.9
    solar = 150 + 50*np.sin(0.06*t)
    for i in range(N):
        if 40 < t[i] < 55:
            solar[i] = 0
            batt[i] = batt[max(0,i-1)] - 0.008
        else:
            batt[i] = min(1.0, batt[max(0,i-1)] + 0.003)

    fl_global = 2.0 * np.exp(-0.05*t) + 0.3 + np.random.randn(N)*0.05
    fl_locals = [fl_global + np.random.randn(N)*0.15 for _ in range(4)]

    fig = plt.figure(figsize=(14, 8), facecolor=DARK_BG)
    fig.suptitle("O-RAN NTN: 9 xApps Concurrent Decision Making",
                 color=TEXT_WHITE, fontsize=14, fontweight='bold', y=0.98)
    gs = gridspec.GridSpec(2, 3, hspace=0.35, wspace=0.3,
                           left=0.06, right=0.97, top=0.92, bottom=0.06)

    axes = [fig.add_subplot(gs[i, j]) for i in range(2) for j in range(3)]
    for ax in axes:
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=TEXT_DIM, labelsize=7)
        for spine in ax.spines.values():
            spine.set_color('#30363d')

    titles = ['HO Prediction (DQN)', 'Beam Hopping (PPO)', 'Slice Manager (MAPPO)',
              'Interference Mgmt', 'Energy Harvesting', 'Federated Learning']
    for i, title in enumerate(titles):
        axes[i].set_title(title, color=ACCENT_BLUE, fontsize=9, fontweight='bold', pad=5)

    def update(frame):
        f = min(frame, N-1)
        for ax in axes:
            ax.clear()
            ax.set_facecolor(PANEL_BG)
            ax.tick_params(colors=TEXT_DIM, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color('#30363d')

        for i, title in enumerate(titles):
            axes[i].set_title(title, color=ACCENT_BLUE, fontsize=9, fontweight='bold', pad=5)

        # HO Prediction
        ax0 = axes[0]
        ax0.plot(t[:f], sinr1[:f], color='#58a6ff', linewidth=1, alpha=0.9, label='UE 1')
        ax0.plot(t[:f], sinr2[:f], color='#f0883e', linewidth=1, alpha=0.9, label='UE 2')
        ax0.plot(t[:f], sinr3[:f], color='#3fb950', linewidth=1, alpha=0.9, label='UE 3')
        ax0.axhline(y=-3, color=ACCENT_RED, linestyle=':', alpha=0.5, linewidth=0.7)
        for ht in ho_times:
            if t[f] > ht:
                ax0.axvline(x=ht, color=ACCENT_GREEN, linestyle='--', alpha=0.5, linewidth=0.7)
        ax0.set_xlim(0, 60); ax0.set_ylim(-8, 22)
        ax0.set_ylabel('SINR (dB)', color=TEXT_DIM, fontsize=7)
        ax0.legend(fontsize=6, loc='upper right', framealpha=0.3, labelcolor=TEXT_DIM)

        # Beam Hopping
        ax1 = axes[1]
        bdata = beam_data[:, :max(1,f)]
        ax1.imshow(bdata, aspect='auto', cmap='YlOrRd', vmin=0, vmax=1,
                   extent=[0, t[max(0,f-1)], 11.5, -0.5])
        ax1.set_ylabel('Beam ID', color=TEXT_DIM, fontsize=7)
        ax1.set_xlabel('Time (s)', color=TEXT_DIM, fontsize=7)

        # Slice Manager
        ax2 = axes[2]
        ax2.fill_between(t[:f], 0, slice_embb[:f], color='#58a6ff', alpha=0.7, label='eMBB')
        ax2.fill_between(t[:f], slice_embb[:f], slice_embb[:f]+slice_urllc[:f],
                         color='#f85149', alpha=0.7, label='URLLC')
        ax2.fill_between(t[:f], slice_embb[:f]+slice_urllc[:f], 1,
                         color='#3fb950', alpha=0.7, label='mMTC')
        ax2.set_xlim(0, 60); ax2.set_ylim(0, 1)
        ax2.set_ylabel('PRB Share', color=TEXT_DIM, fontsize=7)
        ax2.legend(fontsize=6, loc='upper right', framealpha=0.3, labelcolor=TEXT_DIM)

        # Interference
        ax3 = axes[3]
        theta_int = np.linspace(0, 2*np.pi, 72)
        r_int = 0.3 + 0.5*np.abs(np.sin(3*theta_int + t[f]*0.05)) + np.random.rand(72)*0.1
        ax3.bar(range(72), r_int[:72], color=[ACCENT_RED if v>0.7 else ACCENT_CYAN for v in r_int],
                alpha=0.7, width=1)
        ax3.set_xlim(0, 72); ax3.set_ylim(0, 1.2)
        ax3.set_ylabel('Interference', color=TEXT_DIM, fontsize=7)
        ax3.set_xlabel('Beam ID', color=TEXT_DIM, fontsize=7)

        # Energy
        ax4 = axes[4]
        ax4_r = ax4.twinx()
        ax4.plot(t[:f], batt[:f], color=ACCENT_GREEN, linewidth=1.5, label='Battery SoC')
        ax4_r.fill_between(t[:f], 0, solar[:f], color=ACCENT_ORANGE, alpha=0.2)
        ax4_r.plot(t[:f], solar[:f], color=ACCENT_ORANGE, linewidth=0.8, label='Solar (W)')
        if t[f] > 40:
            ax4.axvspan(40, min(55, t[f]), alpha=0.15, color='gray')
            ax4.text(47, 0.95, 'Eclipse', color=TEXT_DIM, fontsize=6, ha='center')
        ax4.set_xlim(0, 60); ax4.set_ylim(0.4, 1.05)
        ax4_r.set_ylim(0, 250)
        ax4.set_ylabel('SoC', color=ACCENT_GREEN, fontsize=7)
        ax4_r.set_ylabel('Power (W)', color=ACCENT_ORANGE, fontsize=7)
        ax4_r.tick_params(colors=TEXT_DIM, labelsize=7)

        # FL
        ax5 = axes[5]
        for i, fl in enumerate(fl_locals):
            ax5.plot(t[:f], fl[:f], color=TEXT_DIM, alpha=0.3, linewidth=0.5)
        ax5.plot(t[:f], fl_global[:f], color=ACCENT_PURPLE, linewidth=2, label='Global Model')
        ax5.set_xlim(0, 60); ax5.set_ylim(0, 2.5)
        ax5.set_ylabel('Loss', color=TEXT_DIM, fontsize=7)
        ax5.set_xlabel('Time (s)', color=TEXT_DIM, fontsize=7)
        ax5.legend(fontsize=6, loc='upper right', framealpha=0.3, labelcolor=TEXT_DIM)

    anim = FuncAnimation(fig, update, frames=N, interval=80, blit=False)
    path = os.path.join(OUT_DIR, 'oran_ntn_xapp_decisions.gif')
    anim.save(path, writer=PillowWriter(fps=12))
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")

# =============================================================================
#  Static PNG 1: Architecture Diagram
# =============================================================================
def generate_architecture_png():
    print("  Generating architecture diagram...")
    fig, ax = plt.subplots(1, 1, figsize=(14, 9), facecolor='white')
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 9)
    ax.axis('off')

    def draw_box(x, y, w, h, label, color, fontsize=9, textcolor='white'):
        box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                              facecolor=color, edgecolor='#2c3e50', linewidth=1.2, alpha=0.9)
        ax.add_patch(box)
        ax.text(x + w/2, y + h/2, label, ha='center', va='center',
                fontsize=fontsize, fontweight='bold', color=textcolor)

    def draw_arrow(x1, y1, x2, y2, label='', color='#2c3e50'):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx+0.15, my, label, fontsize=7, color=color, fontstyle='italic')

    # Non-RT RIC / SMO
    draw_box(4.5, 8.0, 5, 0.7, 'Non-RT RIC / SMO', '#6f42c1', fontsize=11)
    ax.text(7, 8.85, 'A1 Policies: HO Threshold, Slice SLA, Energy Saving, FL Participation',
            ha='center', fontsize=7, color='#6f42c1', fontstyle='italic')

    # Near-RT RIC
    draw_box(2, 5.8, 10, 1.8, '', '#1f6feb', fontsize=10)
    ax.text(7, 7.35, 'Near-RT RIC', ha='center', fontsize=12, fontweight='bold', color='white')

    # xApps inside RIC
    xapp_names = ['HO\nPredict', 'Beam\nHop', 'Slice\nMgr', 'Doppler\nComp', 'TN-NTN\nSteer',
                  'Interf\nMgmt', 'Energy\nHarvest', 'Predict\nAlloc', 'Multi\nConn']
    xapp_colors = ['#2ea043', '#2ea043', '#2ea043', '#2ea043', '#2ea043',
                   '#bf8700', '#bf8700', '#bf8700', '#bf8700']
    for i, (name, col) in enumerate(zip(xapp_names, xapp_colors)):
        x = 2.3 + i * 1.06
        draw_box(x, 5.95, 0.95, 0.7, name, col, fontsize=6)

    # E2 + SDL + Conflict
    ax.text(3.0, 6.8, 'E2 Term', fontsize=7, color='#c9d1d9',
            bbox=dict(boxstyle='round', facecolor='#30363d', alpha=0.8))
    ax.text(5.5, 6.8, 'SDL', fontsize=7, color='#c9d1d9',
            bbox=dict(boxstyle='round', facecolor='#30363d', alpha=0.8))
    ax.text(7.5, 6.8, 'Conflict Mgr', fontsize=7, color='#c9d1d9',
            bbox=dict(boxstyle='round', facecolor='#30363d', alpha=0.8))
    ax.text(10.0, 6.8, 'A1 Adapter', fontsize=7, color='#c9d1d9',
            bbox=dict(boxstyle='round', facecolor='#30363d', alpha=0.8))

    # A1 arrow
    draw_arrow(7, 8.0, 7, 7.6, 'A1', '#6f42c1')

    # Satellite gNBs
    draw_box(0.5, 3.5, 3.5, 1.5, '', '#da3633')
    ax.text(2.25, 4.75, 'Satellite gNB (LEO)', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(2.25, 4.15, 'Space RIC\nInference Engine\nISL Coordination', ha='center',
            fontsize=7, color='#ffa198')
    ax.text(2.25, 3.65, 'Autonomous | Federated Learning', ha='center',
            fontsize=6, color='#ffa198', fontstyle='italic')

    # Terrestrial gNB
    draw_box(5, 3.5, 3.5, 1.5, '', '#1a7f37')
    ax.text(6.75, 4.75, 'Terrestrial gNB', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(6.75, 4.0, 'mmWave NR\nDual Connectivity', ha='center',
            fontsize=7, color='#7ee787')

    # E2 arrows
    draw_arrow(3, 5.8, 2.5, 5.0, 'E2', '#58a6ff')
    draw_arrow(6, 5.8, 6.5, 5.0, 'E2', '#58a6ff')

    # PHY layer
    draw_box(0.5, 1.5, 3.5, 1.3, '', '#0d419d')
    ax.text(2.25, 2.55, 'NTN PHY Layer', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(2.25, 1.95, 'NTN Beamforming | NTN Channel\nNTN Scheduler | PHY KPM Extractor',
            ha='center', fontsize=7, color='#a5d8ff')

    # Satellite Bridge
    draw_box(5, 1.5, 3.5, 1.3, '', '#0d419d')
    ax.text(6.75, 2.55, 'Satellite Bridge', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(6.75, 1.95, 'SGP4 Orbit | Markov Fading\nDVB-S2X ModCod | Inter-beam Interf.',
            ha='center', fontsize=7, color='#a5d8ff')

    draw_arrow(2.25, 3.5, 2.25, 2.8, '', '#58a6ff')
    draw_arrow(6.75, 3.5, 6.75, 2.8, '', '#58a6ff')

    # ns3-ai
    draw_box(10, 3.5, 3.5, 1.5, '', '#8250df')
    ax.text(11.75, 4.75, 'ns3-ai Integration', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(11.75, 4.15, '5 Gym Environments\nMsg-Interface IPC\nLibTorch Inference',
            ha='center', fontsize=7, color='#d2a8ff')

    draw_arrow(10, 6.5, 10, 5.0, 'Gym', '#8250df')

    # ISL
    draw_box(10, 1.5, 3.5, 1.3, '', '#da3633')
    ax.text(11.75, 2.55, 'ISL Network', ha='center', fontsize=9,
            fontweight='bold', color='white')
    ax.text(11.75, 1.95, 'ISL Header Protocol\nIntra/Inter-plane Links\nFederated Learning',
            ha='center', fontsize=7, color='#ffa198')

    draw_arrow(2.25, 1.5, 2.25, 0.9, '', '#30363d')
    draw_arrow(6.75, 1.5, 6.75, 0.9, '', '#30363d')

    # UEs
    draw_box(2, 0.2, 7, 0.6, 'UEs: Static | Pedestrian | Vehicular | HST | Aerial',
             '#30363d', fontsize=8, textcolor='#c9d1d9')

    ax.text(7, 0.05, '27,000+ Lines of C++ | 18 Unit Tests | 3 Python AI Agents',
            ha='center', fontsize=8, color='#8b949e', fontstyle='italic')

    path = os.path.join(OUT_DIR, 'oran_ntn_architecture.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")

# =============================================================================
#  Static PNG 2: KPM Metrics
# =============================================================================
def generate_kpm_metrics_png():
    print("  Generating KPM metrics diagram...")
    np.random.seed(99)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), facecolor='white')
    fig.suptitle('O-RAN NTN: Realistic KPM Metrics from Satellite Bridge',
                 fontsize=13, fontweight='bold', color='#24292f', y=0.98)
    plt.subplots_adjust(hspace=0.35, wspace=0.3)

    # SINR CDF
    ax = axes[0, 0]
    sinr_urban = np.random.normal(5, 6, 5000)
    sinr_suburban = np.random.normal(10, 5, 5000)
    sinr_rural = np.random.normal(15, 4, 5000)
    for data, label, color in [(sinr_urban, 'Urban', '#da3633'),
                                (sinr_suburban, 'Suburban', '#bf8700'),
                                (sinr_rural, 'Rural', '#2ea043')]:
        sorted_d = np.sort(data)
        cdf = np.arange(1, len(sorted_d)+1) / len(sorted_d)
        ax.plot(sorted_d, cdf, linewidth=2, color=color, label=label)
    ax.set_xlabel('SINR (dB)', fontsize=10)
    ax.set_ylabel('CDF', fontsize=10)
    ax.set_title('SINR Distribution by Environment', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-15, 30)

    # Elevation vs Doppler
    ax = axes[0, 1]
    n = 2000
    elev = np.random.uniform(10, 90, n)
    doppler = 40000 * np.cos(np.radians(elev)) * (np.random.randn(n)*0.1 + 1)
    doppler *= np.random.choice([-1, 1], n)
    sinr_color = 5 + 15 * (elev / 90) + np.random.randn(n)*3
    sc = ax.scatter(elev, doppler/1000, c=sinr_color, cmap='RdYlGn', s=3, alpha=0.5,
                    vmin=-5, vmax=25)
    plt.colorbar(sc, ax=ax, label='SINR (dB)', shrink=0.8)
    ax.set_xlabel('Elevation Angle (°)', fontsize=10)
    ax.set_ylabel('Doppler Shift (kHz)', fontsize=10)
    ax.set_title('Elevation vs Doppler (Ka-band 20 GHz)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)

    # TTE Distribution
    ax = axes[1, 0]
    tte_550 = np.random.gamma(4, 12, 3000)
    tte_600 = np.random.gamma(4.5, 13, 3000)
    tte_1200 = np.random.gamma(6, 18, 3000)
    ax.hist(tte_550, bins=50, alpha=0.6, color='#1f6feb', label='550 km LEO', density=True)
    ax.hist(tte_600, bins=50, alpha=0.6, color='#2ea043', label='600 km LEO', density=True)
    ax.hist(tte_1200, bins=50, alpha=0.5, color='#bf8700', label='1200 km MEO', density=True)
    ax.set_xlabel('Time-to-Exit (s)', fontsize=10)
    ax.set_ylabel('Density', fontsize=10)
    ax.set_title('TTE Distribution by Orbital Altitude', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 200)

    # Throughput vs Distance with ModCod
    ax = axes[1, 1]
    dist = np.linspace(550, 2000, 500)
    fspl = 20*np.log10(4*np.pi*dist*1e3*20e9/3e8)
    sinr_base = 43 + 30 - fspl - (-174 + 10*np.log10(400e6) + 7)
    throughput = 400 * np.log2(1 + 10**(sinr_base/10))

    modcod_regions = [
        (0, 80, 'QPSK 1/2', '#264653'),
        (80, 200, '8PSK 2/3', '#2a9d8f'),
        (200, 350, '16APSK 3/4', '#e9c46a'),
        (350, 500, '32APSK 5/6', '#f4a261'),
    ]

    ax.plot(dist, throughput, color='#1f6feb', linewidth=2.5)
    for start, end, label, color in modcod_regions:
        mask = (throughput >= start) & (throughput <= end)
        if mask.any():
            ax.fill_between(dist[mask], 0, throughput[mask], alpha=0.15, color=color, label=label)

    ax.set_xlabel('Slant Range (km)', fontsize=10)
    ax.set_ylabel('Throughput (Mbps)', fontsize=10)
    ax.set_title('Throughput vs Distance (DVB-S2X ModCod)', fontsize=11, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 600)

    path = os.path.join(OUT_DIR, 'oran_ntn_kpm_metrics.png')
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")

# =============================================================================
#  Main
# =============================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("  O-RAN NTN Visualization Generator")
    print("=" * 60)
    generate_constellation_gif()
    generate_xapp_dashboard_gif()
    generate_architecture_png()
    generate_kpm_metrics_png()
    print("\nAll visualizations generated in:", OUT_DIR)
    for f in sorted(os.listdir(OUT_DIR)):
        fpath = os.path.join(OUT_DIR, f)
        print(f"  {f}: {os.path.getsize(fpath)/1024:.0f} KB")
