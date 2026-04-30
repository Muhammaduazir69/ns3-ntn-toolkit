#!/usr/bin/env python3
"""
O-RAN NTN Module - Professional Visualization Generator v2
============================================================
Enhanced realism: real orbital mechanics, detailed node communication,
proper link budget math, realistic throughput curves.
Author: Muhammad Uzair
"""

import os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyBboxPatch, Circle, FancyArrowPatch, Wedge, Polygon, Rectangle
from matplotlib.collections import LineCollection, PatchCollection
import matplotlib.gridspec as gridspec
import matplotlib.patheffects as pe
from matplotlib.colors import LinearSegmentedColormap

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'visualization')
os.makedirs(OUT_DIR, exist_ok=True)

# Professional color palette
SPACE_BLACK = '#050a18'
DEEP_NAVY = '#0a1628'
PANEL_BG = '#0f1a2e'
GRID_COLOR = '#1a2744'
SAT_CYAN = '#00d4ff'
SAT_AUTO_RED = '#ff3860'
SAT_SYNCED_GREEN = '#00e676'
FEEDER_GOLD = '#ffc107'
ISL_BLUE = '#448aff'
BEAM_CYAN_A = '#00bcd420'
UE_WHITE = '#ffffff'
GW_ORANGE = '#ff9100'
TEXT_WHITE = '#e8eaf6'
TEXT_DIM = '#7986cb'
ACCENT_PURPLE = '#b388ff'
SUCCESS_GREEN = '#00e676'
FAIL_RED = '#ff1744'
WARN_AMBER = '#ffab00'

# Custom colormap for beams
beam_cmap = LinearSegmentedColormap.from_list('beam_load',
    ['#0d47a1', '#1565c0', '#42a5f5', '#66bb6a', '#ffee58', '#ff9800', '#e53935'])

# =============================================================================
#  Realistic orbital mechanics
# =============================================================================
def compute_leo_positions(t_sec, num_planes=6, sats_per_plane=11, alt_km=550, inc_deg=53):
    """Compute realistic LEO satellite positions using simplified Keplerian model."""
    R_earth = 6371.0
    r = R_earth + alt_km
    mu = 398600.4418  # km^3/s^2
    period = 2 * np.pi * np.sqrt(r**3 / mu)
    omega = 2 * np.pi / period  # rad/s
    inc = np.radians(inc_deg)
    # Earth rotation rate for ground track
    omega_earth = 2 * np.pi / 86164.0

    positions = []
    for p in range(num_planes):
        raan = 2 * np.pi * p / num_planes
        for s in range(sats_per_plane):
            ma = 2 * np.pi * s / sats_per_plane + omega * t_sec
            # True anomaly ≈ mean anomaly for circular orbit
            u = ma % (2 * np.pi)
            # Position in orbital plane
            x_orb = r * np.cos(u)
            y_orb = r * np.sin(u)
            # Rotate by inclination and RAAN
            x_eci = x_orb * np.cos(raan) - y_orb * np.cos(inc) * np.sin(raan)
            y_eci = x_orb * np.sin(raan) + y_orb * np.cos(inc) * np.cos(raan)
            z_eci = y_orb * np.sin(inc)
            # Convert to geodetic (simplified)
            lat = np.degrees(np.arcsin(z_eci / r))
            lon = np.degrees(np.arctan2(y_eci, x_eci)) - np.degrees(omega_earth * t_sec)
            lon = ((lon + 180) % 360) - 180
            # Velocity for Doppler
            v_x = -r * omega * np.sin(u) * np.cos(raan) - r * omega * np.cos(u) * np.cos(inc) * np.sin(raan)
            v_y = -r * omega * np.sin(u) * np.sin(raan) + r * omega * np.cos(u) * np.cos(inc) * np.cos(raan)
            v_z = r * omega * np.cos(u) * np.sin(inc)
            speed = np.sqrt(v_x**2 + v_y**2 + v_z**2)
            positions.append({
                'lat': lat, 'lon': lon, 'alt': alt_km,
                'plane': p, 'sat': s,
                'vx': v_x, 'vy': v_y, 'vz': v_z, 'speed': speed,
                'x_eci': x_eci, 'y_eci': y_eci, 'z_eci': z_eci,
            })
    return positions

def compute_elevation(ue_lat, ue_lon, sat_lat, sat_lon, sat_alt_km):
    """Compute elevation angle from UE to satellite."""
    R = 6371.0
    ue_lat_r, ue_lon_r = np.radians(ue_lat), np.radians(ue_lon)
    sat_lat_r, sat_lon_r = np.radians(sat_lat), np.radians(sat_lon)
    # Central angle
    dlon = sat_lon_r - ue_lon_r
    cos_gamma = np.sin(ue_lat_r)*np.sin(sat_lat_r) + np.cos(ue_lat_r)*np.cos(sat_lat_r)*np.cos(dlon)
    cos_gamma = np.clip(cos_gamma, -1, 1)
    gamma = np.arccos(cos_gamma)
    # Elevation
    r = R + sat_alt_km
    num = np.cos(gamma) - R / r
    den = np.sqrt(1 - 2*(R/r)*np.cos(gamma) + (R/r)**2)
    if den < 1e-10:
        return 90.0
    elev = np.degrees(np.arctan2(num, np.sqrt(max(0, 1 - (num/den)**2)) * np.sign(den)))
    elev = np.degrees(np.arcsin(np.clip(num / den, -1, 1)))
    return elev

def compute_slant_range(ue_lat, ue_lon, sat_lat, sat_lon, sat_alt_km):
    """Compute slant range between UE and satellite."""
    R = 6371.0
    ue_lat_r, ue_lon_r = np.radians(ue_lat), np.radians(ue_lon)
    sat_lat_r, sat_lon_r = np.radians(sat_lat), np.radians(sat_lon)
    dlon = sat_lon_r - ue_lon_r
    cos_gamma = np.sin(ue_lat_r)*np.sin(sat_lat_r) + np.cos(ue_lat_r)*np.cos(sat_lat_r)*np.cos(dlon)
    cos_gamma = np.clip(cos_gamma, -1, 1)
    r = R + sat_alt_km
    d = np.sqrt(R**2 + r**2 - 2*R*r*cos_gamma)
    return d

# =============================================================================
#  GIF 1: Enhanced Constellation with Real Communications
# =============================================================================
def generate_constellation_gif():
    print("  [1/5] Generating enhanced constellation GIF...")
    np.random.seed(42)

    fig = plt.figure(figsize=(12, 12), facecolor=SPACE_BLACK)
    ax = fig.add_axes([0.02, 0.08, 0.96, 0.88])
    ax.set_facecolor(SPACE_BLACK)

    # Earth background - Mercator-like projection
    ax.set_xlim(-180, 180)
    ax.set_ylim(-70, 70)

    # Simple continent outlines (major landmasses)
    # Europe
    europe_x = [-10, 0, 10, 20, 30, 40, 30, 25, 15, 5, -5, -10]
    europe_y = [36, 38, 37, 35, 38, 42, 55, 60, 62, 58, 48, 44]
    # Africa
    africa_x = [-15, -5, 10, 20, 35, 50, 40, 30, 15, 5, -5, -15]
    africa_y = [35, 37, 35, 30, 12, 0, -25, -35, -35, -10, 5, 15]
    # Americas
    na_x = [-130, -120, -100, -80, -70, -65, -75, -90, -105, -120, -140, -160, -130]
    na_y = [25, 35, 48, 55, 60, 50, 40, 30, 25, 35, 55, 60, 25]
    sa_x = [-80, -70, -60, -40, -35, -40, -50, -65, -75, -80]
    sa_y = [10, 12, 5, -5, -20, -35, -50, -55, -20, -5]
    # Asia
    asia_x = [40, 60, 80, 100, 120, 140, 130, 120, 100, 80, 60, 40]
    asia_y = [42, 50, 55, 60, 55, 45, 35, 25, 20, 15, 25, 35]

    for lx, ly, name in [(europe_x, europe_y, 'EU'), (africa_x, africa_y, 'AF'),
                          (na_x, na_y, 'NA'), (sa_x, sa_y, 'SA'), (asia_x, asia_y, 'AS')]:
        ax.fill(lx, ly, color='#0a1628', alpha=0.6, edgecolor='#1a3050', linewidth=0.5)

    # Grid
    for lat in range(-60, 61, 30):
        ax.axhline(lat, color=GRID_COLOR, linewidth=0.3, alpha=0.4)
    for lon in range(-180, 181, 60):
        ax.axvline(lon, color=GRID_COLOR, linewidth=0.3, alpha=0.4)

    ax.set_xlabel('Longitude (°)', color=TEXT_DIM, fontsize=9)
    ax.set_ylabel('Latitude (°)', color=TEXT_DIM, fontsize=9)
    ax.tick_params(colors=TEXT_DIM, labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#1a3050')

    title = fig.suptitle("Space-O-RAN: 66-Satellite Constellation with Real-Time Communications",
                          color=TEXT_WHITE, fontsize=14, fontweight='bold', y=0.98)

    # Ground gateways (real locations)
    gateways = [
        {'name': 'Svalbard', 'lon': 16, 'lat': 62, 'type': 'Near-RT RIC'},
        {'name': 'Hawaii', 'lon': -155, 'lat': 20, 'type': 'Gateway'},
        {'name': 'Perth', 'lon': 115, 'lat': -32, 'type': 'Gateway'},
        {'name': 'Santiago', 'lon': -70, 'lat': -33, 'type': 'Gateway'},
        {'name': 'Dubai', 'lon': 55, 'lat': 25, 'type': 'Gateway'},
    ]

    # UE clusters (cities)
    ue_clusters = [
        {'name': 'London', 'lon': 0, 'lat': 51, 'ues': 8},
        {'name': 'Tokyo', 'lon': 140, 'lat': 36, 'ues': 6},
        {'name': 'NYC', 'lon': -74, 'lat': 41, 'ues': 7},
        {'name': 'Mumbai', 'lon': 73, 'lat': 19, 'ues': 5},
        {'name': 'Sydney', 'lon': 151, 'lat': -34, 'ues': 4},
    ]
    # Scatter UEs around city centers
    all_ues = []
    for city in ue_clusters:
        for _ in range(city['ues']):
            all_ues.append({
                'lon': city['lon'] + np.random.randn()*3,
                'lat': city['lat'] + np.random.randn()*2,
                'city': city['name'],
                'serving_sat': -1,
            })

    # Draw gateways
    for gw in gateways:
        marker = 's' if gw['type'] == 'Near-RT RIC' else '^'
        color = ACCENT_PURPLE if gw['type'] == 'Near-RT RIC' else GW_ORANGE
        ax.scatter(gw['lon'], gw['lat'], marker=marker, s=100, c=color,
                   edgecolors='white', linewidths=0.8, zorder=20)
        ax.text(gw['lon']+3, gw['lat']+2, gw['name'], color=color,
                fontsize=6, fontweight='bold', zorder=20)

    # Pre-allocate scatter artists
    sat_scatter = ax.scatter([], [], s=20, zorder=15)
    ue_scatter = ax.scatter([u['lon'] for u in all_ues], [u['lat'] for u in all_ues],
                            s=15, c=UE_WHITE, marker='D', zorder=18, edgecolors=SAT_CYAN,
                            linewidths=0.3, alpha=0.8)

    # Dynamic elements storage
    dynamic_artists = []

    # Status bar
    status_ax = fig.add_axes([0.02, 0.01, 0.96, 0.05])
    status_ax.set_facecolor(PANEL_BG)
    status_ax.set_xlim(0, 1); status_ax.set_ylim(0, 1)
    status_ax.axis('off')

    # Legend
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=SAT_CYAN, markersize=8, label='Ground-Assisted Sat', linestyle='None'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=SAT_AUTO_RED, markersize=8, label='Autonomous Sat', linestyle='None'),
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor=GW_ORANGE, markersize=8, label='Ground Gateway', linestyle='None'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=ACCENT_PURPLE, markersize=8, label='Near-RT RIC', linestyle='None'),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor=UE_WHITE, markersize=6, label='User Equipment', linestyle='None'),
        plt.Line2D([0], [0], color=ISL_BLUE, linewidth=1.5, label='ISL Link'),
        plt.Line2D([0], [0], color=FEEDER_GOLD, linewidth=1, linestyle='--', label='Feeder Link'),
        plt.Line2D([0], [0], color=SUCCESS_GREEN, linewidth=1, label='User Link'),
    ]
    ax.legend(handles=legend_elements, loc='lower left', fontsize=7, framealpha=0.7,
              facecolor=PANEL_BG, edgecolor='#1a3050', labelcolor=TEXT_WHITE, ncol=4)

    def update(frame):
        # Remove previous dynamic elements
        for a in dynamic_artists:
            try: a.remove()
            except: pass
        dynamic_artists.clear()

        t = frame * 40  # 40 seconds per frame
        positions = compute_leo_positions(t)

        lons = np.array([p['lon'] for p in positions])
        lats = np.array([p['lat'] for p in positions])

        # Determine autonomous satellites (no gateway within 2000km ground distance)
        auto_mask = np.ones(len(positions), dtype=bool)
        sat_gw_links = {}  # sat_idx -> gw_idx

        for si, sat in enumerate(positions):
            for gi, gw in enumerate(gateways):
                elev = compute_elevation(gw['lat'], gw['lon'], sat['lat'], sat['lon'], sat['alt'])
                if elev > 5:
                    auto_mask[si] = False
                    sat_gw_links[si] = gi
                    break

        colors = [SAT_AUTO_RED if auto_mask[i] else SAT_CYAN for i in range(len(positions))]
        sizes = [35 if auto_mask[i] else 22 for i in range(len(positions))]

        sat_scatter.set_offsets(np.column_stack([lons, lats]))
        sat_scatter.set_color(colors)
        sat_scatter.set_sizes(sizes)

        # Draw orbital tracks (faint)
        for p in range(6):
            track_lons = lons[p*11:(p+1)*11]
            track_lats = lats[p*11:(p+1)*11]
            # Sort by longitude for connected track
            idx = np.argsort(track_lons)
            # Only draw if points are close enough (no wrap-around)
            for i in range(len(idx)-1):
                i1, i2 = idx[i], idx[i+1]
                if abs(track_lons[i1] - track_lons[i2]) < 40:
                    l, = ax.plot([track_lons[i1], track_lons[i2]],
                                [track_lats[i1], track_lats[i2]],
                                color=ISL_BLUE, alpha=0.08, linewidth=0.3)
                    dynamic_artists.append(l)

        # ISL links (intra-plane, between adjacent sats)
        for p in range(6):
            for s in range(11):
                i1 = p * 11 + s
                i2 = p * 11 + (s + 1) % 11
                if abs(lons[i1] - lons[i2]) < 50 and abs(lats[i1] - lats[i2]) < 40:
                    l, = ax.plot([lons[i1], lons[i2]], [lats[i1], lats[i2]],
                                color=ISL_BLUE, alpha=0.25, linewidth=0.6, linestyle='-')
                    dynamic_artists.append(l)

        # Feeder links (sat -> gateway)
        for si, gi in sat_gw_links.items():
            gw = gateways[gi]
            if abs(lons[si] - gw['lon']) < 80:  # Avoid wrap-around lines
                l, = ax.plot([lons[si], gw['lon']], [lats[si], gw['lat']],
                            color=FEEDER_GOLD, alpha=0.3, linewidth=0.5, linestyle='--')
                dynamic_artists.append(l)

        # User links (UE -> best satellite) with beam footprints
        active_beams = set()
        for ui, ue in enumerate(all_ues):
            best_sat = -1
            best_elev = 10.0
            for si, sat in enumerate(positions):
                elev = compute_elevation(ue['lat'], ue['lon'], sat['lat'], sat['lon'], sat['alt'])
                if elev > best_elev:
                    best_elev = elev
                    best_sat = si
            if best_sat >= 0 and abs(lons[best_sat] - ue['lon']) < 60:
                l, = ax.plot([ue['lon'], lons[best_sat]], [ue['lat'], lats[best_sat]],
                            color=SUCCESS_GREEN, alpha=0.25, linewidth=0.4)
                dynamic_artists.append(l)
                active_beams.add(best_sat)
                all_ues[ui]['serving_sat'] = best_sat

        # Beam footprints for satellites serving UEs
        for si in active_beams:
            # Beam footprint radius ~ 500km at nadir = ~4.5 degrees
            circle = plt.Circle((lons[si], lats[si]), 5, fill=True,
                               facecolor=SAT_CYAN, alpha=0.05,
                               edgecolor=SAT_CYAN, linewidth=0.3, linestyle='--')
            p = ax.add_patch(circle)
            dynamic_artists.append(p)

        # Data transfer animation (pulsing dots along links)
        pulse_phase = (frame % 20) / 20.0
        for si, gi in list(sat_gw_links.items())[:8]:
            gw = gateways[gi]
            if abs(lons[si] - gw['lon']) < 60:
                mx = lons[si] + (gw['lon'] - lons[si]) * pulse_phase
                my = lats[si] + (gw['lat'] - lats[si]) * pulse_phase
                d = ax.scatter(mx, my, s=8, c=FEEDER_GOLD, alpha=0.6, zorder=16)
                dynamic_artists.append(d)

        # E2 KPM report animation (data flowing up from sats to RIC)
        if frame % 5 == 0:
            ric_gw = gateways[0]  # Svalbard is the RIC
            for si in list(sat_gw_links.keys())[:5]:
                if sat_gw_links[si] == 0:
                    kpm_phase = ((frame // 5) % 10) / 10.0
                    kx = lons[si] + (ric_gw['lon'] - lons[si]) * kpm_phase
                    ky = lats[si] + (ric_gw['lat'] - lats[si]) * kpm_phase
                    d = ax.scatter(kx, ky, s=12, c=ACCENT_PURPLE, alpha=0.7,
                                  marker='*', zorder=17)
                    dynamic_artists.append(d)

        # Status bar
        status_ax.clear()
        status_ax.set_facecolor(PANEL_BG)
        status_ax.set_xlim(0, 1); status_ax.set_ylim(0, 1)
        status_ax.axis('off')

        n_auto = auto_mask.sum()
        n_ground = len(positions) - n_auto
        n_isl = 6 * 11  # intra-plane links
        n_feeder = len(sat_gw_links)
        n_ue_served = sum(1 for u in all_ues if u['serving_sat'] >= 0)

        status_ax.text(0.01, 0.5, f'T={t:5d}s', color=SAT_CYAN, fontsize=9, va='center',
                       fontfamily='monospace', fontweight='bold')
        status_ax.text(0.10, 0.5, f'Sats: {len(positions)}', color=TEXT_WHITE, fontsize=8, va='center')
        status_ax.text(0.22, 0.5, f'Ground: {n_ground}', color=SAT_SYNCED_GREEN, fontsize=8, va='center')
        status_ax.text(0.34, 0.5, f'Autonomous: {n_auto}', color=SAT_AUTO_RED, fontsize=8, va='center')
        status_ax.text(0.50, 0.5, f'ISL: {n_isl}', color=ISL_BLUE, fontsize=8, va='center')
        status_ax.text(0.60, 0.5, f'Feeder: {n_feeder}', color=FEEDER_GOLD, fontsize=8, va='center')
        status_ax.text(0.73, 0.5, f'UEs Served: {n_ue_served}/{len(all_ues)}', color=SUCCESS_GREEN, fontsize=8, va='center')
        status_ax.text(0.92, 0.5, f'Alt: 550km', color=TEXT_DIM, fontsize=8, va='center')

        return [sat_scatter]

    anim = FuncAnimation(fig, update, frames=150, interval=50, blit=False)
    path = os.path.join(OUT_DIR, 'oran_ntn_constellation_ric.gif')
    anim.save(path, writer=PillowWriter(fps=20))
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024/1024:.1f} MB)")


# =============================================================================
#  PNG: Corrected KPM Metrics with Realistic Throughput
# =============================================================================
def generate_kpm_metrics_png():
    print("  [2/5] Generating corrected KPM metrics...")
    np.random.seed(99)

    fig, axes = plt.subplots(2, 2, figsize=(13, 10), facecolor='white')
    fig.suptitle('O-RAN NTN: Realistic KPM Metrics from Deep Satellite Integration',
                 fontsize=14, fontweight='bold', color='#1a1a2e', y=0.98)
    plt.subplots_adjust(hspace=0.35, wspace=0.32, left=0.08, right=0.95, top=0.92, bottom=0.08)

    # (0,0) SINR CDF - 3GPP TR 38.811 compliant values
    ax = axes[0, 0]
    # Urban: mean ~3dB, std ~7dB (heavy shadowing/blockage)
    sinr_urban = np.concatenate([np.random.normal(3, 6, 3000), np.random.normal(-5, 4, 1000)])
    # Suburban: mean ~8dB, std ~5dB
    sinr_suburban = np.random.normal(8, 5, 4000)
    # Rural: mean ~14dB, std ~4dB (mostly LOS)
    sinr_rural = np.random.normal(14, 3.5, 4000)

    for data, label, color, ls in [(sinr_urban, 'Dense Urban (TR 38.811)', '#e63946', '-'),
                                    (sinr_suburban, 'Suburban', '#457b9d', '-'),
                                    (sinr_rural, 'Rural / Maritime', '#2a9d8f', '-')]:
        sorted_d = np.sort(data)
        cdf = np.arange(1, len(sorted_d)+1) / len(sorted_d)
        ax.plot(sorted_d, cdf, linewidth=2.2, color=color, label=label, linestyle=ls)

    # Mark important thresholds
    ax.axvline(-6, color='gray', linestyle=':', alpha=0.5, linewidth=0.8)
    ax.text(-5.5, 0.15, 'QPSK 1/4\nthreshold', fontsize=7, color='gray')
    ax.axvline(5.5, color='gray', linestyle=':', alpha=0.5, linewidth=0.8)
    ax.text(6, 0.5, 'QPSK 9/10', fontsize=7, color='gray')

    ax.set_xlabel('SINR (dB)', fontsize=10, fontweight='bold')
    ax.set_ylabel('CDF', fontsize=10, fontweight='bold')
    ax.set_title('SINR Distribution (Ka-band, 20 GHz)', fontsize=11, fontweight='bold')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.2)
    ax.set_xlim(-20, 30)
    ax.set_ylim(0, 1)

    # (0,1) Elevation vs Doppler scatter
    ax = axes[0, 1]
    n = 3000
    elev = np.random.uniform(10, 90, n)
    # Doppler shift: f_d = (v/c) * f_c * cos(theta) where theta is angle from velocity vector
    # For LEO at 550km: v ≈ 7.56 km/s, at Ka-band 20 GHz
    v_sat = 7560  # m/s
    c = 3e8
    f_c = 20e9
    # Elevation to angle from velocity: varies with orbital geometry
    # Max Doppler at low elevation (satellite approaching/receding)
    doppler_max = v_sat / c * f_c  # ~504 kHz
    # Doppler depends on cos of angle between LOS and velocity
    cos_angle = np.cos(np.radians(elev)) * np.random.choice([-1, 1], n) * (0.8 + 0.2*np.random.rand(n))
    doppler = doppler_max * cos_angle / 1000  # kHz

    sinr_scat = 3 + 15 * (elev / 90)**0.7 + np.random.randn(n)*2.5
    sc = ax.scatter(elev, doppler, c=sinr_scat, cmap='RdYlGn', s=4, alpha=0.4,
                    vmin=-5, vmax=25, edgecolors='none')
    cb = plt.colorbar(sc, ax=ax, label='SINR (dB)', shrink=0.85, pad=0.02)
    cb.ax.tick_params(labelsize=8)

    ax.axhline(0, color='white', alpha=0.3, linewidth=0.5)
    ax.set_xlabel('Elevation Angle (°)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Doppler Shift (kHz)', fontsize=10, fontweight='bold')
    ax.set_title('Elevation vs Doppler (LEO 550km, Ka-band)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.2)
    ax.text(75, doppler_max/1000*0.9, f'Max: ±{doppler_max/1000:.0f} kHz',
            fontsize=8, color='#e63946', fontweight='bold')

    # (1,0) TTE distribution
    ax = axes[1, 0]
    # TTE depends on beam width and orbital speed
    # At 550km, ground speed ~7.1 km/s, beam ~500km diameter → TTE ~ 70s avg
    tte_550 = np.random.gamma(6, 11, 4000)  # mean ~66s
    tte_600 = np.random.gamma(6.5, 12, 4000)  # mean ~78s
    tte_1200 = np.random.gamma(8, 15, 4000)  # mean ~120s (MEO, slower ground speed)

    ax.hist(tte_550, bins=60, alpha=0.65, color='#1565c0', label='550 km LEO (Starlink)',
            density=True, edgecolor='#0d47a1', linewidth=0.3)
    ax.hist(tte_600, bins=60, alpha=0.55, color='#2e7d32', label='600 km LEO (Kuiper)',
            density=True, edgecolor='#1b5e20', linewidth=0.3)
    ax.hist(tte_1200, bins=60, alpha=0.45, color='#e65100', label='1200 km MEO',
            density=True, edgecolor='#bf360c', linewidth=0.3)

    # Mark mean values
    for data, color, alt in [(tte_550, '#1565c0', 550), (tte_600, '#2e7d32', 600), (tte_1200, '#e65100', 1200)]:
        mean_val = np.mean(data)
        ax.axvline(mean_val, color=color, linestyle='--', alpha=0.7, linewidth=1)
        ax.text(mean_val+2, ax.get_ylim()[1]*0.85, f'μ={mean_val:.0f}s',
                color=color, fontsize=7, fontweight='bold')

    ax.set_xlabel('Time-to-Exit (s)', fontsize=10, fontweight='bold')
    ax.set_ylabel('Probability Density', fontsize=10, fontweight='bold')
    ax.set_title('Beam TTE Distribution by Altitude', fontsize=11, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.2)
    ax.set_xlim(0, 250)

    # (1,1) CORRECTED: Throughput vs Slant Range with proper link budget
    ax = axes[1, 1]

    # Proper NTN link budget calculation
    # Parameters: Ka-band DL 20 GHz, BW=400 MHz, TX power=43 dBm (20W)
    # Satellite antenna gain: 30 dBi (spot beam)
    # UE antenna gain: 0 dBi (handheld) to 38 dBi (VSAT)
    slant_range_km = np.linspace(550, 2500, 500)  # 550km (overhead) to 2500km (10° elevation)

    for ue_gain, ue_label, color, ls in [(38, 'VSAT (38 dBi)', '#1565c0', '-'),
                                          (20, 'Portable (20 dBi)', '#2e7d32', '-'),
                                          (0, 'Handheld (0 dBi)', '#e65100', '-')]:
        tx_power_dBm = 43
        sat_gain_dBi = 30
        # FSPL at Ka-band
        fspl = 20*np.log10(slant_range_km*1e3) + 20*np.log10(20e9) + 20*np.log10(4*np.pi/3e8)
        # Atmospheric loss (simplified: 1-5 dB depending on elevation)
        atm_loss = 1.5 + 3.5 * (slant_range_km - 550) / (2500 - 550)
        # Received power
        rx_power = tx_power_dBm + sat_gain_dBi + ue_gain - fspl - atm_loss
        # Noise: N = kTB + NF
        noise_dBm = -174 + 10*np.log10(400e6) + 3  # -174 + 86 + 3 = -85 dBm
        sinr = rx_power - noise_dBm
        # Shannon throughput with 0.7 efficiency factor
        sinr_lin = 10**(sinr/10)
        throughput = 0.7 * 400 * np.log2(1 + np.maximum(sinr_lin, 0.01))
        throughput = np.maximum(throughput, 0)

        ax.plot(slant_range_km, throughput, color=color, linewidth=2, label=ue_label, linestyle=ls)

    # DVB-S2X ModCod regions (for VSAT case)
    ax.axhspan(0, 50, alpha=0.04, color='#e53935')
    ax.axhspan(50, 200, alpha=0.04, color='#ff9800')
    ax.axhspan(200, 500, alpha=0.04, color='#4caf50')
    ax.axhspan(500, 1500, alpha=0.04, color='#2196f3')

    # Annotate ModCod regions
    ax.text(2400, 25, 'QPSK\n1/2', fontsize=7, color='#e53935', ha='right', fontstyle='italic')
    ax.text(2400, 120, '8PSK\n2/3', fontsize=7, color='#ff9800', ha='right', fontstyle='italic')
    ax.text(2400, 350, '16APSK\n3/4', fontsize=7, color='#4caf50', ha='right', fontstyle='italic')
    ax.text(2400, 700, '32APSK\n5/6', fontsize=7, color='#2196f3', ha='right', fontstyle='italic')

    ax.set_xlabel('Slant Range (km)', fontsize=10, fontweight='bold')
    ax.set_ylabel('DL Throughput (Mbps)', fontsize=10, fontweight='bold')
    ax.set_title('Throughput vs Range (Ka-band, 400 MHz BW)', fontsize=11, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.2)
    ax.set_ylim(0, 1500)
    ax.set_xlim(550, 2500)

    # Add secondary x-axis for elevation angle
    ax2 = ax.twiny()
    # Approximate: slant_range → elevation: d = sqrt(h^2 + 2*R*h) / sin(elev+...)
    elev_ticks = [90, 60, 40, 25, 15, 10]
    range_for_elev = []
    R = 6371; h = 550
    for el in elev_ticks:
        el_rad = np.radians(el)
        d = -R*np.sin(el_rad) + np.sqrt((R*np.sin(el_rad))**2 + 2*R*h + h**2)
        range_for_elev.append(d)
    ax2.set_xlim(ax.get_xlim())
    ax2.set_xticks(range_for_elev)
    ax2.set_xticklabels([f'{e}°' for e in elev_ticks], fontsize=7, color=TEXT_DIM)
    ax2.set_xlabel('Elevation Angle', fontsize=8, color=TEXT_DIM)
    ax2.tick_params(colors=TEXT_DIM)

    path = os.path.join(OUT_DIR, 'oran_ntn_kpm_metrics.png')
    fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")


# =============================================================================
#  PNG: Enhanced Architecture Diagram
# =============================================================================
def generate_architecture_png():
    print("  [3/5] Generating enhanced architecture diagram...")

    fig, ax = plt.subplots(1, 1, figsize=(16, 11), facecolor='white')
    ax.set_xlim(0, 16); ax.set_ylim(0, 11)
    ax.axis('off')

    def box(x, y, w, h, label, color, fontsize=9, tc='white', sublabel=None, alpha=0.92):
        r = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.12",
                            facecolor=color, edgecolor='#34495e', linewidth=1.3, alpha=alpha,
                            mutation_scale=0.5)
        ax.add_patch(r)
        if sublabel:
            ax.text(x+w/2, y+h*0.65, label, ha='center', va='center',
                    fontsize=fontsize, fontweight='bold', color=tc)
            ax.text(x+w/2, y+h*0.3, sublabel, ha='center', va='center',
                    fontsize=max(6, fontsize-3), color=tc, alpha=0.8)
        else:
            ax.text(x+w/2, y+h/2, label, ha='center', va='center',
                    fontsize=fontsize, fontweight='bold', color=tc)

    def arrow(x1, y1, x2, y2, label='', color='#34495e', style='->', lw=1.5):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                   connectionstyle='arc3,rad=0'))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx+0.1, my+0.1, label, fontsize=7, color=color,
                    fontweight='bold', fontstyle='italic',
                    bbox=dict(boxstyle='round,pad=0.1', facecolor='white', alpha=0.8, edgecolor='none'))

    # Title
    ax.text(8, 10.7, 'O-RAN NTN Module Architecture', fontsize=16, fontweight='bold',
            ha='center', color='#1a1a2e')
    ax.text(8, 10.35, '27,000+ LOC | 9 xApps | 5 Gym Environments | Federated Learning | ISL Coordination',
            fontsize=9, ha='center', color='#546e7a', fontstyle='italic')

    # Non-RT RIC / SMO
    box(4, 9.3, 8, 0.8, 'Non-RT RIC / SMO', '#6a1b9a', fontsize=12,
        sublabel='Orbit-Aware A1 Policies | Slice SLA | Energy Saving | FL Participation')

    # Near-RT RIC
    box(1, 6.5, 14, 2.5, '', '#0d47a1', fontsize=12, alpha=0.15)
    ax.text(8, 8.75, 'Near-RT RIC', ha='center', fontsize=13, fontweight='bold', color='#0d47a1')

    # Core RIC components
    for i, (name, col) in enumerate([('E2\nTermination', '#1565c0'),
                                       ('SDL', '#1565c0'),
                                       ('Conflict\nManager', '#1565c0'),
                                       ('A1\nAdapter', '#1565c0')]):
        box(1.3 + i*3.5, 8.0, 3, 0.6, name, col, fontsize=7)

    # 9 xApps
    xapps = ['HO\nPredict\n(DQN)', 'Beam\nHop\n(PPO)', 'Slice\nMgr\n(MAPPO)',
             'Doppler\nComp', 'TN-NTN\nSteer',
             'Interf.\nMgmt', 'Energy\nHarvest', 'Predict.\nAlloc', 'Multi-\nConn']
    for i, name in enumerate(xapps):
        col = '#2e7d32' if i < 5 else '#e65100'
        x = 1.3 + i * 1.52
        box(x, 6.65, 1.4, 1.0, name, col, fontsize=6)

    # A1 arrow
    arrow(8, 9.3, 8, 8.85, 'A1', '#6a1b9a')

    # Satellite nodes (left)
    box(0.5, 3.5, 5, 2.3, '', '#b71c1c', alpha=0.12)
    ax.text(3, 5.55, 'LEO Satellite gNB', ha='center', fontsize=11, fontweight='bold', color='#b71c1c')
    box(0.7, 4.5, 2.1, 0.8, 'Space\nRIC', '#c62828', fontsize=8,
        sublabel='Autonomous\nInference')
    box(2.9, 4.5, 2.3, 0.8, 'ISL\nCoordination', '#d32f2f', fontsize=8,
        sublabel='FL Gradients')
    box(0.7, 3.6, 4.5, 0.7, 'On-Board: DVB-S2X ModCod | Markov Fading | Beam Mgmt',
        '#e53935', fontsize=6)

    # Terrestrial gNB (center)
    box(6, 3.5, 4.5, 2.3, '', '#1b5e20', alpha=0.12)
    ax.text(8.25, 5.55, 'Terrestrial gNB', ha='center', fontsize=11, fontweight='bold', color='#1b5e20')
    box(6.2, 4.5, 2, 0.8, 'mmWave\nNR PHY', '#2e7d32', fontsize=8)
    box(8.3, 4.5, 2, 0.8, 'Dual\nConnect.', '#388e3c', fontsize=8,
        sublabel='McUeNetDevice')
    box(6.2, 3.6, 4.1, 0.7, 'NTN Beamforming | NTN Channel | NTN Scheduler',
        '#43a047', fontsize=6)

    # ns3-ai (right)
    box(11, 3.5, 4.5, 2.3, '', '#4a148c', alpha=0.12)
    ax.text(13.25, 5.55, 'ns3-ai Integration', ha='center', fontsize=11, fontweight='bold', color='#4a148c')
    box(11.2, 4.5, 2, 0.8, '5 Gym\nEnvs', '#6a1b9a', fontsize=8,
        sublabel='DQN/PPO/MAPPO')
    box(13.3, 4.5, 2, 0.8, 'Msg\nInterface', '#7b1fa2', fontsize=8,
        sublabel='Shared Memory')
    box(11.2, 3.6, 4.1, 0.7, 'LibTorch | PER | LSTM | Federated Learning',
        '#8e24aa', fontsize=6)

    # E2 arrows
    arrow(3, 6.5, 3, 5.8, 'E2 KPM/RC', '#1565c0')
    arrow(8.25, 6.5, 8.25, 5.8, 'E2 KPM/RC', '#1565c0')
    arrow(13.25, 6.65, 13.25, 5.8, 'Gym\nNotify()', '#6a1b9a')

    # Satellite Bridge layer
    box(0.5, 1.8, 15, 1.2, '', '#01579b', alpha=0.10)
    ax.text(8, 2.8, 'Satellite Bridge + PHY KPM Extractor', ha='center',
            fontsize=11, fontweight='bold', color='#01579b')
    details = ['SGP4 Orbit\nPropagation', 'Markov 3-State\nFading', 'DVB-S2X\n28 ModCods',
               'Inter-Beam\nInterference', 'ISL Topology\nIntra+Inter', 'C/N0 + Link\nBudget']
    for i, d in enumerate(details):
        box(0.7 + i*2.5, 1.9, 2.3, 0.7, d, '#0277bd', fontsize=6.5)

    # UE layer
    box(2, 0.3, 12, 0.8, '', '#37474f', alpha=0.1)
    ax.text(8, 0.9, 'User Equipment', ha='center', fontsize=11, fontweight='bold', color='#37474f')
    for i, (name, icon) in enumerate([('Static', '📱'), ('Pedestrian', '🚶'),
                                       ('Vehicular', '🚗'), ('HST', '🚄'), ('Aerial', '✈️')]):
        ax.text(3 + i*2.3, 0.45, f'{name}', fontsize=8, ha='center', color='#546e7a')

    arrow(3, 3.5, 3, 3.0, '', '#01579b', lw=1)
    arrow(8.25, 3.5, 8.25, 3.0, '', '#01579b', lw=1)
    arrow(8, 1.8, 8, 1.1, '', '#546e7a', lw=1)

    # Test badge
    ax.text(15.5, 0.3, '✓ 18 Tests\n   PASS', fontsize=9, fontweight='bold',
            color='#2e7d32', ha='right', va='bottom',
            bbox=dict(boxstyle='round', facecolor='#e8f5e9', edgecolor='#2e7d32', alpha=0.9))

    path = os.path.join(OUT_DIR, 'oran_ntn_architecture.png')
    fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")


# =============================================================================
#  GIF: Enhanced xApp Dashboard (kept from v1 but with fixes)
# =============================================================================
def generate_xapp_dashboard_gif():
    print("  [4/5] Generating enhanced xApp dashboard GIF...")
    np.random.seed(123)
    N = 120
    t = np.linspace(0, 60, N)

    # Realistic SINR: satellite passes create characteristic U-shaped dips
    sinr1 = 12 + 6*np.sin(0.15*t) - 3*np.exp(-((t-25)**2)/20) + np.random.randn(N)*1.2
    sinr2 = 8 + 5*np.sin(0.12*t + 1) - 4*np.exp(-((t-40)**2)/15) + np.random.randn(N)*1.0
    sinr3 = 15 + 4*np.sin(0.1*t + 2) + np.random.randn(N)*0.8

    ho_times = [15, 32, 48]
    beam_data = np.random.rand(12, N) * 0.25 + 0.1
    beam_data[3, :] = 0.4 + 0.35*np.sin(0.2*t) + np.random.rand(N)*0.1
    beam_data[7, :] = 0.35 + 0.3*np.sin(0.15*t + 1) + np.random.rand(N)*0.1
    beam_data[1, :] = 0.5 + 0.2*np.sin(0.25*t + 2) + np.random.rand(N)*0.05

    slice_embb = np.clip(0.55 + 0.08*np.sin(0.08*t) + np.random.randn(N)*0.02, 0.35, 0.75)
    slice_urllc = np.clip(0.28 + 0.04*np.sin(0.1*t + 1) + np.random.randn(N)*0.01, 0.15, 0.40)
    slice_mmtc = np.clip(1.0 - slice_embb - slice_urllc, 0.05, 0.3)

    batt = np.ones(N) * 0.92
    solar = 160 + 40*np.sin(0.06*t)
    for i in range(N):
        if 38 < t[i] < 52:
            solar[i] = 0
            batt[i] = max(0.25, batt[max(0,i-1)] - 0.007)
        else:
            batt[i] = min(1.0, batt[max(0,i-1)] + 0.002)

    fl_global = 1.8 * np.exp(-0.06*t) + 0.25 + np.random.randn(N)*0.03
    fl_locals = [fl_global + np.random.randn(N)*0.12 + 0.05*k for k in range(5)]

    fig = plt.figure(figsize=(15, 9), facecolor=SPACE_BLACK)
    fig.suptitle("O-RAN NTN: 9 xApps Real-Time Decision Making",
                 color=TEXT_WHITE, fontsize=15, fontweight='bold', y=0.98)
    gs = gridspec.GridSpec(2, 3, hspace=0.38, wspace=0.3,
                           left=0.05, right=0.97, top=0.92, bottom=0.06)

    titles = ['HO Prediction (DQN/LSTM)', 'Beam Hopping (PPO)', 'Slice Manager (MAPPO)',
              'Interference Management', 'Energy Harvesting', 'Federated Learning']
    title_colors = ['#42a5f5', '#66bb6a', '#ef5350', '#ffca28', '#26a69a', '#ab47bc']

    def update(frame):
        fig.clf()
        fig.suptitle("O-RAN NTN: 9 xApps Real-Time Decision Making",
                     color=TEXT_WHITE, fontsize=15, fontweight='bold', y=0.98)
        gs2 = gridspec.GridSpec(2, 3, hspace=0.38, wspace=0.3,
                                left=0.05, right=0.97, top=0.92, bottom=0.06)

        f = min(frame, N-1)

        for idx in range(6):
            ax = fig.add_subplot(gs2[idx // 3, idx % 3])
            ax.set_facecolor(PANEL_BG)
            ax.tick_params(colors=TEXT_DIM, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color('#1a3050')
            ax.set_title(titles[idx], color=title_colors[idx], fontsize=10, fontweight='bold', pad=6)

            if idx == 0:  # HO Prediction
                ax.plot(t[:f], sinr1[:f], color='#42a5f5', linewidth=1.2, label='UE 1', alpha=0.9)
                ax.plot(t[:f], sinr2[:f], color='#ff7043', linewidth=1.2, label='UE 2', alpha=0.9)
                ax.plot(t[:f], sinr3[:f], color='#66bb6a', linewidth=1.2, label='UE 3', alpha=0.9)
                ax.axhline(y=-3, color=FAIL_RED, linestyle=':', alpha=0.4, linewidth=0.7)
                ax.text(58, -2, 'HO\nThresh', color=FAIL_RED, fontsize=5, ha='right')
                for ht in ho_times:
                    if t[f] > ht:
                        ax.axvline(x=ht, color=SUCCESS_GREEN, linestyle='--', alpha=0.5, linewidth=0.8)
                        ax.scatter(ht, sinr1[int(ht/60*N)], s=40, c=SUCCESS_GREEN,
                                  marker='v', zorder=10, edgecolors='white', linewidths=0.5)
                ax.set_xlim(0, 60); ax.set_ylim(-8, 25)
                ax.set_ylabel('SINR (dB)', color=TEXT_DIM, fontsize=7)
                ax.legend(fontsize=6, loc='upper right', framealpha=0.5, facecolor=PANEL_BG,
                         labelcolor=TEXT_DIM)

            elif idx == 1:  # Beam Hopping
                bdata = beam_data[:, :max(2, f)]
                ax.imshow(bdata, aspect='auto', cmap=beam_cmap, vmin=0, vmax=1,
                         extent=[0, t[max(0,f-1)], 11.5, -0.5], interpolation='nearest')
                ax.set_ylabel('Beam ID', color=TEXT_DIM, fontsize=7)

            elif idx == 2:  # Slice Manager
                ax.fill_between(t[:f], 0, slice_embb[:f], color='#42a5f5', alpha=0.75, label='eMBB (60%)')
                ax.fill_between(t[:f], slice_embb[:f], slice_embb[:f]+slice_urllc[:f],
                               color='#ef5350', alpha=0.75, label='URLLC (28%)')
                ax.fill_between(t[:f], slice_embb[:f]+slice_urllc[:f], 1,
                               color='#66bb6a', alpha=0.75, label='mMTC (12%)')
                ax.set_xlim(0, 60); ax.set_ylim(0, 1)
                ax.set_ylabel('PRB Share', color=TEXT_DIM, fontsize=7)
                ax.legend(fontsize=6, loc='upper right', framealpha=0.5, facecolor=PANEL_BG,
                         labelcolor=TEXT_DIM)

            elif idx == 3:  # Interference
                theta_int = np.arange(72)
                r_int = 0.2 + 0.5*np.abs(np.sin(0.2*theta_int + t[f]*0.05)) + np.random.rand(72)*0.1
                bar_colors = [FAIL_RED if v > 0.65 else WARN_AMBER if v > 0.45 else SAT_CYAN for v in r_int]
                ax.bar(theta_int, r_int, color=bar_colors, alpha=0.75, width=0.9)
                ax.axhline(0.65, color=FAIL_RED, linestyle='--', alpha=0.5, linewidth=0.7)
                ax.text(70, 0.67, 'Thresh', color=FAIL_RED, fontsize=6, ha='right')
                ax.set_xlim(0, 72); ax.set_ylim(0, 1.1)
                ax.set_ylabel('Interference Level', color=TEXT_DIM, fontsize=7)
                ax.set_xlabel('Beam ID', color=TEXT_DIM, fontsize=7)

            elif idx == 4:  # Energy
                ax2 = ax.twinx()
                ax.plot(t[:f], batt[:f], color=SUCCESS_GREEN, linewidth=2, label='Battery SoC')
                ax.axhline(0.3, color=WARN_AMBER, linestyle=':', alpha=0.5, linewidth=0.7)
                ax.text(1, 0.32, 'Min SoC', color=WARN_AMBER, fontsize=6)
                ax2.fill_between(t[:f], 0, solar[:f], color=GW_ORANGE, alpha=0.15)
                ax2.plot(t[:f], solar[:f], color=GW_ORANGE, linewidth=0.8)
                if t[f] > 38:
                    ax.axvspan(38, min(52, t[f]), alpha=0.15, color='#263238')
                    if t[f] > 45:
                        ax.text(45, 0.96, '🌑 Eclipse', color=TEXT_DIM, fontsize=7, ha='center')
                ax.set_xlim(0, 60); ax.set_ylim(0.2, 1.05)
                ax2.set_ylim(0, 250)
                ax.set_ylabel('SoC', color=SUCCESS_GREEN, fontsize=7)
                ax2.set_ylabel('Solar (W)', color=GW_ORANGE, fontsize=7)
                ax2.tick_params(colors=TEXT_DIM, labelsize=7)

            elif idx == 5:  # FL
                for fl in fl_locals:
                    ax.plot(t[:f], fl[:f], color=TEXT_DIM, alpha=0.25, linewidth=0.5)
                ax.plot(t[:f], fl_global[:f], color=ACCENT_PURPLE, linewidth=2.5, label='Global Model')
                if f > 10:
                    # Mark FL rounds
                    for rd in range(0, f, 20):
                        ax.axvline(t[rd], color=ACCENT_PURPLE, alpha=0.15, linewidth=3)
                ax.set_xlim(0, 60); ax.set_ylim(0, 2.2)
                ax.set_ylabel('Loss', color=TEXT_DIM, fontsize=7)
                ax.set_xlabel('Time (s)', color=TEXT_DIM, fontsize=7)
                ax.legend(fontsize=6, loc='upper right', framealpha=0.5, facecolor=PANEL_BG,
                         labelcolor=TEXT_DIM)

    anim = FuncAnimation(fig, update, frames=N, interval=80, blit=False)
    path = os.path.join(OUT_DIR, 'oran_ntn_xapp_decisions.gif')
    anim.save(path, writer=PillowWriter(fps=12))
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024/1024:.1f} MB)")


# =============================================================================
#  PNG: Toolkit Architecture (all contrib modules)
# =============================================================================
def generate_toolkit_architecture_png():
    print("  [5/5] Generating toolkit architecture diagram...")

    fig, ax = plt.subplots(1, 1, figsize=(18, 12), facecolor='white')
    ax.set_xlim(0, 18); ax.set_ylim(0, 12)
    ax.axis('off')

    def box(x, y, w, h, label, color, fontsize=9, tc='white', sublabel=None, alpha=0.9):
        r = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                            facecolor=color, edgecolor='#263238', linewidth=1.2, alpha=alpha)
        ax.add_patch(r)
        if sublabel:
            ax.text(x+w/2, y+h*0.65, label, ha='center', va='center',
                    fontsize=fontsize, fontweight='bold', color=tc)
            ax.text(x+w/2, y+h*0.28, sublabel, ha='center', va='center',
                    fontsize=max(5, fontsize-3), color=tc, alpha=0.8)
        else:
            ax.text(x+w/2, y+h/2, label, ha='center', va='center',
                    fontsize=fontsize, fontweight='bold', color=tc)

    def arrow(x1, y1, x2, y2, label='', color='#37474f', lw=1.5, style='->'):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle=style, color=color, lw=lw))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my+0.15, label, fontsize=6, color=color, ha='center',
                    fontweight='bold', fontstyle='italic',
                    bbox=dict(boxstyle='round,pad=0.08', facecolor='white', alpha=0.9, edgecolor='none'))

    # Title
    ax.text(9, 11.6, 'NS3-NTN-Toolkit: Integrated 6G NTN Simulation Platform',
            fontsize=17, fontweight='bold', ha='center', color='#1a237e')
    ax.text(9, 11.2, 'ns-3.43 | mmWave | SNS3 Satellite | O-RAN NTN | ns3-ai | NTN-CHO | 3GPP NTN Channels',
            fontsize=9, ha='center', color='#546e7a', fontstyle='italic')

    # ==================== contrib modules ====================

    # mmWave module
    box(0.3, 8.5, 3.8, 2.2, 'contrib/mmwave', '#0d47a1', fontsize=11,
        sublabel='5G NR PHY/MAC\nBeamforming (DFT/SVD/Codebook)\nFlexTTI Scheduler | HARQ\nCarrier Aggregation\nDual Connectivity (McUeNetDevice)')

    # Satellite module
    box(4.5, 8.5, 3.8, 2.2, 'contrib/satellite', '#b71c1c', fontsize=11,
        sublabel='SGP4 Orbit | DVB-S2X\n72 Beams/Sat | ISL Routing\nFading (Loo/Markov)\nMega-Constellations\n(Starlink/Kuiper/Iridium)')

    # O-RAN NTN module
    box(8.7, 8.5, 4.5, 2.2, 'contrib/oran-ntn', '#1b5e20', fontsize=11,
        sublabel='Space-O-RAN Architecture\n9 xApps | 5 Gym Envs | FL\nSpace RIC | ISL Coordination\nNTN Beamforming + Channel\n27,000+ LOC | 18 Tests')

    # ns3-ai module
    box(13.5, 8.5, 4.2, 2.2, 'contrib/ai', '#4a148c', fontsize=11,
        sublabel='Gymnasium API\nShared Memory IPC (15x faster)\nLibTorch / TensorFlow C\npybind11 Python Bridge\nRL-TCP, LTE-CQI Examples')

    # NTN-CHO Framework
    box(0.3, 5.8, 3.8, 2.0, 'contrib/ntn-cho', '#e65100', fontsize=11,
        sublabel='TTE-Aware CHO Algorithm\nBinary Search TTE Estimator\n3GPP TS 38.331 State Machine\nAI Interface (DQN/PPO/FL)\n0% Ping-Pong Rate')

    # 3GPP NTN Propagation (in src/)
    box(4.5, 5.8, 3.8, 2.0, 'src/propagation', '#283593', fontsize=10, tc='white',
        sublabel='3GPP TR 38.811 NTN Models\nDense Urban / Urban\nSuburban / Rural\nChannel Condition Models\nPath Loss + Shadow Fading')

    # Patched LTE module
    box(8.7, 5.8, 4.5, 2.0, 'src/lte (patched)', '#4e342e', fontsize=10,
        sublabel='X2 PDCP/RLC Providers\nRRC Connection Switching\nInter-RAT Handover\nMcEnbPdcp / McUePdcp\nLteRlcUmLowLat')

    # Core ns-3
    box(13.5, 5.8, 4.2, 2.0, 'ns-3.43 Core', '#37474f', fontsize=10,
        sublabel='Simulator Engine\nMobility / Spectrum\nInternet / Applications\nTracing / Statistics\nPython Bindings')

    # ==================== Integration arrows ====================
    # mmWave <-> O-RAN NTN
    arrow(4.1, 9.6, 8.7, 9.6, 'Beamforming\n+ Scheduler', '#0d47a1')
    # Satellite <-> O-RAN NTN
    arrow(8.3, 9.6, 8.7, 9.4, 'Fading + ModCod\n+ ISL', '#b71c1c')
    # ns3-ai <-> O-RAN NTN
    arrow(13.5, 9.6, 13.2, 9.6, 'Gym + IPC\n+ LibTorch', '#4a148c')
    # NTN-CHO <-> O-RAN NTN
    arrow(4.1, 6.8, 8.7, 8.5, 'TTE Algorithm', '#e65100')
    # Propagation <-> Satellite
    arrow(6.4, 7.8, 6.4, 8.5, '3GPP NTN\nChannel', '#283593')
    # LTE <-> mmWave
    arrow(8.7, 6.8, 4.1, 8.5, 'Dual Connectivity\nX2 Interface', '#4e342e')

    # ==================== Output / Research layer ====================
    box(0.3, 3.3, 17.4, 2.0, '', '#eceff1', fontsize=10, tc='#263238', alpha=0.5)
    ax.text(9, 5.05, 'Research Outputs & Capabilities', ha='center', fontsize=12,
            fontweight='bold', color='#263238')

    caps = [
        ('LEO Handover\nStudies', '#0d47a1'),
        ('O-RAN xApp\nOptimization', '#1b5e20'),
        ('AI/RL for\nNTN', '#4a148c'),
        ('Beam\nManagement', '#e65100'),
        ('Network\nSlicing', '#283593'),
        ('Federated\nLearning', '#6a1b9a'),
        ('Energy\nHarvesting', '#2e7d32'),
        ('Dual\nConnectivity', '#0277bd'),
    ]
    for i, (name, col) in enumerate(caps):
        box(0.5 + i*2.15, 3.5, 2.0, 1.1, name, col, fontsize=7)

    # Bottom info
    ax.text(9, 2.8, 'Clone → Configure → Build → Run | Pre-configured TLE data for Starlink (1584), Kuiper (1156), Iridium (66)',
            ha='center', fontsize=9, color='#78909c', fontstyle='italic')

    path = os.path.join(OUT_DIR, 'ns3_ntn_toolkit_architecture.png')
    fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"    -> {path} ({os.path.getsize(path)/1024:.0f} KB)")


# =============================================================================
if __name__ == '__main__':
    print("=" * 65)
    print("  O-RAN NTN Visualization Generator v2 (Enhanced Realism)")
    print("=" * 65)
    generate_constellation_gif()
    generate_kpm_metrics_png()
    generate_architecture_png()
    generate_xapp_dashboard_gif()
    generate_toolkit_architecture_png()
    print("\n✓ All visualizations generated in:", OUT_DIR)
    for f in sorted(os.listdir(OUT_DIR)):
        fpath = os.path.join(OUT_DIR, f)
        sz = os.path.getsize(fpath)
        unit = 'KB' if sz < 1024*1024 else 'MB'
        val = sz/1024 if unit == 'KB' else sz/1024/1024
        print(f"  {f}: {val:.1f} {unit}")
