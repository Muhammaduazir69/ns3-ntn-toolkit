#!/usr/bin/env python3
"""
NTN-CHO Analysis Toolkit - Publication-Ready Figure Generation

Generates all paper figures comparing CHO algorithms:
  Fig 1: HO Success/Failure Rate bar chart (all algorithms)
  Fig 2: Ping-Pong Rate bar chart
  Fig 3: Time-of-Stay CDF
  Fig 4: SINR distribution (CDF) per algorithm
  Fig 5: Handover timeline per UE
  Fig 6: KPI timeseries evolution
  Fig 7: Satellite visibility map (geographic)
  Fig 8: Handover event geographic distribution

Author: Muhammad Uzair
License: GPL-2.0

Usage: python3 analyze_results.py --datadir ntn-cho-output --output figures/
"""

import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import matplotlib.ticker as ticker

# Publication style
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'legend.fontsize': 9,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
})

# Color scheme
COLORS = {
    'tte-aware': '#0077CC',
    'location': '#FF8800',
    'a3': '#CC0044',
    'time': '#00AA44',
    'tte-adaptive': '#9933CC'
}
ALGO_LABELS = {
    'tte-aware': 'TTE-Aware CHO (Proposed)',
    'location': 'Location-Only CHO',
    'a3': 'Baseline A3',
    'time': 'Time-Based CHO',
    'tte-adaptive': 'TTE-Adaptive CHO'
}


def load_algorithm_data(datadir, algo):
    """Load all CSV data for an algorithm."""
    adir = os.path.join(datadir, algo)
    data = {}

    for fname in ['handover_events.csv', 'measurements.csv', 'ue_tracks.csv',
                   'kpi_timeseries.csv', 'tte_computations.csv', 'satellite_tracks.csv']:
        fpath = os.path.join(adir, fname)
        if os.path.exists(fpath):
            try:
                df = pd.read_csv(fpath)
                if len(df) > 0:
                    data[fname.replace('.csv', '')] = df
            except Exception as e:
                print(f"  Warning: Could not load {fpath}: {e}")

    # Parse KPI summary
    kpi_path = os.path.join(adir, 'kpi_summary.txt')
    if os.path.exists(kpi_path):
        kpi = {}
        with open(kpi_path) as f:
            for line in f:
                if ':' in line:
                    parts = line.strip().split(':')
                    if len(parts) == 2:
                        key = parts[0].strip()
                        val = parts[1].strip().replace('%', '').replace(' s', '')
                        try:
                            kpi[key] = float(val)
                        except ValueError:
                            kpi[key] = val
        data['kpi'] = kpi

    return data


def fig1_ho_success_failure(all_data, output):
    """Bar chart: HO Success Rate and Failure Rate per algorithm."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

    algos = list(all_data.keys())
    labels = [ALGO_LABELS.get(a, a) for a in algos]
    colors = [COLORS.get(a, '#666') for a in algos]

    success_rates = []
    failure_rates = []
    for a in algos:
        kpi = all_data[a].get('kpi', {})
        success_rates.append(kpi.get('HO Success Rate', 0))
        failure_rates.append(kpi.get('HO Failure Rate', 0))

    x = np.arange(len(algos))
    bars1 = ax1.bar(x, success_rates, color=colors, edgecolor='black', linewidth=0.5)
    ax1.set_ylabel('HO Success Rate (%)')
    ax1.set_title('(a) Handover Success Rate')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=25, ha='right', fontsize=8)
    ax1.set_ylim(0, 110)
    for bar, val in zip(bars1, success_rates):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                f'{val:.1f}%', ha='center', va='bottom', fontsize=9)

    bars2 = ax2.bar(x, failure_rates, color=colors, edgecolor='black', linewidth=0.5)
    ax2.set_ylabel('HO Failure Rate (%)')
    ax2.set_title('(b) Handover Failure Rate')
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=25, ha='right', fontsize=8)
    ax2.set_ylim(0, max(failure_rates + [10]) * 1.3)
    for bar, val in zip(bars2, failure_rates):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{val:.1f}%', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig1_ho_success_failure.pdf'))
    plt.savefig(os.path.join(output, 'fig1_ho_success_failure.png'))
    plt.close()
    print("  Fig 1: HO Success/Failure Rate")


def fig2_pingpong(all_data, output):
    """Bar chart: Ping-Pong Rate."""
    fig, ax = plt.subplots(figsize=(6, 4))

    algos = list(all_data.keys())
    labels = [ALGO_LABELS.get(a, a) for a in algos]
    colors = [COLORS.get(a, '#666') for a in algos]

    pp_rates = [all_data[a].get('kpi', {}).get('Ping-Pong Rate', 0) for a in algos]
    total_hos = [all_data[a].get('kpi', {}).get('Total Handovers', 0) for a in algos]

    x = np.arange(len(algos))
    bars = ax.bar(x, pp_rates, color=colors, edgecolor='black', linewidth=0.5)
    ax.set_ylabel('Ping-Pong Rate (%)')
    ax.set_title('Ping-Pong Handover Rate')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=25, ha='right', fontsize=8)

    for bar, val, total in zip(bars, pp_rates, total_hos):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                f'{val:.1f}%\n(n={int(total)})', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig2_pingpong_rate.pdf'))
    plt.savefig(os.path.join(output, 'fig2_pingpong_rate.png'))
    plt.close()
    print("  Fig 2: Ping-Pong Rate")


def fig3_tos_cdf(all_data, output):
    """CDF of Time-of-Stay per algorithm."""
    fig, ax = plt.subplots(figsize=(7, 5))

    for algo, data in all_data.items():
        ho_df = data.get('handover_events')
        if ho_df is not None and 'time_of_stay_s' in ho_df.columns:
            tos = ho_df['time_of_stay_s'].dropna().sort_values()
            if len(tos) > 0:
                cdf = np.arange(1, len(tos) + 1) / len(tos)
                ax.plot(tos, cdf, label=ALGO_LABELS.get(algo, algo),
                       color=COLORS.get(algo, '#666'), linewidth=2)

    ax.set_xlabel('Time-of-Stay (s)')
    ax.set_ylabel('CDF')
    ax.set_title('CDF of Post-Handover Time-of-Stay')
    ax.legend(loc='lower right')
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.05)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig3_tos_cdf.pdf'))
    plt.savefig(os.path.join(output, 'fig3_tos_cdf.png'))
    plt.close()
    print("  Fig 3: Time-of-Stay CDF")


def fig4_sinr_distribution(all_data, output):
    """CDF of SINR measurements per algorithm."""
    fig, ax = plt.subplots(figsize=(7, 5))

    for algo, data in all_data.items():
        meas_df = data.get('measurements')
        if meas_df is not None and 'sinr_dB' in meas_df.columns:
            sinr = meas_df['sinr_dB'].dropna().sort_values()
            if len(sinr) > 0:
                cdf = np.arange(1, len(sinr) + 1) / len(sinr)
                ax.plot(sinr, cdf, label=ALGO_LABELS.get(algo, algo),
                       color=COLORS.get(algo, '#666'), linewidth=2)

    ax.set_xlabel('SINR (dB)')
    ax.set_ylabel('CDF')
    ax.set_title('CDF of Measured SINR')
    ax.legend(loc='lower right')
    ax.set_ylim(0, 1.05)
    ax.axvline(x=-3, color='red', linestyle='--', alpha=0.5, label='Quality Threshold')

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig4_sinr_cdf.pdf'))
    plt.savefig(os.path.join(output, 'fig4_sinr_cdf.png'))
    plt.close()
    print("  Fig 4: SINR Distribution CDF")


def fig5_ho_timeline(all_data, output):
    """Handover timeline showing events over simulation time."""
    fig, ax = plt.subplots(figsize=(10, 4))

    for algo, data in all_data.items():
        ho_df = data.get('handover_events')
        if ho_df is not None and 'time_s' in ho_df.columns:
            # Cumulative HO count over time
            times = ho_df['time_s'].sort_values()
            cumulative = np.arange(1, len(times) + 1)
            ax.plot(times, cumulative, label=ALGO_LABELS.get(algo, algo),
                   color=COLORS.get(algo, '#666'), linewidth=2)

    ax.set_xlabel('Simulation Time (s)')
    ax.set_ylabel('Cumulative Handovers')
    ax.set_title('Handover Event Timeline')
    ax.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig5_ho_timeline.pdf'))
    plt.savefig(os.path.join(output, 'fig5_ho_timeline.png'))
    plt.close()
    print("  Fig 5: HO Timeline")


def fig6_kpi_timeseries(all_data, output):
    """KPI evolution over time."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    for algo, data in all_data.items():
        kpi_df = data.get('kpi_timeseries')
        if kpi_df is not None:
            color = COLORS.get(algo, '#666')
            label = ALGO_LABELS.get(algo, algo)

            if 'ho_success_rate' in kpi_df.columns:
                axes[0].plot(kpi_df['time_s'], kpi_df['ho_success_rate'],
                           color=color, label=label, linewidth=1.5)
            if 'avg_sinr_dB' in kpi_df.columns:
                axes[1].plot(kpi_df['time_s'], kpi_df['avg_sinr_dB'],
                           color=color, label=label, linewidth=1.5)

    axes[0].set_ylabel('HO Success Rate (%)')
    axes[0].set_title('KPI Evolution Over Time')
    axes[0].legend(loc='lower left', fontsize=8)
    axes[0].set_ylim(0, 110)

    axes[1].set_xlabel('Simulation Time (s)')
    axes[1].set_ylabel('Average SINR (dB)')
    axes[1].legend(loc='lower left', fontsize=8)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig6_kpi_timeseries.pdf'))
    plt.savefig(os.path.join(output, 'fig6_kpi_timeseries.png'))
    plt.close()
    print("  Fig 6: KPI Timeseries")


def fig7_geographic_coverage(all_data, output):
    """Geographic distribution of satellite visibility."""
    fig, ax = plt.subplots(figsize=(10, 6))

    # Use first algorithm's satellite tracks
    first_algo = list(all_data.keys())[0]
    sat_df = all_data[first_algo].get('satellite_tracks')

    if sat_df is not None:
        for sat_id in sat_df['sat_id'].unique():
            sat_track = sat_df[sat_df['sat_id'] == sat_id]
            ax.plot(sat_track['lon'], sat_track['lat'], linewidth=0.5, alpha=0.6,
                   label=f'Sat {sat_id}' if sat_id < 6 else None)

    # Plot UE positions
    ue_df = all_data[first_algo].get('ue_tracks')
    if ue_df is not None:
        # First position of each UE
        first_pos = ue_df.groupby('ue_id').first()
        ax.scatter(first_pos['lon'], first_pos['lat'], c='red', s=20,
                  zorder=5, label='UE Positions', edgecolors='black', linewidth=0.5)

    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.set_title('Satellite Ground Tracks & UE Distribution')
    ax.legend(loc='upper right', fontsize=7, ncol=2)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig7_geographic_coverage.pdf'))
    plt.savefig(os.path.join(output, 'fig7_geographic_coverage.png'))
    plt.close()
    print("  Fig 7: Geographic Coverage")


def fig8_ho_geographic(all_data, output):
    """Geographic distribution of handover events."""
    fig, ax = plt.subplots(figsize=(10, 6))

    for algo, data in all_data.items():
        ho_df = data.get('handover_events')
        if ho_df is not None and 'ue_lat' in ho_df.columns:
            success = ho_df[ho_df['success'] == 1]
            failed = ho_df[ho_df['success'] == 0]
            pp = ho_df[ho_df['ping_pong'] == 1]

            if len(success) > 0:
                ax.scatter(success['ue_lon'], success['ue_lat'], c=COLORS.get(algo, '#666'),
                          s=15, alpha=0.6, label=f'{ALGO_LABELS.get(algo, algo)} (ok)')
            if len(failed) > 0:
                ax.scatter(failed['ue_lon'], failed['ue_lat'], c='red', marker='x',
                          s=30, label=f'{ALGO_LABELS.get(algo, algo)} (fail)')

    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.set_title('Geographic Distribution of Handover Events')
    ax.legend(loc='upper right', fontsize=7)

    plt.tight_layout()
    plt.savefig(os.path.join(output, 'fig8_ho_geographic.pdf'))
    plt.savefig(os.path.join(output, 'fig8_ho_geographic.png'))
    plt.close()
    print("  Fig 8: HO Geographic Distribution")


def generate_summary_table(all_data, output):
    """Generate LaTeX-ready summary table."""
    rows = []
    for algo, data in all_data.items():
        kpi = data.get('kpi', {})
        ho_df = data.get('handover_events')
        total = kpi.get('Total Handovers', 0)
        success = kpi.get('HO Success Rate', 0)
        failure = kpi.get('HO Failure Rate', 0)
        pp = kpi.get('Ping-Pong Rate', 0)
        tos = kpi.get('Avg Time-of-Stay', '-')

        rows.append({
            'Algorithm': ALGO_LABELS.get(algo, algo),
            'Total HOs': int(total),
            'Success (%)': f'{success:.1f}',
            'Failure (%)': f'{failure:.1f}',
            'Ping-Pong (%)': f'{pp:.1f}',
            'Avg ToS (s)': tos
        })

    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(output, 'kpi_comparison_table.csv'), index=False)

    # LaTeX table
    with open(os.path.join(output, 'kpi_comparison_table.tex'), 'w') as f:
        f.write('\\begin{table}[htbp]\n\\centering\n')
        f.write('\\caption{Comparative Performance of CHO Algorithms}\n')
        f.write('\\label{tab:kpi_comparison}\n')
        f.write('\\begin{tabular}{lccccc}\n\\hline\n')
        f.write('Algorithm & Total HOs & Success (\\%) & Failure (\\%) & Ping-Pong (\\%) & Avg ToS (s) \\\\\n\\hline\n')
        for _, row in df.iterrows():
            f.write(f"{row['Algorithm']} & {row['Total HOs']} & {row['Success (%)']} & "
                   f"{row['Failure (%)']} & {row['Ping-Pong (%)']} & {row['Avg ToS (s)']} \\\\\n")
        f.write('\\hline\n\\end{tabular}\n\\end{table}\n')

    print("  Table: KPI comparison (CSV + LaTeX)")


def main():
    parser = argparse.ArgumentParser(description='NTN-CHO Analysis Toolkit')
    parser.add_argument('--datadir', default='ntn-cho-output', help='Root data directory')
    parser.add_argument('--output', default='ntn-cho-figures', help='Output directory for figures')
    parser.add_argument('--algorithms', nargs='+', default=None,
                       help='Algorithms to compare (default: auto-detect)')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # Auto-detect algorithms
    if args.algorithms:
        algos = args.algorithms
    else:
        algos = [d for d in os.listdir(args.datadir)
                if os.path.isdir(os.path.join(args.datadir, d)) and
                os.path.exists(os.path.join(args.datadir, d, 'kpi_summary.txt'))]
        algos.sort()

    if not algos:
        print(f"No algorithm datasets found in {args.datadir}/")
        sys.exit(1)

    print(f"NTN-CHO Analysis Toolkit")
    print(f"========================")
    print(f"Data directory: {args.datadir}")
    print(f"Algorithms: {', '.join(algos)}")
    print(f"Output: {args.output}/")
    print()

    # Load all data
    all_data = {}
    for algo in algos:
        print(f"Loading {algo}...")
        all_data[algo] = load_algorithm_data(args.datadir, algo)

    print(f"\nGenerating figures...")
    fig1_ho_success_failure(all_data, args.output)
    fig2_pingpong(all_data, args.output)
    fig3_tos_cdf(all_data, args.output)
    fig4_sinr_distribution(all_data, args.output)
    fig5_ho_timeline(all_data, args.output)
    fig6_kpi_timeseries(all_data, args.output)
    fig7_geographic_coverage(all_data, args.output)
    fig8_ho_geographic(all_data, args.output)
    generate_summary_table(all_data, args.output)

    print(f"\nAll figures saved to {args.output}/")
    print(f"Generated: 8 figures (PDF + PNG) + comparison table (CSV + LaTeX)")


if __name__ == '__main__':
    main()
