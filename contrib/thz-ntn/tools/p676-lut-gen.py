#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026 Muhammad Uzair
"""ITU-R P.676-13 gaseous-absorption LUT generator for thz-ntn.

This tool is an EXACT Python mirror of the in-process absorption kernel in
``contrib/thz-ntn/model/thz-ntn-molecular-absorption.cc`` — the full ITU-R
P.676-13 Annex 1 line-by-line model (44 oxygen lines from Table 1, 35
water-vapour lines from Table 2, the standard resonant line shape, and the
dry/Debye continuum N''_D). It replaces the earlier ``hitran2024-lut-gen.py``,
which advertised "HITRAN-2024, byte-identical to the C++" but in fact (a) used
a different Van Vleck-Weisskopf exponent than the C++, (b) shipped an
inconsistent, partially duplicated line list that omitted the 183/325 GHz
water lines, and (c) applied no intra-layer pressure/humidity decay. None of
those artifacts were HITRAN data.

There is NO external data file: ITU-R P.676-13 Annex 1 is fully self-contained
and valid to 1000 GHz, so no ``.par`` ingest is needed (or offered). The output
reproduces the ITU-Rpy P.676-13 validation set to within ~2%.

The generated grid samples specific attenuation (dB/km) on a (frequency x
altitude) grid using the same ITU-R P.835 layered atmosphere and intra-layer
exponential decay (5.5 km pressure scale height, 2 km water-vapour scale
height, midlatitude-summer humidity profile) as the C++ model, so a loaded LUT
matches the in-process kernel.

Output columns:
   freq_ghz, alt_km, attenuation_db_per_km
"""
from __future__ import annotations

import argparse
import math
import sys
from typing import Iterable

# ---------------------------------------------------------------------------
#  ITU-R P.835 layered atmosphere — exact match for
#  thz-ntn-molecular-absorption.cc InitAtmosphericLayers().
#  (altLow_km, altHigh_km, T_K, P_hPa, H2O_gm3)
# ---------------------------------------------------------------------------
LAYERS = [
    (0.0,   2.0,   288.15, 1013.25, 7.5),
    (2.0,   5.0,   275.0,   800.0,  4.0),
    (5.0,  10.0,   255.0,   550.0,  1.0),
    (10.0, 20.0,   220.0,   250.0,  0.01),
    (20.0, 50.0,   250.0,    50.0,  0.001),
    (50.0, 100.0,  260.0,     0.5,  0.0),
]

PRESSURE_SCALE_HEIGHT_KM = 5.5   # matches C++ GetAtmosphericConditions
HUMIDITY_SCALE_HEIGHT_KM = 2.0

# ---------------------------------------------------------------------------
#  ITU-R P.676-13 Table 1: oxygen lines (f0_GHz, a1..a6)
# ---------------------------------------------------------------------------
O2_LINES = [
    (50.474214, 0.975, 9.651, 6.69, 0, 2.566, 6.85),
    (50.987745, 2.529, 8.653, 7.17, 0, 2.246, 6.8),
    (51.503360, 6.193, 7.709, 7.64, 0, 1.947, 6.729),
    (52.021429, 14.32, 6.819, 8.11, 0, 1.667, 6.64),
    (52.542418, 31.24, 5.983, 8.58, 0, 1.388, 6.526),
    (53.066934, 64.29, 5.201, 9.06, 0, 1.349, 6.206),
    (53.595775, 124.6, 4.474, 9.55, 0, 2.227, 5.085),
    (54.130025, 227.3, 3.8, 9.96, 0, 3.17, 3.75),
    (54.671180, 389.7, 3.182, 10.37, 0, 3.558, 2.654),
    (55.221384, 627.1, 2.618, 10.89, 0, 2.56, 2.952),
    (55.783815, 945.3, 2.109, 11.34, 0, -1.172, 6.135),
    (56.264774, 543.4, 0.014, 17.03, 0, 3.525, -0.978),
    (56.363399, 1331.8, 1.654, 11.89, 0, -2.378, 6.547),
    (56.968211, 1746.6, 1.255, 12.23, 0, -3.545, 6.451),
    (57.612486, 2120.1, 0.91, 12.62, 0, -5.416, 6.056),
    (58.323877, 2363.7, 0.621, 12.95, 0, -1.932, 0.436),
    (58.446588, 1442.1, 0.083, 14.91, 0, 6.768, -1.273),
    (59.164204, 2379.9, 0.387, 13.53, 0, -6.561, 2.309),
    (59.590983, 2090.7, 0.207, 14.08, 0, 6.957, -0.776),
    (60.306056, 2103.4, 0.207, 14.15, 0, -6.395, 0.699),
    (60.434778, 2438, 0.386, 13.39, 0, 6.342, -2.825),
    (61.150562, 2479.5, 0.621, 12.92, 0, 1.014, -0.584),
    (61.800158, 2275.9, 0.91, 12.63, 0, 5.014, -6.619),
    (62.411220, 1915.4, 1.255, 12.17, 0, 3.029, -6.759),
    (62.486253, 1503, 0.083, 15.13, 0, -4.499, 0.844),
    (62.997984, 1490.2, 1.654, 11.74, 0, 1.856, -6.675),
    (63.568526, 1078, 2.108, 11.34, 0, 0.658, -6.139),
    (64.127775, 728.7, 2.617, 10.88, 0, -3.036, -2.895),
    (64.678910, 461.3, 3.181, 10.38, 0, -3.968, -2.59),
    (65.224078, 274, 3.8, 9.96, 0, -3.528, -3.68),
    (65.764779, 153, 4.473, 9.55, 0, -2.548, -5.002),
    (66.302096, 80.4, 5.2, 9.06, 0, -1.66, -6.091),
    (66.836834, 39.8, 5.982, 8.58, 0, -1.68, -6.393),
    (67.369601, 18.56, 6.818, 8.11, 0, -1.956, -6.475),
    (67.900868, 8.172, 7.708, 7.64, 0, -2.216, -6.545),
    (68.431006, 3.397, 8.652, 7.17, 0, -2.492, -6.6),
    (68.960312, 1.334, 9.65, 6.69, 0, -2.773, -6.65),
    (118.750334, 940.3, 0.01, 16.64, 0, -0.439, 0.079),
    (368.498246, 67.4, 0.048, 16.4, 0, 0, 0),
    (424.763020, 637.7, 0.044, 16.4, 0, 0, 0),
    (487.249273, 237.4, 0.049, 16, 0, 0, 0),
    (715.392902, 98.1, 0.145, 16, 0, 0, 0),
    (773.839490, 572.3, 0.141, 16.2, 0, 0, 0),
    (834.145546, 183.1, 0.145, 14.7, 0, 0, 0),
]

# ---------------------------------------------------------------------------
#  ITU-R P.676-13 Table 2: water-vapour lines (f0_GHz, b1..b6)
# ---------------------------------------------------------------------------
H2O_LINES = [
    (22.235080, 0.1079, 2.144, 26.38, 0.76, 5.087, 1),
    (67.803960, 0.0011, 8.732, 28.58, 0.69, 4.93, 0.82),
    (119.995940, 0.0007, 8.353, 29.48, 0.7, 4.78, 0.79),
    (183.310087, 2.273, 0.668, 29.06, 0.77, 5.022, 0.85),
    (321.225630, 0.047, 6.179, 24.04, 0.67, 4.398, 0.54),
    (325.152888, 1.514, 1.541, 28.23, 0.64, 4.893, 0.74),
    (336.227764, 0.001, 9.825, 26.93, 0.69, 4.74, 0.61),
    (380.197353, 11.67, 1.048, 28.11, 0.54, 5.063, 0.89),
    (390.134508, 0.0045, 7.347, 21.52, 0.63, 4.81, 0.55),
    (437.346667, 0.0632, 5.048, 18.45, 0.6, 4.23, 0.48),
    (439.150807, 0.9098, 3.595, 20.07, 0.63, 4.483, 0.52),
    (443.018343, 0.192, 5.048, 15.55, 0.6, 5.083, 0.5),
    (448.001085, 10.41, 1.405, 25.64, 0.66, 5.028, 0.67),
    (470.888999, 0.3254, 3.597, 21.34, 0.66, 4.506, 0.65),
    (474.689092, 1.26, 2.379, 23.2, 0.65, 4.804, 0.64),
    (488.490108, 0.2529, 2.852, 25.86, 0.69, 5.201, 0.72),
    (503.568532, 0.0372, 6.731, 16.12, 0.61, 3.98, 0.43),
    (504.482692, 0.0124, 6.731, 16.12, 0.61, 4.01, 0.45),
    (547.676440, 0.9785, 0.158, 26, 0.7, 4.5, 1),
    (552.020960, 0.184, 0.158, 26, 0.7, 4.5, 1),
    (556.935985, 497, 0.159, 30.86, 0.69, 4.552, 1),
    (620.700807, 5.015, 2.391, 24.38, 0.71, 4.856, 0.68),
    (645.766085, 0.0067, 8.633, 18, 0.6, 4, 0.5),
    (658.005280, 0.2732, 7.816, 32.1, 0.69, 4.14, 1),
    (752.033113, 243.4, 0.396, 30.86, 0.68, 4.352, 0.84),
    (841.051732, 0.0134, 8.177, 15.9, 0.33, 5.76, 0.45),
    (859.965698, 0.1325, 8.055, 30.6, 0.68, 4.09, 0.84),
    (899.303175, 0.0547, 7.914, 29.85, 0.68, 4.53, 0.9),
    (902.611085, 0.0386, 8.429, 28.65, 0.7, 5.1, 0.95),
    (906.205957, 0.1836, 5.11, 24.08, 0.7, 4.7, 0.53),
    (916.171582, 8.4, 1.441, 26.73, 0.7, 5.15, 0.78),
    (923.112692, 0.0079, 10.293, 29, 0.7, 5, 0.8),
    (970.315022, 9.009, 1.919, 25.5, 0.64, 4.94, 0.67),
    (987.926764, 134.6, 0.257, 29.85, 0.68, 4.55, 0.9),
    (1780.000000, 17506, 0.952, 196.3, 2, 24.15, 5),
]


def atmospheric_conditions(alt_km: float) -> tuple[float, float, float]:
    """Local (T[K], P_total[hPa], rho[g/m^3]) — mirrors the C++
    GetAtmosphericConditions() with intra-layer exponential decay and the
    default midlatitude-summer humidity profile (scale 1.0)."""
    if alt_km >= 100.0:
        return 210.0, 0.001, 0.0
    for lo, hi, t, p, h in LAYERS:
        if lo <= alt_km < hi:
            press = p * math.exp(-(alt_km - lo) / PRESSURE_SCALE_HEIGHT_KM)
            hum = h * math.exp(-(alt_km - lo) / HUMIDITY_SCALE_HEIGHT_KM)
            return t, press, hum
    return 288.15, 1013.25, 7.5


def specific_attenuation_db_km(f_ghz: float, temp_K: float, press_hPa: float,
                               hum_gm3: float) -> float:
    """ITU-R P.676-13 Annex 1 specific attenuation (dB/km). Identical maths to
    ThzNtnMolecularAbsorption::ComputeAbsorptionCoefficient()."""
    if temp_K < 1.0 or press_hPa < 1e-6:
        return 0.0
    theta = 300.0 / temp_K
    e = max(0.0, hum_gm3) * temp_K / 216.7   # water-vapour partial pressure
    p = max(0.0, press_hPa - e)              # dry-air partial pressure
    f = f_ghz

    npp = 0.0
    for (f0, a1, a2, a3, a4, a5, a6) in O2_LINES:
        Si = a1 * 1e-7 * p * theta ** 3 * math.exp(a2 * (1.0 - theta))
        df = a3 * 1e-4 * (p * theta ** (0.8 - a4) + 1.1 * e * theta)
        df = math.sqrt(df * df + 2.25e-6)
        delta = (a5 + a6 * theta) * 1e-4 * (p + e) * theta ** 0.8
        fm, fp = f0 - f, f0 + f
        Fi = (f / f0) * ((df - delta * fm) / (fm * fm + df * df) +
                         (df - delta * fp) / (fp * fp + df * df))
        npp += Si * Fi
    for (f0, b1, b2, b3, b4, b5, b6) in H2O_LINES:
        Si = b1 * 1e-1 * e * theta ** 3.5 * math.exp(b2 * (1.0 - theta))
        df = b3 * 1e-4 * (p * theta ** b4 + b5 * e * theta ** b6)
        df = 0.535 * df + math.sqrt(0.217 * df * df + 2.1316e-12 * f0 * f0 / theta)
        fm, fp = f0 - f, f0 + f
        Fi = (f / f0) * (df / (fm * fm + df * df) + df / (fp * fp + df * df))
        npp += Si * Fi

    d0 = 5.6e-4 * (p + e) * theta ** 0.8
    nd = f * p * theta * theta * (
        6.14e-5 / (d0 * (1.0 + (f / d0) ** 2)) +
        1.4e-12 * p * theta ** 1.5 / (1.0 + 1.9e-5 * f ** 1.5))
    npp += nd

    return max(0.0, 0.1820 * f * npp)


def absorption_db_per_km(f_ghz: float, alt_km: float) -> float:
    t, p, h = atmospheric_conditions(alt_km)
    return specific_attenuation_db_km(f_ghz, t, p, h)


def generate(freqs_ghz: Iterable[float], alts_km: Iterable[float],
             release: str = "ITU-R-P.676-13") -> str:
    out = [
        f"# release: {release}",
        "# generated by contrib/thz-ntn/tools/p676-lut-gen.py",
        "# model: ITU-R P.676-13 Annex 1 line-by-line (44 O2 + 35 H2O lines)",
        "# columns: freq_ghz, alt_km, attenuation_db_per_km",
        "freq_ghz,alt_km,attenuation_db_per_km",
    ]
    for f in freqs_ghz:
        for h in alts_km:
            out.append(f"{f:.3f},{h:.2f},{absorption_db_per_km(f, h):.6f}")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--out", required=True)
    p.add_argument("--freqs", default="100:500:10",
                   help="freq grid spec start:stop:step (GHz)")
    p.add_argument("--alts", default="0:30:1",
                   help="alt grid spec start:stop:step (km)")
    args = p.parse_args(argv)

    def grid(spec: str) -> list[float]:
        start, stop, step = (float(x) for x in spec.split(":"))
        n = int(round((stop - start) / step)) + 1
        return [start + i * step for i in range(n)]

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(generate(grid(args.freqs), grid(args.alts)))
    print(f"wrote ITU-R P.676-13 LUT to {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
