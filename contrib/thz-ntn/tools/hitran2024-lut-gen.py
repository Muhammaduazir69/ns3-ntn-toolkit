#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2026 Muhammad Uzair
"""Sub-THz absorption LUT generator for thz-ntn (Roadmap §4.3.1).

NOTE ON LABELLING: the bundled CSV is a continuum-approximation curve, NOT a
HITRAN-2024 line-by-line product. The sampled grid is smooth and does not
resolve the 183/325/380 GHz water-vapour peaks, so the output is tagged
"continuum-approximation" rather than "HITRAN-2024 line-by-line".

Reproduces, in pure Python, the toolkit's in-process Van Vleck-Weisskopf +
P.676-13-calibrated continuum model
(contrib/thz-ntn/model/thz-ntn-molecular-absorption.cc) so the resulting
CSV is byte-identical to what the C++ ThzNtnMolecularAbsorption would
compute. Used to regenerate the bundled
   contrib/thz-ntn/data/hitran2024-lut-subthz.csv

Output columns:
   freq_ghz, alt_km, attenuation_db_per_km

Future work (§4.3.1 follow-up): accept a HITRAN-2024 .par file via --par
and ingest the full 61-molecule line database. The v2.1 baseline keeps the
toolkit's 23-line approximation since per-line deltas between HITRAN-2020
and HITRAN-2024 are < 0.5% (Gordon et al., 2026).
"""
from __future__ import annotations

import argparse
import math
import sys
from typing import Iterable

# ---------------------------------------------------------------------------
#  Constants — exact match for thz-ntn-molecular-absorption.cc
# ---------------------------------------------------------------------------

BOLTZMANN = 1.380649e-23     # J/K
PLANCK = 6.62607015e-34      # J s
SPEED_OF_LIGHT = 2.99792458e8  # m/s
NP_TO_DB = 4.342944819

HITRAN_REF_TEMP = 296.0         # K
HITRAN_REF_PRESSURE = 1013.25   # hPa

CONTINUUM_K_REF = 0.012      # Np/km at f_ref (P.676-13 calibrated)
CONTINUUM_F_REF = 100.0e9    # Hz
CONTINUUM_P_REF = 1013.25    # hPa
CONTINUUM_T_REF = 296.0      # K
CONTINUUM_TEMP_EXPONENT = 3.0

S_SI_FACTOR = 4.979e-20      # m^2 Hz per molecule, from S_HITRAN

LAYERS = [
    (0.0,   2.0,   288.15, 1013.25, 7.5),
    (2.0,   5.0,   275.0,   800.0,  4.0),
    (5.0,  10.0,   255.0,   550.0,  1.0),
    (10.0, 20.0,   220.0,   250.0,  0.01),
    (20.0, 50.0,   250.0,    50.0,  0.001),
    (50.0, 100.0,  260.0,     0.5,  0.0),
]

# (center_Hz, intensity, half_width_Hz, is_h2o) — same 23 lines as
# InitAbsorptionLines() in the C++ implementation.
LINES = [
    (556.936e9,  2.36e-18, 2.85e9, True),
    (752.033e9,  6.10e-19, 2.68e9, True),
    (1097.365e9, 1.53e-18, 2.95e9, True),
    (1228.789e9, 8.44e-19, 2.72e9, True),
    (1410.618e9, 1.12e-18, 2.88e9, True),
    (1602.219e9, 4.25e-19, 2.61e9, True),
    (1716.770e9, 3.87e-19, 2.54e9, True),
    (380.197e9,  1.27e-19, 2.80e9, True),
    (448.001e9,  2.56e-19, 2.75e9, True),
    (620.701e9,  1.44e-19, 2.70e9, True),
    (916.172e9,  4.08e-19, 2.82e9, True),
    (970.315e9,  3.15e-19, 2.78e9, True),
    (1153.127e9, 5.92e-19, 2.90e9, True),
    (1669.904e9, 7.18e-19, 2.58e9, True),
    (118.750e9,  9.40e-20, 1.58e9, False),
    (368.498e9,  2.14e-20, 1.45e9, False),
    (424.763e9,  7.63e-20, 1.52e9, False),
    (487.249e9,  1.87e-20, 1.40e9, False),
    (56.265e9,   1.82e-20, 1.60e9, False),
    (62.486e9,   2.04e-20, 1.58e9, False),
    (368.498e9,  2.14e-20, 1.45e9, False),
    (487.249e9,  1.87e-20, 1.40e9, False),
    (715.393e9,  6.79e-20, 1.55e9, False),
]


def atmospheric_conditions(alt_km: float) -> tuple[float, float, float]:
    for lo, hi, t, p, h in LAYERS:
        if lo <= alt_km < hi:
            return t, p, h
    return LAYERS[-1][2], LAYERS[-1][3], LAYERS[-1][4]


def van_vleck_weisskopf(f_hz: float, fc_hz: float, gamma_hz: float) -> float:
    dfm = f_hz - fc_hz
    dfp = f_hz + fc_hz
    term1 = gamma_hz / (dfm * dfm + gamma_hz * gamma_hz)
    term2 = gamma_hz / (dfp * dfp + gamma_hz * gamma_hz)
    return (f_hz / fc_hz) * (f_hz / fc_hz) * (term1 + term2) / math.pi


def absorption_coefficient_np_km(f_hz: float, temp_K: float, press_hPa: float,
                                  hum_gm3: float) -> float:
    if temp_K < 1.0 or press_hPa < 1e-6:
        return 0.0
    N_total = (press_hPa * 100.0) / (BOLTZMANN * temp_K)
    N_h2o = hum_gm3 * 3.343e22
    N_o2 = 0.2095 * max(0.0, N_total - N_h2o)

    pressureScale = press_hPa / HITRAN_REF_PRESSURE
    tempScale = math.sqrt(HITRAN_REF_TEMP / max(temp_K, 1.0))

    k_lines = 0.0  # m^-1
    for fc, intensity, gamma_ref, is_h2o in LINES:
        gamma = gamma_ref * pressureScale * tempScale
        shape = van_vleck_weisskopf(f_hz, fc, gamma)
        tempCorr = pow(HITRAN_REF_TEMP / max(temp_K, 1.0), 1.5)
        expArg = -(PLANCK * fc / (2.0 * BOLTZMANN)) * (
            1.0 / max(temp_K, 1.0) - 1.0 / HITRAN_REF_TEMP)
        expArg = max(-50.0, min(50.0, expArg))
        boltz = tempCorr * math.exp(expArg)
        S_local = intensity * boltz
        N = N_h2o if is_h2o else N_o2
        S_SI = S_local * S_SI_FACTOR
        k_lines += N * S_SI * shape

    k_lines_np_km = k_lines * 1.0e3  # m^-1 -> Np/km

    freqRatio = f_hz / CONTINUUM_F_REF
    k_cont = (freqRatio * freqRatio *
              (press_hPa / CONTINUUM_P_REF) *
              pow(CONTINUUM_T_REF / max(temp_K, 1.0),
                   CONTINUUM_TEMP_EXPONENT) *
              CONTINUUM_K_REF)
    h2oFrac = N_h2o / N_total if N_total > 0.0 else 0.0
    k_cont *= 0.2 + 0.8 * h2oFrac / 0.01
    k_cont = max(0.0, k_cont)
    return k_lines_np_km + k_cont


def absorption_db_per_km(f_hz: float, alt_km: float) -> float:
    t, p, h = atmospheric_conditions(alt_km)
    return absorption_coefficient_np_km(f_hz, t, p, h) * NP_TO_DB


def generate(freqs_ghz: Iterable[float], alts_km: Iterable[float],
             release: str = "continuum-approximation (NOT line-by-line; smooth fit, no 183/325/380 GHz peaks)") -> str:
    out = [
        f"# release: {release}",
        "# generated by contrib/thz-ntn/tools/hitran2024-lut-gen.py",
        "# columns: freq_ghz, alt_km, attenuation_db_per_km",
        "freq_ghz,alt_km,attenuation_db_per_km",
    ]
    for f in freqs_ghz:
        for h in alts_km:
            out.append(f"{f:.3f},{h:.2f},{absorption_db_per_km(f * 1e9, h):.6f}")
    return "\n".join(out) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--out", required=True)
    p.add_argument("--freqs", default="100:500:10",
                   help="freq grid spec start:stop:step (GHz)")
    p.add_argument("--alts", default="0:30:1",
                   help="alt grid spec start:stop:step (km)")
    p.add_argument("--par", default=None,
                   help="HITRAN-2024 .par path (not yet implemented)")
    args = p.parse_args(argv)

    def grid(spec: str) -> list[float]:
        start, stop, step = (float(x) for x in spec.split(":"))
        n = int(round((stop - start) / step)) + 1
        return [start + i * step for i in range(n)]

    if args.par is not None:
        sys.stderr.write(
            "Warning: --par HITRAN-2024 .par ingest not implemented in v2.1; "
            "falling back to the built-in line + continuum approximation. "
            "Output is a continuum-approximation curve (NOT line-by-line; it "
            "does not resolve the 183/325/380 GHz peaks).\n")

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(generate(grid(args.freqs), grid(args.alts)))
    print(f"wrote LUT to {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
