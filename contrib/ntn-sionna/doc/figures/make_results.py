#!/usr/bin/env python3
"""Results figures of ntn-sionna, rendered from the stored study figures (no new simulation).

Source PDFs: papers/paper9_sionna_urban_ntn/figures/ in the toolkit tree; their data and scripts are in papers/paper9_sionna_urban_ntn/sim/.
Each PDF is rasterized at 200 dpi with pdftoppm (poppler-utils).

    nice -n 19 python3 make_results.py [toolkit_root]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, *[".."] * 5))
SRC = os.path.join(ROOT, "papers", "paper9_sionna_urban_ntn", "figures")
FIGS = {"fig2_los": "ntn-sionna-los-probability", "fig4_loss": "ntn-sionna-excess-loss"}

for src, dst in FIGS.items():
    out = os.path.join(HERE, dst)
    subprocess.run(["pdftoppm", "-png", "-r", "200", "-singlefile", os.path.join(SRC, src + ".pdf"), out], check=True)
    print("wrote", out + ".png")
