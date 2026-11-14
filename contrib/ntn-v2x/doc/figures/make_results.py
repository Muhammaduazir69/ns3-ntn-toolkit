#!/usr/bin/env python3
"""Results figures for the ntn-v2x README: page renders (200 dpi) of figures of the study built on this module.

The figures are drawn by that study's own scripts from stored ns-3 outputs; this file only renders them.
Source PDFs: papers/paper8_v2x_sidelink_relay/figures/ in the toolkit workspace.
Usage: python3 make_results.py [WORKSPACE]   (needs pdftoppm from poppler-utils)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
SRC = os.path.join(WS, "papers", "paper8_v2x_sidelink_relay", "figures")
MAP = {"fig3_model": "ntn_v2x_model_vs_ns3","fig4_period": "ntn_v2x_period_capacity"}

for pdf, png in MAP.items():
    subprocess.run(["pdftoppm", "-png", "-r", "200", "-singlefile", os.path.join(SRC, pdf + ".pdf"),
                    os.path.join(HERE, png)], check=True)
    print("wrote", png + ".png")
