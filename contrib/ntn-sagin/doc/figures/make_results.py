#!/usr/bin/env python3
"""Results figures for the ntn-sagin README: page renders (200 dpi) of figures of the study built on this module.

The figures are drawn by that study's own scripts from stored ns-3 outputs; this file only renders them.
Source PDFs: papers/paper6_iotj_sf_iot/figures/ in the toolkit workspace.
Usage: python3 make_results.py [WORKSPACE]   (needs pdftoppm from poppler-utils)
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
SRC = os.path.join(WS, "papers", "paper6_iotj_sf_iot", "figures")
MAP = {"fig4_ns3": "ntn_sagin_ns3_custody","fig5_storage": "ntn_sagin_storage"}

for pdf, png in MAP.items():
    subprocess.run(["pdftoppm", "-png", "-r", "200", "-singlefile", os.path.join(SRC, pdf + ".pdf"),
                    os.path.join(HERE, png)], check=True)
    print("wrote", png + ".png")
