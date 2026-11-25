#!/usr/bin/env python3
# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

"""Add width/height (from the viewBox) to seqdiag-generated SVGs.

seqdiag emits an <svg> with a viewBox but no width/height, so the image has no
intrinsic size; embedded via <img> with the docs' height:auto CSS it collapses
and does not render. Injecting width/height from the viewBox gives it intrinsic
dimensions so it renders (and still scales to the container)."""

import re
import sys

for path in sys.argv[1:]:
    svg = open(path).read()
    head = svg.split(">", 1)[0] if ">" in svg else svg
    if " width=" in head and " height=" in head:
        continue
    m = re.search(r'viewBox="0 0 ([0-9.]+) ([0-9.]+)"', svg)
    if not m:
        continue
    svg = re.sub(r"<svg ", f'<svg width="{m.group(1)}" height="{m.group(2)}" ', svg, count=1)
    open(path, "w").write(svg)
