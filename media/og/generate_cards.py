#!/usr/bin/env python3
"""Generate per-page Open Graph social-card preview images (1200x630) for the
ns3-ntn-toolkit site. On-theme dark/indigo/orange with a satellite-orbit motif.
Pure Pillow (no cairosvg / system deps). Output: <slug>.png in this directory.

Run: python3 generate_cards.py
"""
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
W, H = 1200, 630
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# (slug, title, subtitle)
PAGES = [
    ("home", "ns3-ntn-toolkit", "Open-source ns-3 toolkit for 6G non-terrestrial networks"),
    ("getting-started", "Getting started", "Build the complete ns-3 NTN simulation stack in minutes"),
    ("architecture", "Architecture", "How fourteen ns-3 modules compose the 6G NTN stack"),
    ("modules", "Modules", "Fourteen integrated ns-3 modules for satellite networks"),
    ("ntn-constellation", "ntn-constellation", "SGP4 LEO/MEO/GEO orbits + Walker mega-constellations"),
    ("ntn-rrc", "ntn-rrc", "3GPP Rel-17/18 NR-NTN control plane: SIB19, TA, DRX"),
    ("ntn-observability", "ntn-observability", "Measured-KPI observability: InfluxDB, Grafana, NetSimulyzer"),
    ("ns3-ai-ntn", "ns3-ai-ntn", "AI/ML and reinforcement learning for satellite RANs"),
    ("ntn-sagin", "ntn-sagin", "Space-air-ground: HAPS, UAV, aviation, maritime mobility"),
    ("ntn-slice", "ntn-slice", "5G network slicing over NTN (eMBB / URLLC / mMTC)"),
    ("ntn-v2x", "ntn-v2x", "Vehicle-to-everything over LEO satellites"),
    ("oran-ntn", "oran-ntn", "Space O-RAN: E2 and A1 loops that actuate a real radio"),
    ("ntn-sionna", "ntn-sionna", "NVIDIA Sionna RT ray-traced channels bridged into ns-3"),
    ("ntn-digital-twin", "ntn-digital-twin", "Live constellation digital twin + prediction API"),
    ("ntn-cho", "ntn-cho", "Rel-17/18 conditional handover for LEO (full trigger set)"),
    ("thz-ntn", "thz-ntn", "100 GHz-1 THz on the ITU-R recommendations: RIS, ISAC, beams"),
    ("satellite", "satellite (SNS3)", "DVB-S2/RCS2 + SatSGP4 satellite base reused by the toolkit"),
    ("papers", "Papers", "Peer-reviewed publications built on ns3-ntn-toolkit"),
    ("community", "Community", "Contribute to the open-source ns-3 NTN toolkit"),
    ("cite", "Cite", "How to cite ns3-ntn-toolkit in your research"),
    ("ntn-traffic", "ntn-traffic", "The real NR NTN data plane: every KPI measured in band"),
    ("ntn-fapi", "ntn-fapi", "SCF-222 FAPI MAC-PHY interface over a satellite link"),
]


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def background():
    img = Image.new("RGB", (W, H), (7, 10, 20))
    px = img.load()
    top, bot = (9, 12, 28), (4, 6, 14)
    for y in range(H):
        c = lerp(top, bot, y / H)
        for x in range(W):
            px[x, y] = c
    d = ImageDraw.Draw(img, "RGBA")
    # indigo glow upper-left
    for r in range(520, 0, -8):
        a = int(34 * (1 - r / 520))
        d.ellipse([-260 - r//6, -260 - r//6, -260 + r, -260 + r], fill=(91, 107, 245, a))
    # orbit motif (concentric arcs + satellites) lower-right
    cx, cy = 1000, 470
    for i, rad in enumerate((150, 230, 320)):
        d.arc([cx - rad, cy - rad, cx + rad, cy + rad], 180, 360, fill=(120, 140, 255, 70), width=2)
        ang = math.radians(210 + i * 38)
        sx, sy = cx + rad * math.cos(ang), cy + rad * math.sin(ang)
        col = (255, 112, 67) if i == 1 else (120, 200, 255)
        d.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=col + (255,))
    d.ellipse([cx - 16, cy - 16, cx + 16, cy + 16], fill=(80, 110, 200, 200))  # Earth
    # accent bar
    bw = 1
    for x in range(80, 360):
        c = lerp((91, 107, 245), (255, 112, 67), (x - 80) / 280)
        d.rectangle([x, 150, x + bw, 156], fill=c + (255,))
    return img


def wrap(draw, text, font, max_w):
    words, lines, cur = text.split(), [], ""
    for w in words:
        t = (cur + " " + w).strip()
        if draw.textlength(t, font=font) <= max_w:
            cur = t
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def make(slug, title, subtitle):
    img = background()
    d = ImageDraw.Draw(img)
    brand = ImageFont.truetype(FONT_B, 34)
    tf = ImageFont.truetype(FONT_B, 78 if len(title) < 18 else 62)
    sf = ImageFont.truetype(FONT, 33)
    foot = ImageFont.truetype(FONT, 25)
    # brand wordmark
    d.text((80, 96), "ns3-ntn-toolkit", font=brand, fill=(150, 165, 255))
    # title (wrapped)
    tlines = wrap(d, title, tf, 1000)
    y = 210
    for ln in tlines[:2]:
        d.text((80, y), ln, font=tf, fill=(245, 247, 255))
        y += tf.size + 8
    # subtitle
    for ln in wrap(d, subtitle, sf, 1000)[:2]:
        d.text((82, y + 6), ln, font=sf, fill=(170, 180, 205))
        y += sf.size + 6
    # footer
    d.text((80, 560), "muhammaduazir69.github.io/ns3-ntn-toolkit", font=foot, fill=(120, 132, 160))
    d.text((720, 560), "Muhammad Uzair  -  GPL-2.0", font=foot, fill=(110, 122, 150))
    img.save(OUT / f"{slug}.png", "PNG", optimize=True)
    return (OUT / f"{slug}.png").stat().st_size


def main():
    print(f"{'card':<22}{'KB':>6}")
    for slug, title, sub in PAGES:
        kb = make(slug, title, sub) / 1024
        print(f"{slug:<22}{kb:6.0f}")


if __name__ == "__main__":
    main()
