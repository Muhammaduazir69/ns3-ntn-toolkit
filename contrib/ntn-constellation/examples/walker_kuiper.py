"""Generate a Kuiper Phase-1 Walker preset (no TLE fetch) and export to SNS3.

Run inside the venv:
    python examples/walker_kuiper.py
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from ntn_constellation.isl import grid_isl_topology_walker
from ntn_constellation.ns3_export import write_sns3_scenario
from ntn_constellation.presets import PRESETS, from_preset


def main() -> None:
    preset = PRESETS["kuiper"]
    print(preset.description)
    print(f"  total satellites: {preset.total_satellites} "
          f"({preset.num_planes} x {preset.sats_per_plane})")

    start = datetime.now(tz=timezone.utc)
    tles = from_preset(preset, epoch=start)

    isls = grid_isl_topology_walker(
        num_planes=preset.num_planes,
        sats_per_plane=preset.sats_per_plane,
    )

    out_dir = Path("data/preset-kuiper")
    pos = write_sns3_scenario(
        scenario_dir=out_dir,
        tles=tles,
        start_date=start,
        isls=isls,
        gw_positions_lla_m=[
            (35.6895, 139.6917, 0.0),
            (28.6667, 77.2167, 0.0),
            (-23.5475, -46.6361, 0.0),
            (40.7128, -74.0060, 0.0),
            (51.5074, -0.1278, 0.0),
        ],
    )
    print(f"wrote {len(tles)} TLEs and {len(isls)} +grid ISL edges to {pos}")


if __name__ == "__main__":
    main()
