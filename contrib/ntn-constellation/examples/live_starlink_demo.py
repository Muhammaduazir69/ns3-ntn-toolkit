"""End-to-end live demo: fetch current Starlink TLEs from CelesTrak, propagate
them with SGP4, and write both an SNS3 scenario and a CZML file for the
CesiumJS viewer in `contrib/ntn-cho/visualization/`.

Run from inside the venv:
    python examples/live_starlink_demo.py
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
from pathlib import Path

from ntn_constellation.cesium_export import write_czml
from ntn_constellation.feeds import CelesTrakFeed, TleCache
from ntn_constellation.isl import build_isl_topology
from ntn_constellation.ns3_export import write_sns3_scenario
from ntn_constellation.propagator import Constellation


def main() -> None:
    out_dir = Path("data/live-starlink")
    out_dir.mkdir(parents=True, exist_ok=True)

    cache = TleCache(out_dir / ".tle-cache")
    feed = CelesTrakFeed(cache=cache)

    print("fetching current Starlink TLEs from CelesTrak ...")
    tles = feed.fetch_group("starlink")
    print(f"  got {len(tles)} satellites")

    sample = tles[:50]
    print(f"keeping the first {len(sample)} for the demo")

    start = datetime.now(tz=timezone.utc)
    constellation = Constellation.from_tles(sample)

    print("computing ISL topology (k=4, <=5000 km) ...")
    isls = build_isl_topology(constellation, when=start, k_nearest=4, max_range_km=5000.0)
    print(f"  {len(isls)} ISL edges")

    print("writing SNS3 scenario ...")
    pos_dir = write_sns3_scenario(
        scenario_dir=out_dir / "scenario",
        tles=sample,
        start_date=start,
        isls=isls,
    )
    print(f"  -> {pos_dir}")

    print("writing CZML for CesiumJS (2 hours, 30 s samples) ...")
    czml_path = write_czml(
        constellation=constellation,
        start=start,
        duration=timedelta(hours=2),
        sample_step=timedelta(seconds=30),
        out_path=out_dir / "starlink.czml",
        document_name="live-starlink-50",
    )
    print(f"  -> {czml_path}")

    print("done.")


if __name__ == "__main__":
    main()
