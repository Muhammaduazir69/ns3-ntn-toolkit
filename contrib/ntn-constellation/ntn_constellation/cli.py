"""Command-line entry point: `ntn-fetch <preset|group> ...`."""

from __future__ import annotations

import argparse
import logging
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

from ntn_constellation.cesium_export import write_czml
from ntn_constellation.feeds import (
    CELESTRAK_GROUPS,
    CelesTrakFeed,
    SpaceTrackFeed,
    TleCache,
    write_tle_file,
)
from ntn_constellation.isl import build_isl_topology, grid_isl_topology_walker
from ntn_constellation.ns3_export import write_sns3_scenario
from ntn_constellation.presets import PRESETS, from_preset
from ntn_constellation.propagator import Constellation


def _parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="ntn-fetch",
        description="Fetch TLEs (CelesTrak/Space-Track) or generate Walker presets, "
                    "then export to ns-3/SNS3 scenario layout and/or CZML for CesiumJS.",
    )
    p.add_argument("source", help=f"CelesTrak group ({', '.join(sorted(CELESTRAK_GROUPS))}) "
                                  f"or preset ({', '.join(sorted(PRESETS))})")
    p.add_argument("--out", required=True, type=Path, help="Output scenario directory")
    p.add_argument("--cache", type=Path, default=None,
                   help="On-disk cache root (default: <out>/.tle-cache)")
    p.add_argument("--use-spacetrack", action="store_true",
                   help="Use Space-Track instead of CelesTrak (needs SPACETRACK_USER/PASS)")
    p.add_argument("--max-sats", type=int, default=None, help="Cap number of satellites")
    p.add_argument("--isl-k", type=int, default=4, help="k-nearest ISL neighbours (default 4)")
    p.add_argument("--isl-max-km", type=float, default=5000.0,
                   help="ISL range cap in km (default 5000)")
    p.add_argument("--isl-walker", action="store_true",
                   help="For presets, emit closed-form Walker grid ISLs instead of k-NN")
    p.add_argument("--czml", action="store_true", help="Also write a CZML file for CesiumJS")
    p.add_argument("--czml-duration-min", type=float, default=120.0)
    p.add_argument("--czml-step-sec", type=float, default=30.0)
    p.add_argument("--start", default=None,
                   help="Scenario start UTC (ISO-8601). Default: now.")
    p.add_argument("--force", action="store_true", help="Bypass TLE cache")
    p.add_argument("-v", "--verbose", action="count", default=0)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=logging.WARNING - 10 * min(args.verbose, 2),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    out_dir: Path = args.out
    cache_dir = args.cache or (out_dir / ".tle-cache")
    cache = TleCache(cache_dir)

    start = (
        datetime.fromisoformat(args.start).astimezone(timezone.utc)
        if args.start
        else datetime.now(tz=timezone.utc)
    )

    source = args.source.lower()
    walker_geom: tuple[int, int] | None = None
    if source in PRESETS:
        preset = PRESETS[source]
        tles = from_preset(preset, epoch=start)
        walker_geom = (preset.num_planes, preset.sats_per_plane)
    elif args.use_spacetrack:
        feed = SpaceTrackFeed(cache=cache)
        tles = feed.fetch_group(source, force=args.force)
    else:
        feed = CelesTrakFeed(cache=cache)
        tles = feed.fetch_group(source, force=args.force)

    if args.max_sats is not None:
        tles = tles[: args.max_sats]
    if not tles:
        print(f"error: no TLEs returned for source {args.source!r}", file=sys.stderr)
        return 2

    constellation = Constellation.from_tles(tles)

    if args.isl_walker and walker_geom is not None:
        isls = grid_isl_topology_walker(num_planes=walker_geom[0],
                                        sats_per_plane=walker_geom[1])
    else:
        isls = build_isl_topology(constellation, when=start,
                                  k_nearest=args.isl_k,
                                  max_range_km=args.isl_max_km)

    pos_dir = write_sns3_scenario(
        scenario_dir=out_dir,
        tles=tles,
        start_date=start,
        isls=isls,
    )
    flat_tle = write_tle_file(tles, out_dir / "raw.tle")

    print(f"wrote {len(tles)} satellites + {len(isls)} ISL edges")
    print(f"  scenario : {pos_dir}")
    print(f"  raw tle  : {flat_tle}")

    if args.czml:
        czml_path = write_czml(
            constellation=constellation,
            start=start,
            duration=timedelta(minutes=args.czml_duration_min),
            sample_step=timedelta(seconds=args.czml_step_sec),
            out_path=out_dir / "constellation.czml",
            document_name=f"{args.source}",
        )
        print(f"  czml     : {czml_path}")

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
