"""Write SNS3-compatible constellation scenarios.

Target layout (matches `contrib/satellite/data/scenarios/<name>/positions/`):
    positions/start_date.txt    "YYYY-MM-DD HH:MM:SS"
    positions/tles.txt          <count>\\n<name>\\n<line1>\\n<line2>\\n...
    positions/isls.txt          <count>\\n<i> <j>\\n...
    positions/gw_positions.txt  <lat> <lon> <alt_m>\\n...
    positions/ut_positions.txt  <lat> <lon> <alt_m>\\n...
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence

from ntn_constellation.feeds import TleRecord


def write_sns3_scenario(
    *,
    scenario_dir: Path | str,
    tles: Sequence[TleRecord],
    start_date: datetime,
    isls: Iterable[tuple[int, int]] | None = None,
    gw_positions_lla_m: Iterable[tuple[float, float, float]] | None = None,
    ut_positions_lla_m: Iterable[tuple[float, float, float]] | None = None,
) -> Path:
    """Materialise one SNS3 scenario directory. Returns the `positions/` path.

    `start_date` is converted to UTC and written without timezone (SNS3 parses
    naive timestamps as UTC).
    """
    scenario_dir = Path(scenario_dir)
    pos_dir = scenario_dir / "positions"
    pos_dir.mkdir(parents=True, exist_ok=True)

    sd = start_date.astimezone(timezone.utc).replace(tzinfo=None)
    (pos_dir / "start_date.txt").write_text(sd.strftime("%Y-%m-%d %H:%M:%S") + "\n", encoding="utf-8")

    tles = list(tles)
    lines = [str(len(tles))]
    for r in tles:
        lines.extend([r.name, r.line1, r.line2])
    (pos_dir / "tles.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    edges = sorted({(min(i, j), max(i, j)) for i, j in (isls or [])})
    isl_lines = [str(len(edges))]
    isl_lines.extend(f"{i} {j}" for i, j in edges)
    (pos_dir / "isls.txt").write_text("\n".join(isl_lines) + "\n", encoding="utf-8")

    if gw_positions_lla_m is not None:
        (pos_dir / "gw_positions.txt").write_text(
            "\n".join(f"{lat:.6f} {lon:.6f} {alt:.6f}" for lat, lon, alt in gw_positions_lla_m) + "\n",
            encoding="utf-8",
        )
    if ut_positions_lla_m is not None:
        (pos_dir / "ut_positions.txt").write_text(
            "\n".join(f"{lat:.6f} {lon:.6f} {alt:.6f}" for lat, lon, alt in ut_positions_lla_m) + "\n",
            encoding="utf-8",
        )

    return pos_dir
