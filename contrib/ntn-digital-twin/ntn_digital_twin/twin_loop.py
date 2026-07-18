"""Live digital-twin refresher.

Every ``--interval`` seconds:
  1. Pull TLEs via :mod:`ntn_constellation.feeds` (CelesTrak).
  2. Propagate the constellation at the current wall-clock time.
  3. Emit a fresh CZML packet stream so the CesiumJS viewer auto-reloads.
  4. Push InfluxDB line-protocol points (W3) — measurement ``ntn_sat_pos``.

The loop is crash-tolerant — any single iteration's exception is logged
and the next iteration starts on schedule. Designed to run as a systemd
timer or under tmux for a 24-hour validation window.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import logging
import socket
import sys
import time
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path

# Sibling-package import — works after `pip install -e ../ntn-constellation`.
from ntn_constellation.feeds import CelesTrakFeed, TleCache
from ntn_constellation.propagator import Constellation, Satellite
from ntn_constellation.cesium_export import write_czml

LOG = logging.getLogger("ntn-digital-twin.twin-loop")


@dataclass
class LoopStats:
    started_at_iso: str = ""
    iterations: int = 0
    last_tle_count: int = 0
    last_iteration_iso: str = ""
    last_error: str = ""
    error_count: int = 0
    last_iteration_seconds: float = 0.0
    cumulative_iteration_seconds: float = 0.0


@dataclass
class LoopConfig:
    group: str = "starlink"
    max_sats: int = 50
    interval_sec: float = 60.0
    czml_path: Path = field(default_factory=lambda: Path("/tmp/ntn-twin.czml"))
    influx_lp_path: Path | None = None  # if set, append LP each iteration
    influx_udp_host: str | None = None  # alternative: UDP push to InfluxDB
    influx_udp_port: int = 8089
    bucket: str = "ntn"
    run_id: str = "twin"
    tle_cache_dir: Path = field(default_factory=lambda: Path("/tmp/.ntn-twin-cache"))
    # W1: run the SAME elements as an ns-3 scenario. When source != "celestrak",
    # the twin builds a deterministic Walker-Delta constellation with a fixed
    # epoch instead of pulling live TLEs, so twin and sim propagate identical
    # orbits and their handover sequences are comparable.
    source: str = "celestrak"            # "celestrak" | "walker" | "tle_file"
    tle_file: Path | None = None         # source="tle_file": a 3LE/TLE file on disk
    # Walker parameters (source="walker"), matching the C++ WalkerConfig fields.
    walker_planes: int = 1
    walker_sats_per_plane: int = 24
    walker_altitude_km: float = 600.0
    walker_inclination_deg: float = 53.0
    walker_phasing_factor: int = 1
    epoch_unix_s: float | None = None    # shared epoch; None = now


def _walker_records(cfg: LoopConfig):
    from datetime import datetime, timezone

    from ntn_constellation import presets

    epoch = (datetime.fromtimestamp(cfg.epoch_unix_s, tz=timezone.utc)
             if cfg.epoch_unix_s is not None else None)
    return presets.walker_delta(
        name_prefix="TWIN",
        altitude_km=cfg.walker_altitude_km,
        inclination_deg=cfg.walker_inclination_deg,
        num_planes=cfg.walker_planes,
        sats_per_plane=cfg.walker_sats_per_plane,
        phasing_factor=cfg.walker_phasing_factor,
        epoch=epoch,
    )


def fetch_constellation(cfg: LoopConfig) -> tuple[Constellation, int]:
    if cfg.source == "walker":
        records = _walker_records(cfg)
    elif cfg.source == "tle_file":
        if not cfg.tle_file:
            raise RuntimeError("source='tle_file' requires tle_file")
        from ntn_constellation.feeds import parse_tle_text
        records = parse_tle_text(Path(cfg.tle_file).read_text())
    else:
        cache = TleCache(cfg.tle_cache_dir)
        feed = CelesTrakFeed(cache=cache)
        records = feed.fetch_group(cfg.group)
    if not records:
        raise RuntimeError(f"empty TLE set for source={cfg.source!r} group={cfg.group!r}")
    if cfg.max_sats > 0:
        records = records[: cfg.max_sats]
    sats = [Satellite(r) for r in records]
    return Constellation(sats), len(records)


def emit_czml(cons: Constellation, when: dt.datetime, path: Path) -> None:
    # 5-min CZML window starting `when`, sampled at 30-s cadence.
    write_czml(
        constellation=cons,
        start=when,
        duration=dt.timedelta(minutes=5),
        sample_step=dt.timedelta(seconds=30),
        out_path=path,
    )


def emit_influx_lp(cons: Constellation, when: dt.datetime, cfg: LoopConfig) -> int:
    """Append `ntn_sat_pos` line-protocol points; return number written."""
    ts_ns = int(when.replace(tzinfo=dt.timezone.utc).timestamp() * 1e9)
    lines: list[str] = []
    for sat in cons:
        norad = sat.norad_id
        # gap B3: emit true ECEF (Earth-fixed), not the inertial TEME r_eci_km,
        # so the points overlay correctly on a fixed-frame globe.
        x_m, y_m, z_m = sat.ecef_m(when)
        # Schema lifted from contrib/ntn-observability/model/ntn-metric-schema.h
        line = (
            f"ntn_sat_pos,sat_norad={norad},run_id={cfg.run_id} "
            f"sat_x_m={x_m},sat_y_m={y_m},sat_z_m={z_m} {ts_ns}"
        )
        lines.append(line)

    if cfg.influx_lp_path:
        cfg.influx_lp_path.parent.mkdir(parents=True, exist_ok=True)
        with cfg.influx_lp_path.open("a") as f:
            f.write("\n".join(lines) + "\n")
    if cfg.influx_udp_host:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                payload = ("\n".join(lines) + "\n").encode("utf-8")
                # InfluxDB UDP listener accepts up to 64 KiB datagrams; chunk just in case.
                while payload:
                    chunk, payload = payload[:60000], payload[60000:]
                    s.sendto(chunk, (cfg.influx_udp_host, cfg.influx_udp_port))
        except OSError as exc:
            LOG.warning("UDP push to %s:%d failed: %s",
                        cfg.influx_udp_host, cfg.influx_udp_port, exc)
    return len(lines)


def emit_predictions_file(
    cfg: LoopConfig,
    *,
    observer_lat_deg: float,
    observer_lon_deg: float,
    observer_alt_m: float,
    out_path: Path,
    horizon_s: float,
    step_s: float = 10.0,
    ue_id: int = 0,
    min_confidence: float = 0.0,
) -> int:
    """Export the twin's handover schedule in the C++ consumer's file contract.

    Format (read by ns3::OranNtnTwinPredictionConsumer::LoadPredictionsFromFile):

        # ntn-twin handover predictions  epoch_unix=<...>
        # t_s,ueId,recommendedGnbId,confidence
        <t_s>,<ueId>,<recommendedGnbId>,<confidence>

    ``t_s`` is seconds from the SHARED epoch (cfg.epoch_unix_s) — identical to
    ns-3 simulation time — so the C++ side actuates each handover at the matching
    instant. The recommended cell is the 1-indexed satellite position in the
    constellation's iteration order (the same order the ns-3 scenario builds its
    1-indexed E2 nodes). A line is written only when the serving satellite CHANGES
    (a handover), with confidence set from the elevation margin over the runner-up.
    Returns the number of predictions written.
    """
    cons, _n = fetch_constellation(cfg)
    sats = list(cons)
    epoch_unix = cfg.epoch_unix_s if cfg.epoch_unix_s is not None else time.time()
    epoch = dt.datetime.fromtimestamp(epoch_unix, tz=dt.timezone.utc)

    predictions: list[tuple[float, int, int, float]] = []
    prev_serving: int | None = None
    t = 0.0
    while t <= horizon_s + 1e-9:
        when = epoch + dt.timedelta(seconds=t)
        elevs = [
            s.elevation_deg(
                when,
                observer_lat_deg=observer_lat_deg,
                observer_lon_deg=observer_lon_deg,
                observer_alt_m=observer_alt_m,
            )
            for s in sats
        ]
        # argmax elevation = serving; runner-up sets the confidence margin.
        best_idx = max(range(len(elevs)), key=lambda i: elevs[i])
        best = elevs[best_idx]
        others = [e for i, e in enumerate(elevs) if i != best_idx]
        runner = max(others) if others else -90.0
        # Confidence: how decisively the best beats the runner-up (0..1 over 30 deg).
        confidence = max(0.0, min(1.0, (best - runner) / 30.0))
        if best > 0.0 and best_idx != prev_serving:
            predictions.append((t, ue_id, best_idx + 1, round(confidence, 3)))
            prev_serving = best_idx
        t += step_s

    out_path.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with out_path.open("w") as f:
        f.write(f"# ntn-twin handover predictions  epoch_unix={epoch_unix:.3f}\n")
        f.write("# t_s,ueId,recommendedGnbId,confidence\n")
        for t_s, uid, gnb, conf in predictions:
            if conf < min_confidence:
                continue
            f.write(f"{t_s:.3f},{uid},{gnb},{conf}\n")
            written += 1
    LOG.info("emit_predictions_file: wrote %d handover predictions to %s", written, out_path)
    return written


def run_iteration(cfg: LoopConfig, stats: LoopStats) -> None:
    t0 = time.time()
    when = dt.datetime.now(tz=dt.timezone.utc)
    cons, nTle = fetch_constellation(cfg)
    emit_czml(cons, when, cfg.czml_path)
    nLp = emit_influx_lp(cons, when, cfg)
    stats.iterations += 1
    stats.last_iteration_iso = when.isoformat()
    stats.last_tle_count = nTle
    dt_s = time.time() - t0
    stats.last_iteration_seconds = dt_s
    stats.cumulative_iteration_seconds += dt_s
    LOG.info("iter=%d  tle=%d  lp=%d  czml=%s  dt=%.2fs",
             stats.iterations, nTle, nLp, cfg.czml_path, dt_s)


def run_loop(cfg: LoopConfig, max_iterations: int | None = None) -> LoopStats:
    """Run the loop forever (or up to max_iterations). Crash-tolerant."""
    stats = LoopStats()
    stats.started_at_iso = dt.datetime.now(tz=dt.timezone.utc).isoformat()
    while True:
        try:
            run_iteration(cfg, stats)
        except Exception as exc:  # noqa: BLE001 — crash tolerance is the point
            stats.error_count += 1
            stats.last_error = f"{type(exc).__name__}: {exc}"
            LOG.error("iteration failed: %s\n%s", stats.last_error, traceback.format_exc())
        if max_iterations is not None and stats.iterations + stats.error_count >= max_iterations:
            break
        time.sleep(cfg.interval_sec)
    return stats


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--group", default="starlink")
    parser.add_argument("--max-sats", type=int, default=50)
    parser.add_argument("--interval", type=float, default=60.0)
    parser.add_argument("--czml", type=Path, default=Path("/tmp/ntn-twin.czml"))
    parser.add_argument("--lp", type=Path, default=None,
                        help="if set, append InfluxDB line-protocol to this file")
    parser.add_argument("--udp-host", default=None,
                        help="InfluxDB UDP host (port 8089 default)")
    parser.add_argument("--udp-port", type=int, default=8089)
    parser.add_argument("--run-id", default="twin")
    parser.add_argument("--cache-dir", type=Path, default=Path("/tmp/.ntn-twin-cache"))
    parser.add_argument("--max-iterations", type=int, default=None,
                        help="stop after N iterations (CI-friendly)")
    parser.add_argument("--stats-out", type=Path, default=None,
                        help="write LoopStats JSON when the loop exits")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = LoopConfig(
        group=args.group,
        max_sats=args.max_sats,
        interval_sec=args.interval,
        czml_path=args.czml,
        influx_lp_path=args.lp,
        influx_udp_host=args.udp_host,
        influx_udp_port=args.udp_port,
        run_id=args.run_id,
        tle_cache_dir=args.cache_dir,
    )
    stats = run_loop(cfg, max_iterations=args.max_iterations)
    if args.stats_out:
        args.stats_out.write_text(json.dumps(asdict(stats), indent=2),
                                  encoding="utf-8")
    print(json.dumps(asdict(stats), indent=2))
    return 0 if stats.iterations > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
