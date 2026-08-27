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
import math
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
from ntn_digital_twin import a3_guard

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
    # TWIN-03: predictions written on the last iteration. Zero when the loop is
    # not configured to emit them, which is the default.
    last_prediction_count: int = 0


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

    # ---- TWIN-03: prediction export from the shipped CLI ----
    # emit_predictions_file() had exactly one caller, a test. run_iteration()
    # never called it and main() exposed no flag for it, so the file the C++
    # OranNtnTwinPredictionConsumer reads could not be produced by any shipped
    # command. Setting predictions_path turns it on.
    predictions_path: Path | None = None
    # TWIN-02: dump the exact TLEs the twin propagated, so ns-3 can load the
    # SAME orbits and the two handover sequences become comparable at all.
    #
    # Until TWIN-01 this could not have helped: Sgp4MobilityModel defaulted to
    # Kepler + J2 even when handed a TLE, so feeding it the twin's elements
    # still propagated a different orbit. With SetTle() now selecting SGP4,
    # both sides run the same propagator on the same elements, and a
    # disagreement in handover instants is a disagreement about MOBILITY
    # DECISIONS rather than about where the satellites are.
    tle_dump_path: Path | None = None
    predictions_horizon_s: float = 2700.0   # 45 min, the exporter's own horizon
    predictions_step_s: float = 10.0
    observer_lat_deg: float = 0.0
    observer_lon_deg: float = 0.0
    observer_alt_m: float = 0.0

    # ---- TWIN-04: an A3 guard on the path that actuates ----
    # The exporter took a bare argmax over elevation and appended a prediction
    # on every serving change: no hysteresis, no time-to-trigger, no minimum
    # service time. The guarded REST path used elevation DEGREES, while the
    # simulator's A3 uses SINR in dB - so the two admitted different handovers
    # and the claim that the guard enforces "exactly the conditions the sim's A3
    # algorithm enforces" held in neither units nor code path.
    #
    # The twin now computes a link budget from the ephemeris it already has, so
    # its trigger quantity is a dB SNR like the simulator's, and applies A3 with
    # hysteresis and time-to-trigger on top.
    a3_offset_db: float = 3.0
    a3_time_to_trigger_s: float = 0.0
    min_service_time_s: float = 0.0
    # Link-budget terms for the twin's SNR. Defaults are the toolkit's S-band
    # reference point; a scenario should set them to whatever it is modelling.
    link_eirp_dbm: float = 62.0
    link_gt_db_per_k: float = 1.1
    link_bandwidth_hz: float = 20.0e6
    link_frequency_hz: float = 2.0e9


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


def _a3_params(cfg: "LoopConfig") -> a3_guard.A3Params:
    """The loop config's guard settings as the shared A3 parameters."""
    return a3_guard.A3Params(
        margin_quantity="db",
        a3_offset_db=cfg.a3_offset_db,
        time_to_trigger_s=cfg.a3_time_to_trigger_s,
        min_service_s=cfg.min_service_time_s,
        min_elevation_deg=0.0,
        link_eirp_dbm=cfg.link_eirp_dbm,
        link_gt_db_per_k=cfg.link_gt_db_per_k,
        link_bandwidth_hz=cfg.link_bandwidth_hz,
        link_frequency_hz=cfg.link_frequency_hz,
    )


def _slant_range_m(elev_deg: float, alt_km: float, earth_radius_m: float = 6371e3) -> float:
    """Slant range to a satellite at `alt_km` seen at `elev_deg`.

    TWIN-04: delegates to ntn_digital_twin.a3_guard, which the REST endpoint
    also calls. Two copies of this budget existed and had already drifted.
    """
    return a3_guard.slant_range_m(elev_deg, alt_km, earth_radius_m)


def _snr_db(cfg: "LoopConfig", elev_deg: float, alt_km: float) -> float:
    """Link-budget SNR in dB, so the twin triggers on the sim's quantity.

    TWIN-04: delegates to the shared guard. See a3_guard for what this budget
    does and does not model.
    """
    return a3_guard.snr_db(_a3_params(cfg), elev_deg, alt_km)


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
    ev = a3_guard.A3Evaluator(_a3_params(cfg))

    # Propagate each satellite across the WHOLE horizon in one call rather than
    # per tick. This is the path the C++ OranNtnTwinPredictionConsumer reads, so
    # it is swept over long horizons and large shells; the per-instant loop it
    # replaced spent its time in Skyfield's per-call setup.
    n_ticks = int(math.floor((horizon_s + 1e-9) / step_s)) + 1
    tick_s = [k * step_s for k in range(n_ticks)]
    times = [epoch + dt.timedelta(seconds=t) for t in tick_s]
    elev_tracks = [
        s_.elevation_deg_series(
            times,
            observer_lat_deg=observer_lat_deg,
            observer_lon_deg=observer_lon_deg,
            observer_alt_m=observer_alt_m,
        )
        for s_ in sats
    ]
    ecef_tracks = [s_.ecef_m_series(times) for s_ in sats]

    for k, t in enumerate(tick_s):
        elevs = [elev_tracks[i][k] for i in range(len(sats))]
        # TWIN-04: the guard is ntn_digital_twin.a3_guard, the same evaluator
        # the REST endpoint runs. This loop used to be a bare argmax over
        # elevation that appended a prediction on every serving change - no
        # hysteresis, no time-to-trigger, no minimum service time - and it is
        # the path the C++ OranNtnTwinPredictionConsumer actuates on. So the
        # twin's foresight reached ns-3 in its most ping-pong-prone form while
        # the guarded REST variant that nothing consumed triggered on elevation
        # DEGREES against a simulator whose A3 compares SINR in dB.
        #
        # Altitude is measured per satellite, not taken from the preset's
        # nominal value: the endpoint measures it, and a constant here made the
        # two paths disagree by one tick on a 45 minute horizon.
        cands = [
            a3_guard.Candidate(
                key=i, elev_deg=e,
                alt_km=a3_guard.altitude_km_from_ecef(ecef_tracks[i][k]))
            for i, e in enumerate(elevs)
        ]
        hit = ev.step(t, cands)
        if hit is not None:
            # Confidence from how far the incoming satellite clears the best of
            # the rest, in elevation, capped at a 30 degree spread.
            others = [e for i, e in enumerate(elevs) if i != hit.key_in]
            runner = max(others) if others else -90.0
            confidence = max(0.0, min(1.0, (hit.elev_in_deg - runner) / 30.0))
            # E2 node ids are 1-indexed.
            predictions.append((t, ue_id, hit.key_in + 1, round(confidence, 3)))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with out_path.open("w") as f:
        f.write(f"# ntn-twin handover predictions  epoch_unix={epoch_unix:.3f}\n")
        # TWIN-02: the observer and window travel WITH the predictions, so an
        # ns-3 gate can replay the identical geometry instead of being told
        # them out of band and silently diverging.
        f.write(f"# observer_lat_deg={observer_lat_deg:.6f} "
                f"observer_lon_deg={observer_lon_deg:.6f} "
                f"observer_alt_m={observer_alt_m:.3f}\n")
        f.write(f"# horizon_s={horizon_s:.3f} step_s={step_s:.3f}\n")
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

    # TWIN-03: emit the prediction file the C++ consumer reads.
    #
    # emit_predictions_file() had exactly one caller - a test - and
    # run_iteration() never called it, so the twin->sim loop both READMEs
    # advertise as closed could not be executed by any shipped command.
    # TWIN-02: write the orbits alongside the predictions.
    if cfg.tle_dump_path is not None:
        cfg.tle_dump_path.parent.mkdir(parents=True, exist_ok=True)
        with cfg.tle_dump_path.open("w") as f:
            n = 0
            for sat in cons:  # Constellation is iterable
                rec = sat.tle
                f.write(f"{rec.name}\n{rec.line1}\n{rec.line2}\n")
                n += 1
        LOG.info("wrote %d TLEs to %s", n, cfg.tle_dump_path)

    nPred = 0
    if cfg.predictions_path is not None:
        nPred = emit_predictions_file(
            cfg,
            observer_lat_deg=cfg.observer_lat_deg,
            observer_lon_deg=cfg.observer_lon_deg,
            observer_alt_m=cfg.observer_alt_m,
            out_path=cfg.predictions_path,
            horizon_s=cfg.predictions_horizon_s,
            step_s=cfg.predictions_step_s,
        )
        stats.last_prediction_count = nPred

    stats.iterations += 1
    stats.last_iteration_iso = when.isoformat()
    stats.last_tle_count = nTle
    dt_s = time.time() - t0
    stats.last_iteration_seconds = dt_s
    stats.cumulative_iteration_seconds += dt_s
    LOG.info("iter=%d  tle=%d  lp=%d  pred=%d  czml=%s  dt=%.2fs",
             stats.iterations, nTle, nLp, nPred, cfg.czml_path, dt_s)


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
    # ---- TWIN-03: the flags the shipped CLI was missing ----
    # main() built LoopConfig with source='celestrak' and exposed no way to
    # select Walker elements, pin an epoch, or emit predictions - so the
    # deterministic run the C++ consumer is designed to read was unreachable
    # from any shipped command, and the twin->sim loop both READMEs advertise
    # as closed could not actually be executed.
    parser.add_argument("--source", choices=["celestrak", "walker", "tle_file"],
                        default="celestrak",
                        help="orbit source; walker is deterministic and sim-comparable")
    parser.add_argument("--tle-file", type=Path, default=None)
    parser.add_argument("--walker-planes", type=int, default=1)
    parser.add_argument("--walker-sats-per-plane", type=int, default=24)
    parser.add_argument("--walker-altitude-km", type=float, default=600.0)
    parser.add_argument("--walker-inclination-deg", type=float, default=53.0)
    parser.add_argument("--walker-phasing-factor", type=int, default=1)
    parser.add_argument("--epoch-unix", type=float, default=None,
                        help="shared epoch with the ns-3 scenario; omit for now()")
    parser.add_argument("--tle-dump", type=Path, default=None,
                        help="write the exact TLEs the twin propagated, so an ns-3 "
                             "scenario can load the SAME orbits (TWIN-02)")
    parser.add_argument("--predictions", type=Path, default=None,
                        help="write the handover-prediction file the C++ "
                             "OranNtnTwinPredictionConsumer reads")
    parser.add_argument("--predictions-horizon-s", type=float, default=2700.0)
    parser.add_argument("--predictions-step-s", type=float, default=10.0)
    parser.add_argument("--observer-lat", type=float, default=0.0)
    parser.add_argument("--observer-lon", type=float, default=0.0)
    parser.add_argument("--observer-alt-m", type=float, default=0.0)
    # ---- TWIN-04: the A3 guard on the path that actuates ----
    parser.add_argument("--a3-offset-db", type=float, default=3.0,
                        help="candidate must beat serving by this margin, in dB")
    parser.add_argument("--a3-ttt-s", type=float, default=0.0,
                        help="time-to-trigger: the A3 condition must HOLD this long")
    parser.add_argument("--min-service-time-s", type=float, default=0.0,
                        help="minimum dwell on a cell before another switch is emitted")
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
        source=args.source,
        tle_file=args.tle_file,
        walker_planes=args.walker_planes,
        walker_sats_per_plane=args.walker_sats_per_plane,
        walker_altitude_km=args.walker_altitude_km,
        walker_inclination_deg=args.walker_inclination_deg,
        walker_phasing_factor=args.walker_phasing_factor,
        epoch_unix_s=args.epoch_unix,
        predictions_path=args.predictions,
        tle_dump_path=args.tle_dump,
        predictions_horizon_s=args.predictions_horizon_s,
        predictions_step_s=args.predictions_step_s,
        observer_lat_deg=args.observer_lat,
        observer_lon_deg=args.observer_lon,
        observer_alt_m=args.observer_alt_m,
        a3_offset_db=args.a3_offset_db,
        a3_time_to_trigger_s=args.a3_ttt_s,
        min_service_time_s=args.min_service_time_s,
    )
    stats = run_loop(cfg, max_iterations=args.max_iterations)
    if args.stats_out:
        args.stats_out.write_text(json.dumps(asdict(stats), indent=2),
                                  encoding="utf-8")
    print(json.dumps(asdict(stats), indent=2))
    return 0 if stats.iterations > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
