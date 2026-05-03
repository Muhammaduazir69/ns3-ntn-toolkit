"""TLE data feeds: CelesTrak (public) and Space-Track (credentialed)."""

from __future__ import annotations

import dataclasses
import logging
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Iterator

import requests

logger = logging.getLogger(__name__)

CELESTRAK_GP_URL = "https://celestrak.org/NORAD/elements/gp.php"
SPACETRACK_BASE = "https://www.space-track.org"

# CelesTrak's terms of use require an identifying User-Agent on automated
# requests. They will return HTTP 403 to default urllib/requests UAs.
DEFAULT_UA = (
    "ns3-ntn-toolkit/0.1 (+https://github.com/Muhammaduazir69/ns3-ntn-toolkit) "
    "python-requests"
)

CELESTRAK_GROUPS = {
    "starlink": "starlink",
    "oneweb": "oneweb",
    "kuiper": "amazon-kuiper",
    "telesat": "telesat",
    "globalstar": "globalstar",
    "iridium": "iridium-NEXT",
    "galileo": "galileo",
    "gps-ops": "gps-ops",
    "active": "active",
    "stations": "stations",
}


@dataclasses.dataclass(frozen=True)
class TleRecord:
    """A single Two-Line Element set."""

    name: str
    line1: str
    line2: str

    def __post_init__(self) -> None:
        if len(self.line1) != 69 or self.line1[0] != "1":
            raise ValueError(f"line1 not a valid TLE line: {self.line1!r}")
        if len(self.line2) != 69 or self.line2[0] != "2":
            raise ValueError(f"line2 not a valid TLE line: {self.line2!r}")

    @property
    def norad_id(self) -> int:
        return int(self.line1[2:7])

    @property
    def epoch(self) -> datetime:
        year = int(self.line1[18:20])
        year += 2000 if year < 57 else 1900
        doy = float(self.line1[20:32])
        ts = datetime(year, 1, 1, tzinfo=timezone.utc).timestamp() + (doy - 1) * 86400.0
        return datetime.fromtimestamp(ts, tz=timezone.utc)


def parse_tle_text(text: str) -> list[TleRecord]:
    """Parse a 3-line-per-sat TLE text block (CelesTrak GP format)."""
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    records: list[TleRecord] = []
    i = 0
    while i + 2 < len(lines) + 1:
        if i + 2 >= len(lines):
            break
        name, l1, l2 = lines[i], lines[i + 1], lines[i + 2]
        if l1.startswith("1 ") and l2.startswith("2 "):
            try:
                records.append(TleRecord(name=name.strip(), line1=l1, line2=l2))
            except ValueError as err:
                logger.warning("skipping malformed TLE near line %d: %s", i, err)
            i += 3
        else:
            i += 1
    return records


class TleCache:
    """On-disk cache for fetched TLEs. Default TTL 6h matches CelesTrak's update cadence."""

    def __init__(self, root: Path, ttl_seconds: float = 6 * 3600.0) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.ttl_seconds = ttl_seconds

    def _path(self, key: str) -> Path:
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", key)
        return self.root / f"{safe}.tle"

    def is_fresh(self, key: str) -> bool:
        p = self._path(key)
        if not p.exists():
            return False
        return (time.time() - p.stat().st_mtime) < self.ttl_seconds

    def read(self, key: str) -> str | None:
        p = self._path(key)
        return p.read_text(encoding="utf-8") if p.exists() else None

    def write(self, key: str, text: str) -> Path:
        p = self._path(key)
        p.write_text(text, encoding="utf-8")
        return p


class CelesTrakFeed:
    """Public TLE source. No credentials. Rate-limited per CelesTrak terms (~1 req/s)."""

    def __init__(
        self,
        cache: TleCache | None = None,
        timeout: float = 30.0,
        min_interval_seconds: float = 1.0,
    ) -> None:
        self.cache = cache
        self.timeout = timeout
        self.min_interval_seconds = min_interval_seconds
        self._last_fetch_ts: float = 0.0

    def _throttle(self) -> None:
        elapsed = time.monotonic() - self._last_fetch_ts
        if elapsed < self.min_interval_seconds:
            time.sleep(self.min_interval_seconds - elapsed)
        self._last_fetch_ts = time.monotonic()

    def fetch_group(self, group: str, force: bool = False) -> list[TleRecord]:
        celestrak_name = CELESTRAK_GROUPS.get(group.lower(), group)
        cache_key = f"celestrak_{celestrak_name}"
        if self.cache is not None and not force and self.cache.is_fresh(cache_key):
            text = self.cache.read(cache_key)
            if text is not None:
                logger.info("celestrak: using cached %s (%d bytes)", group, len(text))
                return parse_tle_text(text)

        self._throttle()
        params = {"GROUP": celestrak_name, "FORMAT": "tle"}
        headers = {"User-Agent": DEFAULT_UA}
        logger.info("celestrak: fetching group=%s", celestrak_name)
        resp = requests.get(CELESTRAK_GP_URL,
                            params=params,
                            headers=headers,
                            timeout=self.timeout)
        resp.raise_for_status()
        text = resp.text
        if "No GP data found" in text or len(text.strip()) < 50:
            raise RuntimeError(f"CelesTrak returned no data for group {group!r}")
        if self.cache is not None:
            self.cache.write(cache_key, text)
        return parse_tle_text(text)

    def fetch_norad(self, norad_ids: Iterable[int], force: bool = False) -> list[TleRecord]:
        ids = sorted({int(x) for x in norad_ids})
        cache_key = f"celestrak_norad_{'-'.join(str(i) for i in ids)}"
        if self.cache is not None and not force and self.cache.is_fresh(cache_key):
            text = self.cache.read(cache_key)
            if text is not None:
                return parse_tle_text(text)
        records: list[TleRecord] = []
        headers = {"User-Agent": DEFAULT_UA}
        for nid in ids:
            self._throttle()
            params = {"CATNR": str(nid), "FORMAT": "tle"}
            resp = requests.get(CELESTRAK_GP_URL,
                                params=params,
                                headers=headers,
                                timeout=self.timeout)
            resp.raise_for_status()
            records.extend(parse_tle_text(resp.text))
        if self.cache is not None:
            self.cache.write(cache_key, _records_to_text(records))
        return records


class SpaceTrackFeed:
    """Credentialed TLE source. Required for full historical / GP_HISTORY queries.

    Reads SPACETRACK_USER and SPACETRACK_PASS env vars unless passed explicitly.
    """

    def __init__(
        self,
        username: str | None = None,
        password: str | None = None,
        cache: TleCache | None = None,
        timeout: float = 60.0,
    ) -> None:
        self.username = username or os.environ.get("SPACETRACK_USER")
        self.password = password or os.environ.get("SPACETRACK_PASS")
        if not self.username or not self.password:
            raise RuntimeError(
                "Space-Track credentials missing: set SPACETRACK_USER and SPACETRACK_PASS, "
                "or pass username=/password= explicitly."
            )
        self.cache = cache
        self.timeout = timeout
        self._session: requests.Session | None = None

    def _login(self) -> requests.Session:
        if self._session is not None:
            return self._session
        s = requests.Session()
        r = s.post(
            f"{SPACETRACK_BASE}/ajaxauth/login",
            data={"identity": self.username, "password": self.password},
            timeout=self.timeout,
        )
        r.raise_for_status()
        self._session = s
        return s

    def fetch_group(self, group: str, force: bool = False) -> list[TleRecord]:
        cache_key = f"spacetrack_{group.lower()}"
        if self.cache is not None and not force and self.cache.is_fresh(cache_key):
            text = self.cache.read(cache_key)
            if text is not None:
                return parse_tle_text(text)
        sess = self._login()
        query = (
            f"{SPACETRACK_BASE}/basicspacedata/query/class/gp/decay_date/null-val/"
            f"epoch/%3Enow-30/OBJECT_NAME/~~{group}~~/orderby/NORAD_CAT_ID/format/tle"
        )
        r = sess.get(query, timeout=self.timeout)
        r.raise_for_status()
        text = r.text
        if self.cache is not None:
            self.cache.write(cache_key, text)
        return parse_tle_text(text)


def _records_to_text(records: Iterable[TleRecord]) -> str:
    return "\n".join(f"{r.name}\n{r.line1}\n{r.line2}" for r in records) + "\n"


def write_tle_file(records: Iterable[TleRecord], path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_records_to_text(records), encoding="utf-8")
    return path


def iter_tle_file(path: Path) -> Iterator[TleRecord]:
    yield from parse_tle_text(Path(path).read_text(encoding="utf-8"))
