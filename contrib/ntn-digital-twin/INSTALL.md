# Installing ntn-digital-twin

`ntn-digital-twin` is a Python package: a refresher loop plus a FastAPI
prediction service for live LEO constellations.

## Requirements

- Python >= 3.10
- The sibling `ntn-constellation` package (installed from the same toolkit)
- Dependencies are declared in `pyproject.toml`: `fastapi>=0.110`,
  `uvicorn>=0.27`, `pydantic>=2.6`, `requests>=2.31`.

## Install

From the package directory:

```bash
pip install -e .
```

To include the test dependencies (`pytest`, `httpx`):

```bash
pip install -e .[test]
```

## Run the API

```bash
ntn-twin-api --host 0.0.0.0 --port 8090
```

Then check it is alive:

```bash
curl http://localhost:8090/health
```

## Run the refresher loop

```bash
ntn-twin-loop --max-iterations=3 \
    --czml /tmp/ntn-twin.czml --lp /tmp/twin.lp \
    --max-sats=50 --interval=2
```

The API reloads its constellation when the loop writes a newer CZML file
(the file's mtime is the cache key). The default CZML path is
`/tmp/ntn-twin.czml`; override it for the API with the `NTN_TWIN_CZML`
environment variable.

## InfluxDB (optional)

The loop can emit InfluxDB line-protocol over UDP (default port **8089**) for
the Grafana dashboards. This is optional — line-protocol can also be written to
a file, and the API does not require InfluxDB at all.

## Test

```bash
pytest tests/
```

## systemd (optional, for long-running deployments)

Install the unit files from `systemd/` and enable them:

```bash
systemctl --user enable --now ntn-twin.service ntn-twin-api.service
```

## Part of the toolkit

`ntn-digital-twin` is one module of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).
