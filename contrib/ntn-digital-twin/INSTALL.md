# Installing ntn-digital-twin

<p align="center">
  <a href="README.md">Module README</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit">Toolkit</a>
  &nbsp;·&nbsp;
  <a href="https://github.com/Muhammaduazir69/ns3-ntn-toolkit/blob/ntn-integration-v2/INSTALL.md">Toolkit install guide</a>
  &nbsp;·&nbsp;
  <a href="https://muhammaduazir69.github.io/ns3-ntn-toolkit/">Docs site</a>
</p>

> **The fastest path is the container.** `docker pull uzairdocker69/ns3-ntn-toolkit:latest`
> ships this module already built alongside the other thirteen and the vendored
> stacks, so nothing below is needed to simply run the examples. Build from source
> when you intend to change the module.

---

`ntn-digital-twin` is a Python package: a refresher loop plus a FastAPI
prediction service for live LEO constellations. It is one module of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit) (ns-3.43,
branch `ntn-integration-v2`). Being a pure-Python service package, it has no
ns-3 C++ build step of its own.

## Requirements

- Python >= 3.10
- The sibling `ntn-constellation` package (installed from the same toolkit)
- Dependencies are declared in `pyproject.toml`: `fastapi>=0.110`,
  `uvicorn>=0.27`, `pydantic>=2.6`, `requests>=2.31`.

## Get the module

It already ships in `contrib/ntn-digital-twin` inside the toolkit tree. To clone
it standalone:

```bash
git clone -b ntn-digital-twin-v2 \
  https://github.com/Muhammaduazir69/ntn-digital-twin.git ntn-digital-twin
cd ntn-digital-twin
```

> GitLab mirror: the umbrella toolkit (with this module under `contrib/`) is
> mirrored at
> [gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit](https://gitlab.com/ns3-ntn-toolkit/ns3-ntn-toolkit),
> and shipped as the Docker image `uzairdocker69/ns3-ntn-toolkit:latest`
> (or `:latest`) with this package preinstalled:
> `docker run -it uzairdocker69/ns3-ntn-toolkit:latest`.

## Install

From the package directory (install the sibling `ntn-constellation` package
first — it is a hard dependency):

```bash
pip install -e .
```

To include the test dependencies (`pytest>=8`, `httpx>=0.26`):

```bash
pip install -e .[test]
```

This installs two console entry points (`pyproject.toml [project.scripts]`):
`ntn-twin-loop` → `ntn_digital_twin.twin_loop:main` and
`ntn-twin-api` → `ntn_digital_twin.api.server:main`.

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
pytest tests/      # 10 cases (LP schema, outage tolerance, /health,
                   # /constellation/state, /predict/handover latency + events,
                   # A3 hysteresis guard, shared-epoch Walker source,
                   # emit_predictions_file export contract, and the gate-9
                   # twin/sim handover-agreement check)
```

## systemd (optional, for long-running deployments)

Install the unit files from `systemd/` and enable them:

```bash
systemctl --user enable --now ntn-twin.service ntn-twin-api.service
```

## Part of the toolkit

`ntn-digital-twin` is one module of the
[ns3-ntn-toolkit](https://github.com/Muhammaduazir69/ns3-ntn-toolkit).