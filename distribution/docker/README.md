# Docker distribution

Pre-built ns-3.43 + 13 contrib modules + Python utilities.

## Build the image

From the repository root:

```bash
docker build -t uzairdocker69/ns3-ntn-toolkit:latest \
    -f distribution/docker/Dockerfile .
```

First build is **20–40 minutes** (ns-3 compile time).

## Run

### Just a shell

```bash
docker run --rm -it uzairdocker69/ns3-ntn-toolkit:latest
# inside the container:
ns3-ntn-toolkit info
./ns3 run scratch/scratch-simulator
```

### Full stack with observability backend

```bash
docker compose -f distribution/docker/docker-compose.yml up -d
# Toolkit API → http://localhost:8000
# InfluxDB    → http://localhost:8086  (admin / adminadmin)
# Grafana     → http://localhost:3000  (admin / admin)
```

## Push to Docker Hub

```bash
docker login
docker push uzairdocker69/ns3-ntn-toolkit:latest
docker tag uzairdocker69/ns3-ntn-toolkit:latest uzairdocker69/ns3-ntn-toolkit:2.0.0
docker push uzairdocker69/ns3-ntn-toolkit:2.0.0
```

## Image size

Multi-stage build keeps the runtime image lean: the builder stage clones, configures, and builds; the runtime stage copies only the produced binaries + shared libs + Python utilities. Expect ~2.5–3 GB compressed (ns-3.43 is large; this is unavoidable).

## What's NOT in the image

- **NVIDIA Sionna RT (W9)** — requires NVIDIA Container Toolkit + CUDA. Run separately on a GPU host; toolkit talks to it over UDP.
- **FlexRIC live mode (W8)** — has its own docker-compose at `contrib/oran-ntn/flexric-bridge/docker/`. Bring up alongside.
