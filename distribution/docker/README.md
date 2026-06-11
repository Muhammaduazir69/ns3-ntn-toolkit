# Docker distribution

Pre-built ns-3.43 + 15 bundled modules + Python utilities, published as
[`uzairdocker69/ns3-ntn-toolkit`](https://hub.docker.com/r/uzairdocker69/ns3-ntn-toolkit).

## Build the image

The Dockerfile copies `ns-3-dev/` from the build context, so the context must
be the **parent directory of a clone named `ns-3-dev`** (a `.dockerignore` in
that parent should keep run outputs and papers out of the context):

```bash
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git ns-3-dev
cd ns-3-dev/contrib && git clone https://github.com/sns3/sns3-satellite.git satellite && cd ../..
docker build -t uzairdocker69/ns3-ntn-toolkit:latest \
    -f ns-3-dev/distribution/docker/Dockerfile .
```

First build is **~15–40 minutes** (ns-3 compile time, hardware-dependent);
rebuilds with a cached builder layer take ~8 minutes.

The multi-stage build keeps `./ns3 run` working inside the runtime image:
cmake/ninja/g++ and the same dev headers as the builder stage are installed in
the runtime stage, the build path matches the runtime path
(`/home/ntn/ns-3-dev`, UID 1000), and `LD_LIBRARY_PATH` is baked into the
image environment. Don't strip any of these — `./ns3 run` re-drives CMake on
every invocation and breaks without them.

## Run

### Just a shell

```bash
docker run --rm -it uzairdocker69/ns3-ntn-toolkit:latest bash
# inside the container:
./ns3 show profile
./ns3 run "ntn-tn-integrated-analysis --algorithm=tte-aware --simTime=10 --numTnUes=4"
./ns3 run "ntn-oran-qos-flows"          # 4 5QI flows + C&C on a real NR NTN cell
./ns3 run "oran-ntn-ric-placement-ab"   # measured RIC-placement reaction times
```

### Full stack with observability backend

```bash
docker compose -f distribution/docker/docker-compose.yml up -d
# Toolkit API → http://localhost:8090
# InfluxDB    → http://localhost:8086  (admin / adminadmin)
# Grafana     → http://localhost:3000  (admin / admin)
```

## Push to Docker Hub

```bash
docker login
docker tag uzairdocker69/ns3-ntn-toolkit:latest uzairdocker69/ns3-ntn-toolkit:2.1.0
docker push uzairdocker69/ns3-ntn-toolkit:2.1.0
docker push uzairdocker69/ns3-ntn-toolkit:latest
```

## Image size

Multi-stage build: the builder stage configures and compiles; the runtime
stage carries the built tree plus the toolchain `./ns3 run` needs. Expect
**~7.5 GB extracted** — the SNS3 satellite module's antenna-pattern and
constellation data accounts for most of it.

## What's NOT in the image

- **NVIDIA Sionna RT** — requires NVIDIA Container Toolkit + CUDA. Run
  separately on a GPU host; the toolkit talks to it over UDP.
- **FlexRIC live mode** — has its own docker-compose at
  `contrib/oran-ntn/flexric-bridge/docker/`. Bring up alongside.
- Papers, run outputs, and internal planning documents — excluded by
  `.dockerignore`; the image contains only the published source tree plus the
  SNS3 satellite dependency.
