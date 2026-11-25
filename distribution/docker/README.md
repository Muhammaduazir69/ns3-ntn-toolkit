# Docker distribution

Pre-built ns-3.49 + 5G-LENA v5.2 + 14 bundled modules + Python utilities, published as
[`uzairdocker69/ns3-ntn-toolkit`](https://hub.docker.com/r/uzairdocker69/ns3-ntn-toolkit).

## Build the image

Release images are built from committed source only, never from the working
tree, so run outputs, virtualenvs and untracked files can not reach the image:

```bash
distribution/docker/build-release.sh 2.7.0     # tags 2.7.0 and latest
```

The script stages a context next to the clone (`git archive HEAD` of
ns-3-dev, each nested module repository overlaid with its own `git archive`,
the SNS3 satellite tree without its 1.4 GB of sample outputs), removes paper
and draft folders, aborts on any forbidden path or secret-looking string, and
runs `docker build`. By hand, the context is the **parent directory of a clone
named `ns-3-dev`**:

```bash
git clone https://github.com/Muhammaduazir69/ns3-ntn-toolkit.git ns-3-dev
cd ns-3-dev/contrib && git clone https://github.com/sns3/sns3-satellite.git satellite && cd ../..
docker build -t uzairdocker69/ns3-ntn-toolkit:latest \
    -f ns-3-dev/distribution/docker/Dockerfile .
```

The image is compiled with the `release` profile (-O3 without
`-march=native`), so it runs on any x86-64 CPU, not only on the build host.
A first build takes 15 to 40 minutes depending on the machine.

The multi-stage build keeps `./ns3 run` working inside the runtime image:
cmake/ninja/g++ and the same dev headers as the builder stage are installed in
the runtime stage, the build path matches the runtime path
(`/home/ntn/ns-3-dev`, UID 1000), and `LD_LIBRARY_PATH` is baked into the
image environment. Don't strip any of these: `./ns3 run` re-drives CMake on
every invocation and breaks without them.

## Run

### Just a shell

```bash
docker run --rm -it uzairdocker69/ns3-ntn-toolkit:latest bash
# inside the container:
./ns3 show profile
./ns3 run "ntn-constellation-walker-traffic --simSeconds=10"   # 44-sat Walker shell, NR cell, ISL routes
./ns3 run "ntn-real-stack-smoke"        # TR 38.821 Set-1 NR NTN cell, measured KPIs
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
docker push uzairdocker69/ns3-ntn-toolkit:2.7.0
docker push uzairdocker69/ns3-ntn-toolkit:latest
```

## Image size

Multi-stage build: the builder stage configures and compiles; the runtime
stage carries the built tree plus the toolchain `./ns3 run` needs. Expect
**~7.5 GB extracted**: the SNS3 satellite module's antenna-pattern and
constellation data accounts for most of it.

## What's NOT in the image

- **NVIDIA Sionna RT**: requires NVIDIA Container Toolkit + CUDA. Run
  separately on a GPU host; the toolkit talks to it over UDP. The ntn-sionna
  examples stop with an error when no server answers, rather than silently
  using free space. To try them without a GPU, start the protocol stub (closed-form
  FSPL) on the default port first:
  `python3 contrib/ntn-sionna/test/sionna-stub-server.py --port 8765 &`,
  or pass `--allowFallback=1` to the example.
- **FlexRIC live mode**: has its own docker-compose at
  `contrib/oran-ntn/flexric-bridge/docker/`. Bring up alongside.
- Papers, run outputs, and internal planning documents: `build-release.sh`
  stages only committed source and aborts on any such path; the image contains
  the published source tree plus the SNS3 satellite dependency.
