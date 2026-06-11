# syntax=docker/dockerfile:1.7
#
# ns3-ntn-toolkit — pre-built ns-3.43 + 13 contrib modules + Python utilities
#
# Build:  docker build -t uzairdocker69/ns3-ntn-toolkit:latest -f distribution/docker/Dockerfile .
# Run:    docker run --rm -it -p 8090:8090 -p 3000:3000 uzairdocker69/ns3-ntn-toolkit:latest
#
# Maintainer: Muhammad Uzair, Independent Researcher
# License:    GPL-2.0-only

ARG UBUNTU_VERSION=22.04
ARG NS3_VERSION=3.43

# -----------------------------------------------------------------------------
# Stage 1: builder — clone, configure, build ns-3 + contrib modules
#
# IMPORTANT: the builder and runtime stages share the SAME absolute install
# path (/home/ntn/ns-3-dev) and the SAME UID (1000). ns-3 bakes absolute
# paths into `.lock-ns3_*` and `cmake-cache/CMakeCache.txt` at configure time,
# so building under one path and copying to another silently breaks
# `./ns3 run` (it'll look for binaries under the builder path and report
# "Executable has not been built yet"). See
# https://github.com/Muhammaduazir69/ns3-ntn-toolkit/issues/2.
# -----------------------------------------------------------------------------
FROM ubuntu:${UBUNTU_VERSION} AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake ninja-build pkg-config \
        g++ gcc \
        python3 python3-dev python3-venv python3-pip \
        libgsl-dev libxml2-dev libsqlite3-dev libpcap-dev \
        libboost-all-dev \
        pybind11-dev \
        protobuf-compiler libprotobuf-dev \
        libabsl-dev \
        libsctp-dev \
        ca-certificates git \
        wget curl \
    && rm -rf /var/lib/apt/lists/*

# Match the runtime UID/GID so the copied tree and the user agree without a
# chown sweep at the end of the build.
RUN groupadd --gid 1000 ntn \
    && useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash ntn

USER ntn
WORKDIR /home/ntn/ns-3-dev

# Copy the toolkit source tree into the image.
# (The .dockerignore at repo root excludes build artefacts and venvs.)
COPY --chown=ntn:ntn ns-3-dev/ /home/ntn/ns-3-dev/

# Configure libraries + tests + examples. As of v2 the image ships with
# examples built so every reference scenario (TTE-aware CHO, orbital RRC,
# SAGIN flight-LEO, O-RAN full scenario, THz, the new cross-module
# *-traffic examples, …) is runnable out of the box with `./ns3 run "<name>"`.
# The multi-bss WiFi VR-app under ns3-ai-ntn pulls in Abseil, hence the
# `libabsl-dev` dependency installed above.
RUN ./ns3 configure \
        --enable-tests \
        --enable-examples \
        --build-profile=optimized \
    && ./ns3 build -j"$(nproc)"

# -----------------------------------------------------------------------------
# Stage 2: runtime — slim image with the built ns-3 + Python utilities
# -----------------------------------------------------------------------------
FROM ubuntu:${UBUNTU_VERSION} AS runtime

ARG DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.title="ns3-ntn-toolkit"
LABEL org.opencontainers.image.description="Open-source ns-3.43 distribution for 6G non-terrestrial network research: 15 integrated modules covering SGP4 LEO constellations, 3GPP NR-NTN, O-RAN + FlexRIC, measured in-band QoS-flow KPIs, multi-tier RIC, sub-THz physics, NVIDIA Sionna, SAGIN, V2X, slicing, AI/ML, digital twin."
LABEL org.opencontainers.image.url="https://ns3-ntn-toolkit.dev/"
LABEL org.opencontainers.image.source="https://github.com/Muhammaduazir69/ns3-ntn-toolkit"
LABEL org.opencontainers.image.documentation="https://ns3-ntn-toolkit.dev/"
LABEL org.opencontainers.image.authors="Muhammad Uzair <muhammaduzairr69@gmail.com>"
LABEL org.opencontainers.image.licenses="GPL-2.0-only"
LABEL org.opencontainers.image.version="2.1.0"

# Runtime deps — note this is intentionally NOT a slim "runtime-only" set.
# ns-3's `./ns3 run` wrapper unconditionally invokes the CMake build system
# to perform an incremental rebuild check before launching a scenario (see
# Muhammaduazir69/ns3-ntn-toolkit#2). That step:
#   1. requires cmake + ninja + g++ on PATH, otherwise every `./ns3 run …`
#      aborts with "Error:  CMake not found; please install …";
#   2. re-runs every `find_package(...)` from contrib modules' CMakeLists
#      — so removing dev headers (Boost, GSL, libxml2, sqlite, libpcap,
#      protobuf, pybind11) makes the reconfigure fail with errors like
#      "Could NOT find Boost: Found unsuitable version 0.0.0".
# Mirroring the builder's apt install in the runtime stage is the only way
# to keep the documented `./ns3 run …` UX working AND to let users opt back
# into examples with `./ns3 configure --enable-examples` inside the
# container. Cost: ~300 MB of headers/toolchain on top of the runtime libs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake ninja-build pkg-config \
        g++ gcc \
        python3 python3-dev python3-venv python3-pip \
        libgsl-dev libxml2-dev libsqlite3-dev libpcap-dev \
        libboost-all-dev \
        pybind11-dev \
        protobuf-compiler libprotobuf-dev \
        libabsl-dev \
        libsctp-dev \
        ca-certificates git \
        wget curl \
    && rm -rf /var/lib/apt/lists/*

# Non-root user for the runtime container — same UID/GID as builder so the
# tree we COPY --from=builder keeps its ownership and the embedded absolute
# paths in .lock-ns3 / CMakeCache.txt stay valid.
RUN groupadd --gid 1000 ntn \
    && useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash ntn

USER ntn
WORKDIR /home/ntn

# Copy the built ns-3 tree (built at the same /home/ntn/ns-3-dev path).
COPY --from=builder --chown=ntn:ntn /home/ntn/ns-3-dev /home/ntn/ns-3-dev

# Install Python utilities (in-tree, editable).
RUN python3 -m venv /home/ntn/.venv \
    && /home/ntn/.venv/bin/pip install --upgrade pip wheel setuptools \
    && /home/ntn/.venv/bin/pip install \
        /home/ntn/ns-3-dev/contrib/ntn-constellation \
        /home/ntn/ns-3-dev/contrib/ntn-digital-twin

ENV PATH="/home/ntn/.venv/bin:/home/ntn/ns-3-dev:${PATH}"
# ns-3's `./ns3 run` wrapper sets LD_LIBRARY_PATH for its child processes,
# but users invoking the built binaries directly (e.g. with `gdb` or
# `valgrind`, or scripting them from outside ns-3) need this on their
# environment. Setting it image-wide is cheap and removes a footgun.
ENV LD_LIBRARY_PATH="/home/ntn/ns-3-dev/build/lib"
ENV PYTHONUNBUFFERED=1

# Useful default workdir for users.
WORKDIR /home/ntn/ns-3-dev

# Smoke test (lightweight): ns-3 reports its version/profile, contrib modules
# are linked into libraries, and the Python utilities are importable.
# We intentionally don't run a full scenario here — scenario failures should
# surface in the user's `docker run`, not as an image-build failure (so users
# can still pull the image, drop into a shell, and debug).
RUN ./ns3 show version \
    && ./ns3 show profile \
    && test -e build/lib/libns3.43-ntn-cho-optimized.so \
    && test -e build/lib/libns3.43-oran-ntn-optimized.so \
    && test -e build/lib/libns3.43-thz-ntn-optimized.so \
    && test -e build/contrib/ntn-constellation/examples/ns3.43-ntn-constellation-walker-traffic-optimized \
    && test -e build/contrib/ntn-fapi/examples/ns3.43-ntn-fapi-leo-pass-slotloop-optimized \
    && python3 -c "import ntn_constellation, ntn_digital_twin; print('Python utilities OK')"

# Ports:
#   8090 — ntn-twin-api (FastAPI)
#   3000 — Grafana (when observability stack is brought up)
EXPOSE 8090 3000

# Default: drop into a shell. Users can override with `docker run … cmd`.
CMD ["/bin/bash"]
