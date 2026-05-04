# SPDX-License-Identifier: GPL-2.0-only
# Builds FlexRIC (EURECOM) + the asn1c toolchain + this bridge.
#
# Two-stage:
#   1. flexric-builder — clones FlexRIC, builds asn1c-compiled libs.
#   2. ntn-flexric    — runtime image with the bridge installed.
#
# The image lets us launch the live RIC (`flexric-builder`) and the
# bridge / xApps (`ntn-flexric`) under one docker-compose invocation.
# Tested against FlexRIC tag `br-flexric-ntn-2026.05` — change as needed.

FROM ubuntu:22.04 AS flexric-builder
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake pkg-config \
        gcc-12 g++-12 \
        autoconf automake libtool \
        libsctp-dev lksctp-tools \
        flex bison \
        libpcre2-dev libssl-dev \
        ca-certificates \
        python3 python3-dev \
    && rm -rf /var/lib/apt/lists/*

# FlexRIC's mac_dec_generic.h triggers a GCC 11 internal compiler error
# (segfault on heavily nested generic macros). GCC 12+ compiles it cleanly.
ENV CC=gcc-12 CXX=g++-12

# asn1c — used by FlexRIC at build time. autoreconf must succeed (no `|| true`)
# so an actual failure surfaces instead of getting masked into a configure error.
RUN git clone --depth=1 https://github.com/mouse07410/asn1c.git /tmp/asn1c \
    && cd /tmp/asn1c \
    && autoreconf -iv \
    && ./configure --prefix=/usr/local && make -j"$(nproc)" && make install \
    && rm -rf /tmp/asn1c

# SWIG ≥ 4.1 — FlexRIC's src/xApp/swig CMake step rejects 4.0.x (Jammy ships 4.0.2).
# We build 4.2.1 from source rather than pulling a backports PPA so the image stays
# pinned and reproducible.
RUN git clone --depth=1 --branch=v4.2.1 https://github.com/swig/swig.git /tmp/swig \
    && cd /tmp/swig \
    && ./autogen.sh \
    && ./configure --prefix=/usr/local && make -j"$(nproc)" && make install \
    && rm -rf /tmp/swig

# FlexRIC.
ARG FLEXRIC_REF=master
RUN git clone --depth=1 --branch="${FLEXRIC_REF}" \
        https://gitlab.eurecom.fr/mosaic5g/flexric.git /opt/flexric
RUN mkdir /opt/flexric/build \
    && cd /opt/flexric/build \
    && cmake -DCMAKE_BUILD_TYPE=Release \
             -DE2AP_VERSION=E2AP_V2 \
             -DKPM_VERSION=KPM_V2_03 .. \
    && make -j"$(nproc)" \
    && make install

# ----------------------------------------------------------------------------
FROM ubuntu:22.04 AS ntn-flexric
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
        libsctp1 lksctp-tools \
        python3 python3-pip python3-venv \
        ca-certificates iproute2 tcpdump \
    && rm -rf /var/lib/apt/lists/*

# Bring over FlexRIC binaries.
COPY --from=flexric-builder /usr/local /usr/local
COPY --from=flexric-builder /opt/flexric/build/examples/ric/nearRT-RIC /usr/local/bin/nearRT-RIC

# Install the bridge. Upgrade setuptools first — Jammy ships 59.x which predates
# PEP 621, so the [project] table in pyproject.toml gets ignored and the build
# silently produces an UNKNOWN-0.0.0 wheel with no entry-points.
WORKDIR /opt/ntn-flexric-bridge
COPY pyproject.toml ./
COPY src ./src
RUN pip3 install --no-cache-dir --upgrade "setuptools>=68" "pip>=23" \
    && pip3 install --no-cache-dir .

EXPOSE 36421/tcp 36421/sctp 36422/sctp
CMD ["nearRT-RIC", "-c", "/etc/flexric/ric.conf"]
