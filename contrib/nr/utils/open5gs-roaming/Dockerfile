# Copyright (c) 2026 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
#
# SPDX-License-Identifier: GPL-2.0-only

# Open5GS (visited network) and nr-bridge (NGAP/NAS of the gNBs and UEs of ns-3, and SEPP of
# their home network) in one image.
FROM docker.io/library/golang:1.23 AS bridge
# the "packetrusher" build context: the nr-bridge branch of PacketRusher (docker-compose.yml)
COPY --from=packetrusher . /packetrusher
WORKDIR /packetrusher
RUN go build -o /nr-bridge ./cmd/nr-bridge

FROM docker.io/library/ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common gpg-agent \
    && add-apt-repository -y ppa:open5gs/latest \
    && apt-get update \
    && apt-get install -y --no-install-recommends open5gs iproute2 iputils-ping ethtool iptables \
    && rm -rf /var/lib/apt/lists/*
COPY --from=bridge /nr-bridge /usr/local/bin/nr-bridge
COPY open5gs.yaml /etc/open5gs/roaming.yaml
COPY nr-bridge.json /etc/nr-bridge.json
# run.py sets up the network and starts the network functions
ENTRYPOINT ["sleep", "infinity"]
