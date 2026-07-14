#!/usr/bin/env bash
#
# sync-publish.sh — incremental, in-place update of the PUBLISHED toolkit image.
# =============================================================================
#
# THE PROBLEM THIS SOLVES
# -----------------------
# A from-scratch `docker build` recompiles every shared library, so the whole
# built-tree layer (and, with the layer split, the 5 GB satellite corpus too)
# gets a brand-new content digest and re-uploads on EVERY push — gigabytes a
# day even when one .cc changed. Docker can only skip a layer whose exact bytes
# already exist on the registry; a clean rebuild changes those bytes wholesale.
#
# THE FIX
# -------
# Rebase today's working tree ON TOP OF the image that is already on Docker Hub,
# rebuild only what changed (incremental ninja), and commit the FILESYSTEM DIFF
# as one thin layer. `docker push` then uploads only that diff; every invariant
# layer is reused:
#
#     satellite TLE corpus (5 GB)  -> reused from base, never re-pushed
#     apt toolchain / python venv  -> reused from base, never re-pushed
#     only changed source + the .so ninja relinked  -> uploaded
#
# Typical first run after a big feature week: a couple hundred MB to ~2 GB (the
# binaries that genuinely changed). Every day after that: tens of MB.
#
# WHY SOURCE-ONLY SYNC (important)
# --------------------------------
# ns-3 bakes the absolute install path (/home/ntn/ns-3-dev) into build/,
# cmake-cache/ and .lock-ns3_* at configure time. The host tree is configured
# for a DIFFERENT path (/home/uzair/...), so its build artifacts are NOT
# portable into the container. We therefore sync SOURCE ONLY and let the build
# run inside the container against its own correctly-pathed build/ dir.
#
# USAGE
# -----
#   sync-publish.sh [BASE_TAG] [PUSH_TAG] [FORCE_CONFIGURE]
#
#   sync-publish.sh                    # rebase :latest -> push :latest
#   sync-publish.sh 2.2.1 latest 1     # rebase the published 2.2.1, force a
#                                      # configure (new modules), push :latest
#   sync-publish.sh latest latest      # daily steady-state: rebase + push :latest
#
# BASE_TAG must be a tag whose layers are ALREADY on the registry (so they are
# reused, not re-pushed). After the first successful run, :latest itself is that
# tag, so the steady-state daily call is just `sync-publish.sh`.
#
# Run it from anywhere; paths below are absolute. Requires: docker, host rsync.
#
# PERIODIC RE-BASELINE
# --------------------
# Each run adds one thin commit layer on top of the previous :latest. PUSH size
# stays small, but the layer COUNT (and therefore a fresh `docker pull` size)
# grows slowly over time as shadowed copies accumulate. Roughly monthly, collapse
# it: do ONE from-scratch build with distribution/docker/Dockerfile and push that
# (a new flat base). The next sync-publish run rebases on it automatically.
# =============================================================================
set -euo pipefail

IMAGE="uzairdocker69/ns3-ntn-toolkit"
# BASE_TAG  = the image already on the registry that we rebase onto (its layers
#             are reused, never re-pushed). Pass the real published tag here.
# PUSH_TAG  = the tag we commit + push the updated image to (same tag daily;
#             no new version is created).
BASE_TAG="${1:-latest}"
PUSH_TAG="${2:-latest}"
FORCE_CONFIGURE="${3:-0}"
BASE="${IMAGE}:${BASE_TAG}"
TARGET="${IMAGE}:${PUSH_TAG}"
SRC="/home/uzair/6g_ntn_ns3/ns-3-dev"
CONTAINER="ntn-toolkit-live"

say() { printf '\n\033[1;36m>> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------------------
# 1. Builder container created FRESH from ${BASE} on every run.
#
#    This is deliberate, not wasteful. `docker commit` snapshots the container's
#    writable layer = everything that changed since the container was CREATED.
#    If we reused a long-lived container, that writable layer would accumulate
#    EVERY day's changes and re-push a growing cumulative blob. By recreating
#    from the image we are about to update (BASE defaults to :latest = yesterday's
#    push), the commit captures ONLY today's delta, so the pushed layer stays
#    thin day after day. The image's own build/ dir (preserved in BASE) keeps
#    ninja incremental, so the rebuild is still fast.
#
#    We only pull if BASE is absent locally (after the first run :latest is local
#    anyway). rsync is baked into the image after the first run, so the apt
#    install only happens if it is somehow missing.
# ---------------------------------------------------------------------------
say "creating fresh builder from ${BASE}"
docker image inspect "${BASE}" >/dev/null 2>&1 || docker pull "${BASE}"
docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
docker run -d --name "${CONTAINER}" -u root "${BASE}" sleep infinity
if ! docker exec "${CONTAINER}" sh -c 'command -v rsync' >/dev/null 2>&1; then
  say "installing rsync into the builder (one-time; baked into image after commit)"
  docker exec -u root "${CONTAINER}" bash -lc \
    'apt-get update && apt-get install -y --no-install-recommends rsync && rm -rf /var/lib/apt/lists/*'
fi

# ---------------------------------------------------------------------------
# 2. Sync SOURCE into the container, preserving mtimes (so ninja stays
#    incremental) and deleting files removed on the host. Everything that
#    bakes an absolute path or is huge-and-invariant is excluded so it is
#    reused from the base layers and never re-pushed.
# ---------------------------------------------------------------------------
# NOTE: *.pb.h/*.pb.cc/*_pb2.py are protoc-GENERATED (not tracked) and the host
# protoc (3.21) emits code incompatible with the container's libprotobuf (3.12).
# Excluding them lets the container regenerate matching pb sources with its own
# protoc — otherwise the synced host pb files cause "abstract type DataContainer".
say "rsync source into builder (build/, cmake-cache/, satellite corpus, generated pb excluded)"
rsync -a --delete --info=stats1 \
  --rsh='docker exec -i' \
  --exclude 'build/' \
  --exclude 'cmake-cache/' \
  --exclude '.lock-ns3_*' \
  --exclude 'contrib/satellite/data/' \
  --exclude '.git/' \
  --exclude '**/.venv/' \
  --exclude '**/__pycache__/' \
  --exclude '*.pcap' \
  --exclude '*_kpm_series.*' \
  --exclude '*.pb.h' \
  --exclude '*.pb.cc' \
  --exclude '*_pb2.py' \
  "${SRC}/" "${CONTAINER}:/home/ntn/ns-3-dev/"

# rsync ran as root; hand the SYNCED SOURCE back to the runtime user (uid 1000).
#
# CRITICAL: do NOT `chown -R` the whole tree. overlayfs copies up a file on ANY
# chown syscall — even a no-op one to the same owner — so a blanket recursive
# chown over build/ (already 1000:1000, ~6 GB) would copy up the entire build
# tree into the commit layer, making every push a full-image (~6 GB) upload
# instead of the intended thin delta. Prune build/ and cmake-cache/ (never
# rsync'd, already 1000-owned) and only touch files that are genuinely not
# owned by uid/gid 1000 — i.e. just the freshly rsync'd source.
docker exec -u root "${CONTAINER}" bash -lc \
  'find /home/ntn/ns-3-dev \
      -path /home/ntn/ns-3-dev/build -prune -o \
      -path /home/ntn/ns-3-dev/cmake-cache -prune -o \
      \( -not -uid 1000 -o -not -gid 1000 \) -exec chown 1000:1000 {} +'

# ---------------------------------------------------------------------------
# 3. Build inside the container. A reconfigure is needed the first time a brand
#    new contrib module (e.g. nr) or new example .cc appears; after that a plain
#    incremental build only recompiles what changed.
# ---------------------------------------------------------------------------
if [ "${FORCE_CONFIGURE}" = "1" ] || \
   ! docker exec -u ntn "${CONTAINER}" test -d /home/ntn/ns-3-dev/cmake-cache; then
  say "configure (new modules/examples detected or forced)"
  docker exec -u ntn "${CONTAINER}" bash -lc \
    'cd /home/ntn/ns-3-dev && ./ns3 configure --enable-tests --enable-examples --build-profile=optimized'
fi
say "incremental ./ns3 build"
docker exec -u ntn "${CONTAINER}" bash -lc 'cd /home/ntn/ns-3-dev && ./ns3 build -j"$(nproc)"'

# ---------------------------------------------------------------------------
# 4. Commit the filesystem diff back onto the SAME tag (no new version). The
#    base image config (USER/WORKDIR/ENV/EXPOSE/CMD/LABELS) is inherited.
# ---------------------------------------------------------------------------
say "commit diff -> ${TARGET}"
docker commit "${CONTAINER}" "${TARGET}"

# ---------------------------------------------------------------------------
# 5. Push. Base layers report "Layer already exists"; only the diff uploads.
# ---------------------------------------------------------------------------
say "push ${TARGET} (only the changed-file layer uploads)"
docker push "${TARGET}"

say "done — satellite corpus + toolchain reused from base; pushed only the delta."
