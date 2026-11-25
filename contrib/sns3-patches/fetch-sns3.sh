#!/usr/bin/env bash
# Fetch the SNS3 satellite module exactly as the toolkit builds it, into contrib/satellite.
#
# usage (from the ns-3 root):  contrib/sns3-patches/fetch-sns3.sh [--no-data]
#
# 1. clones sns3-satellite and checks out the pinned base commit;
# 2. applies the toolkit patch series (01-*.patch ...);
# 3. fetches the sns3-data submodule at the commit the base pins (about 2 GB; build-only users and
#    CI jobs that run no SNS3-backed test can skip it with --no-data).
#
# The toolkit's published results were produced with this tree. Cloning upstream HEAD instead
# gives a different SNS3 (newer upstream code, and the `traffic` module where the toolkit uses
# `ntn-traffic`).
set -euo pipefail

BASE=2c0c6d4d7fd1628eda1b923a7e135749513769fa   # sns3-satellite tag 3.48 (validated with ns-3.48), 8 Jul 2026
REPO=https://github.com/sns3/sns3-satellite.git
DATA_REPO=https://github.com/sns3/sns3-data.git
HERE=$(cd "$(dirname "$0")" && pwd)
DEST=${SNS3_DEST:-contrib/satellite}
WITH_DATA=1
[ "${1:-}" = "--no-data" ] && WITH_DATA=0

if [ -e "$DEST/CMakeLists.txt" ]; then
  echo "$DEST already exists; remove it first to fetch a fresh copy" >&2
  exit 1
fi
# Only the pinned commit (depth 1): a full-history clone is a large transfer that can drop
# ("early EOF"). Retried, because a dropped transfer cannot resume.
retry() { local n; for n in 1 2 3 4 5; do "$@" && return 0; echo "attempt $n failed; retrying" >&2; sleep $((n * 10)); done; return 1; }
git init --quiet "$DEST"
retry git -C "$DEST" fetch --quiet --depth 1 "$REPO" "$BASE"
git -C "$DEST" checkout --quiet FETCH_HEAD
for p in "$HERE"/[0-9][0-9]-*.patch; do
  (cd "$DEST" && patch -p1 --quiet --no-backup-if-mismatch < "$p")
  echo "applied $(basename "$p")"
done
if [ "$WITH_DATA" = 1 ]; then
  DATA_COMMIT=$(git -C "$DEST" ls-tree HEAD data | awk '{print $3}')
  rm -rf "$DEST/data"
  mkdir -p "$DEST/data"
  # A plain HTTPS tarball with retries: a 2 GB shallow git fetch of sns3-data can drop mid-transfer
  # ("early EOF"), and git cannot resume it.
  TGZ="$DEST/.sns3-data.tar.gz"
  if retry curl -fL --retry 8 --retry-delay 5 --retry-all-errors -sS -o "$TGZ" \
        "https://codeload.github.com/sns3/sns3-data/tar.gz/$DATA_COMMIT" \
      && tar -xzf "$TGZ" -C "$DEST/data" --strip-components=1; then
    rm -f "$TGZ"
  else
    rm -f "$TGZ"
    echo "tarball download failed; trying git" >&2
    rm -rf "$DEST/data"
    git init --quiet "$DEST/data"
    retry git -C "$DEST/data" fetch --quiet --depth 1 "$DATA_REPO" "$DATA_COMMIT"
    git -C "$DEST/data" checkout --quiet FETCH_HEAD
  fi
  echo "sns3-data at $DATA_COMMIT"
fi
echo "SNS3 ready in $DEST (base $BASE + toolkit patches)"
