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

BASE=165b49152a25195d3ee04d77699a7bc184fb9ddf   # sns3-satellite, 21 Jul 2025
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
git clone --quiet --no-checkout "$REPO" "$DEST"
git -C "$DEST" checkout --quiet "$BASE"
for p in "$HERE"/[0-9][0-9]-*.patch; do
  (cd "$DEST" && patch -p1 --quiet --no-backup-if-mismatch < "$p")
  echo "applied $(basename "$p")"
done
if [ "$WITH_DATA" = 1 ]; then
  DATA_COMMIT=$(git -C "$DEST" ls-tree "$BASE" data | awk '{print $3}')
  rm -rf "$DEST/data"
  git init --quiet "$DEST/data"
  git -C "$DEST/data" fetch --quiet --depth 1 "$DATA_REPO" "$DATA_COMMIT"
  git -C "$DEST/data" checkout --quiet FETCH_HEAD
  echo "sns3-data at $DATA_COMMIT"
fi
echo "SNS3 ready in $DEST (base $BASE + toolkit patches)"
