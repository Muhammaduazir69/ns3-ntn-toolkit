#!/usr/bin/env bash
#
# release-v2.5.0.sh — publish the v2.5.0 artifact set in one command.
# ===================================================================
#
# WHY THIS EXISTS
# ---------------
# The OJ-COMS manuscript's Data and Code Availability section pins
#
#     GitHub release   v2.5.0
#     Docker image     uzairdocker69/ns3-ntn-toolkit:2.5.0
#     Figshare DOI     10.6084/m9.figshare.32927594
#
# and NONE of those three resolve to code that produced the paper's tables until
# this runs. Before the 24 August audit the situation was worse and worth
# recording: the manuscript pinned v2.1.0, GitHub had no such release (only
# v2.0.2 existed), the Docker 2.1.0 image predated the paper's own experiment
# commits by six weeks, and the Figshare deposit was frozen at a snapshot older
# still. Three pins, three different wrong answers.
#
# Nothing here is destructive and nothing runs without asking. Every step prints
# what it is about to do and stops on the first failure.
#
# USAGE
#     ./distribution/release-v2.5.0.sh --check      # verify only, publish nothing
#     ./distribution/release-v2.5.0.sh --tag        # + create and push the tag
#     ./distribution/release-v2.5.0.sh --all        # + build and push the image
#
# The Figshare step cannot be automated here: it needs an interactive login.
# The script prints the exact steps at the end.

set -euo pipefail

VERSION="2.5.0"
TAG="v${VERSION}"
IMAGE="uzairdocker69/ns3-ntn-toolkit"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:---check}"

cd "$REPO_ROOT"

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
die() { printf '\033[31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- preflight
say "Preflight"

[ -z "$(git status --porcelain)" ] || die "working tree is dirty; commit or stash first"
git rev-parse --verify HEAD >/dev/null || die "not a git repository"

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
HEAD_SHA="$(git rev-parse --short HEAD)"
echo "  branch      : $BRANCH"
echo "  HEAD        : $HEAD_SHA"
echo "  unpushed    : $(git rev-list --count "origin/${BRANCH}..HEAD" 2>/dev/null || echo '?') commits"

if git rev-parse "$TAG" >/dev/null 2>&1; then
    die "$TAG already exists locally; delete it first if you mean to re-cut it"
fi

# The authorship rule is not negotiable and is cheapest to enforce here, before
# anything leaves the machine. A single AI-credit trailer makes GitHub list a
# bot as a repository contributor, and that cannot be undone by a later commit.
say "Authorship sweep"
RANGE="v2.0.2..HEAD"
if git log --format='%B' "$RANGE" | grep -in 'claude\|anthropic\|co-authored\|🤖'; then
    die "AI-credit text found in commit messages; strip it before publishing"
fi
BAD_AUTHORS="$(git log --format='%an <%ae>' "$RANGE" | sort -u | grep -v 'muhammaduzairr69@gmail.com' || true)"
[ -z "$BAD_AUTHORS" ] || die "unexpected commit authors: $BAD_AUTHORS"
echo "  clean: sole author, no AI credit"

# ---------------------------------------------------------------- test gate
say "Test gate: all NTN suites must be green"
SUITES="ntn-cho ntn-constellation ntn-fapi ntn-observability ntn-oran-ai-flow-monitor
        ntn-oran-application ntn-real-stack-helper ntn-rrc ntn-sagin ntn-sionna
        ntn-slice ntn-spectrum-seam ntn-standards-validation ntn-v2x oran-ntn
        oran-ntn-airan-inference oran-ntn-multi-tier-ric oran-ntn-ws4 thz-ntn
        three-gpp-ntn-propagation-loss-model"
./ns3 build >/dev/null || die "build failed"
FAILED=""
for s in $SUITES; do
    if ./test.py --no-build -s "$s" >/tmp/rel-$s.log 2>&1; then
        printf '  ok   %s\n' "$s"
    else
        printf '  FAIL %s\n' "$s"
        FAILED="$FAILED $s"
    fi
done
[ -z "$FAILED" ] || die "failing suites:$FAILED — a release must not ship red tests"

say "Provenance gates"
python3 tools/check_dashboard_producers.py --repo . || die "dashboard producer check failed"

if [ "$MODE" = "--check" ]; then
    say "Check-only mode: nothing published"
    echo "  re-run with --tag or --all to publish"
    exit 0
fi

# ---------------------------------------------------------------- tag
say "Creating annotated tag $TAG"
git tag -a "$TAG" -m "Release ${VERSION}

All 37 critical findings from the 24 August 2026 R&D audit are fixed, with
regression tests that fail against the code they replace. See
RD_AUDIT_AND_ROADMAP_2026-08-24.md for the full record and
rd-audit-2026-08-24/GAP_REGISTER.md for per-finding evidence.

Headline corrections in this release:
  * satellite EIRP was 18-21 dB above TR 38.821 Set-1 in every example; the
    calibration offset falls from 21.39 dB to 3.74 dB and the elevation-slope
    agreement from 1.25 dB to 0.06 dB
  * rain attenuation above 100 GHz used unsourced coefficients overstating loss
    by 1.7-2x; now ITU-R P.838-3
  * fog attenuation stepped 50.8% across 2 kHz at 100 GHz; now continuous P.840
  * scintillation used P.618's frequency exponent for elevation, under-
    predicting 39% at the cell edge, and advanced per call rather than per
    simulated second
  * control decisions across O-RAN, CHO, DRX and V2X computed outcomes that
    never reached the data plane; they now actuate and are measured
  * CI had never run against this branch and its gate script could not parse
    the current schema"

git push origin "$TAG"
git push gitlab "$TAG" 2>/dev/null || echo "  (gitlab remote push skipped)"
say "Pushing branch $BRANCH"
git push origin "$BRANCH"
git push gitlab "$BRANCH" 2>/dev/null || echo "  (gitlab branch push skipped)"

if [ "$MODE" != "--all" ]; then
    say "Tag published; image not built (use --all)"
    exit 0
fi

# ---------------------------------------------------------------- image
say "Publishing Docker image ${IMAGE}:${VERSION}"
# sync-publish rebases on the published :latest and pushes only the thin delta,
# so this is minutes rather than a 4 GB upload. See its header for why.
./distribution/docker/sync-publish.sh "$VERSION"

# ---------------------------------------------------------------- figshare
say "Figshare — manual, needs an interactive login"
cat <<'EOF'
  The DOI 10.6084/m9.figshare.32927594 is version-independent: uploading a new
  version keeps the same DOI, so the manuscript citation does not change.

    1. https://figshare.com/account/articles/32927594
    2. "Edit item" -> upload a fresh archive of this tag:
         git archive --format=zip --prefix=ns3-ntn-toolkit/ v2.5.0 -o /tmp/ns3-ntn-toolkit-2.5.0.zip
    3. Version note: "v2.5.0 - all 37 critical audit findings fixed; see
       RD_AUDIT_AND_ROADMAP_2026-08-24.md"
    4. Publish. Confirm the DOI still resolves and now lists version 2.

  Until this is done, the Figshare deposit predates the paper's experiments and
  a reviewer following the DOI gets code that cannot reproduce its tables.
EOF

say "Done. Verify all three pins resolve:"
echo "  https://github.com/Muhammaduazir69/ns3-ntn-toolkit/releases/tag/${TAG}"
echo "  https://hub.docker.com/r/${IMAGE}/tags?name=${VERSION}"
echo "  https://doi.org/10.6084/m9.figshare.32927594"
