#!/usr/bin/env bash
# Build the submission image.
#
#     ./scripts/build.sh                      -> autodrive_racer:iros-2026-final
#     IMAGE=me/roboracer TAG=final ./scripts/build.sh
#
# Run it from anywhere; the build context is the repo root, which is what the
# COPY paths in the Dockerfile are written against.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-iros-2026-final}"
BASE_TAG="${BASE_TAG:-2026-iros-practice}"

echo "building ${IMAGE}:${TAG} from autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}"
exec docker build \
    -f "$REPO/Dockerfile" \
    --build-arg "BASE_TAG=${BASE_TAG}" \
    -t "${IMAGE}:${TAG}" \
    "$REPO"
