#!/usr/bin/env bash
# Build the multi-track image.
#
#     ./scripts/build.sh                      -> autodrive_racer:multi-track
#     IMAGE=me/roboracer TAG=mt ./scripts/build.sh
#
# Run it from anywhere; the build context is the repo root.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-autodrive_racer}"
TAG="${TAG:-multi-track}"
BASE_TAG="${BASE_TAG:-2026-iros-practice}"

echo "building ${IMAGE}:${TAG} from autodriveecosystem/autodrive_roboracer_api:${BASE_TAG}"
exec docker build \
    -f "$REPO/Dockerfile" \
    --build-arg "BASE_TAG=${BASE_TAG}" \
    -t "${IMAGE}:${TAG}" \
    "$REPO"
