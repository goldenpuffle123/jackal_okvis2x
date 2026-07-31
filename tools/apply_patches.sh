#!/usr/bin/env bash
# Apply the local OKVIS2-X patches, skipping any that are already in the tree.
# Run from the repository root (pixi tasks always do).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${REPO}/okvis_ws/src/OKVIS2-X"

if [ ! -d "${SRC}" ]; then
    echo "error: ${SRC} not found -- run the clone task first" >&2
    exit 1
fi

for patch in "${REPO}"/patches/*.patch; do
    name="$(basename "${patch}")"
    if git -C "${SRC}" apply --reverse --check "${patch}" 2>/dev/null; then
        echo "skip  ${name} (already applied)"
    elif git -C "${SRC}" apply --check "${patch}" 2>/dev/null; then
        git -C "${SRC}" apply "${patch}"
        echo "apply ${name}"
    else
        echo "error: ${name} does not apply cleanly to ${SRC}" >&2
        echo "       (upstream moved? re-create it from your local changes)" >&2
        exit 1
    fi
done
