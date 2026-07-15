#!/usr/bin/env bash
# Calibrated GEM anatomical segments -> original GMR IK -> Unitree E1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALIBRATED="$ROOT/config/ik_configs/gem_segments_to_e1_calibrated.json"

if [[ ! -f "$CALIBRATED" ]]; then
    echo "[ERROR] calibrated GEM segment config not found: $CALIBRATED" >&2
    echo "Run scripts/calibrate_gem_segments_to_e1.py while holding the E1 neutral reference pose." >&2
    exit 1
fi

exec "$ROOT/run_gem.sh" \
    --ik-config "$CALIBRATED" \
    --vis \
    --viewer-follow-body base_link \
    "$@"
