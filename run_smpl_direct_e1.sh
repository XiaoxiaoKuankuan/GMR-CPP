#!/usr/bin/env bash
# GEM2 SMPL joint centers -> original GMR IK -> E1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IK_CONFIG="${SMPL_DIRECT_IK_CONFIG:-$ROOT/config/ik_configs/smpl_direct_to_e1.json}"
ADAPTER_CONFIG="${SMPL_DIRECT_ADAPTER_CONFIG:-/home/weili/GENMO/config/gmr/smpl_direct_e1_adapter.json}"

if [[ ! -f "$IK_CONFIG" ]]; then
    echo "[ERROR] SMPL-direct IK config not found: $IK_CONFIG" >&2
    exit 1
fi
if [[ ! -f "$ADAPTER_CONFIG" ]]; then
    echo "[ERROR] SMPL-direct adapter config not found: $ADAPTER_CONFIG" >&2
    exit 1
fi

echo "[SMPL direct] IK=$IK_CONFIG"
echo "[SMPL direct] GENMO adapter=$ADAPTER_CONFIG"

exec "$ROOT/run_gem.sh" \
    --ik-config "$IK_CONFIG" \
    --gem-protocol gem2 \
    --viewer-follow-body base_link \
    "$@"
