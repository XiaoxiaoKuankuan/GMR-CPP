#!/usr/bin/env bash
# Fixed SMP1 -> BUMI3 jump chain. Per-frame grounding is forcibly disabled so
# simultaneous vertical foot/pelvis motion remains observable.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/smplx_bumi3_server"
XML="${BUMI3_XML:-$ROOT/assets/bumi3/mjcf/bumi3.xml}"
IK_CONFIG="${BUMI3_JUMP_IK_CONFIG:-$ROOT/config/ik_configs/smplx_to_bumi3_jump.json}"
UDP_PORT="${BUMI3_JUMP_UDP_PORT:-7007}"
GROUND_OFFSET="${BUMI3_JUMP_GROUND_OFFSET:-0.6}"
REDIS_KEY="${BUMI3_REDIS_KEY:-smplx_online_frame_bumi3_jump}"

for path in "$EXECUTABLE" "$XML" "$IK_CONFIG"; do
    if [[ ! -e "$path" ]]; then
        echo "[ERROR] required SMPL-X BUMI3 jump file not found: $path" >&2
        echo "Run ./build.sh if the executable is missing." >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"
echo "[run_smplx_bumi3_jump] mode=jump SMP1=0.0.0.0:$UDP_PORT -> BUMI3"
echo "[run_smplx_bumi3_jump] XML=$XML"
echo "[run_smplx_bumi3_jump] IK=$IK_CONFIG"
echo "[run_smplx_bumi3_jump] offset_to_ground=off fixed_ground_offset=${GROUND_OFFSET}m ground_clearance=0.02m"
echo "[run_smplx_bumi3_jump] Redis disabled by default: publish order is unverified"

exec "$EXECUTABLE" \
    --xml "$XML" \
    --ik-config "$IK_CONFIG" \
    --port "$UDP_PORT" \
    --redis-key "$REDIS_KEY" \
    --no-redis \
    "$@" \
    --no-offset-to-ground \
    --fixed-ground-offset "$GROUND_OFFSET" \
    --ground-clearance 0.02
