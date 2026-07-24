#!/usr/bin/env bash
# Fixed SMPL-X SMP1 -> BUMI3 viewer/IK chain. Redis remains opt-in so merely
# opening the Viewer never starts a downstream GMT stream.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/smplx_bumi3_server"
XML="${BUMI3_XML:-$ROOT/assets/bumi3/mjcf/bumi3.xml}"
IK_CONFIG="${BUMI3_IK_CONFIG:-$ROOT/config/ik_configs/smplx_to_bumi3.json}"
UDP_PORT="${BUMI3_UDP_PORT:-7006}"
REDIS_KEY="${BUMI3_REDIS_KEY:-gmt_online_frame_bumi}"

for path in "$EXECUTABLE" "$XML" "$IK_CONFIG"; do
    if [[ ! -e "$path" ]]; then
        echo "[ERROR] required SMPL-X BUMI3 file not found: $path" >&2
        echo "Run ./build.sh if the executable is missing." >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"
echo "[run_smplx_bumi3] SMP1 0.0.0.0:$UDP_PORT -> BUMI3"
echo "[run_smplx_bumi3] XML=$XML"
echo "[run_smplx_bumi3] IK=$IK_CONFIG"
echo "[run_smplx_bumi3] mode=grounded ground_clearance=0.04m offset_to_ground=on"
echo "[run_smplx_bumi3] Redis disabled by default; GMT 21-joint reorder is configured"

exec "$EXECUTABLE" \
    --xml "$XML" \
    --ik-config "$IK_CONFIG" \
    --port "$UDP_PORT" \
    --redis-key "$REDIS_KEY" \
    --offset-to-ground \
    --ground-clearance 0.04 \
    --no-redis \
    "$@"
