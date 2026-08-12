#!/usr/bin/env bash
# OMG G1 qpos Redis -> G1 FK -> GMR BodyMap IK -> BUMI3 -> GMT Redis.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/g1_bumi3_server"
SOURCE_XML="${G1_SOURCE_XML:-$ROOT/assets/unitree_g1/g1_mocap_29dof.xml}"
TARGET_XML="${BUMI3_XML:-$ROOT/assets/bumi3/mjcf/bumi3.xml}"
IK_CONFIG="${G1_BUMI3_IK_CONFIG:-$ROOT/config/ik_configs/g1_to_bumi3.json}"
INPUT_KEY="${OMG_G1_REDIS_KEY:-omg_online_frame_g1}"
OUTPUT_KEY="${BUMI3_REDIS_KEY:-gmt_online_frame_bumi}"

for path in "$EXECUTABLE" "$SOURCE_XML" "$TARGET_XML" "$IK_CONFIG"; do
    if [[ ! -e "$path" ]]; then
        echo "[ERROR] required OMG G1 -> BUMI3 file not found: $path" >&2
        echo "Run ./build.sh if the executable is missing." >&2
        exit 1
    fi
done

# This repository contains historical build artifacts.  Their timestamps can
# be newer than freshly checked-out source files, causing an incremental make
# to keep an incompatible binary.  Fail early with an actionable message when
# the dedicated OMG flags are requested but the executable predates them.
if printf '%s\n' "$@" | grep -qx -- '--foot-contact-constraints'; then
    if ! "$EXECUTABLE" --help 2>&1 |
            grep -q -- '--foot-contact-weight-scale'; then
        echo "[ERROR] build/g1_bumi3_server is stale and lacks current soft-contact support." >&2
        echo "Rebuild it with:" >&2
        echo "  cmake --build build --target g1_bumi3_server -- -B -j\$(nproc)" >&2
        exit 1
    fi
fi

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"
echo "[run_g1_bumi3] OMG G1 Redis key=$INPUT_KEY"
echo "[run_g1_bumi3] source=$SOURCE_XML"
echo "[run_g1_bumi3] target=$TARGET_XML"
echo "[run_g1_bumi3] IK=$IK_CONFIG"
echo "[run_g1_bumi3] BUMI3 GMT Redis key=$OUTPUT_KEY (21-joint verified reorder)"
if printf '%s\n' "$@" | grep -qx -- '--foot-contact-constraints'; then
    echo "[run_g1_bumi3] retarget mode=gait-aware (explicit opt-in)"
else
    echo "[run_g1_bumi3] retarget mode=legacy (no gait-aware/contact flag)"
fi

exec "$EXECUTABLE" \
    --source-xml "$SOURCE_XML" \
    --target-xml "$TARGET_XML" \
    --config "$IK_CONFIG" \
    --input-redis-key "$INPUT_KEY" \
    --output-redis-key "$OUTPUT_KEY" \
    "$@"
