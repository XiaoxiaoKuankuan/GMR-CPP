#!/usr/bin/env bash
# Original-GMR SMPL-X SMP1 -> E1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/smplx_e1_server"
XML="$ROOT/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$ROOT/config/ik_configs/smplx_to_e1.json"

for path in "$EXECUTABLE" "$XML" "$IK_CONFIG"; do
    if [[ ! -e "$path" ]]; then
        echo "[ERROR] required SMPL-X E1 file not found: $path" >&2
        echo "Run ./build.sh if the executable is missing." >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"
echo "[run_smplx_e1] SMP1 0.0.0.0:7005 -> E1; IK=$IK_CONFIG"
exec "$EXECUTABLE" --xml "$XML" --ik-config "$IK_CONFIG" "$@"
