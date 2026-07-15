#!/usr/bin/env bash
# Original-GMR SMPL-X SMP1 -> G1.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/smplx_g1_server"
XML="$ROOT/assets/unitree_g1/g1_mocap_29dof.xml"
IK_CONFIG="$ROOT/config/ik_configs/smplx_to_g1.json"

for path in "$EXECUTABLE" "$XML" "$IK_CONFIG"; do
    if [[ ! -e "$path" ]]; then
        echo "[ERROR] required SMPL-X G1 file not found: $path" >&2
        echo "Run ./build.sh if the executable is missing." >&2
        exit 1
    fi
done

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"
echo "[run_smplx_g1] SMP1 0.0.0.0:7004 -> G1; IK=$IK_CONFIG"
exec "$EXECUTABLE" --xml "$XML" --ik-config "$IK_CONFIG" "$@"
