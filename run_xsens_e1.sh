#!/bin/bash
# run_xsens_e1.sh — Launch GMR-CPP xsens mocap server for E1
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Default config ─────────────────────────────────────────────────────────────
PORT="9763"
PRESET="e1"                                         # ← 关键:必须传 preset,否则默认 g1
XML="$SCRIPT_DIR/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/xsens_to_e1.json"
REDIS_HOST="192.168.50.102"                         # ← 改成同网段,Legion 能连到
REDIS_PORT="6379"
REDIS_KEY="gmt_online_frame_e1"
HZ="50"
TTL_MS="200"
LIN_VEL_ALPHA="1"
ANG_VEL_ALPHA="1"
LIN_VEL_MAX="0"
ANG_VEL_MAX="0"
ALWAYS=""
VIS=""
# 这两个原脚本写死但没传给可执行文件,属 bug。改成 CLI 可控,默认不传。
NO_OFFSET_TO_GROUND=""
NO_SPINE_OFFSET=""
EXTRA_ARGS=""

# ── Parse arguments ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)                 PORT="$2";         shift 2 ;;
        --preset)               PRESET="$2";       shift 2 ;;
        --xml)                  XML="$2";          shift 2 ;;
        --ik-config)            IK_CONFIG="$2";    shift 2 ;;
        --redis-host)           REDIS_HOST="$2";   shift 2 ;;
        --redis-port)           REDIS_PORT="$2";   shift 2 ;;
        --redis-key)            REDIS_KEY="$2";    shift 2 ;;
        --hz)                   HZ="$2";           shift 2 ;;
        --ttl-ms)               TTL_MS="$2";       shift 2 ;;
        --lin-vel-alpha)        LIN_VEL_ALPHA="$2"; shift 2 ;;
        --ang-vel-alpha)        ANG_VEL_ALPHA="$2"; shift 2 ;;
        --lin-vel-max)          LIN_VEL_MAX="$2";   shift 2 ;;
        --ang-vel-max)          ANG_VEL_MAX="$2";   shift 2 ;;
        --always)               ALWAYS="--always"; shift 1 ;;
        --vis)                  VIS="--vis";       shift 1 ;;
        --no-offset-to-ground)  NO_OFFSET_TO_GROUND="--no-offset-to-ground"; shift 1 ;;
        --no-spine-offset)      NO_SPINE_OFFSET="--no-spine-offset"; shift 1 ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Optional:"
            echo "  --port <port>            Xsens MVN UDP port (default: 9763)"
            echo "  --preset <name>          Robot preset: g1 | e1 (default: e1)"
            echo "  --xml <path>             MuJoCo XML (default: assets/e1/mjcf/e1_24dof.xml)"
            echo "  --ik-config <path>       IK config JSON (default: config/ik_configs/xsens_to_e1.json)"
            echo "  --redis-host <ip>        Redis host (default: 192.168.22.220)"
            echo "  --redis-port <port>      Redis port (default: 6379)"
            echo "  --redis-key <key>        Redis key (default: gmt_online_frame_e1)"
            echo "  --hz <hz>                Publish rate (default: 50)"
            echo "  --ttl-ms <ms>            Redis TTL ms (default: 200)"
            echo "  --lin-vel-alpha <a>      Linear velocity EMA alpha (default: 1=no filter)"
            echo "  --ang-vel-alpha <a>      Angular velocity EMA alpha (default: 1=no filter)"
            echo "  --lin-vel-max <m/s>      Reject linear velocity spikes above this (default: 0=off)"
            echo "  --ang-vel-max <rad/s>    Reject angular velocity spikes above this (default: 0=off)"
            echo "  --always                 Publish without joystick gate"
            echo "  --vis                    Open MuJoCo viewer"
            echo "  --no-offset-to-ground    Disable offset-to-ground"
            echo "  --no-spine-offset        Disable spine pitch offset"
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift 1
            ;;
    esac
done

# ── Validate ───────────────────────────────────────────────────────────────────
EXECUTABLE="$SCRIPT_DIR/build/xsens_mocap_server"
if [ ! -f "$EXECUTABLE" ]; then
    echo "[ERROR] Executable not found: $EXECUTABLE"
    echo "Run ./build.sh first"
    exit 1
fi

if [ ! -f "$XML" ]; then
    echo "[ERROR] XML not found: $XML"
    exit 1
fi

if [ ! -f "$IK_CONFIG" ]; then
    echo "[ERROR] IK config not found: $IK_CONFIG"
    exit 1
fi

# ── Set library paths ──────────────────────────────────────────────────────────
export LD_LIBRARY_PATH="$SCRIPT_DIR/third_party/mujoco/lib:$LD_LIBRARY_PATH"

# ── Print config ───────────────────────────────────────────────────────────────
echo "=============================================="
echo "  GMR-CPP Xsens Mocap Server  [$PRESET]"
echo "=============================================="
echo "  UDP Port         : $PORT"
echo "  Preset           : $PRESET"
echo "  XML              : $XML"
echo "  IK config        : $IK_CONFIG"
echo "  Redis            : $REDIS_HOST:$REDIS_PORT  key=$REDIS_KEY"
echo "  Publish hz       : $HZ"
echo "  Vel filter       : lin_alpha=$LIN_VEL_ALPHA ang_alpha=$ANG_VEL_ALPHA lin_max=$LIN_VEL_MAX ang_max=$ANG_VEL_MAX"
echo "  Always           : ${ALWAYS:-off}"
echo "  Viewer           : ${VIS:-off}"
echo "  NoOffsetToGround : ${NO_OFFSET_TO_GROUND:-off}"
echo "  NoSpineOffset    : ${NO_SPINE_OFFSET:-off}"
echo "=============================================="
echo ""

# ── Launch ─────────────────────────────────────────────────────────────────────
exec "$EXECUTABLE" \
    --port       "$PORT" \
    --preset     "$PRESET" \
    --xml        "$XML" \
    --ik-config  "$IK_CONFIG" \
    --redis-host "$REDIS_HOST" \
    --redis-port "$REDIS_PORT" \
    --redis-key  "$REDIS_KEY" \
    --hz         "$HZ" \
    --ttl-ms     "$TTL_MS" \
    --lin-vel-alpha "$LIN_VEL_ALPHA" \
    --ang-vel-alpha "$ANG_VEL_ALPHA" \
    --lin-vel-max "$LIN_VEL_MAX" \
    --ang-vel-max "$ANG_VEL_MAX" \
    $ALWAYS \
    $VIS \
    $NO_OFFSET_TO_GROUND \
    $NO_SPINE_OFFSET \
    $EXTRA_ARGS
