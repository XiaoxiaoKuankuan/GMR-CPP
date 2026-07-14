#!/bin/bash
# run_xsens.sh — Launch GMR-CPP xsens mocap server
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Default config ─────────────────────────────────────────────────────────────
PORT="9763"
XML="$SCRIPT_DIR/assets/unitree_g1/g1_mocap_29dof.xml"
# XML="/home/user/Workspace/GMR/assets/unitree_g1/g1_mocap_29dof.xml"
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/xsens_to_g1.json"

REDIS_HOST="127.0.0.1"
REDIS_PORT="6379"
REDIS_KEY="mmocap_motion_frame_g1"
HZ="50"
TTL_MS="200"
LIN_VEL_ALPHA="1"
ANG_VEL_ALPHA="1"
LIN_VEL_MAX="0"
ANG_VEL_MAX="0"
ALWAYS=""
VIS=""
NO_OFFSET_TO_GROUND=""
EXTRA_ARGS=""

# ── Parse arguments ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)               PORT="$2";                          shift 2 ;;
        --xml)                XML="$2";                           shift 2 ;;
        --ik-config)          IK_CONFIG="$2";                     shift 2 ;;
        --redis-host)         REDIS_HOST="$2";                    shift 2 ;;
        --redis-port)         REDIS_PORT="$2";                    shift 2 ;;
        --redis-key)          REDIS_KEY="$2";                     shift 2 ;;
        --hz)                 HZ="$2";                            shift 2 ;;
        --ttl-ms)             TTL_MS="$2";                        shift 2 ;;
        --lin-vel-alpha)      LIN_VEL_ALPHA="$2";                 shift 2 ;;
        --ang-vel-alpha)      ANG_VEL_ALPHA="$2";                 shift 2 ;;
        --lin-vel-max)        LIN_VEL_MAX="$2";                   shift 2 ;;
        --ang-vel-max)        ANG_VEL_MAX="$2";                   shift 2 ;;
        --always)             ALWAYS="--always";                  shift 1 ;;
        --vis)                VIS="--vis";                        shift 1 ;;
        --no-offset-to-ground) NO_OFFSET_TO_GROUND="--no-offset-to-ground"; shift 1 ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Optional:"
            echo "  --port <port>           Xsens MVN UDP port (default: 9763)"
            echo "  --xml <path>            MuJoCo XML"
            echo "  --ik-config <path>      IK config JSON"
            echo "  --redis-host <ip>       Redis host (default: 127.0.0.1)"
            echo "  --redis-port <port>     Redis port (default: 6379)"
            echo "  --redis-key <key>       Redis key"
            echo "  --hz <hz>               Publish rate (default: 50)"
            echo "  --ttl-ms <ms>           Redis TTL ms (default: 200)"
            echo "  --lin-vel-alpha <a>     Linear velocity EMA alpha (default: 1=no filter)"
            echo "  --ang-vel-alpha <a>     Angular velocity EMA alpha (default: 1=no filter)"
            echo "  --lin-vel-max <m/s>     Reject linear velocity spikes above this (default: 0=off)"
            echo "  --ang-vel-max <rad/s>   Reject angular velocity spikes above this (default: 0=off)"
            echo "  --always                Publish without joystick gate"
            echo "  --vis                   Open MuJoCo viewer"
            echo "  --no-offset-to-ground   Disable offset to ground (default: enabled)"
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
echo "  GMR-CPP Xsens Mocap Server"
echo "=============================================="
echo "  UDP Port         : $PORT"
echo "  XML              : $XML"
echo "  IK config        : $IK_CONFIG"
echo "  Redis            : $REDIS_HOST:$REDIS_PORT  key=$REDIS_KEY"
echo "  Publish hz       : $HZ"
echo "  Vel filter       : lin_alpha=$LIN_VEL_ALPHA ang_alpha=$ANG_VEL_ALPHA lin_max=$LIN_VEL_MAX ang_max=$ANG_VEL_MAX"
echo "  Always           : ${ALWAYS:-off}"
echo "  Viewer           : ${VIS:-off}"
echo "  Offset to ground : ${NO_OFFSET_TO_GROUND:-on}"
echo "=============================================="
echo ""

# ── Launch ─────────────────────────────────────────────────────────────────────
exec "$EXECUTABLE" \
    --port      "$PORT" \
    --xml       "$XML" \
    --ik-config "$IK_CONFIG" \
    --redis-host "$REDIS_HOST" \
    --redis-port "$REDIS_PORT" \
    --redis-key  "$REDIS_KEY" \
    --hz        "$HZ" \
    --ttl-ms    "$TTL_MS" \
    --lin-vel-alpha "$LIN_VEL_ALPHA" \
    --ang-vel-alpha "$ANG_VEL_ALPHA" \
    --lin-vel-max "$LIN_VEL_MAX" \
    --ang-vel-max "$ANG_VEL_MAX" \
    $ALWAYS \
    $VIS \
    $NO_OFFSET_TO_GROUND \
    $EXTRA_ARGS