#!/bin/bash
# run_fzmotion_e1.sh - Launch GMR-CPP FZMotion mocap server for E1
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FZ_SERVER=""
FZ_SUBJECT=""
FZ_SCALE="0.001"
PRESET="e1"
XML="$SCRIPT_DIR/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/fbx_to_e1_fzmotion.json"
REDIS_HOST="127.0.0.1"
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
NO_OFFSET_TO_GROUND=""
NO_SPINE_OFFSET=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --fz-server)          FZ_SERVER="$2"; shift 2 ;;
        --fz-subject)         FZ_SUBJECT="$2"; shift 2 ;;
        --fz-scale)           FZ_SCALE="$2"; shift 2 ;;
        --preset)             PRESET="$2"; shift 2 ;;
        --xml)                XML="$2"; shift 2 ;;
        --ik-config)          IK_CONFIG="$2"; shift 2 ;;
        --redis-host)         REDIS_HOST="$2"; shift 2 ;;
        --redis-port)         REDIS_PORT="$2"; shift 2 ;;
        --redis-key)          REDIS_KEY="$2"; shift 2 ;;
        --hz)                 HZ="$2"; shift 2 ;;
        --ttl-ms)             TTL_MS="$2"; shift 2 ;;
        --lin-vel-alpha)      LIN_VEL_ALPHA="$2"; shift 2 ;;
        --ang-vel-alpha)      ANG_VEL_ALPHA="$2"; shift 2 ;;
        --lin-vel-max)        LIN_VEL_MAX="$2"; shift 2 ;;
        --ang-vel-max)        ANG_VEL_MAX="$2"; shift 2 ;;
        --always)             ALWAYS="--always"; shift 1 ;;
        --vis)                VIS="--vis"; shift 1 ;;
        --no-offset-to-ground) NO_OFFSET_TO_GROUND="--no-offset-to-ground"; shift 1 ;;
        --no-spine-offset)    NO_SPINE_OFFSET="--no-spine-offset"; shift 1 ;;
        --help|-h)
            echo "Usage: $0 --fz-server <ip> [options]"
            echo ""
            echo "Required:"
            echo "  --fz-server <ip>       FZMotion host PC IP"
            echo ""
            echo "Optional:"
            echo "  --fz-subject <name>    BodyName filter (default: any tracked subject)"
            echo "  --fz-scale <float>     Position scale (default: 0.001 = mm->m)"
            echo "  --xml <path>           MuJoCo XML"
            echo "  --ik-config <path>     IK config JSON"
            echo "  --redis-host <ip>      Redis host (default: 127.0.0.1)"
            echo "  --redis-port <port>    Redis port (default: 6379)"
            echo "  --redis-key <key>      Redis key (default: gmt_online_frame_e1)"
            echo "  --hz <hz>              Publish rate (default: 50)"
            echo "  --ttl-ms <ms>          Redis TTL ms (default: 200)"
            echo "  --lin-vel-alpha <a>    Linear velocity EMA alpha (default: 1=no filter)"
            echo "  --ang-vel-alpha <a>    Angular velocity EMA alpha (default: 1=no filter)"
            echo "  --lin-vel-max <m/s>    Reject linear velocity spikes above this (default: 0=off)"
            echo "  --ang-vel-max <rad/s>  Reject angular velocity spikes above this (default: 0=off)"
            echo "  --always               Publish without joystick gate"
            echo "  --vis                  Open MuJoCo viewer"
            echo "  --no-offset-to-ground  Disable offset-to-ground"
            echo "  --no-spine-offset      Disable spine pitch offset"
            exit 0
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift 1
            ;;
    esac
done

if [ -z "$FZ_SERVER" ]; then
    echo "[ERROR] --fz-server is required"
    echo "Usage: $0 --fz-server <ip> [--always] [--vis]"
    exit 1
fi

EXECUTABLE="$SCRIPT_DIR/build/fzmotion_mocap_server"
if [ ! -f "$EXECUTABLE" ]; then
    echo "[ERROR] Executable not found: $EXECUTABLE"
    echo "Run ./build.sh first (or: cmake --build build --target fzmotion_mocap_server)"
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

export LD_LIBRARY_PATH="$SCRIPT_DIR/third_party/mujoco/lib:$SCRIPT_DIR/third_party/LuMoSDK/lib:$LD_LIBRARY_PATH"

echo "=============================================="
echo "  GMR-CPP FZMotion Mocap Server  [$PRESET]"
echo "=============================================="
echo "  FZ Server        : $FZ_SERVER"
echo "  FZ Subject       : ${FZ_SUBJECT:-<any>}"
echo "  FZ Scale         : $FZ_SCALE"
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

CMD=(
    "$EXECUTABLE"
    --fz-server "$FZ_SERVER"
    --fz-scale "$FZ_SCALE"
    --preset "$PRESET"
    --xml "$XML"
    --ik-config "$IK_CONFIG"
    --redis-host "$REDIS_HOST"
    --redis-port "$REDIS_PORT"
    --redis-key "$REDIS_KEY"
    --hz "$HZ"
    --ttl-ms "$TTL_MS"
    --lin-vel-alpha "$LIN_VEL_ALPHA"
    --ang-vel-alpha "$ANG_VEL_ALPHA"
    --lin-vel-max "$LIN_VEL_MAX"
    --ang-vel-max "$ANG_VEL_MAX"
)

if [ -n "$FZ_SUBJECT" ]; then
    CMD+=(--fz-subject "$FZ_SUBJECT")
fi
if [ -n "$ALWAYS" ]; then CMD+=("$ALWAYS"); fi
if [ -n "$VIS" ]; then CMD+=("$VIS"); fi
if [ -n "$NO_OFFSET_TO_GROUND" ]; then CMD+=("$NO_OFFSET_TO_GROUND"); fi
if [ -n "$NO_SPINE_OFFSET" ]; then CMD+=("$NO_SPINE_OFFSET"); fi
CMD+=("${EXTRA_ARGS[@]}")

exec "${CMD[@]}"
