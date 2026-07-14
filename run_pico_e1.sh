#!/bin/bash
# run_pico_e1.sh - Launch Pico mocap server for Unitree E1
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXE="$SCRIPT_DIR/build/pico_mocap_server"

if [ ! -f "$EXE" ]; then
    echo "[ERROR] pico_mocap_server not found. Run ./build.sh first."
    exit 1
fi

XML="$SCRIPT_DIR/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/xrobot_to_e1.json"
HEIGHT="1.6"
GMR_HZ="30"
PUB_HZ="50"
BUFFER_MS="200"
REDIS_HOST="192.168.0.44"
REDIS_PORT="6379"
REDIS_KEY="gmt_online_frame_e1"
TTL_MS="200"
LIN_VEL_ALPHA="1"
ANG_VEL_ALPHA="1"
LIN_VEL_MAX="0"
ANG_VEL_MAX="0"
VIS=false
VIEWER_WIDTH="640"
VIEWER_HEIGHT="480"
OFFSET_TO_GROUND="--offset-to-ground"
RAW_BONES="--raw-bones"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --height=*) HEIGHT="${1#*=}"; shift ;;
        --height) HEIGHT="$2"; shift 2 ;;
        --gmr-hz=*) GMR_HZ="${1#*=}"; shift ;;
        --gmr-hz) GMR_HZ="$2"; shift 2 ;;
        --hz=*) PUB_HZ="${1#*=}"; shift ;;
        --hz) PUB_HZ="$2"; shift 2 ;;
        --buffer-ms=*) BUFFER_MS="${1#*=}"; shift ;;
        --buffer-ms) BUFFER_MS="$2"; shift 2 ;;
        --redis-host=*) REDIS_HOST="${1#*=}"; shift ;;
        --redis-host) REDIS_HOST="$2"; shift 2 ;;
        --redis-port=*) REDIS_PORT="${1#*=}"; shift ;;
        --redis-port) REDIS_PORT="$2"; shift 2 ;;
        --redis-key=*) REDIS_KEY="${1#*=}"; shift ;;
        --redis-key) REDIS_KEY="$2"; shift 2 ;;
        --ttl-ms=*) TTL_MS="${1#*=}"; shift ;;
        --ttl-ms) TTL_MS="$2"; shift 2 ;;
        --lin-vel-alpha=*) LIN_VEL_ALPHA="${1#*=}"; shift ;;
        --lin-vel-alpha) LIN_VEL_ALPHA="$2"; shift 2 ;;
        --ang-vel-alpha=*) ANG_VEL_ALPHA="${1#*=}"; shift ;;
        --ang-vel-alpha) ANG_VEL_ALPHA="$2"; shift 2 ;;
        --lin-vel-max=*) LIN_VEL_MAX="${1#*=}"; shift ;;
        --lin-vel-max) LIN_VEL_MAX="$2"; shift 2 ;;
        --ang-vel-max=*) ANG_VEL_MAX="${1#*=}"; shift ;;
        --ang-vel-max) ANG_VEL_MAX="$2"; shift 2 ;;
        --xml=*) XML="${1#*=}"; shift ;;
        --xml) XML="$2"; shift 2 ;;
        --ik-config=*) IK_CONFIG="${1#*=}"; shift ;;
        --ik-config) IK_CONFIG="$2"; shift 2 ;;
        --viewer-width=*) VIEWER_WIDTH="${1#*=}"; shift ;;
        --viewer-width) VIEWER_WIDTH="$2"; shift 2 ;;
        --viewer-height=*) VIEWER_HEIGHT="${1#*=}"; shift ;;
        --viewer-height) VIEWER_HEIGHT="$2"; shift 2 ;;
        --vis) VIS=true; shift ;;
        --no-vis) VIS=false; shift ;;
        --offset-to-ground) OFFSET_TO_GROUND="--offset-to-ground"; shift ;;
        --no-offset-to-ground) OFFSET_TO_GROUND="--no-offset-to-ground"; shift ;;
        --raw-bones) RAW_BONES="--raw-bones"; shift ;;
        --no-raw-bones) RAW_BONES="--no-raw-bones"; shift ;;
        --always) shift ;;
        --help|-h)
            echo "Usage: ./run_pico_e1.sh [options]"
            echo ""
            echo "Options:"
            echo "  --height <m>             Actual human height (default: 1.6)"
            echo "  --hz <hz>                Redis publish frequency (default: 50)"
            echo "  --redis-host <ip>        Redis host (default: 127.0.0.1)"
            echo "  --redis-port <port>      Redis port (default: 6379)"
            echo "  --redis-key <key>        Redis key (default: gmt_online_frame_e1)"
            echo "  --ttl-ms <ms>            Redis TTL ms (default: 200)"
            echo "  --lin-vel-alpha <a>      Linear velocity EMA alpha (default: 1=no filter)"
            echo "  --ang-vel-alpha <a>      Angular velocity EMA alpha (default: 1=no filter)"
            echo "  --lin-vel-max <m/s>      Reject linear velocity spikes above this (default: 0=off)"
            echo "  --ang-vel-max <rad/s>    Reject angular velocity spikes above this (default: 0=off)"
            echo "  --xml <path>             MuJoCo XML model path"
            echo "  --ik-config <path>       IK config JSON path"
            echo "  --vis / --no-vis         Enable or disable MuJoCo viewer"
            echo "  --viewer-width <px>      Viewer render width (default: 640)"
            echo "  --viewer-height <px>     Viewer render height (default: 480)"
            echo "  --offset-to-ground       Enable offset-to-ground (default)"
            echo "  --no-offset-to-ground    Disable offset-to-ground"
            echo "  --raw-bones              Publish raw bones JSON (default)"
            echo "  --no-raw-bones           Disable raw bones JSON publishing"
            echo "  --always                 Accepted for compatibility; Pico has no gate"
            exit 0
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ ! -f "$XML" ]; then
    echo "[ERROR] XML not found: $XML"
    exit 1
fi

if [ ! -f "$IK_CONFIG" ]; then
    echo "[ERROR] IK config not found: $IK_CONFIG"
    exit 1
fi

export LD_LIBRARY_PATH="$SCRIPT_DIR/third_party/mujoco/lib:$SCRIPT_DIR/third_party/pico_sdk/lib:$LD_LIBRARY_PATH"

echo "=============================================="
echo "  GMR-CPP Pico Mocap Server  [E1]"
echo "=============================================="
echo "  Preset           : e1"
echo "  XML              : $XML"
echo "  IK config        : $IK_CONFIG"
echo "  Height           : $HEIGHT m"
echo "  Publish hz       : $PUB_HZ"
echo "  Redis            : $REDIS_HOST:$REDIS_PORT key=$REDIS_KEY"
echo "  Vel filter       : lin_alpha=$LIN_VEL_ALPHA ang_alpha=$ANG_VEL_ALPHA lin_max=$LIN_VEL_MAX ang_max=$ANG_VEL_MAX"
echo "  Viewer           : $VIS ${VIEWER_WIDTH}x${VIEWER_HEIGHT}"
echo "  OffsetToGround   : $OFFSET_TO_GROUND"
echo "  RawBones         : $RAW_BONES"
echo "=============================================="
echo ""

CMD=(
    "$EXE"
    --preset e1
    --xml "$XML"
    --ik-config "$IK_CONFIG"
    --height "$HEIGHT"
    --gmr-hz "$GMR_HZ"
    --hz "$PUB_HZ"
    --buffer-ms "$BUFFER_MS"
    --redis-host "$REDIS_HOST"
    --redis-port "$REDIS_PORT"
    --redis-key "$REDIS_KEY"
    --ttl-ms "$TTL_MS"
    --lin-vel-alpha "$LIN_VEL_ALPHA"
    --ang-vel-alpha "$ANG_VEL_ALPHA"
    --lin-vel-max "$LIN_VEL_MAX"
    --ang-vel-max "$ANG_VEL_MAX"
    --viewer-width "$VIEWER_WIDTH"
    --viewer-height "$VIEWER_HEIGHT"
    "$OFFSET_TO_GROUND"
    "$RAW_BONES"
)

if [ "$VIS" = true ]; then
    CMD+=(--vis)
fi

CMD+=("${EXTRA_ARGS[@]}")

exec "${CMD[@]}"
