#!/bin/bash
# run_pico.sh — Launch Pico mocap server
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXE="$SCRIPT_DIR/build/pico_mocap_server"

if [ ! -f "$EXE" ]; then
    echo "[ERROR] pico_mocap_server not found. Run ./build.sh first."
    exit 1
fi

# ── Default parameters ────────────────────────────────────────────────────────
XML="$SCRIPT_DIR/assets/unitree_g1/g1_mocap_29dof.xml"
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/xrobot_to_g1.json"
HEIGHT=1.6
GMR_HZ=30          # compatibility only; C++ Pico now uses the unified MotionBuffer path
PUB_HZ=50
BUFFER_MS=200     # compatibility only; no interpolation buffer is used
REDIS_HOST="192.168.0.139"
REDIS_PORT=6379
REDIS_KEY="mmocap_motion_frame_g1"
TTL_MS=200
VIS=true
VIEWER_WIDTH=640
VIEWER_HEIGHT=480

# ── Parse arguments ───────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --height=*)     HEIGHT="${arg#*=}" ;;
        --gmr-hz=*)     GMR_HZ="${arg#*=}" ;;
        --hz=*)         PUB_HZ="${arg#*=}" ;;
        --buffer-ms=*)  BUFFER_MS="${arg#*=}" ;;
        --redis-host=*) REDIS_HOST="${arg#*=}" ;;
        --redis-port=*) REDIS_PORT="${arg#*=}" ;;
        --redis-key=*)  REDIS_KEY="${arg#*=}" ;;
        --ttl-ms=*)     TTL_MS="${arg#*=}" ;;
        --xml=*)        XML="${arg#*=}" ;;
        --ik-config=*)  IK_CONFIG="${arg#*=}" ;;
        --viewer-width=*) VIEWER_WIDTH="${arg#*=}" ;;
        --viewer-height=*) VIEWER_HEIGHT="${arg#*=}" ;;
        --no-vis)       VIS=false ;;
        --help)
            echo "Usage: ./run_pico.sh [options]"
            echo ""
            echo "Options:"
            echo "  --height=1.6        Actual human height (meters)"
            echo "  --gmr-hz=30         Compatibility option (not used by unified pipeline)"
            echo "  --hz=50             Redis publish frequency"
            echo "  --buffer-ms=200     Compatibility option (no interpolation buffer)"
            echo "  --redis-host=IP     Redis host (default: 127.0.0.1)"
            echo "  --redis-port=PORT   Redis port (default: 6379)"
            echo "  --redis-key=KEY     Redis key name"
            echo "  --ttl-ms=200        Redis key TTL"
            echo "  --xml=PATH          MuJoCo XML model path"
            echo "  --ik-config=PATH    IK config JSON path"
            echo "  --viewer-width=640  Viewer render width"
            echo "  --viewer-height=480 Viewer render height"
            echo "  --no-vis            Disable MuJoCo viewer"
            exit 0
            ;;
        *)
            echo "[WARN] Unknown argument: $arg"
            ;;
    esac
done

# ── Validate paths ────────────────────────────────────────────────────────────
if [ ! -f "$XML" ]; then
    echo "[ERROR] XML not found: $XML"
    exit 1
fi
if [ ! -f "$IK_CONFIG" ]; then
    # Fallback: try common locations
    ALT="$HOME/Workspace/GMR/general_motion_retargeting/ik_configs/xrobot_to_g1.json"
    if [ -f "$ALT" ]; then
        IK_CONFIG="$ALT"
    else
        echo "[ERROR] IK config not found: $IK_CONFIG"
        echo "  Specify with: --ik-config=/path/to/xrobot_to_g1.json"
        exit 1
    fi
fi

# ── Build command ─────────────────────────────────────────────────────────────
CMD="$EXE \
  --xml $XML \
  --ik-config $IK_CONFIG \
  --height $HEIGHT \
  --gmr-hz $GMR_HZ \
  --hz $PUB_HZ \
  --buffer-ms $BUFFER_MS \
  --redis-host $REDIS_HOST \
  --redis-port $REDIS_PORT \
  --redis-key $REDIS_KEY \
  --ttl-ms $TTL_MS \
  --viewer-width $VIEWER_WIDTH \
  --viewer-height $VIEWER_HEIGHT"

if [ "$VIS" = true ]; then
    CMD="$CMD --vis"
fi

echo "=============================================="
echo "  Pico Mocap Server"
echo "=============================================="
echo "  Height:     $HEIGHT m"
echo "  Pipeline:   unified MotionBuffer (no interpolation)"
echo "  GMR Hz:     input-driven (arg $GMR_HZ ignored)"
echo "  Publish Hz: $PUB_HZ"
echo "  Buffer:     no interpolation (arg $BUFFER_MS ignored)"
echo "  Redis:      $REDIS_HOST:$REDIS_PORT key=$REDIS_KEY"
echo "  Viewer:     $VIS ${VIEWER_WIDTH}x${VIEWER_HEIGHT}"
echo "=============================================="
echo ""

exec $CMD
