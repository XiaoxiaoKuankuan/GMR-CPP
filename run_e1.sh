#!/bin/bash
# run_e1.sh — Launch GMR-CPP optitrack mocap server for Unitree E1
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── E1-specific defaults ──────────────────────────────────────────────────────
SERVER_IP=""
CLIENT_IP=""
XML="$SCRIPT_DIR/assets/e1/mjcf/e1_24dof.xml"                   # TODO: update when ready
# XML="$SCRIPT_DIR/assets/e1/mjcf/e1_24dof_from_urdf.xml"                   # TODO: update when ready
IK_CONFIG="$SCRIPT_DIR/config/ik_configs/fbx_to_e1.json"           # TODO: update when ready
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
EXTRA_ARGS=""

# ── Parse arguments ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --server)     SERVER_IP="$2";    shift 2 ;;
        --client)     CLIENT_IP="$2";    shift 2 ;;
        --xml)        XML="$2";          shift 2 ;;
        --ik-config)  IK_CONFIG="$2";    shift 2 ;;
        --redis-host) REDIS_HOST="$2";   shift 2 ;;
        --redis-port) REDIS_PORT="$2";   shift 2 ;;
        --redis-key)  REDIS_KEY="$2";    shift 2 ;;
        --hz)         HZ="$2";           shift 2 ;;
        --ttl-ms)     TTL_MS="$2";       shift 2 ;;
        --lin-vel-alpha) LIN_VEL_ALPHA="$2"; shift 2 ;;
        --ang-vel-alpha) ANG_VEL_ALPHA="$2"; shift 2 ;;
        --lin-vel-max)   LIN_VEL_MAX="$2";   shift 2 ;;
        --ang-vel-max)   ANG_VEL_MAX="$2";   shift 2 ;;
        --always)     ALWAYS="--always"; shift 1 ;;
        --vis)        VIS="--vis";       shift 1 ;;
        --help|-h)
            echo "Usage: $0 --server <ip> --client <ip> [options]"
            echo ""
            echo "Required:"
            echo "  --server <ip>       Motive PC IP address"
            echo "  --client <ip>       This machine IP address"
            echo ""
            echo "Optional:"
            echo "  --xml <path>        MuJoCo XML (default: assets/unitree_e1/e1_mocap.xml)"
            echo "  --ik-config <path>  IK config JSON (default: config/ik_configs/fbx_to_e1.json)"
            echo "  --redis-host <ip>   Redis host (default: 127.0.0.1)"
            echo "  --redis-port <port> Redis port (default: 6379)"
            echo "  --redis-key <key>   Redis key (default: mmocap_motion_frame_e1)"
            echo "  --hz <hz>           Publish rate (default: 50)"
            echo "  --ttl-ms <ms>       Redis TTL ms (default: 200)"
            echo "  --lin-vel-alpha <a> Linear velocity EMA alpha (default: 1=no filter)"
            echo "  --ang-vel-alpha <a> Angular velocity EMA alpha (default: 1=no filter)"
            echo "  --lin-vel-max <m/s> Reject linear velocity spikes above this (default: 0=off)"
            echo "  --ang-vel-max <rad/s> Reject angular velocity spikes above this (default: 0=off)"
            echo "  --always            Publish without joystick gate"
            echo "  --vis               Open MuJoCo viewer"
            exit 0
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift 1
            ;;
    esac
done

# ── Validate ───────────────────────────────────────────────────────────────────
if [ -z "$SERVER_IP" ] || [ -z "$CLIENT_IP" ]; then
    echo "[ERROR] --server and --client are required"
    echo "Usage: $0 --server <motive_ip> --client <this_ip> [--always] [--vis]"
    exit 1
fi

EXECUTABLE="$SCRIPT_DIR/build/optitrack_mocap_server"
if [ ! -f "$EXECUTABLE" ]; then
    echo "[ERROR] Executable not found: $EXECUTABLE"
    echo "Run ./build.sh first"
    exit 1
fi

if [ ! -f "$XML" ]; then
    echo "[ERROR] XML not found: $XML"
    echo "Please add E1 model to: $XML"
    exit 1
fi

if [ ! -f "$IK_CONFIG" ]; then
    echo "[ERROR] IK config not found: $IK_CONFIG"
    echo "Please add E1 IK config to: $IK_CONFIG"
    exit 1
fi

# ── Set library paths ──────────────────────────────────────────────────────────
export LD_LIBRARY_PATH="$SCRIPT_DIR/third_party/mujoco/lib:$SCRIPT_DIR/third_party/NatNet_SDK_4.4_ubuntu/lib:$LD_LIBRARY_PATH"

# ── Print config ───────────────────────────────────────────────────────────────
echo "=============================================="
echo "  GMR-CPP OptiTrack Mocap Server  [Unitree E1]"
echo "=============================================="
echo "  Server IP  : $SERVER_IP"
echo "  Client IP  : $CLIENT_IP"
echo "  Preset     : e1 (24 DOF, no joint reorder)"
echo "  XML        : $XML"
echo "  IK config  : $IK_CONFIG"
echo "  Redis      : $REDIS_HOST:$REDIS_PORT  key=$REDIS_KEY"
echo "  Publish hz : $HZ"
echo "  Vel filter : lin_alpha=$LIN_VEL_ALPHA ang_alpha=$ANG_VEL_ALPHA lin_max=$LIN_VEL_MAX ang_max=$ANG_VEL_MAX"
echo "  Always     : ${ALWAYS:-off}"
echo "  Viewer     : ${VIS:-off}"
echo "=============================================="
echo ""

# ── Launch ─────────────────────────────────────────────────────────────────────
exec "$EXECUTABLE" \
    --server    "$SERVER_IP" \
    --client    "$CLIENT_IP" \
    --preset    e1 \
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
    $EXTRA_ARGS