#!/usr/bin/env bash
# GEM UDP -> GMR -> Unitree E1 launcher.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="$ROOT/build/gem_mocap_server"

BIND="0.0.0.0"
PORT="7001"
PRESET="e1"
XML="$ROOT/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$ROOT/config/ik_configs/gem_to_e1_position.json"
REDIS_HOST="127.0.0.1"
REDIS_PORT="6379"
REDIS_DB="0"
REDIS_KEY="gmt_online_frame_e1"
HZ="50"
TTL_MS="200"
STALE_MS="250"
LIN_VEL_ALPHA="1"
ANG_VEL_ALPHA="1"
LIN_VEL_MAX="0"
ANG_VEL_MAX="0"
VIEWER_WIDTH="640"
VIEWER_HEIGHT="480"
HUMAN_HEIGHT="1.8"
DAMPING="1.0"
ALWAYS=false
VIS=false
GROUND_ARG=""
EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./run_gem.sh [options]

GEM input:
  --bind <IPv4>             UDP bind address (default: 0.0.0.0)
  --port <port>             UDP port (default: 7001)

E1/GMR:
  --preset <e1>             E1 only (default: e1)
  --xml <path>              E1 MuJoCo XML
  --ik-config <path>        E1 IK config (default: GEM position-only)
  --human-height <m>        human-height assumption (default: 1.8)
  --damping <value>         IK damping (default: 1.0)
  --offset-to-ground        enable per-frame grounding (off by default)
  --no-offset-to-ground     preserve sender's initial ground

Redis/publishing:
  --redis-host <host>       default: 127.0.0.1
  --redis-port <port>       default: 6379
  --redis-db <db>           default: 0
  --redis-key <key>         default: gmt_online_frame_e1
  --hz <rate>               default: 50
  --ttl-ms <ms>             default: 200
  --stale-ms <ms>           default: 250
  --lin-vel-alpha <0..1>    default: 1
  --ang-vel-alpha <0..1>    default: 1
  --lin-vel-max <m/s>       default: 0 (disabled)
  --ang-vel-max <rad/s>     default: 0 (disabled)

Runtime:
  --always                  bypass A+R1 joystick gate
  --vis                     open E1 MuJoCo viewer
  --viewer-width <px>       default: 640
  --viewer-height <px>      default: 480
  -h, --help
EOF
}

need_value() {
    if [[ $# -lt 2 ]]; then
        echo "[ERROR] missing value for $1" >&2
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bind)              need_value "$@"; BIND="$2"; shift 2 ;;
        --port)              need_value "$@"; PORT="$2"; shift 2 ;;
        --preset)            need_value "$@"; PRESET="$2"; shift 2 ;;
        --xml)               need_value "$@"; XML="$2"; shift 2 ;;
        --ik-config)         need_value "$@"; IK_CONFIG="$2"; shift 2 ;;
        --human-height)      need_value "$@"; HUMAN_HEIGHT="$2"; shift 2 ;;
        --damping)           need_value "$@"; DAMPING="$2"; shift 2 ;;
        --redis-host)        need_value "$@"; REDIS_HOST="$2"; shift 2 ;;
        --redis-port)        need_value "$@"; REDIS_PORT="$2"; shift 2 ;;
        --redis-db)          need_value "$@"; REDIS_DB="$2"; shift 2 ;;
        --redis-key)         need_value "$@"; REDIS_KEY="$2"; shift 2 ;;
        --hz)                need_value "$@"; HZ="$2"; shift 2 ;;
        --ttl-ms)            need_value "$@"; TTL_MS="$2"; shift 2 ;;
        --stale-ms)          need_value "$@"; STALE_MS="$2"; shift 2 ;;
        --lin-vel-alpha)     need_value "$@"; LIN_VEL_ALPHA="$2"; shift 2 ;;
        --ang-vel-alpha)     need_value "$@"; ANG_VEL_ALPHA="$2"; shift 2 ;;
        --lin-vel-max)       need_value "$@"; LIN_VEL_MAX="$2"; shift 2 ;;
        --ang-vel-max)       need_value "$@"; ANG_VEL_MAX="$2"; shift 2 ;;
        --viewer-width)      need_value "$@"; VIEWER_WIDTH="$2"; shift 2 ;;
        --viewer-height)     need_value "$@"; VIEWER_HEIGHT="$2"; shift 2 ;;
        --always)            ALWAYS=true; shift ;;
        --vis)               VIS=true; shift ;;
        --offset-to-ground)  GROUND_ARG="--offset-to-ground"; shift ;;
        --no-offset-to-ground) GROUND_ARG="--no-offset-to-ground"; shift ;;
        -h|--help)           usage; exit 0 ;;
        *)                   EXTRA_ARGS+=("$1"); shift ;;
    esac
done

if [[ "$PRESET" != "e1" ]]; then
    echo "[ERROR] run_gem.sh is E1-only; --preset must be e1" >&2
    exit 2
fi
if [[ ! -x "$EXECUTABLE" ]]; then
    echo "[ERROR] executable not found: $EXECUTABLE" >&2
    echo "Run: ./build.sh --clean" >&2
    exit 1
fi
if [[ ! -f "$XML" ]]; then
    echo "[ERROR] E1 XML not found: $XML" >&2
    exit 1
fi
if [[ ! -f "$IK_CONFIG" ]]; then
    echo "[ERROR] E1 IK config not found: $IK_CONFIG" >&2
    exit 1
fi

export LD_LIBRARY_PATH="$ROOT/third_party/mujoco/lib:${LD_LIBRARY_PATH:-}"

CMD=(
    "$EXECUTABLE"
    --bind "$BIND"
    --port "$PORT"
    --preset "$PRESET"
    --xml "$XML"
    --ik-config "$IK_CONFIG"
    --human-height "$HUMAN_HEIGHT"
    --damping "$DAMPING"
    --redis-host "$REDIS_HOST"
    --redis-port "$REDIS_PORT"
    --redis-db "$REDIS_DB"
    --redis-key "$REDIS_KEY"
    --hz "$HZ"
    --ttl-ms "$TTL_MS"
    --stale-ms "$STALE_MS"
    --lin-vel-alpha "$LIN_VEL_ALPHA"
    --ang-vel-alpha "$ANG_VEL_ALPHA"
    --lin-vel-max "$LIN_VEL_MAX"
    --ang-vel-max "$ANG_VEL_MAX"
    --viewer-width "$VIEWER_WIDTH"
    --viewer-height "$VIEWER_HEIGHT"
)

[[ "$ALWAYS" == true ]] && CMD+=(--always)
[[ "$VIS" == true ]] && CMD+=(--vis)
[[ -n "$GROUND_ARG" ]] && CMD+=("$GROUND_ARG")
CMD+=("${EXTRA_ARGS[@]}")

echo "[run_gem] GEM UDP $BIND:$PORT -> E1 -> Redis $REDIS_HOST:$REDIS_PORT/$REDIS_DB key=$REDIS_KEY"
echo "[run_gem] XML=$XML"
echo "[run_gem] IK=$IK_CONFIG stale=${STALE_MS}ms ttl=${TTL_MS}ms vis=$VIS"

exec "${CMD[@]}"
