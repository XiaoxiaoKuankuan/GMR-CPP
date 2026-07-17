#!/bin/bash
# build.sh — Build GMR-CPP mocap servers
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "  GMR-CPP Build Script"
echo "=============================================="
echo ""

# ── Parse arguments ───────────────────────────────────────────────────────────
BUILD_TYPE="Release"
CLEAN=false

for arg in "$@"; do
    case "$arg" in
        --debug)  BUILD_TYPE="Debug" ;;
        --clean)  CLEAN=true ;;
        --help)
            echo "Usage: ./build.sh [options]"
            echo ""
            echo "Options:"
            echo "  --debug   Build in Debug mode"
            echo "  --clean   Clean build directory first"
            echo ""
            echo "Examples:"
            echo "  ./build.sh              # release build"
            echo "  ./build.sh --clean      # clean + release build"
            echo "  ./build.sh --debug      # debug build"
            exit 0
            ;;
        *)
            echo "[WARN] Unknown argument: $arg"
            ;;
    esac
done

# ── Step 1: Check system dependencies ─────────────────────────────────────────
echo "[1/3] Checking system dependencies..."

missing=()
pkg-config --exists eigen3 2>/dev/null || missing+=("libeigen3-dev")
pkg-config --exists glfw3 2>/dev/null  || \
    [ -f /usr/include/GLFW/glfw3.h ]   || missing+=("libglfw3-dev")
[ -f /usr/include/hiredis/hiredis.h ]  || missing+=("libhiredis-dev")
[ -f /usr/include/nlohmann/json.hpp ]  || missing+=("nlohmann-json3-dev")

if [ ${#missing[@]} -gt 0 ]; then
    echo ""
    echo "[ERROR] Missing packages: ${missing[*]}"
    echo "Install with:"
    echo "  sudo apt install ${missing[*]}"
    exit 1
fi
echo "  ✓ All system dependencies found"
echo ""

# ── Step 2: Build daqp ────────────────────────────────────────────────────────
echo "[2/3] Building daqp QP solver..."

if [ "$CLEAN" = true ]; then
    rm -rf "$SCRIPT_DIR/third_party/daqp/build"
fi

if [ -f "$SCRIPT_DIR/third_party/daqp/build/libdaqpstat.a" ]; then
    echo "  ✓ daqp already built, skipping"
else
    mkdir -p "$SCRIPT_DIR/third_party/daqp/build"
    cd "$SCRIPT_DIR/third_party/daqp/build"
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_C_FLAGS="-w" \
        > /dev/null
    make -j$(nproc)
    echo "  ✓ daqp built successfully"
fi
echo ""

# ── Step 3: Build mocap servers ───────────────────────────────────────────────
echo "[3/3] Building mocap servers..."

if [ "$CLEAN" = true ]; then
    rm -rf "$SCRIPT_DIR/build"
fi

mkdir -p "$SCRIPT_DIR/build"
cd "$SCRIPT_DIR/build"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"

# Auto-detect Pico SDK in third_party
PICO_SDK="$SCRIPT_DIR/third_party/pico_sdk"
if [ -d "$PICO_SDK" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DPICO_SDK_DIR=$PICO_SDK"
    echo "  ✓ Pico SDK found: third_party/pico_sdk"
else
    echo "  ⚠ Pico SDK not found in third_party/pico_sdk"
    echo "    pico_mocap_server will be skipped."
    echo "    To enable pico, copy the SDK:"
    echo "      cp -r <XRoboToolkit-PC-Service-Pybind> third_party/pico_sdk"
fi

# FZMotion is enabled by CMake when the LuMo SDK shared library is present.
LUMO_SDK_LIB="$SCRIPT_DIR/third_party/LuMoSDK/lib/libLuMoSDK.so"
if [ -f "$LUMO_SDK_LIB" ]; then
    echo "  ✓ LuMo SDK found: third_party/LuMoSDK"
else
    echo "  ⚠ LuMo SDK library not found: third_party/LuMoSDK/lib/libLuMoSDK.so"
    echo "    fzmotion_mocap_server will be skipped."
fi

cmake .. $CMAKE_ARGS
make -j$(nproc)

echo ""
echo "=============================================="
echo "  ✓ Build completed! ($BUILD_TYPE)"
echo ""
echo "  Executables:"
[ -f "$SCRIPT_DIR/build/optitrack_mocap_server" ] && \
    echo "    build/optitrack_mocap_server"
[ -f "$SCRIPT_DIR/build/xsens_mocap_server" ] && \
    echo "    build/xsens_mocap_server"
[ -f "$SCRIPT_DIR/build/pico_mocap_server" ] && \
    echo "    build/pico_mocap_server"
[ -f "$SCRIPT_DIR/build/fzmotion_mocap_server" ] && \
    echo "    build/fzmotion_mocap_server"
[ -f "$SCRIPT_DIR/build/smplx_g1_server" ] && \
    echo "    build/smplx_g1_server"
[ -f "$SCRIPT_DIR/build/smplx_e1_server" ] && \
    echo "    build/smplx_e1_server"
[ -f "$SCRIPT_DIR/build/smplx_bumi3_server" ] && \
    echo "    build/smplx_bumi3_server"
echo "=============================================="
