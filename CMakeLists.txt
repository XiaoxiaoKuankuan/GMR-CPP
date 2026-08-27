cmake_minimum_required(VERSION 3.15)
project(optitrack_mocap_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
#  Paths  (relative to GMR-CPP root)
# ==============================================================================
set(GMR_ROOT       "${CMAKE_SOURCE_DIR}")
set(THIRD_PARTY    "${GMR_ROOT}/third_party")
set(NATNET_SDK_DIR "${THIRD_PARTY}/NatNet_SDK_4.4_ubuntu")
set(MUJOCO_DIR     "${THIRD_PARTY}/mujoco")
set(LUMO_SDK_DIR   "${THIRD_PARTY}/LuMoSDK")

# ==============================================================================
#  Find packages
# ==============================================================================

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# nlohmann/json
find_package(nlohmann_json 3.9 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2)
    FetchContent_MakeAvailable(json)
endif()

# MuJoCo
set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_LIBRARY     "${MUJOCO_DIR}/lib/libmujoco.so")
if(NOT EXISTS "${MUJOCO_LIBRARY}")
    message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_LIBRARY}")
endif()
add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION             "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")

# GLFW
find_library(GLFW_LIBRARY NAMES glfw glfw3
    HINTS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h
    HINTS /usr/include /usr/local/include)
if(NOT GLFW_LIBRARY)
    message(FATAL_ERROR "GLFW not found. Install: sudo apt install libglfw3-dev")
endif()
add_library(glfw UNKNOWN IMPORTED)
set_target_properties(glfw PROPERTIES
    IMPORTED_LOCATION             "${GLFW_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLFW_INCLUDE_DIR}")

# hiredis
find_path(HIREDIS_INCLUDE_DIR hiredis/hiredis.h
    HINTS /usr/include /usr/local/include)
find_library(HIREDIS_LIBRARY NAMES hiredis
    HINTS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
if(NOT HIREDIS_LIBRARY)
    message(FATAL_ERROR "hiredis not found. Install: sudo apt install libhiredis-dev")
endif()
add_library(hiredis::hiredis UNKNOWN IMPORTED)
set_target_properties(hiredis::hiredis PROPERTIES
    IMPORTED_LOCATION             "${HIREDIS_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HIREDIS_INCLUDE_DIR}")

# NatNet
add_library(NatNet SHARED IMPORTED)
set_target_properties(NatNet PROPERTIES
    IMPORTED_LOCATION             "${NATNET_SDK_DIR}/lib/libNatNet.so"
    INTERFACE_INCLUDE_DIRECTORIES "${NATNET_SDK_DIR}/include")

# daqp (static)
set(DAQP_LIB "${THIRD_PARTY}/daqp/build/libdaqpstat.a")
if(NOT EXISTS "${DAQP_LIB}")
    message(FATAL_ERROR "daqp not built. Run build.sh first.")
endif()

# ==============================================================================
#  Server executable
# ==============================================================================
add_executable(optitrack_mocap_server
    apps/mocap_server.cpp
    readers/optitrack_reader.cpp
)

target_include_directories(optitrack_mocap_server PRIVATE
    ${GMR_ROOT}                      # for gmr_mink.hpp (include via #include "gmr/...")
    ${GMR_ROOT}/include              # for gmr/body_map.hpp etc.
    ${GMR_ROOT}/readers              # for readers/base_reader.hpp etc.
    ${NATNET_SDK_DIR}/include
    ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(optitrack_mocap_server PRIVATE
    NatNet
    mujoco::mujoco
    glfw
    hiredis::hiredis
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    pthread
    GL
    ${DAQP_LIB}
)

target_compile_options(optitrack_mocap_server PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)


# ==============================================================================
#  Pico server (optional — only built if -DPICO_SDK_DIR=<path> is passed)
# ==============================================================================
set(PICO_SDK_DIR "" CACHE PATH "XRoboToolkit-PC-Service-Pybind root (contains include/ and lib/)")

if(NOT PICO_SDK_DIR STREQUAL "")
    set(PXREA_INCLUDE "${PICO_SDK_DIR}/include")
    set(PXREA_LIB     "${PICO_SDK_DIR}/lib/libPXREARobotSDK.so")

    if(EXISTS "${PXREA_LIB}")
        add_library(PXREARobotSDK SHARED IMPORTED)
        set_target_properties(PXREARobotSDK PROPERTIES
            IMPORTED_LOCATION             "${PXREA_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${PXREA_INCLUDE}")

        add_executable(pico_mocap_server
            apps/pico_mocap_server.cpp
            readers/pico_reader.cpp)

        target_include_directories(pico_mocap_server PRIVATE
            ${GMR_ROOT}
            ${GMR_ROOT}/include
            ${GMR_ROOT}/readers
            ${PXREA_INCLUDE}
            ${MUJOCO_INCLUDE_DIR})

        target_link_libraries(pico_mocap_server PRIVATE
            PXREARobotSDK
            mujoco::mujoco
            glfw
            hiredis::hiredis
            Eigen3::Eigen
            nlohmann_json::nlohmann_json
            pthread
            GL
            ${DAQP_LIB})

        target_compile_options(pico_mocap_server PRIVATE
            -O2 -Wall -Wextra -Wno-unused-parameter)

        message(STATUS "=== pico_mocap_server ENABLED ===")
        message(STATUS "  PICO SDK : ${PICO_SDK_DIR}")
    else()
        message(STATUS "[pico] lib not found at ${PXREA_LIB}, skipping")
    endif()
else()
    message(STATUS "[pico] PICO_SDK_DIR not set — pass -DPICO_SDK_DIR=<path> to build pico_mocap_server")
endif()

# ==============================================================================
#  Xsens server
# ==============================================================================
add_executable(xsens_mocap_server
    apps/xsens_server.cpp
    readers/xsens_reader.cpp
)

target_include_directories(xsens_mocap_server PRIVATE
    ${GMR_ROOT}
    ${GMR_ROOT}/include
    ${GMR_ROOT}/readers
    ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(xsens_mocap_server PRIVATE
    mujoco::mujoco
    glfw
    hiredis::hiredis
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    pthread
    GL
    ${DAQP_LIB}
)

target_compile_options(xsens_mocap_server PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

message(STATUS "=== xsens_mocap_server ENABLED ===")

# ==============================================================================
#  FZMotion server (optional - only built when LuMo SDK library is present)
# ==============================================================================
set(LUMO_SDK_LIB "${LUMO_SDK_DIR}/lib/libLuMoSDK.so")

if(EXISTS "${LUMO_SDK_LIB}")
    add_library(LuMoSDK SHARED IMPORTED)
    set_target_properties(LuMoSDK PROPERTIES
        IMPORTED_LOCATION             "${LUMO_SDK_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${LUMO_SDK_DIR}/include")

    add_executable(fzmotion_mocap_server
        apps/fzmotion_server.cpp
        readers/fzmotion_reader.cpp
    )

    target_include_directories(fzmotion_mocap_server PRIVATE
        ${GMR_ROOT}
        ${GMR_ROOT}/include
        ${GMR_ROOT}/readers
        ${LUMO_SDK_DIR}/include
        ${MUJOCO_INCLUDE_DIR}
    )

    target_link_libraries(fzmotion_mocap_server PRIVATE
        LuMoSDK
        mujoco::mujoco
        glfw
        hiredis::hiredis
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        pthread
        GL
        ${DAQP_LIB}
    )

    set_target_properties(fzmotion_mocap_server PROPERTIES
        BUILD_RPATH   "${LUMO_SDK_DIR}/lib"
        INSTALL_RPATH "${LUMO_SDK_DIR}/lib"
    )

    target_compile_options(fzmotion_mocap_server PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)

    message(STATUS "=== fzmotion_mocap_server ENABLED ===")
    message(STATUS "  LuMo SDK      : ${LUMO_SDK_DIR}")
else()
    message(STATUS "[fzmotion] LuMo SDK library not found at ${LUMO_SDK_LIB}, skipping fzmotion_mocap_server")
endif()

# ==============================================================================
#  Summary
# ==============================================================================
message(STATUS "")
message(STATUS "=== optitrack_mocap_server ===")
message(STATUS "  GMR root      : ${GMR_ROOT}")
message(STATUS "  NatNet SDK    : ${NATNET_SDK_DIR}")
message(STATUS "  MuJoCo        : ${MUJOCO_DIR}")
message(STATUS "  hiredis       : ${HIREDIS_LIBRARY}")
message(STATUS "  daqp          : ${DAQP_LIB}")
message(STATUS "")
message(STATUS "  Run:")
message(STATUS "    ./run.sh --server <motive_ip> --client <this_ip> --always --vis")
message(STATUS "")
# ==============================================================================
#  Original-GMR SMPL-X SMP1 servers (separate G1, E1 and BUMI3 defaults)
# ==============================================================================
foreach(SMPLX_TARGET smplx_g1_server smplx_e1_server smplx_bumi3_server)
    add_executable(${SMPLX_TARGET}
        apps/smplx_server.cpp
        readers/smplx_reader.cpp
    )
    target_include_directories(${SMPLX_TARGET} PRIVATE
        ${GMR_ROOT}
        ${GMR_ROOT}/include
        ${GMR_ROOT}/readers
        ${MUJOCO_INCLUDE_DIR}
    )
    target_link_libraries(${SMPLX_TARGET} PRIVATE
        mujoco::mujoco
        glfw
        hiredis::hiredis
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        pthread
        GL
        ${DAQP_LIB}
    )
    target_compile_options(${SMPLX_TARGET} PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
endforeach()
target_compile_definitions(smplx_g1_server PRIVATE
    SMPLX_DEFAULT_PRESET="g1" SMPLX_DEFAULT_PORT=7004)
target_compile_definitions(smplx_e1_server PRIVATE
    SMPLX_DEFAULT_PRESET="e1" SMPLX_DEFAULT_PORT=7005)
target_compile_definitions(smplx_bumi3_server PRIVATE
    SMPLX_DEFAULT_PRESET="bumi3" SMPLX_DEFAULT_PORT=7006)

message(STATUS "=== smplx_g1_server / smplx_e1_server / smplx_bumi3_server ENABLED ===")

# Persistent one-input/one-output SMP1 service used by GENMO sliding-window
# deployment.  It intentionally has no Redis publisher or viewer: stdout is a
# checksummed binary response channel and all diagnostics go to stderr.
add_executable(smplx_bumi3_batch_server
    apps/smplx_bumi3_batch_server.cpp
    readers/smplx_reader.cpp
)
target_include_directories(smplx_bumi3_batch_server PRIVATE
    ${GMR_ROOT}
    ${GMR_ROOT}/include
    ${GMR_ROOT}/readers
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(smplx_bumi3_batch_server PRIVATE
    mujoco::mujoco
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    pthread
    ${DAQP_LIB}
)
target_compile_options(smplx_bumi3_batch_server PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

message(STATUS "=== smplx_bumi3_batch_server ENABLED ===")

# Real-time visualization sidecar for the GENMO safety bridge.  It consumes
# native BUMI3 qpos[28] records at the bridge playback clock; it is deliberately
# separate from the ahead-of-time batch IK process.
add_executable(bumi3_qpos_viewer apps/bumi3_qpos_viewer.cpp)
target_include_directories(bumi3_qpos_viewer PRIVATE
    ${GMR_ROOT}
    ${GMR_ROOT}/include
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(bumi3_qpos_viewer PRIVATE
    mujoco::mujoco
    glfw
    Eigen3::Eigen
    GL
)
target_compile_options(bumi3_qpos_viewer PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

message(STATUS "=== bumi3_qpos_viewer ENABLED ===")

# ==============================================================================
#  OMG Unitree G1 qpos -> source FK BodyMap -> BUMI3
# ==============================================================================
add_library(g1_motion_source STATIC
    readers/g1_motion_reader.cpp
    readers/g1_redis_receiver.cpp
    adapters/g1_motion_adapter.cpp
)
target_include_directories(g1_motion_source PUBLIC
    ${GMR_ROOT}
    ${GMR_ROOT}/include
    ${GMR_ROOT}/readers
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(g1_motion_source PUBLIC
    mujoco::mujoco
    hiredis::hiredis
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    pthread
)
target_compile_options(g1_motion_source PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

add_executable(g1_bumi3_server apps/g1_bumi3_server.cpp)
target_include_directories(g1_bumi3_server PRIVATE
    ${GMR_ROOT} ${GMR_ROOT}/include ${GMR_ROOT}/readers ${MUJOCO_INCLUDE_DIR})
target_link_libraries(g1_bumi3_server PRIVATE
    g1_motion_source mujoco::mujoco glfw hiredis::hiredis Eigen3::Eigen
    nlohmann_json::nlohmann_json pthread GL ${DAQP_LIB})
target_compile_options(g1_bumi3_server PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

add_executable(generate_g1_to_bumi3_config
    tools/generate_g1_to_bumi3_config.cpp)
target_include_directories(generate_g1_to_bumi3_config PRIVATE
    ${GMR_ROOT} ${GMR_ROOT}/include ${MUJOCO_INCLUDE_DIR})
target_link_libraries(generate_g1_to_bumi3_config PRIVATE
    mujoco::mujoco Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_options(generate_g1_to_bumi3_config PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

add_executable(test_g1_to_bumi3 tools/test_g1_to_bumi3.cpp)
target_include_directories(test_g1_to_bumi3 PRIVATE
    ${GMR_ROOT} ${GMR_ROOT}/include ${GMR_ROOT}/readers ${MUJOCO_INCLUDE_DIR})
target_link_libraries(test_g1_to_bumi3 PRIVATE
    g1_motion_source mujoco::mujoco Eigen3::Eigen nlohmann_json::nlohmann_json
    pthread ${DAQP_LIB})
target_compile_options(test_g1_to_bumi3 PRIVATE
    -O2 -Wall -Wextra -Wno-unused-parameter)

message(STATUS "=== g1_bumi3_server / test_g1_to_bumi3 ENABLED ===")

# ==============================================================================
#  Fixed-size SMP1 protocol parser self-test (no camera or Redis required)
# ==============================================================================
option(BUILD_TESTING "Build local protocol self-tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_executable(smplx_reader_protocol_test
        tests/smplx_reader_protocol_test.cpp
        readers/smplx_reader.cpp
    )
    target_include_directories(smplx_reader_protocol_test PRIVATE
        ${GMR_ROOT}
        ${GMR_ROOT}/include
        ${GMR_ROOT}/readers
    )
    target_link_libraries(smplx_reader_protocol_test PRIVATE
        Eigen3::Eigen
        pthread
    )
    target_compile_options(smplx_reader_protocol_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME smplx_reader_protocol_test COMMAND smplx_reader_protocol_test)

    add_executable(smplx_bumi3_config_test
        tests/smplx_bumi3_config_test.cpp
    )
    target_include_directories(smplx_bumi3_config_test PRIVATE
        ${GMR_ROOT}
        ${GMR_ROOT}/include
        ${MUJOCO_INCLUDE_DIR}
    )
    target_link_libraries(smplx_bumi3_config_test PRIVATE
        mujoco::mujoco
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ${DAQP_LIB}
    )
    target_compile_definitions(smplx_bumi3_config_test PRIVATE
        BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(smplx_bumi3_config_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME smplx_bumi3_config_test COMMAND smplx_bumi3_config_test)

    add_executable(ground_clearance_test
        tests/ground_clearance_test.cpp
    )
    target_include_directories(ground_clearance_test PRIVATE
        ${GMR_ROOT}
        ${GMR_ROOT}/include
        ${MUJOCO_INCLUDE_DIR}
    )
    target_link_libraries(ground_clearance_test PRIVATE
        mujoco::mujoco
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        ${DAQP_LIB}
    )
    target_compile_definitions(ground_clearance_test PRIVATE
        BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(ground_clearance_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME ground_clearance_test COMMAND ground_clearance_test)

    add_executable(g1_motion_source_test
        tests/g1_motion_source_test.cpp)
    target_include_directories(g1_motion_source_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include ${GMR_ROOT}/readers ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(g1_motion_source_test PRIVATE
        g1_motion_source mujoco::mujoco Eigen3::Eigen
        nlohmann_json::nlohmann_json pthread)
    target_compile_definitions(g1_motion_source_test PRIVATE
        G1_BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(g1_motion_source_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME g1_motion_source_test COMMAND g1_motion_source_test)

    add_executable(foot_contact_detector_test
        tests/foot_contact_detector_test.cpp)
    target_include_directories(foot_contact_detector_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(foot_contact_detector_test PRIVATE
        mujoco::mujoco Eigen3::Eigen)
    target_compile_options(foot_contact_detector_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME foot_contact_detector_test COMMAND foot_contact_detector_test)

    add_executable(qpos_interpolator_test
        tests/qpos_interpolator_test.cpp)
    target_include_directories(qpos_interpolator_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include)
    target_link_libraries(qpos_interpolator_test PRIVATE Eigen3::Eigen)
    target_compile_options(qpos_interpolator_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME qpos_interpolator_test COMMAND qpos_interpolator_test)

    add_executable(realtime_motion_guard_test
        tests/realtime_motion_guard_test.cpp)
    target_include_directories(realtime_motion_guard_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(realtime_motion_guard_test PRIVATE
        mujoco::mujoco Eigen3::Eigen)
    target_compile_definitions(realtime_motion_guard_test PRIVATE
        BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(realtime_motion_guard_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME realtime_motion_guard_test COMMAND realtime_motion_guard_test)

    add_executable(g1_to_bumi3_config_test
        tests/g1_to_bumi3_config_test.cpp)
    target_include_directories(g1_to_bumi3_config_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include ${GMR_ROOT}/readers ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(g1_to_bumi3_config_test PRIVATE
        g1_motion_source mujoco::mujoco Eigen3::Eigen
        nlohmann_json::nlohmann_json pthread ${DAQP_LIB})
    target_compile_definitions(g1_to_bumi3_config_test PRIVATE
        G1_BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(g1_to_bumi3_config_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME g1_to_bumi3_config_test COMMAND g1_to_bumi3_config_test)

    add_executable(g1_bumi3_foot_constraint_test
        tests/g1_bumi3_foot_constraint_test.cpp)
    target_include_directories(g1_bumi3_foot_constraint_test PRIVATE
        ${GMR_ROOT} ${GMR_ROOT}/include ${GMR_ROOT}/readers ${MUJOCO_INCLUDE_DIR})
    target_link_libraries(g1_bumi3_foot_constraint_test PRIVATE
        g1_motion_source mujoco::mujoco Eigen3::Eigen
        nlohmann_json::nlohmann_json pthread ${DAQP_LIB})
    target_compile_definitions(g1_bumi3_foot_constraint_test PRIVATE
        G1_BUMI3_REPO_ROOT="${GMR_ROOT}")
    target_compile_options(g1_bumi3_foot_constraint_test PRIVATE
        -O2 -Wall -Wextra -Wno-unused-parameter)
    add_test(NAME g1_bumi3_foot_constraint_test
        COMMAND g1_bumi3_foot_constraint_test)
endif()
