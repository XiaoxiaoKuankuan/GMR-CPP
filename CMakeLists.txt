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
endif()
