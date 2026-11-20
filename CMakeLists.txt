cmake_minimum_required(VERSION 3.21)
project(hw_emulator LANGUAGES C CXX)

# hw-emulator: one sim interface over Achilles and MuJoCo, wire-accurate
# device emulation for the MCB (aruw-mcb, hosted) and the Jetson
# (project-otto), and the hub process that ties them together.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(HWEMU_WITH_ACHILLES "Build the Achilles backend" ON)
option(HWEMU_WITH_MUJOCO "Build the MuJoCo backend" ON)
option(HWEMU_WITH_DISPLAY "Build the window displays (Achilles's renderer, MuJoCo's simulate)" ON)
option(HWEMU_WITH_PYTHON "Build the pybind11 module" ON)
option(HWEMU_BUILD_TESTS "Build the GoogleTest suite" ${PROJECT_IS_TOP_LEVEL})

# Achilles: which repository and ref. The default carries the sensor kinds
# hw-emulator needs (attitude, switch, a sized camera, motor temperature) and
# the speedups that let lockstep run in real time (Newton contact solver,
# lane-packed copies, blocked 6x6 fields); a local checkout can be used with
# -DFETCHCONTENT_SOURCE_DIR_ACHILLES=/path/to/achilles.
set(HWEMU_ACHILLES_REPOSITORY "https://github.com/EricJGoossen/Achilles.git" CACHE STRING "Achilles git repository")
set(HWEMU_ACHILLES_TAG "claude/realtime-integration" CACHE STRING "Achilles git ref")
set(HWEMU_MUJOCO_VERSION "3.3.0" CACHE STRING "MuJoCo release")

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif ()

include(FetchContent)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(FETCHCONTENT_QUIET ON)

find_package(Threads REQUIRED)

# ---- dependencies ---------------------------------------------------------
if (HWEMU_WITH_ACHILLES)
    # Achilles brings yaml-cpp 0.8.0 with it; with Achilles off, fetch it here.
    set(ACHILLES_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(ACHILLES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(ACHILLES_WITH_GPU ON CACHE BOOL "" FORCE)
    set(ACHILLES_WITH_DISPLAY ${HWEMU_WITH_DISPLAY} CACHE BOOL "" FORCE)
    FetchContent_Declare(achilles
        GIT_REPOSITORY ${HWEMU_ACHILLES_REPOSITORY}
        GIT_TAG ${HWEMU_ACHILLES_TAG}
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(achilles)
else ()
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
        GIT_SHALLOW TRUE
    )
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(yaml-cpp)
endif ()

if (HWEMU_WITH_MUJOCO)
    include(cmake/mujoco.cmake)
endif ()

# ---- the library ----------------------------------------------------------
# Shared, so the hub, the Python module and device plugins (devices.yaml
# `plugins:`) all use one copy -- one device registry, one gap report.
add_library(hwemu SHARED
    src/gaps.cpp
    src/protocol/crc.cpp
    src/protocol/dji_serial.cpp
    src/protocol/can_codecs.cpp
    src/protocol/vision.cpp
    src/protocol/ld19.cpp
    src/protocol/spi_imu.cpp
    src/transport/can_bus.cpp
    src/transport/serial_port.cpp
    src/transport/frame_shm.cpp
    src/transport/pinbus.cpp
    src/clock/sim_clock.cpp
    src/sim/create_sim.cpp
    src/sim/fake_sim.cpp
    src/devices/registry.cpp
    src/devices/can_devices.cpp
    src/devices/board_devices.cpp
    src/devices/stream_devices.cpp
    src/config/config.cpp
    src/hub/hub.cpp
)
add_library(hwemu::hwemu ALIAS hwemu)
target_include_directories(hwemu PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(hwemu PUBLIC yaml-cpp::yaml-cpp)
target_compile_options(hwemu PRIVATE -Wall -Wextra)
target_compile_definitions(hwemu PRIVATE
    HWEMU_SOURCE_PRESET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/presets"
)

target_link_libraries(hwemu PUBLIC Threads::Threads rt ${CMAKE_DL_LIBS})

if (HWEMU_WITH_ACHILLES)
    target_sources(hwemu PRIVATE src/sim/achilles_backend.cpp)
    target_compile_definitions(hwemu PUBLIC HWEMU_WITH_ACHILLES=1)
    target_link_libraries(hwemu PUBLIC achilles::achilles)
    if (HWEMU_WITH_DISPLAY)
        target_sources(hwemu PRIVATE src/display/achilles_display.cpp)
        target_compile_definitions(hwemu PUBLIC HWEMU_WITH_DISPLAY=1)
        target_link_libraries(hwemu PUBLIC achilles::display)
    endif ()
endif ()

if (HWEMU_WITH_MUJOCO)
    target_sources(hwemu PRIVATE src/sim/mujoco_backend.cpp src/sim/egl_context.cpp)
    target_compile_definitions(hwemu PUBLIC HWEMU_WITH_MUJOCO=1)
    target_link_libraries(hwemu PUBLIC hwemu_mujoco)
    find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
    target_link_libraries(hwemu PRIVATE OpenGL::EGL OpenGL::OpenGL)
    if (HWEMU_WITH_DISPLAY)
        target_sources(hwemu PRIVATE src/display/mujoco_display.cpp)
        target_compile_definitions(hwemu PUBLIC HWEMU_WITH_DISPLAY=1)
        target_link_libraries(hwemu PRIVATE hwemu_mujoco_simulate)
    endif ()
endif ()

# ---- the hub --------------------------------------------------------------
add_executable(hwemu_hub apps/hwemu.cpp)
set_target_properties(hwemu_hub PROPERTIES OUTPUT_NAME hwemu)
target_link_libraries(hwemu_hub PRIVATE hwemu)

# ---- the client hooks (for the hosted MCB) --------------------------------
add_library(hwemu_client client/hwemu_client.cpp)
target_include_directories(hwemu_client PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/client" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(hwemu_client PRIVATE Threads::Threads rt)

# ---- Python ---------------------------------------------------------------
if (HWEMU_WITH_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development.Module)
    if (Python3_FOUND)
        FetchContent_Declare(pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.13.6
            GIT_SHALLOW TRUE
        )
        FetchContent_MakeAvailable(pybind11)
        pybind11_add_module(_hwemu python/hwemu_py.cpp)
        target_link_libraries(_hwemu PRIVATE hwemu)
        set_target_properties(_hwemu PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python/hwemu")
    else ()
        message(WARNING "Python 3 development files not found: not building the hwemu module")
    endif ()
endif ()

# ---- tests ----------------------------------------------------------------
if (HWEMU_BUILD_TESTS)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    add_subdirectory(tests)
endif ()
