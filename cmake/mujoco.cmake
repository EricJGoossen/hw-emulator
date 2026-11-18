# MuJoCo, from its release tarball (no source build): an imported target
# hwemu_mujoco for libmujoco.so and its headers, and -- with the display on --
# hwemu_mujoco_simulate, a static library of the tarball's simulate/ sources
# (MuJoCo's own viewer), which the MuJoCo display runs in passive mode.
#
# -DHWEMU_MUJOCO_DIR=/path/to/mujoco-3.3.0 uses an unpacked release instead.

set(HWEMU_MUJOCO_DIR "" CACHE PATH "An unpacked MuJoCo release (empty: download one)")
if (HWEMU_MUJOCO_DIR)
    set(_mj_root "${HWEMU_MUJOCO_DIR}")
else ()
    if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|aarch64|arm64")
        message(FATAL_ERROR "No MuJoCo release for ${CMAKE_SYSTEM_PROCESSOR}; set HWEMU_MUJOCO_DIR")
    endif ()
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(_mj_arch aarch64)
    else ()
        set(_mj_arch x86_64)
    endif ()
    set(_mj_url "https://github.com/google-deepmind/mujoco/releases/download/${HWEMU_MUJOCO_VERSION}/mujoco-${HWEMU_MUJOCO_VERSION}-linux-${_mj_arch}.tar.gz")
    if (HWEMU_MUJOCO_VERSION STREQUAL "3.3.0" AND _mj_arch STREQUAL "x86_64")
        set(_mj_hash URL_HASH SHA256=0ba6ed0934f3eb90bb017b0ed7689d676f34449b868ec01ad410a7aeae6e1ab4)
    else ()
        set(_mj_hash "")
    endif ()
    FetchContent_Declare(mujoco_release URL "${_mj_url}" ${_mj_hash} DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(mujoco_release)
    set(_mj_root "${mujoco_release_SOURCE_DIR}")
endif ()

find_library(HWEMU_MUJOCO_LIBRARY mujoco PATHS "${_mj_root}/lib" NO_DEFAULT_PATH REQUIRED)
add_library(hwemu_mujoco SHARED IMPORTED GLOBAL)
set_target_properties(hwemu_mujoco PROPERTIES
    IMPORTED_LOCATION "${HWEMU_MUJOCO_LIBRARY}"
    IMPORTED_NO_SONAME FALSE
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_mj_root}/include"
    INTERFACE_INCLUDE_DIRECTORIES "${_mj_root}/include"
)
set(HWEMU_MUJOCO_ROOT "${_mj_root}" CACHE INTERNAL "")

if (HWEMU_WITH_DISPLAY)
    find_package(glfw3 3.3 QUIET)
    if (NOT glfw3_FOUND)
        FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(glfw)
    endif ()
    FetchContent_Declare(lodepng
        GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
        GIT_TAG 7b2893e35891c0c85bee5595caafaf3c2044b409
    )
    FetchContent_MakeAvailable(lodepng)
    set(_sim "${_mj_root}/simulate")
    add_library(hwemu_mujoco_simulate STATIC
        "${_sim}/simulate.cc"
        "${_sim}/glfw_adapter.cc"
        "${_sim}/glfw_dispatch.cc"
        "${_sim}/platform_ui_adapter.cc"
        "${lodepng_SOURCE_DIR}/lodepng.cpp"
    )
    target_include_directories(hwemu_mujoco_simulate SYSTEM PUBLIC "${_sim}" "${lodepng_SOURCE_DIR}")
    target_compile_definitions(hwemu_mujoco_simulate PUBLIC mjUSEPLATFORMSIMD=0)
    target_link_libraries(hwemu_mujoco_simulate PUBLIC hwemu_mujoco glfw Threads::Threads ${CMAKE_DL_LIBS})
    set_target_properties(hwemu_mujoco_simulate PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif ()
