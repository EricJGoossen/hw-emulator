# hw-emulator development image: everything the library, the hub, the
# Python module and the tests need, on Debian bookworm (the same base as
# Achilles's devcontainer). Achilles, MuJoCo, yaml-cpp, pybind11 and
# GoogleTest are fetched by CMake, not installed here.
FROM debian:bookworm

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git ca-certificates curl pkg-config ccache \
        python3 python3-dev python3-pip python3-numpy python3-serial python3-pytest python3-yaml \
        libxsimd-dev \
        libgl1-mesa-dev libegl-dev libegl1 libgles-dev libgl1-mesa-dri libglx-mesa0 mesa-utils \
        libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev \
        xvfb xauth x11-apps imagemagick \
        can-utils iproute2 kmod \
    && rm -rf /var/lib/apt/lists/*

# Mesa's software rasterizer for headless EGL and for Xvfb.
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    MESA_GL_VERSION_OVERRIDE=4.5
WORKDIR /work
