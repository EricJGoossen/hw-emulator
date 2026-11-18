#!/usr/bin/env bash
# Configures, builds and tests hw-emulator. Extra arguments go to cmake's
# configure step, e.g.
#   scripts/build.sh -DHWEMU_WITH_MUJOCO=OFF
#   scripts/build.sh -DFETCHCONTENT_SOURCE_DIR_ACHILLES=../achilles
# HWEMU_BUILD_DIR picks the build directory (default build/); HWEMU_NO_TEST=1
# skips ctest.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
build="${HWEMU_BUILD_DIR:-build}"
cmake -S . -B "$build" -G Ninja "$@"
cmake --build "$build" -j "$(nproc)"
if [ "${HWEMU_NO_TEST:-0}" != "1" ]; then
  ctest --test-dir "$build" --output-on-failure -j "$(nproc)"
fi
