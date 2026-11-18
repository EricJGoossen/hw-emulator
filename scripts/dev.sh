#!/usr/bin/env bash
# Runs a command inside the hw-emulator dev container (.devcontainer/Dockerfile),
# with the workspace (this repo's parent directory) mounted at /work.
#
#   scripts/dev.sh                         # a shell
#   scripts/dev.sh scripts/build.sh        # build and test
#   HWEMU_ROOT=1 scripts/dev.sh scripts/vcan.sh   # bring up vcan0/vcan1
#
# The container shares the host's network namespace (so vcan interfaces made
# once are seen by every process and container) and its IPC namespace (so
# /dev/shm rings are too). It runs as the calling user, so files it writes
# stay yours; HWEMU_ROOT=1 runs as root, which vcan setup needs.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$(dirname "$here")"
image="${HWEMU_IMAGE:-hwemu-dev:latest}"
if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -t "$image" "$here/.devcontainer"
fi
tty_flags=()
if [ -t 0 ] && [ -t 1 ]; then tty_flags=(-it); fi
user_flags=(--user "$(id -u):$(id -g)" -e HOME=/tmp)
if [ "${HWEMU_ROOT:-0}" = "1" ]; then user_flags=(); fi
exec docker run --rm "${tty_flags[@]}" "${user_flags[@]}" --privileged --network host --ipc host \
  -e DISPLAY="${DISPLAY:-}" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$workspace:/work" -w "/work/$(basename "$here")" \
  -e HWEMU_BUILD_DIR="${HWEMU_BUILD_DIR:-build}" \
  "$image" "${@:-bash}"
