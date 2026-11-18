#!/usr/bin/env bash
# Brings up virtual CAN interfaces (default vcan0 and vcan1) for the hub and
# the hosted MCB to share. Needs root (or the dev container's --privileged).
set -euo pipefail
for ifname in "${@:-vcan0 vcan1}"; do
  for name in $ifname; do
    if ! ip link show "$name" >/dev/null 2>&1; then
      modprobe vcan 2>/dev/null || true
      ip link add dev "$name" type vcan
    fi
    ip link set "$name" up
    echo "$name up"
  done
done
