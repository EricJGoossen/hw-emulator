"""A reader for the hub's frame ring in POSIX shared memory.

The layout (include/hwemu/transport/frame_shm.hpp), little-endian:

    header (64 B):  "HWEMUFR1" | u32 version (1) | u32 slot_count
                    | u64 meta_capacity | u64 payload_capacity | u64 latest
    frame s in slot s % slot_count, at 64 + slot * (64 + meta_capacity + payload_capacity):
                    u64 lock | u64 sequence | u32 meta_size | u32 payload_size
                    | f64 timestamp | (pad to 64) | meta | payload

Each slot is a seqlock: a copy is kept only if its lock was the same even
value before and after copying it.
"""

from __future__ import annotations

import mmap
import os
import struct
from dataclasses import dataclass
from typing import Optional

_HEADER = struct.Struct("<8sIIQQQ")
_SLOT = struct.Struct("<QQIId")
_HEADER_SIZE = 64
_SLOT_HEADER_SIZE = 64
_MAGIC = b"HWEMUFR1"


@dataclass
class ShmFrame:
    sequence: int
    timestamp: float
    meta: bytes
    payload: bytes


class FrameShmReader:
    """Maps /dev/shm/<name>, written by the hub. Raises FileNotFoundError if
    the hub hasn't made it yet."""

    def __init__(self, name: str, shm_dir: str = "/dev/shm"):
        path = os.path.join(shm_dir, name.lstrip("/"))
        fd = os.open(path, os.O_RDONLY)
        try:
            size = os.fstat(fd).st_size
            self._map = mmap.mmap(fd, size, prot=mmap.PROT_READ)
        finally:
            os.close(fd)
        magic, version, slots, meta_cap, payload_cap, _ = _HEADER.unpack_from(self._map, 0)
        if magic != _MAGIC or version != 1:
            raise ValueError(f"{path} is not a hw-emulator frame ring (v1)")
        self.slots = slots
        self.meta_capacity = meta_cap
        self.payload_capacity = payload_cap
        self._stride = _SLOT_HEADER_SIZE + meta_cap + payload_cap

    def close(self) -> None:
        self._map.close()

    def __enter__(self) -> "FrameShmReader":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def latest(self) -> int:
        return struct.unpack_from("<Q", self._map, 32)[0]

    def read_latest(self, after: int = 0) -> Optional[ShmFrame]:
        """The newest whole frame newer than `after`, or None."""
        for _ in range(4):  # retry if the writer laps us mid-copy
            seq = self.latest()
            if seq == 0 or seq <= after:
                return None
            base = _HEADER_SIZE + (seq % self.slots) * self._stride
            lock0, sequence, meta_size, payload_size, timestamp = _SLOT.unpack_from(self._map, base)
            if lock0 % 2 == 1 or sequence != seq:
                continue
            start = base + _SLOT_HEADER_SIZE
            meta = bytes(self._map[start:start + meta_size])
            payload_start = start + self.meta_capacity
            payload = bytes(self._map[payload_start:payload_start + payload_size])
            lock1 = struct.unpack_from("<Q", self._map, base)[0]
            if lock1 == lock0:
                return ShmFrame(sequence, timestamp, meta, payload)
        return None
