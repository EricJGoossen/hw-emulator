"""The hub's sim clock, from Python: read it, and take part in lockstep.

    from hwemu import clock
    c = clock.SimClock()        # maps /dev/shm/$HWEMU_CLOCK_SHM (default hwemu_clock)
    c.seconds(), c.micros()     # sim time; None until the hub has published
    c.wait_until(t)             # blocks until sim time reaches t (s)

    p = clock.LockstepParticipant("otto")   # joins <runtime dir>/clock.sock
    p.start()                   # a daemon thread acks every tick as it comes
    p.stop()

The shared-memory layout (include/hwemu/clock/sim_clock.hpp), little-endian:
    "HWEMUCK1" | u64 micros | u64 tick | f64 seconds | u64 dt_us | u64 seqlock
with the seqlock at offset 40, odd while the hub writes.

The lockstep protocol (same header): participant -> hub JOIN (0x01, name),
hub -> participant WELCOME (0x81) and TICK (0x02), each with
u64 tick | u64 micros | u64 dt_us; participant -> hub ACK (0x03, u64 tick)
and LEAVE (0x04). Every message is one SOCK_SEQPACKET packet:
    u8 type | u8 status | u16 name length | u32 request id | u32 payload length | name | payload
"""

from __future__ import annotations

import mmap
import os
import socket
import struct
import threading
import time as _time
from typing import Callable, Optional

from .serial import runtime_dir

_MAGIC = b"HWEMUCK1"
_LAYOUT = struct.Struct("<8sQQdQQ")  # magic, micros, tick, seconds, dt_us, seq
_SEQ_OFFSET = 40

# Real time, captured before anything (sitecustomize) replaces the time module's
# functions with sim time.
_real_monotonic = _time.monotonic
_real_sleep = _time.sleep

JOIN, TICK, ACK, LEAVE, WELCOME = 0x01, 0x02, 0x03, 0x04, 0x81
_HEADER = struct.Struct("<BBHII")


def shm_name() -> str:
    return os.environ.get("HWEMU_CLOCK_SHM", "hwemu_clock")


class SimClock:
    """Reads the hub's published clock. Every read returns None until the hub
    has made its shared memory (it keeps looking, at most every 100 ms)."""

    def __init__(self, name: Optional[str] = None, shm_dir: str = "/dev/shm"):
        self._path = os.path.join(shm_dir, (name or shm_name()).lstrip("/"))
        self._map: Optional[mmap.mmap] = None
        self._next_try = 0.0
        self._lock = threading.Lock()

    def _mapped(self) -> Optional[mmap.mmap]:
        if self._map is not None:
            return self._map
        now = _real_monotonic()
        if now < self._next_try:
            return None
        with self._lock:
            if self._map is not None:
                return self._map
            self._next_try = now + 0.1
            try:
                fd = os.open(self._path, os.O_RDONLY)
            except FileNotFoundError:
                return None
            try:
                if os.fstat(fd).st_size < _LAYOUT.size:
                    return None
                m = mmap.mmap(fd, _LAYOUT.size, prot=mmap.PROT_READ)
            finally:
                os.close(fd)
            if m[:8] != _MAGIC:
                m.close()
                return None
            self._map = m
            return m

    def read(self) -> Optional[tuple[int, int, float, int]]:
        """(micros, tick, seconds, dt_us), or None without a hub clock."""
        m = self._mapped()
        if m is None:
            return None
        for _ in range(1000):
            seq0 = struct.unpack_from("<Q", m, _SEQ_OFFSET)[0]
            if seq0 % 2:
                continue
            _, micros, tick, seconds, dt_us, seq1 = _LAYOUT.unpack_from(m, 0)
            if seq1 == seq0:
                return micros, tick, seconds, dt_us
        return None

    def available(self) -> bool:
        return self.read() is not None

    def micros(self) -> Optional[int]:
        r = self.read()
        return None if r is None else r[0]

    def seconds(self) -> Optional[float]:
        r = self.read()
        return None if r is None else r[0] * 1e-6

    def wait_until(self, seconds: float, poll: float = 0.0005) -> bool:
        """Blocks until sim time reaches `seconds`; False (at once) without a hub clock."""
        while True:
            now = self.seconds()
            if now is None:
                return False
            if now >= seconds:
                return True
            _real_sleep(poll)


class ProcessClock:
    """This process's view of sim time, as sitecustomize gives it to `time.*`:

    - **Continuous.** The hub publishes whole ticks (1 ms). Between them, the
      time moves on with the wall clock from when this process first saw the
      tick, capped just short of the next tick, so two reads a few
      microseconds apart differ, as on the robot.
    - **Strictly increasing** by at least 1 us per read, so timestamps taken
      in a row never tie (robot code rejects samples "not newer" than the
      last). It can run ahead of the hub by that much per read within a tick;
      it never runs backwards.
    - **On a wall-clock epoch.** `wall(t)` turns a sim time into what
      `time.time()` reads at it: the wall time when this process started plus
      the sim time since. Camera frames' host timestamps use it, so they match
      `time.time()`.
    """

    def __init__(self, sim: Optional[SimClock] = None):
        self.sim = sim or SimClock()
        self._lock = threading.Lock()
        self._tick_micros = -1
        self._tick_seen = 0.0
        self._last_us = -1
        self._epoch: Optional[float] = None

    def micros(self) -> Optional[int]:
        r = self.sim.read()
        if r is None:
            return None
        micros, _tick, _seconds, dt_us = r
        with self._lock:
            now = _real_monotonic()
            if micros != self._tick_micros:
                self._tick_micros, self._tick_seen = micros, now
            within = min(int((now - self._tick_seen) * 1e6), max(dt_us - 1, 0))
            t = max(micros + within, self._last_us + 1)
            self._last_us = t
            return t

    def seconds(self) -> Optional[float]:
        us = self.micros()
        return None if us is None else us * 1e-6

    def epoch(self) -> float:
        """Wall seconds at sim time 0, fixed the first time it's asked for."""
        if self._epoch is None:
            r = self.sim.read()
            self._epoch = _real_time() - (r[0] * 1e-6 if r else 0.0)
        return self._epoch

    def wall(self, sim_seconds: float) -> float:
        return self.epoch() + sim_seconds


_real_time = _time.time
_process_clock: Optional[ProcessClock] = None


def process_clock() -> ProcessClock:
    """The one ProcessClock of this process (sitecustomize's and the shims')."""
    global _process_clock
    if _process_clock is None:
        _process_clock = ProcessClock()
    return _process_clock


def _pack(kind: int, name: bytes = b"", payload: bytes = b"") -> bytes:
    return _HEADER.pack(kind, 0, len(name), 0, len(payload)) + name + payload


def _unpack(packet: bytes) -> tuple[int, bytes, bytes]:
    kind, _status, name_len, _rid, payload_len = _HEADER.unpack_from(packet, 0)
    name = packet[_HEADER.size:_HEADER.size + name_len]
    payload = packet[_HEADER.size + name_len:_HEADER.size + name_len + payload_len]
    return kind, name, payload


class LockstepParticipant:
    """Joins the hub's lockstep as `name` and acks each tick from a daemon
    thread, so the hub's clock advances only while this process runs: stopped
    at a breakpoint (debugpy stops every thread), the whole sim waits.

    `on_tick(tick, micros)`, when given, runs before each ack."""

    def __init__(self, name: str, socket_path: Optional[str] = None,
                 on_tick: Optional[Callable[[int, int], None]] = None):
        self.name = name
        self.socket_path = socket_path or os.path.join(runtime_dir(), "clock.sock")
        self.on_tick = on_tick
        self.tick = 0
        self.micros = 0
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def join(self, wait_s: float = 60.0) -> None:
        deadline = _real_monotonic() + wait_s
        while True:
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                s.connect(self.socket_path)
                break
            except OSError:
                s.close()
                if _real_monotonic() > deadline:
                    raise TimeoutError(f"no hub lockstep socket at {self.socket_path} after {wait_s:g} s")
                _real_sleep(0.1)
        s.send(_pack(JOIN, self.name.encode()))
        self._sock = s

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                packet = self._sock.recv(65536)
            except OSError:
                break
            if not packet:
                break
            kind, _, payload = _unpack(packet)
            if kind in (TICK, WELCOME) and len(payload) >= 16:
                self.tick, self.micros = struct.unpack_from("<QQ", payload, 0)
                if kind == TICK:
                    if self.on_tick:
                        self.on_tick(self.tick, self.micros)
                    try:
                        self._sock.send(_pack(ACK, payload=struct.pack("<Q", self.tick)))
                    except OSError:
                        break

    def start(self, wait_s: float = 60.0) -> "LockstepParticipant":
        if self._sock is None:
            self.join(wait_s)
        self._thread = threading.Thread(target=self._run, name=f"hwemu-lockstep-{self.name}", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.send(_pack(LEAVE))
            except OSError:
                pass
            self._sock.close()
            self._sock = None
