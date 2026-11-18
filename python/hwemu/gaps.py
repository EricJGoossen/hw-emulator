"""Sim-gap reports from Python, in the hub's format (include/hwemu/gaps.hpp).

    from hwemu import gaps
    gaps.report("arducam", "v4l2-controls", "V4L2 camera controls aren't simulated")

Each gap is printed once, when first reported, as `[hwemu GAP] <source>:
<message>` (to stderr and to the `logging` warning stream), and appended once
to <runtime dir>/gaps/<$HWEMU_PROCESS or "python">.jsonl as
{"process", "source", "key", "message", "time"} -- `time` in sim seconds when
the hub's clock is up, else 0.
"""

from __future__ import annotations

import json
import logging
import os
import sys
import threading

from .serial import runtime_dir

_seen: set[tuple[str, str]] = set()
_lock = threading.Lock()
_clock = None


def _sim_seconds() -> float:
    global _clock
    try:
        if _clock is None:
            from .clock import SimClock
            _clock = SimClock()
        t = _clock.seconds()
        return t if t is not None else 0.0
    except Exception:  # noqa: BLE001 -- a gap report must never raise
        return 0.0


def process() -> str:
    return os.environ.get("HWEMU_PROCESS", "python")


def report(source: str, key: str, message: str) -> None:
    with _lock:
        if (source, key) in _seen:
            return
        _seen.add((source, key))
    line = f"[hwemu GAP] {source}: {message}"
    print(line, file=sys.stderr, flush=True)
    try:
        d = os.path.join(runtime_dir(), "gaps")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, f"{process()}.jsonl"), "a") as f:
            f.write(json.dumps({"process": process(), "source": source, "key": key, "message": message,
                                "time": _sim_seconds()}) + "\n")
    except OSError as e:
        logging.getLogger("hwemu").warning("couldn't record gap (%s): %s", e, line)


def all_reported() -> list[tuple[str, str]]:
    with _lock:
        return sorted(_seen)


class GapLogHandler(logging.Handler):
    """Turns `[hwemu GAP] source: message` log records -- what patched robot
    code logs, so it needs no import of this package -- into reports."""

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = record.getMessage()
        except Exception:  # noqa: BLE001
            return
        i = msg.find("[hwemu GAP] ")
        if i < 0:
            return
        body = msg[i + len("[hwemu GAP] "):]
        source, _, message = body.partition(": ")
        with _lock:
            if (source, message) in _seen:
                return
            _seen.add((source, message))
        try:
            d = os.path.join(runtime_dir(), "gaps")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{process()}.jsonl"), "a") as f:
                f.write(json.dumps({"process": process(), "source": source, "key": message, "message": message,
                                    "time": _sim_seconds()}) + "\n")
        except OSError:
            pass
