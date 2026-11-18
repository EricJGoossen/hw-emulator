"""The hub's serial ports, by name.

The hub makes each serial device a PTY and links it at
<runtime_dir>/<name> (a device's transport.port, or its name). Point the
code under test at serial_port("ld19") instead of /dev/ttyUSB0 -- or set its
port name to that path in its config.
"""

from __future__ import annotations

import os

DEFAULT_RUNTIME_DIR = "/tmp/hwemu"


def runtime_dir() -> str:
    """$HWEMU_RUNTIME_DIR, or the hub's default."""
    return os.environ.get("HWEMU_RUNTIME_DIR", DEFAULT_RUNTIME_DIR)


def serial_port(name: str) -> str:
    """The path of the hub's port `name`: an absolute path is returned as is."""
    if os.path.isabs(name):
        return name
    return os.path.join(runtime_dir(), name)


def open_serial(name: str, baudrate: int = 115200, **kwargs):
    """A pyserial Serial on the hub's port `name` (pyserial must be installed).
    A PTY ignores the baud rate; it's accepted so call sites stay unchanged."""
    import serial  # type: ignore

    return serial.Serial(serial_port(name), baudrate=baudrate, **kwargs)
