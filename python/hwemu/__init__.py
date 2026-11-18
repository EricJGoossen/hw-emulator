"""hw-emulator from Python.

Pure Python, nothing compiled needed:
    hwemu.FrameShmReader   read a hub's frame ring (cameras) from /dev/shm
    hwemu.serial_port      the path of a hub-made serial port, by name
    hwemu.runtime_dir      where the hub puts its ports, sockets and sidecars

With the compiled module (_hwemu, built with HWEMU_WITH_PYTHON=ON):
    hwemu.Hub              run the hub in-process
    hwemu.make_sim         a headless sim from sim.yaml + devices.yaml
"""

from .frame_shm import FrameShmReader, ShmFrame
from .serial import open_serial, runtime_dir, serial_port

try:
    from ._hwemu import Hub, Sim, backends, make_sim  # noqa: F401
except ImportError:  # the compiled module isn't built; the pure-Python parts still work
    pass

__all__ = ["FrameShmReader", "ShmFrame", "open_serial", "runtime_dir", "serial_port"]
