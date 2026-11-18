#!/usr/bin/env python3
"""Generates tests/golden/*.hex from project-otto's own code.

Golden frames come from otto's Transceiver.send() and message classes (and
its CRC functions), so the C++ codecs are checked against the code on the
other end of the wire, not against themselves. Modules otto imports that
aren't needed to serialize (cpp_serial, the logger) are stubbed.

    python3 -I tools/make_golden.py <path to project-otto> tests/golden
"""

import importlib
import struct
import sys
import types
from pathlib import Path


def stub(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


def main():
    otto = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(otto))

    # cpp_serial is a compiled extension otto builds on the Jetson.
    stub("cpp_serial", BufferedSerialByteProvider=object)
    # Load the comms and lidar modules straight from their files: the package
    # __init__s pull in all of otto (config loaders, numpy, OpenCV, ...),
    # none of which serializing a frame needs.
    from typing import Generic, TypeVar

    T = TypeVar("T")

    class Generic1(Generic[T]):
        pass

    def package(name, path=None):
        mod = stub(name)
        mod.__path__ = [str(path)] if path else []
        return mod

    package("project_otto", otto / "project_otto")
    package("project_otto.core", otto / "project_otto" / "core")
    stub("project_otto.core.timestamps", Timestamp=Generic1, JetsonTimestamp=Generic1)
    stub("project_otto.core.time", Timestamp=Generic1)
    stub("project_otto.core.frames", ChassisOdometryReferencePointFrame=object)
    stub("project_otto.core.spatial", Orientation=Generic1, Position=Generic1)
    stub("project_otto.core.data_logging", ModuleLoggers=object)
    package("project_otto.comms", otto / "project_otto" / "comms")
    package("project_otto.comms.uart", otto / "project_otto" / "comms" / "uart")
    package("project_otto.autonav", otto / "project_otto" / "autonav")
    package("project_otto.autonav.lidar", otto / "project_otto" / "autonav" / "lidar")

    crc = importlib.import_module("project_otto.comms.uart._crc")
    golden = {}
    check = b"123456789"
    golden["crc8_check"] = bytes([crc.crc8(check)])
    golden["crc16_check"] = struct.pack("<H", crc.crc16(check))

    # Frames through otto's own Transceiver.send, into a fake serial.
    serial_mod = importlib.import_module("project_otto.comms.uart._serial")
    config_mod = importlib.import_module("project_otto.comms.uart._config")
    tx_mod = importlib.import_module("project_otto.comms.uart._transceiver")

    class Raw:
        def __init__(self, type_id, data):
            self._type_id = type_id
            self._data = data

        def get_type_id(self):
            return self._type_id

        def serialize(self):
            return self._data

    def frame(type_id, data, seq=0):
        fake = serial_mod.FakeSerialBlocking()
        t = tx_mod.Transceiver(config_mod.TransceiverConfig(), [], fake, lambda: 0)
        t.send(Raw(type_id, data), seq)
        return bytes(fake.buffer_out)

    # Odometry exactly as otto's OdometryMessage.parse() reads it.
    odom = struct.pack("<L6fB", 123456, 1.0, 2.0, 0.5, 0.1, -0.2, 3.0, 1) + struct.pack(
        "<6f", 0.1, 0.2, 0.3, 0.0, 0.25, -1.5
    )
    golden["frame_odometry"] = frame(0x01, odom, seq=7)
    # Turret aim exactly as otto's AutoAimTargetUpdateMessage serializes it.
    aim = struct.pack("<cI", (1).to_bytes(1, "big"), 99) + struct.pack(
        "<B17fB", 2, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0.5, 0.25, 0.2, 0.2, 0.1, 0.1, 0.1, 0.1, 3
    )
    aim_frame = struct.pack("<cIB17fB", (1).to_bytes(1, "big"), 99, 2, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0.5, 0.25,
                            0.2, 0.2, 0.1, 0.1, 0.1, 0.1, 3)
    assert aim == aim_frame
    golden["frame_turret_aim"] = frame(0x02, aim_frame + struct.pack("<cI", (0).to_bytes(1, "big"), 100))
    golden["frame_auto_nav"] = frame(0x0D, struct.pack("<IfI4f", 5, 1.5, 2, 1.0, 2.0, 3.0, 4.0))
    golden["frame_world_to_turret"] = frame(0x0A, struct.pack("<7fQB", 1, 2, 3, 1, 0, 0, 0, 555, 1))
    golden["frame_health"] = frame(0x0C, struct.pack("<14H", *range(100, 1500, 100)))
    golden["frame_empty_reboot"] = frame(0x08, b"")

    # LD19: otto's CRC table and parse format.
    lidar_cfg = None
    try:
        lidar_cfg = importlib.import_module("project_otto.autonav.lidar._config").LD19Config
    except Exception as e:  # its frames module may need more of otto than is installed
        print(f"note: LD19Config not importable ({e}); using its table by value", file=sys.stderr)
    if lidar_cfg is not None:
        body = struct.pack(lidar_cfg.MSG_BODY_FORMAT[:-1], 3600, 1000,
                           *sum(([1000 + i, 200] for i in range(12)), []), 2100, 1234)
        packet = bytes([lidar_cfg.START_BYTE, lidar_cfg.VERSION]) + body
        packet += bytes([crc.crc8(packet, 0, lidar_cfg.CRC_TABLE)])
        assert len(packet) == lidar_cfg.MESSAGE_LEN
        golden["ld19_packet"] = packet

    for name, data in golden.items():
        (out / f"{name}.hex").write_text(data.hex() + "\n")
        print(f"{name}: {len(data)} bytes")


if __name__ == "__main__":
    main()
