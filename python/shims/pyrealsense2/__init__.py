"""A pyrealsense2 stand-in backed by hw-emulator's simulated D435.

Put python/shims (and python/) on PYTHONPATH ahead of the real package and
`import pyrealsense2 as rs` gets this instead. Each realsense_d435 device in
the hub writes <runtime_dir>/<name>.json (its streams and intrinsics) and a
frame ring in /dev/shm; this module reads them.

It covers what project-otto's RealsenseImagesetSource uses: context /
query_devices / hardware_reset, config.enable_device / enable_stream,
pipeline.start / wait_for_frames / get_active_profile / stop, align (a
no-op: the sim's depth is already registered to its color image),
video_stream_profile.get_intrinsics, sensors' set_option (accepted, not
applied: the sim's camera has no exposure model), frame.get_data and
get_frame_metadata(backend_timestamp), and rs2_project_point_to_pixel /
rs2_deproject_pixel_to_point for a distortion-free pinhole. The color stream
is served as rgb8 or bgr8, whichever was asked for; depth as z16.
"""

from __future__ import annotations

import enum
import glob
import json
import os
import struct
import sys
import time
from typing import List, Optional, Sequence

import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(_here)))  # python/, for hwemu
from hwemu.frame_shm import FrameShmReader  # noqa: E402
from hwemu.serial import runtime_dir  # noqa: E402

__version__ = "2.55.1+hwemu"
IS_HWEMU_SHIM = True


class stream(enum.Enum):
    any = 0
    depth = 1
    color = 2
    infrared = 3


class format(enum.Enum):  # noqa: A001 -- pyrealsense2's own name
    any = 0
    z16 = 1
    rgb8 = 5
    bgr8 = 6
    rgba8 = 7
    bgra8 = 8


class distortion(enum.Enum):
    none = 0
    modified_brown_conrady = 1
    inverse_brown_conrady = 2
    ftheta = 3
    brown_conrady = 4
    kannala_brandt4 = 5


class option(enum.Enum):
    backlight_compensation = 0
    brightness = 1
    contrast = 2
    exposure = 3
    gain = 4
    gamma = 5
    hue = 6
    saturation = 7
    sharpness = 8
    white_balance = 9
    enable_auto_exposure = 10
    enable_auto_white_balance = 11


class frame_metadata_value(enum.Enum):
    frame_counter = 0
    frame_timestamp = 1
    sensor_timestamp = 2
    backend_timestamp = 3
    time_of_arrival = 4


class camera_info(enum.Enum):
    name = 0
    serial_number = 1
    product_line = 2


class intrinsics:
    def __init__(self, width=0, height=0, ppx=0.0, ppy=0.0, fx=0.0, fy=0.0):
        self.width, self.height = width, height
        self.ppx, self.ppy, self.fx, self.fy = ppx, ppy, fx, fy
        self.model = distortion.none
        self.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

    def __repr__(self):
        return (f"[ {self.width}x{self.height}  p[{self.ppx} {self.ppy}]  f[{self.fx} {self.fy}]  "
                f"{self.model.name} {self.coeffs} ]")


def rs2_project_point_to_pixel(intrin: intrinsics, point: Sequence[float]) -> List[float]:
    x, y, z = point
    return [x / z * intrin.fx + intrin.ppx, y / z * intrin.fy + intrin.ppy]


def rs2_deproject_pixel_to_point(intrin: intrinsics, pixel: Sequence[float], depth: float) -> List[float]:
    u, v = pixel
    return [(u - intrin.ppx) / intrin.fx * depth, (v - intrin.ppy) / intrin.fy * depth, depth]


def _devices() -> List[dict]:
    out = []
    for path in sorted(glob.glob(os.path.join(runtime_dir(), "*.json"))):
        try:
            with open(path) as f:
                info = json.load(f)
        except (OSError, ValueError):
            continue
        if "shm" in info and "depth_scale" in info:
            info["name"] = os.path.splitext(os.path.basename(path))[0]
            out.append(info)
    return out


class _Sensor:
    def __init__(self, depth: bool, info: dict):
        self._depth = depth
        self._info = info
        self._options = {}

    def is_depth_sensor(self) -> bool:
        return self._depth

    def as_depth_sensor(self):
        return self

    def get_depth_scale(self) -> float:
        return float(self._info["depth_scale"])

    def set_option(self, opt, value) -> None:
        self._options[opt] = value  # the sim's camera has no exposure or color model

    def get_option(self, opt):
        return self._options.get(opt, 0.0)

    def supports(self, opt) -> bool:
        return True


class device:
    def __init__(self, info: dict):
        self._info = info
        self.sensors = [_Sensor(True, info), _Sensor(False, info)]

    def get_info(self, what: camera_info) -> str:
        if what == camera_info.serial_number:
            return str(self._info["serial"])
        if what == camera_info.name:
            return "Intel RealSense D435 (hw-emulator)"
        return "D400"

    def query_sensors(self):
        return self.sensors

    def first_depth_sensor(self):
        return self.sensors[0]

    def hardware_reset(self) -> None:
        pass

    def __repr__(self):
        return f"<pyrealsense2.device: Intel RealSense D435 (hw-emulator) (S/N: {self._info['serial']})>"


class context:
    def query_devices(self) -> List[device]:
        return [device(i) for i in _devices()]

    @property
    def devices(self) -> List[device]:
        return self.query_devices()


class config:
    def __init__(self):
        self.serial: Optional[str] = None
        self.streams = {}

    def enable_device(self, serial: str) -> None:
        self.serial = str(serial)

    def enable_stream(self, s: stream, *args) -> None:
        # (stream), (stream, format, fps), (stream, w, h, format, fps), ...
        fmt = next((a for a in args if isinstance(a, format)), None)
        self.streams[s] = fmt


class stream_profile:
    def __init__(self, s: stream, info: dict, fmt: format):
        self._stream, self._info, self._format = s, info, fmt

    def stream_type(self) -> stream:
        return self._stream

    def format(self) -> format:  # noqa: A003
        return self._format

    def fps(self) -> int:
        return 30

    def as_video_stream_profile(self) -> "video_stream_profile":
        return video_stream_profile(self)

    def get_intrinsics(self) -> intrinsics:
        i = self._info
        return intrinsics(int(i["width"]), int(i["height"]), float(i["ppx"]), float(i["ppy"]),
                          float(i["fx"]), float(i["fy"]))


class video_stream_profile(stream_profile):
    def __init__(self, profile: stream_profile):
        super().__init__(profile._stream, profile._info, profile._format)

    def width(self) -> int:
        return int(self._info["width"])

    def height(self) -> int:
        return int(self._info["height"])


class pipeline_profile:
    def __init__(self, dev: device, profiles: dict):
        self._device, self._profiles = dev, profiles

    def get_device(self) -> device:
        return self._device

    def get_stream(self, s: stream) -> stream_profile:
        return self._profiles[s]

    def get_streams(self) -> List[stream_profile]:
        return list(self._profiles.values())


class frame:
    def __init__(self, data: np.ndarray, number: int, timestamp_ms: float, profile: stream_profile):
        self._data, self._number, self._timestamp, self._profile = data, number, timestamp_ms, profile

    def get_data(self):
        return self._data

    def get_frame_number(self) -> int:
        return self._number

    def get_timestamp(self) -> float:
        return self._timestamp

    def get_frame_metadata(self, what: frame_metadata_value) -> int:
        if what == frame_metadata_value.frame_counter:
            return self._number
        # The sim time the image was rendered at, in ms, as the D435's clocks.
        return int(round(self._timestamp))

    def supports_frame_metadata(self, what) -> bool:
        return True

    def get_profile(self) -> stream_profile:
        return self._profile

    def get_width(self) -> int:
        return self._data.shape[1]

    def get_height(self) -> int:
        return self._data.shape[0]

    def __bool__(self) -> bool:
        return self._data is not None


class video_frame(frame):
    pass


class depth_frame(video_frame):
    def __init__(self, data, number, timestamp_ms, profile, depth_scale):
        super().__init__(data, number, timestamp_ms, profile)
        self._scale = depth_scale

    def get_distance(self, x: int, y: int) -> float:
        return float(self._data[y, x]) * self._scale


class composite_frame(frame):
    def __init__(self, color: Optional[video_frame], depth: Optional[depth_frame]):
        self._color, self._depth = color, depth

    def get_color_frame(self) -> Optional[video_frame]:
        return self._color

    def get_depth_frame(self) -> Optional[depth_frame]:
        return self._depth

    def __iter__(self):
        return iter([f for f in (self._depth, self._color) if f is not None])

    def size(self) -> int:
        return len(list(iter(self)))


frameset = composite_frame


class align:
    """A no-op: the simulated depth is rendered from the color camera's own
    pose and intrinsics, so it's already aligned to color."""

    def __init__(self, align_to: stream):
        self.align_to = align_to

    def process(self, frames: composite_frame) -> composite_frame:
        return frames


_META = struct.Struct("<IIIII fffff Q d Q")


class pipeline:
    def __init__(self, ctx: Optional[context] = None):
        self._reader: Optional[FrameShmReader] = None
        self._last = 0

    def start(self, cfg: Optional[config] = None) -> pipeline_profile:
        cfg = cfg or config()
        deadline = time.monotonic() + float(os.environ.get("HWEMU_REALSENSE_WAIT", "10"))
        while True:
            devices = _devices()
            if cfg.serial is not None:
                devices = [d for d in devices if str(d["serial"]) == cfg.serial]
            if devices:
                break
            if time.monotonic() > deadline:
                want = f" with serial {cfg.serial}" if cfg.serial else ""
                raise RuntimeError(f"No device connected{want} (no hw-emulator D435 under {runtime_dir()})")
            time.sleep(0.1)
        self._info = devices[0]
        self._device = device(self._info)
        streams = cfg.streams or {stream.depth: format.z16, stream.color: format.rgb8}
        self._color_format = streams.get(stream.color) or format.rgb8
        if self._color_format not in (format.rgb8, format.bgr8):
            raise RuntimeError(f"hw-emulator's D435 serves color as rgb8 or bgr8, not {self._color_format.name}")
        self._profiles = {}
        if stream.color in streams:
            self._profiles[stream.color] = stream_profile(stream.color, self._info, self._color_format)
        if stream.depth in streams:
            self._profiles[stream.depth] = stream_profile(stream.depth, self._info, format.z16)
        while True:
            try:
                self._reader = FrameShmReader(self._info["shm"])
                break
            except FileNotFoundError:
                if time.monotonic() > deadline:
                    raise RuntimeError(f"hw-emulator D435 frame ring /dev/shm/{self._info['shm']} never appeared")
                time.sleep(0.1)
        self._profile = pipeline_profile(self._device, self._profiles)
        return self._profile

    def get_active_profile(self) -> pipeline_profile:
        return self._profile

    def poll_for_frames(self) -> Optional[composite_frame]:
        f = self._reader.read_latest(self._last) if self._reader else None
        return self._unpack(f) if f else None

    def try_wait_for_frames(self, timeout_ms: int = 5000):
        try:
            return True, self.wait_for_frames(timeout_ms)
        except RuntimeError:
            return False, None

    def wait_for_frames(self, timeout_ms: int = 5000) -> composite_frame:
        if self._reader is None:
            raise RuntimeError("wait_for_frames cannot be called before start()")
        deadline = time.monotonic() + timeout_ms / 1000.0
        while True:
            f = self._reader.read_latest(self._last)
            if f is not None:
                return self._unpack(f)
            if time.monotonic() > deadline:
                raise RuntimeError(f"Frame didn't arrive within {timeout_ms}")
            time.sleep(0.001)

    def stop(self) -> None:
        if self._reader is not None:
            self._reader.close()
            self._reader = None

    def _unpack(self, f) -> composite_frame:
        self._last = f.sequence
        (_, w, h, _, _, _, _, _, _, scale, number, capture_time, _) = _META.unpack_from(f.meta, 0)
        pixels = w * h
        rgb = np.frombuffer(f.payload, dtype=np.uint8, count=3 * pixels).reshape(h, w, 3)
        depth = np.frombuffer(f.payload, dtype=np.uint16, count=pixels, offset=3 * pixels).reshape(h, w)
        ms = capture_time * 1000.0
        color = None
        if stream.color in self._profiles:
            image = rgb[:, :, ::-1] if self._color_format == format.bgr8 else rgb
            color = video_frame(np.ascontiguousarray(image), number, ms, self._profiles[stream.color])
        dframe = None
        if stream.depth in self._profiles:
            dframe = depth_frame(depth.copy(), number, ms, self._profiles[stream.depth], scale)
        return composite_frame(color, dframe)
