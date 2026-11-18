"""The Python side, end to end: the compiled module runs the hub in-process
on the turret testbed, and the pyrealsense2 shim reads its D435 the way
project-otto's RealsenseImagesetSource does. Run by ctest (HWEMU_WITH_PYTHON)
with python/ and python/shims on PYTHONPATH."""

import os
import tempfile
import unittest

import numpy as np

import hwemu

EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "..", "examples", "turret_testbed")
SIM, DEVICES = os.path.join(EXAMPLE, "sim.yaml"), os.path.join(EXAMPLE, "devices.yaml")
BACKEND = "mujoco" if "mujoco" in hwemu.backends() else "achilles"


class MakeSim(unittest.TestCase):
    def test_a_voltage_turns_the_yaw_and_the_attitude_follows(self):
        sim = hwemu.make_sim(SIM, DEVICES, BACKEND)
        sim.motor("yaw_motor").command_voltage(6.0)
        for _ in range(300):
            sim.step(0.001)
        enc = sim.encoder("yaw_encoder").read()
        self.assertGreater(enc.velocity, 1.0)
        self.assertAlmostEqual(sim.attitude("turret_attitude").read().yaw, enc.position, delta=0.02)
        self.assertIn("encoder:yaw_encoder", [repr(i) for i in sim.list()])


class RealsenseShim(unittest.TestCase):
    def test_otto_style_capture(self):
        runtime = tempfile.mkdtemp(prefix="hwemu-py-")
        os.environ["HWEMU_RUNTIME_DIR"] = runtime
        import pyrealsense2 as rs

        self.assertTrue(rs.IS_HWEMU_SHIM)
        hub = hwemu.Hub(SIM, DEVICES, BACKEND, runtime)
        hub.start()
        try:
            pipe, cfg = rs.pipeline(), rs.config()
            cfg.enable_device("000000000001")
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            profile = pipe.start(cfg)
            color_sensor = next(s for s in profile.get_device().sensors if not s.is_depth_sensor())
            color_sensor.set_option(rs.option.exposure, 100)
            intr = rs.video_stream_profile(pipe.get_active_profile().get_stream(rs.stream.color)).get_intrinsics()
            self.assertEqual((intr.width, intr.height), (640, 480))
            self.assertAlmostEqual(intr.fy, 615.0, delta=0.5)
            self.assertEqual(intr.model.value, 0)
            frames = rs.align(rs.stream.color).process(pipe.wait_for_frames())
            color = np.asanyarray(frames.get_color_frame().get_data(), dtype=np.uint8)
            depth = np.asanyarray(frames.get_depth_frame().get_data(), dtype=np.uint16)
            self.assertEqual(color.shape, (480, 640, 3))
            self.assertEqual(depth.shape, (480, 640))
            if BACKEND == "mujoco":  # the URDF has a wall 2 m ahead
                self.assertAlmostEqual(frames.get_depth_frame().get_distance(320, 240), 2.0, delta=0.02)
            pipe.stop()
        finally:
            hub.stop()


if __name__ == "__main__":
    unittest.main()
