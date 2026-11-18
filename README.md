# hw-emulator

A simulated robot that the real code can't tell from the real one. The hub
runs a physics sim ([Achilles](https://github.com/EricJGoossen/Achilles) or
[MuJoCo](https://mujoco.org)) and puts each simulated part on the wire the
way the real part talks: DJI motors and the turret MCB on CAN, IMUs on SPI,
the LD19 lidar and the MCB↔Jetson link on serial, and the RealSense D435
through a `pyrealsense2` stand-in. Two programs run against it unchanged,
apart from their HAL:

- **aruw-mcb**, built for taproot's hosted platform. It reaches the hub through
  `client/hwemu_client.h`: SocketCAN, PTYs, the pinbus socket for SPI and
  GPIO, and the sim clock.
- **project-otto**, which opens the hub's PTYs as its serial ports and gets
  its camera from `python/shims/pyrealsense2`.

```
        aruw-mcb (hosted)                     project-otto (Jetson code)
   CAN │ SPI/GPIO │ UART │ clock               UART │ pyrealsense2
       ▼          ▼      ▼                          ▼       ▼
  ┌────────────────────────────── hub ───────────────────────────────┐
  │ transports: SocketCAN / loopback · PTYs · pinbus · frame shm ring│
  │ devices:    dji_can_motor · turret_mcb · spi_imu · ld19_lidar ·  │
  │             realsense_d435 · vision_link · limit_switch · …      │
  │ sim::ISim:  motor · encoder · gyro · accel · mag · attitude ·    │
  │             camera · lidar · switch                              │
  │ backends:   Achilles (.arow)        MuJoCo (URDF + bind.mujoco)  │
  └──────────────────────────────────────────────────────────────────┘
```

Each device only converts units between its wire format (`protocol/`) and
backend-neutral sim kinds (`sim/sim.hpp`). The only quantization it adds is
what the wire format itself imposes. All sensor error lives in the backends.
Achilles models it with its error chains. MuJoCo stores a sensor's `noise`
but never applies it, so the MuJoCo backend adds it from a seeded generator.

## Building

```sh
scripts/build.sh                                      # everything, then ctest
scripts/build.sh -DHWEMU_WITH_MUJOCO=OFF              # Achilles only
scripts/build.sh -DFETCHCONTENT_SOURCE_DIR_ACHILLES=../achilles   # a local Achilles
```

| Option | Default | |
|---|---|---|
| `HWEMU_WITH_ACHILLES` | ON | Fetches Achilles at `HWEMU_ACHILLES_TAG` (branch `claude/hwemu-support`) |
| `HWEMU_WITH_MUJOCO` | ON | Downloads the MuJoCo `HWEMU_MUJOCO_VERSION` (3.3.0) release tarball, hash-pinned; `HWEMU_MUJOCO_DIR` uses an unpacked one |
| `HWEMU_WITH_DISPLAY` | ON | Window displays: Achilles's renderer, MuJoCo's `simulate` (GLFW) |
| `HWEMU_WITH_PYTHON` | ON | The pybind11 module `_hwemu` |

Needs CMake ≥ 3.21, a C++20 compiler, and the EGL, OpenGL and GLFW development
packages (`libegl-dev libgl-dev libglfw3-dev` on Ubuntu). Cameras render
headless through EGL on both backends, so no display server is needed, and
Mesa's software renderer is enough. Linking the Achilles tests takes a few
GB per job; on a small machine, build with `-j1` or `-j2`.

## Running the example

```sh
./build/hwemu --sim examples/turret_testbed/sim.yaml \
              --devices examples/turret_testbed/devices.yaml \
              [--backend achilles|mujoco] [--window] [--seconds 10] [--status 1]
```

`examples/turret_testbed` is a yaw/pitch turret with two GM6020s, a turret
MCB IMU, a pitch limit switch and a D435. It has the same masses and inertias
in `robot.arow` (Achilles) and `robot.urdf` (MuJoCo), and the URDF adds a wall
2 m ahead for the camera. `devices.yaml` puts the motors on `loopback:can1`,
an in-process bus that needs no kernel module. Use `vcan0` (see
`scripts/vcan.sh`) once a hosted MCB needs to reach the bus from another
process.

`tests/test_backend_parity.cpp` drives the testbed identically on both
backends and checks that they agree: rotation directions, rates, currents,
temperatures, gravity, the attitude and the switch angle. It also checks that
MuJoCo's depth reads 2.00 m at the wall.

## Configuration

Two files; their schemas are in `schemas/`, and `tests/python/test_schemas.py`
validates the example against them.

- **`sim.yaml`** (`schemas/sim.schema.json`): the backend, the display, the
  models, the clock (`realtime`, or `lockstep` with the participants named),
  the runtime directory, and the sensor windows to open.
- **`devices.yaml`** (`schemas/devices.schema.json`): each device's `type`,
  `transport`, `bind` (the sim kinds behind it) and `params`. A `preset`
  (from `presets/`, e.g. `presets/dji_motors.yaml`; `presets.schema.json`) is
  merged under its params.

For Achilles, `bind` names attachments that the `.arow` declares. MuJoCo's
URDF has no motors or sensors, so each binding also declares its MuJoCo
element under `bind.mujoco`, and the backend adds them to the model's mjSpec
before compiling:

| `type` | Becomes | Keys |
|---|---|---|
| `motor` | a joint actuator driven by a DC-motor model (as Achilles's `DcMotor`, thermal model included) | `joint`, `torque_constant`, `back_emf_constant`, `resistance`, `bus_voltage`, `max_current`, `gear_ratio`, `gearbox_efficiency`, `viscous_friction`, `ambient_temperature`, `thermal_resistance`, `thermal_time_constant` |
| `encoder` | `jointpos` + `jointvel` sensors | `joint`, `noise`, `velocity_noise`, `resolution` |
| `gyro` / `accel` / `mag` | a site and its sensor | `body`, `pos`, `quat`, `noise` |
| `attitude` | a site and a `framequat` sensor, reported as ZYX roll/pitch/yaw | `body`, `pos`, `quat`, `noise` |
| `switch` | a `jointpos` sensor, thresholded with hysteresis | `joint`, `press_above` / `press_below`, `hysteresis`, `normally_closed`, `noise` |
| `camera` | a camera, rendered offscreen; pose given x-forward/z-up, as in Achilles | `body`, `pos`, `quat`, `width`, `height`, `fx`, `fy` (or `fovy`), `cx`, `cy`, `near`, `far`, `period` |
| `lidar` | a site, cast as an `mj_ray` fan | `body`, `pos`, `quat`, `samples`, `fov`, `min_range`, `max_range`, `period`, `noise` |

## Backends and displays

| | Achilles | MuJoCo |
|---|---|---|
| Model | `.arow` (+ `sim_config.yaml`) | URDF or MJCF, plus `bind.mujoco` |
| Sensor error | Achilles's error chains (bias, noise, latency, …) | Gaussian `noise` and `resolution`, applied by hw-emulator |
| Camera | Achilles's GPU camera, pinhole intrinsics | EGL offscreen render; square pixels (`fy` sets the FOV, `fx` is reported as given) and a centred principal point |
| Window | Achilles's renderer, with a window per sensor (`windows:` in sim.yaml) | MuJoCo's `simulate`, passive, on its own thread: its own sensor plot and pause, no per-sensor windows |

Code that needs a backend's own objects can get them with
`sim::AchillesExtras` (the `Simulation` and the renderer) or
`sim::MujocoExtras` (`mjModel`, `mjData` and the mutex that guards them).

## Python

```sh
export PYTHONPATH=$PWD/python/shims:$PWD/python   # shims first, to stand in for pyrealsense2
```

- **`hwemu`** (pure Python): `FrameShmReader` maps a hub frame ring,
  `serial_port(name)` / `open_serial(name)` resolve a hub PTY, and
  `runtime_dir()` reads `$HWEMU_RUNTIME_DIR` (default `/tmp/hwemu`).
- **`hwemu._hwemu`** (compiled): `Hub(sim_yaml, devices_yaml, backend,
  runtime_dir)` runs the hub in-process (`tick()`, or `start()` on a
  thread); `make_sim(...)` gives a bare sim whose kinds can be read and
  commanded directly. Camera frames come back as numpy arrays.
- **`pyrealsense2` shim**: covers what project-otto's
  `RealsenseImagesetSource` calls: `context().query_devices()`,
  `config.enable_device` / `enable_stream`, `pipeline.start` /
  `wait_for_frames`, `align` (a no-op, since the sim's depth is already
  registered to color), `video_stream_profile.get_intrinsics` (distortion
  `none`), `set_option` (accepted but ignored), frame data as `rgb8`/`bgr8`
  and `z16`, and `backend_timestamp` metadata in sim milliseconds.

## Not emulated

The aruw-mcb devices that hw-emulator doesn't emulate yet. The hosted
firmware's HAL has to stub them, or the code using them has to be
configured out:

| Device (aruw-mcb / taproot) | Interface | Why not yet |
|---|---|---|
| DR16 remote receiver (`tap::communication::serial::Remote`) | UART (DBUS, 100 kbaud, inverted) | Needs an input source (keyboard or gamepad) to send; the frame codec is simple to add on a PTY |
| Referee system (`RefSerial`, `RefSerialTransmitter`) | UART, DJI serial framing | The framing exists (`protocol/dji_serial`), but the referee's game state, power buffer and HP model don't |
| PWM servos and flywheel ESCs (`tap::motor::Servo`, `tap::gpio::Pwm`) | PWM | The pinbus has no PWM message, and the sims have no position-servo kind |
| Analog voltage and current sensors (`VoltageSensorInterface`, `CurrentSensorInterface`, power limiting) | ADC | No analog pins on the pinbus, and no electrical model of the chassis supply |
| LEDs, buzzer, digital outputs (`tap::gpio::Leds`, `Digital::OutputPin`) | GPIO out | The pinbus accepts GPIO writes and passes them to output handlers, but no device registers one, so nothing reacts |
| OLED display and its buttons (`tap::display`, sh1106) | SPI + ADC | UI only |
| T-Motor AK80-9's other servo-mode commands (duty, speed, position, position-speed) | CAN, extended ids | `tmotor_ak80` implements the current and origin commands, the ones aruwsrc's `Tmotor_AK809` sends |

## Known discrepancy between the codebases

Message **0x0A** on the MCB↔Jetson link doesn't match between the two repos:
project-otto sends it as `world_to_turret`, 37 bytes, while aruw-mcb's
aruwsrc reads 0x0A as a 45-byte ArUco packet. `vision_link` relays bytes
unchanged, so the hub neither causes nor hides this; aruw-mcb will reject or
misparse otto's 0x0A frames until one side changes.
`tests/golden/frame_world_to_turret.hex` holds otto's version, generated by
`tools/make_golden.py`.

## Layout

```
include/hwemu/  protocol/ transport/ clock/ sim/ devices/ config/ display/ hub/
src/            the same, plus sim/{achilles,mujoco}_backend.cpp, display/{achilles,mujoco}_display.cpp
apps/hwemu.cpp  the hub's command line
client/         hwemu_client.{h,cpp}: the C library the hosted MCB links
python/         _hwemu module, hwemu package, shims/pyrealsense2
presets/        device params presets       schemas/  config JSON schemas
examples/       turret_testbed              tests/    GoogleTest + Python tests, golden frames
```
