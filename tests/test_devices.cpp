// Every device's unit conversions, against a FakeSim, over a loopback CAN
// bus standing in for vcan0 and the firmware on its far end.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <numbers>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "hwemu/devices/builtin.hpp"
#include "hwemu/protocol/can_encoder.hpp"
#include "hwemu/protocol/dji_motor_can.hpp"
#include "hwemu/protocol/ld19.hpp"
#include "hwemu/protocol/tmotor_can.hpp"
#include "hwemu/protocol/turret_mcb_can.hpp"
#include "hwemu/sim/fake_sim.hpp"
#include "hwemu/transport/frame_shm.hpp"
#include "hwemu_client.h"

using namespace hwemu;
using protocol::CanFrame;

namespace {

std::string TempDir(const std::string& name) {
  auto dir = std::filesystem::temp_directory_path() / ("hwemu_test_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  return dir.string();
}

devices::DeviceSpec Spec(const std::string& yaml) {
  const YAML::Node n = YAML::Load(yaml);
  devices::DeviceSpec s;
  s.name = n["name"].as<std::string>();
  s.type = n["type"].as<std::string>();
  s.transport = n["transport"];
  s.bind = n["bind"];
  s.params = n["params"] ? n["params"] : YAML::Node(YAML::NodeType::Map);
  return s;
}

clock::SimTime At(double seconds, std::uint64_t tick = 0) {
  return {tick, static_cast<std::uint64_t>(seconds * 1e6), seconds, 0.001};
}

struct Rig {
  sim::FakeSim sim;
  devices::Transports transports;
  std::shared_ptr<transport::LoopbackCanBus> firmware;
  explicit Rig(const std::string& name) : transports(TempDir(name)) {
    transports.Can("loopback:can1");
    firmware = transports.CanBuses().Peer("can1");
  }
  std::vector<CanFrame> Drain() {
    std::vector<CanFrame> out;
    CanFrame f;
    while (firmware->Receive(&f)) out.push_back(f);
    return out;
  }
};

}  // namespace

TEST(DjiCanMotor, Gm6020VoltageCommandAndFeedback) {
  Rig rig("dji");
  devices::DjiCanMotor motor;
  motor.Configure(Spec(R"(
    name: yaw
    type: dji_can_motor
    transport: {bus: "loopback:can1", id: 0x205}
    bind: {motor: yaw_motor, encoder: yaw_encoder}
    params: {mode: voltage, command_scale: 0.00096, max_command: 25000, feedback_current_scale: 0.000183,
             reduction: 1.0, feedback_rate_hz: 1000, command_timeout: 0.05})"));
  motor.Bind(rig.sim, rig.transports);

  // The firmware commands -12500 on slot 0 of 0x1FF: half of -24 V.
  rig.firmware->Send(protocol::dji_motor::EncodeCommand(0x1FF, {-12500, 0, 0, 0}));
  rig.transports.Router().Poll();
  const auto cmd = rig.sim.LastCommand("yaw_motor");
  EXPECT_EQ(cmd.mode, sim::MotorCommand::Mode::kVoltage);
  EXPECT_NEAR(cmd.value, -12.0, 1e-9);

  // The motor sits at 90 deg turning 2 rad/s with 1.5 A in the winding.
  rig.sim.SetEncoder("yaw_encoder", {std::numbers::pi / 2, 2.0});
  rig.sim.SetMotorReading("yaw_motor", {1.5, 0.0, 45.0});
  motor.Tick(At(0.0));
  const auto frames = rig.Drain();
  ASSERT_EQ(frames.size(), 1U);
  const auto fb = protocol::dji_motor::DecodeFeedback(frames[0]);
  ASSERT_TRUE(fb);
  EXPECT_EQ(frames[0].id, 0x205U);
  EXPECT_EQ(fb->encoder, 2048);
  EXPECT_EQ(fb->rpm, static_cast<std::int16_t>(std::lround(2.0 * 60 / (2 * std::numbers::pi))));
  EXPECT_NEAR(fb->current * 0.000183, 1.5, 0.001);
  EXPECT_EQ(fb->temperature, 45);

  // Feedback runs at 1 kHz: nothing more until 1 ms later.
  motor.Tick(At(0.0004));
  EXPECT_TRUE(rig.Drain().empty());
  motor.Tick(At(0.001));
  EXPECT_EQ(rig.Drain().size(), 1U);

  // No command for longer than command_timeout: output to zero.
  motor.Tick(At(0.2));
  EXPECT_NEAR(rig.sim.LastCommand("yaw_motor").value, 0.0, 1e-12);
}

TEST(DjiCanMotor, M3508RotorThroughGearbox) {
  Rig rig("m3508");
  devices::DjiCanMotor motor;
  motor.Configure(Spec(R"(
    name: wheel
    type: dji_can_motor
    transport: {bus: "loopback:can1", id: 0x202}
    bind: {motor: m, encoder: e}
    params: {mode: current, command_scale: 0.001220703125, max_command: 16384, reduction: 19.2032})"));
  motor.Bind(rig.sim, rig.transports);
  rig.firmware->Send(protocol::dji_motor::EncodeCommand(0x200, {0, 8192, 0, 0}));
  rig.transports.Router().Poll();
  EXPECT_NEAR(rig.sim.LastCommand("m").value, 10.0, 1e-9);  // 8192 of 16384 is 10 A of 20
  // One output turn is 19.2 rotor turns: the rotor encoder wraps.
  rig.sim.SetEncoder("e", {2 * std::numbers::pi / 19.2032 * 0.25, 1.0});
  motor.Tick(At(0));
  const auto fb = protocol::dji_motor::DecodeFeedback(rig.Drain().at(0));
  EXPECT_NEAR(fb->encoder, 2048, 1);
  EXPECT_NEAR(fb->rpm, 1.0 * 19.2032 * 60 / (2 * std::numbers::pi), 1);
}

TEST(TmotorAk80, CurrentCommandAndServoFeedback) {
  Rig rig("tmotor");
  devices::TmotorAk80 motor;
  motor.Configure(Spec(R"(
    name: arm
    type: tmotor_ak80
    transport: {bus: "loopback:can1", id: 2}
    bind: {motor: m, encoder: e})"));
  motor.Bind(rig.sim, rig.transports);
  rig.firmware->Send(protocol::tmotor::EncodeCurrentCommand(2, 2500));
  rig.transports.Router().Poll();
  EXPECT_NEAR(rig.sim.LastCommand("m").value, 2.5, 1e-9);
  rig.sim.SetEncoder("e", {std::numbers::pi / 2, 1.0});
  rig.sim.SetMotorReading("m", {2.5, 0.0, std::nullopt});
  motor.Tick(At(0));
  const auto fb = protocol::tmotor::DecodeFeedback(rig.Drain().at(0));
  ASSERT_TRUE(fb);
  EXPECT_EQ(fb->position, 900);  // 90.0 deg in 0.1 deg
  EXPECT_EQ(fb->current, 250);
  // aruwsrc: velocity = rpm * 2pi / 60 * (1/9)
  EXPECT_NEAR(fb->speed * 2 * std::numbers::pi / 60.0 / 9.0, 1.0, 0.01);
}

TEST(CanEncoders, CountsAndCentidegrees) {
  Rig rig("enc");
  devices::CanEncoderDevice plain(false), lamprey(true);
  plain.Configure(Spec(R"({name: a, type: can_encoder, transport: {bus: "loopback:can1", id: 0x1E1},
                           bind: {encoder: e}, params: {gauss: 650}})"));
  lamprey.Configure(Spec(R"({name: b, type: lamprey_encoder, transport: {bus: "loopback:can1", id: 0x1E2},
                             bind: {encoder: e}})"));
  plain.Bind(rig.sim, rig.transports);
  lamprey.Bind(rig.sim, rig.transports);
  rig.sim.SetEncoder("e", {-std::numbers::pi / 2, 0});
  plain.Tick(At(0));
  lamprey.Tick(At(0));
  const auto frames = rig.Drain();
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(protocol::can_encoder::Decode(frames[0])->counts, 3072);
  EXPECT_EQ(protocol::can_encoder::Decode(frames[0])->gauss, 650);
  EXPECT_EQ(*protocol::can_encoder::DecodeLamprey(frames[1]), 27000);
}

TEST(TurretMcb, ImuFramesHandshakesAndRecalibration) {
  Rig rig("turret");
  devices::TurretMcb turret;
  turret.Configure(Spec(R"(
    name: turret_mcb
    type: turret_mcb
    transport: {bus: "loopback:can1"}
    bind: {gyro: g, accel: a, attitude: att, kicker_switch: kick}
    params: {imu_rate_hz: 500, status_rate_hz: 100, calibration_time: 0.5, temperature: 41.5})"));
  turret.Bind(rig.sim, rig.transports);
  rig.sim.SetGyro("g", {0.1, -0.2, 3.0});
  rig.sim.SetAccel("a", {0.0, 0.0, 9.81});
  sim::AttitudeReading att;
  att.roll = 0.01;
  att.pitch = -0.3;
  att.yaw = 2.5;
  rig.sim.SetAttitude("att", att);
  rig.sim.SetSwitch("kick", true);

  turret.Tick(At(0));
  using namespace protocol::turret_mcb;
  std::map<std::uint32_t, CanFrame> seen;
  for (const auto& f : rig.Drain()) seen[f.id] = f;
  ASSERT_TRUE(seen.count(kXAxisId) && seen.count(kYAxisId) && seen.count(kZAxisId) && seen.count(kStatusId));
  EXPECT_TRUE(seen.count(kCalibrationSamplesRequestId) && seen.count(kImuMountingRequestId));
  const auto z = *DecodeAxis(seen[kZAxisId]);
  EXPECT_NEAR(FixedToAngle(z.angle), 2.5, 1e-4);
  EXPECT_NEAR(RawToRate(z.rate), 3.0, 1e-3);
  EXPECT_NEAR(RawToAcceleration(z.acceleration), 9.81, 0.01);
  EXPECT_EQ(DecodeAxis(seen[kXAxisId])->seq, z.seq);
  const auto status = *DecodeStatus(seen[kStatusId]);
  EXPECT_EQ(status.bitmask & kKickerLimitSwitch, kKickerLimitSwitch);
  EXPECT_EQ(status.imu_state, ImuState::kCalibrated);
  EXPECT_EQ(status.temperature_centi_c, 4150);

  // The chassis answers the handshakes and asks for a recalibration.
  rig.firmware->Send(EncodeCalibrationSamples(1500));
  rig.firmware->Send(EncodeCommand(kRecalibrateImu));
  rig.transports.Router().Poll();
  EXPECT_EQ(*turret.CalibrationSamples(), 1500);
  EXPECT_EQ(turret.State(), ImuState::kCalibrating);
  turret.Tick(At(0.1));
  for (const auto& f : rig.Drain()) {
    EXPECT_NE(f.id, kXAxisId) << "no IMU data while calibrating";
    if (f.id == kStatusId) EXPECT_EQ(DecodeStatus(f)->imu_state, ImuState::kCalibrating);
  }
  turret.Tick(At(0.6));
  EXPECT_EQ(turret.State(), ImuState::kCalibrated);
}

TEST(PinbusDevices, SpiImuAndLimitSwitchThroughTheClient) {
  Rig rig("pinbus");
  devices::SpiImu imu;
  imu.Configure(Spec(R"({name: imu, type: spi_imu, transport: {device: imu},
                         bind: {gyro: g, accel: a}, params: {chip: mpu6500, temperature: 30}})"));
  devices::LimitSwitchDevice sw;
  sw.Configure(Spec(R"({name: ls, type: limit_switch, transport: {pin: B12}, bind: {switch: s},
                        params: {active_low: true}})"));
  imu.Bind(rig.sim, rig.transports);
  sw.Bind(rig.sim, rig.transports);
  rig.sim.SetGyro("g", {0, 0, 1.0});
  rig.sim.SetAccel("a", {0, 0, 9.80665});
  imu.Tick(At(0));
  sw.Tick(At(0));

  const int h = hwemu_pinbus_connect(rig.transports.PinBus().Path().c_str(), "test");
  ASSERT_GE(h, 0);
  std::uint8_t tx[2] = {0x75 | 0x80, 0}, rx[2] = {};
  ASSERT_EQ(hwemu_spi_transfer(h, "imu", tx, rx, 2), 0);
  EXPECT_EQ(rx[1], 0x70);
  std::uint8_t cfg[2] = {0x1B, 0x18};
  ASSERT_EQ(hwemu_spi_transfer(h, "imu", cfg, rx, 2), 0);
  imu.Tick(At(0.001));
  std::uint8_t burst_tx[15] = {0x3B | 0x80}, burst_rx[15] = {};
  ASSERT_EQ(hwemu_spi_transfer(h, "imu", burst_tx, burst_rx, 15), 0);
  const auto gz = static_cast<std::int16_t>(burst_rx[13] << 8 | burst_rx[14]);
  EXPECT_NEAR(gz / 16.4 * std::numbers::pi / 180.0, 1.0, 1e-3);
  EXPECT_EQ(hwemu_spi_transfer(h, "nope", tx, rx, 2), -1);

  // Active low: released reads high, pressed reads low.
  EXPECT_EQ(hwemu_gpio_read(h, "B12"), 1);
  rig.sim.SetSwitch("s", true);
  sw.Tick(At(0.002));
  EXPECT_EQ(hwemu_gpio_read(h, "B12"), 0);
  EXPECT_EQ(hwemu_gpio_write(h, "laser", 1), 0);
  EXPECT_TRUE(rig.transports.PinBus().Output("laser"));
  hwemu_pinbus_close(h);
}

TEST(Ld19Lidar, PacketsOnThePty) {
  Rig rig("ld19");
  devices::Ld19Lidar lidar;
  lidar.Configure(Spec(R"({name: ld19, type: ld19_lidar, transport: {port: ld19}, bind: {lidar: l},
                           params: {scan_rate_hz: 10}})"));
  lidar.Bind(rig.sim, rig.transports);
  const int fd = hwemu_serial_open(lidar.PortPath().c_str());
  ASSERT_GE(fd, 0);
  sim::LidarScan scan;
  for (int i = 0; i < 36; ++i) {
    scan.ranges.push_back(1.0F + 0.01F * static_cast<float>(i));
    scan.angles.push_back(static_cast<float>(i * 2 * std::numbers::pi / 36));
  }
  scan.sequence = 1;
  rig.sim.SetLidar("l", scan);
  lidar.Tick(At(1.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::vector<std::uint8_t> bytes(4096);
  const ssize_t n = ::read(fd, bytes.data(), bytes.size());
  ::close(fd);
  ASSERT_EQ(n, 3 * 47);
  auto p = protocol::ld19::Decode(std::span(bytes).first(47));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->speed_dps, 3600);
  EXPECT_EQ(p->timestamp_ms, 1000);
  EXPECT_EQ(p->points[0].distance_mm, 1000);  // beam 0, at 0 deg either way
}

TEST(RealsenseD435, FramesInSharedMemory) {
  Rig rig("d435");
  devices::RealsenseD435 cam;
  cam.Configure(Spec(R"({name: d435, type: realsense_d435, transport: {shm: hwemu_test_d435},
                         bind: {camera: c}})"));
  sim::CameraFrame frame;
  frame.width = 4;
  frame.height = 2;
  frame.rgb.assign(4 * 2 * 3, 7);
  frame.depth = {1.0F, 2.5F, 0.0F, std::numeric_limits<float>::infinity(), 0.3F, 0.3F, 0.3F, 0.3F};
  frame.capture_time = 0.5;
  frame.sequence = 1;
  rig.sim.SetCamera("c", frame, {4, 2, 3.0, 3.0, 2.0, 1.0});
  cam.Bind(rig.sim, rig.transports);
  cam.Tick(At(0.5));

  transport::FrameShmReader reader("hwemu_test_d435");
  transport::FrameShmReader::Frame got;
  ASSERT_TRUE(reader.ReadLatest(&got));
  ASSERT_EQ(got.payload.size(), 4U * 2 * 5);
  EXPECT_EQ(got.payload[0], 7);
  const auto* depth = reinterpret_cast<const std::uint16_t*>(got.payload.data() + 24);
  EXPECT_EQ(depth[0], 1000);
  EXPECT_EQ(depth[1], 2500);
  EXPECT_EQ(depth[2], 0);
  EXPECT_EQ(depth[3], 0);  // no return
  EXPECT_FALSE(reader.ReadLatest(&got, got.sequence));
  EXPECT_TRUE(std::filesystem::exists(rig.transports.Resolve("d435.json")));
}
