#pragma once

// The built-in device emulators, one per real part with a sim equivalent.
// Their YAML is documented in docs/devices.md and checked by
// schemas/devices.schema.json; the table:
//
//   type               transport                 sim kinds
//   dji_can_motor      CAN (feedback id)         Motor + Encoder
//   tmotor_ak80        CAN (motor id, extended)  Motor + Encoder
//   can_encoder        CAN (0x1E0..)             Encoder
//   lamprey_encoder    CAN (0x1E0..)             Encoder
//   turret_mcb         CAN                       Gyro + Accel + Attitude (+ Switch)
//   spi_imu            pinbus SPI                Gyro + Accel
//   limit_switch       pinbus GPIO               Switch
//   ld19_lidar         PTY                       Lidar
//   realsense_d435     shared memory             Camera
//   vision_link        PTY <-> PTY               (none: a relay)

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "hwemu/devices/device.hpp"
#include "hwemu/protocol/dji_motor_can.hpp"
#include "hwemu/protocol/dji_serial.hpp"
#include "hwemu/protocol/spi_imu.hpp"
#include "hwemu/protocol/turret_mcb_can.hpp"

namespace hwemu::devices {

// A command that lapses: a motor controller that stops hearing from the
// firmware zeroes its output.
struct CommandWatchdog {
  double timeout = 0.1;  // s; 0 disables
  double last = -1.0;
  bool expired = true;
  void Fed(double now) {
    last = now;
    expired = false;
  }
  // True once, the tick the command lapses.
  bool Lapsed(double now) {
    if (timeout <= 0.0 || expired || last < 0.0) return false;
    if (now - last > timeout) {
      expired = true;
      return true;
    }
    return false;
  }
};

class DjiCanMotor final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

  // For tests: the last raw command, and the feedback it would send now.
  std::int16_t LastRawCommand() const { return last_raw_; }
  protocol::dji_motor::Feedback MakeFeedback() const;

 private:
  void OnCommand(const protocol::CanFrame& frame);

  std::string bus_name_;
  std::uint32_t feedback_id_ = 0;
  protocol::dji_motor::CommandSlot slot_;
  sim::MotorCommand::Mode mode_ = sim::MotorCommand::Mode::kCurrent;
  double command_scale_ = 0.0;  // A or V per LSB
  std::int32_t max_command_ = 32767;
  double feedback_current_scale_ = 0.0;  // A per LSB
  double reduction_ = 1.0;               // rotor turns per encoder-joint turn
  double direction_ = 1.0;               // -1: mounted reversed relative to the joint axis
  double encoder_offset_ = 0.0;          // counts reported at joint zero
  double default_temperature_ = 30.0;
  std::string motor_name_, encoder_name_;
  Every feedback_;
  CommandWatchdog watchdog_;
  double now_ = 0.0;
  std::int16_t last_raw_ = 0;
  std::uint64_t commands_ = 0, feedbacks_ = 0;

  transport::ICanBus* bus_ = nullptr;
  sim::MotorHandle motor_;
  sim::EncoderHandle encoder_;
};

class TmotorAk80 final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

 private:
  std::string bus_name_;
  std::uint8_t motor_id_ = 1;
  double reduction_ = 9.0;
  double default_temperature_ = 30.0;
  double origin_ = 0.0;  // output rad that reads as 0
  std::string motor_name_, encoder_name_;
  Every feedback_;
  CommandWatchdog watchdog_;
  double now_ = 0.0;
  std::int32_t last_ma_ = 0;
  transport::ICanBus* bus_ = nullptr;
  sim::MotorHandle motor_;
  sim::EncoderHandle encoder_;
};

class CanEncoderDevice final : public IDevice {
 public:
  explicit CanEncoderDevice(bool lamprey) : lamprey_(lamprey) {}
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;

 private:
  bool lamprey_;
  std::string bus_name_, encoder_name_;
  std::uint32_t id_ = 0x1E0;
  std::uint16_t counts_ = 4096;
  std::uint16_t gauss_ = 0;
  double reduction_ = 1.0;
  double direction_ = 1.0;
  double offset_ = 0.0;
  Every period_;
  transport::ICanBus* bus_ = nullptr;
  sim::EncoderHandle encoder_;
};

class TurretMcb final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

  // What the chassis MCB has told it.
  std::uint8_t Command() const { return command_; }
  std::optional<std::uint16_t> CalibrationSamples() const { return calibration_samples_; }
  std::optional<std::uint32_t> ChassisMicros() const { return chassis_micros_; }
  const std::vector<protocol::turret_mcb::MountingMessage>& Mountings() const { return mountings_; }
  protocol::turret_mcb::ImuState State() const { return state_; }

 private:
  void OnFrame(const protocol::CanFrame& frame, double now);

  std::string bus_name_;
  std::string gyro_name_, accel_name_, attitude_name_, kicker_name_, agitator_name_;
  Every axis_, status_, requests_, sync_;
  double temperature_ = 40.0;
  double calibration_time_ = 1.0;
  double calibrating_until_ = -1.0;
  std::uint8_t seq_ = 0;
  protocol::turret_mcb::ImuState state_ = protocol::turret_mcb::ImuState::kCalibrated;
  std::uint8_t command_ = 0;
  std::optional<std::uint16_t> calibration_samples_;
  std::optional<std::uint32_t> chassis_micros_;
  std::vector<protocol::turret_mcb::MountingMessage> mountings_;
  bool mounting_received_ = false;
  double now_ = 0.0;

  transport::ICanBus* bus_ = nullptr;
  sim::Vec3Handle gyro_, accel_;
  sim::AttitudeHandle attitude_;
  sim::SwitchHandle kicker_, agitator_;
};

class SpiImu final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  protocol::spi_imu::RegisterFile* Registers() { return registers_.get(); }

 private:
  std::string chip_, device_, gyro_name_, accel_name_, spi_bus_, cs_pin_;
  double temperature_ = 40.0;
  Every odr_;
  std::shared_ptr<protocol::spi_imu::RegisterFile> registers_;
  sim::Vec3Handle gyro_, accel_;
};

class LimitSwitchDevice final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;

 private:
  std::string pin_, switch_name_;
  bool active_low_ = false;
  transport::PinBusServer* pinbus_ = nullptr;
  sim::SwitchHandle switch_;
};

class Ld19Lidar final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;
  std::string PortPath() const;

 private:
  std::string port_spec_, lidar_name_;
  std::uint8_t intensity_ = 200;
  double scan_rate_hz_ = 0.0;  // 0: from the sim's scan period
  double min_range_ = 0.02, max_range_ = 12.0;
  // The sector it sees out of (deg, counterclockwise from its +x, from ->
  // to): beams outside it, where its own robot blocks the view, are sent as
  // no return. Without one, all of the scan.
  bool has_usable_ = false;
  double usable_from_ = 0.0, usable_to_ = 0.0;
  std::uint64_t last_sequence_ = 0;
  double last_scan_time_ = -1.0;
  std::uint64_t packets_ = 0;
  std::unique_ptr<transport::ISerialPort> port_;
  sim::LidarHandle lidar_;
};

class RealsenseD435 final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

 private:
  std::string shm_name_, camera_name_, serial_, info_path_;
  double depth_scale_ = 0.001;
  std::uint32_t slots_ = 4;
  std::uint64_t last_sequence_ = 0;
  std::uint64_t frames_ = 0;
  std::vector<std::uint16_t> depth_;
  sim::CameraFrame frame_;  // reused, so a frame's buffers are allocated once
  std::unique_ptr<transport::FrameShmWriter> shm_;
  sim::CameraHandle camera_;
};

// A UVC camera as otto's ArducamImagesetSource captures it through
// GStreamer: BGRx frames (the GStreamer shim, python/shims/gi, serves them to
// the appsink). Frames go into a frame ring; a sidecar,
// <runtime_dir>/arducam/<v4l>.json, maps the V4L path otto opens
// (/dev/v4l/by-path/<v4l>) to the ring and the stream's size.
class Arducam final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

 private:
  std::string shm_name_, camera_name_, v4l_, info_path_;
  std::uint32_t slots_ = 4;
  std::uint64_t last_sequence_ = 0, frames_ = 0;
  std::vector<std::uint8_t> bgrx_;
  sim::CameraFrame frame_;  // reused, so a frame's buffers are allocated once
  std::unique_ptr<transport::FrameShmWriter> shm_;
  sim::CameraHandle camera_;
};

class VisionLink final : public IDevice {
 public:
  struct Message {
    double time = 0.0;
    bool from_mcb = true;
    protocol::DjiFrame frame;
  };
  using Listener = std::function<void(const Message&)>;

  void Configure(const DeviceSpec& spec) override;
  void Bind(sim::ISim& sim, Transports& transports) override;
  void Tick(const SimTime& now) override;
  std::string Status() const override;

  void AddListener(Listener listener) { listeners_.push_back(std::move(listener)); }
  std::string McbPath() const;
  std::string OttoPath() const;

 private:
  void Relay(transport::ISerialPort& from, transport::ISerialPort& to, protocol::DjiFrameParser& parser,
             bool from_mcb, double now);

  std::string mcb_spec_, otto_spec_, log_path_;
  std::unique_ptr<transport::ISerialPort> mcb_, otto_;
  protocol::DjiFrameParser from_mcb_, from_otto_;
  std::ofstream log_;
  std::vector<Listener> listeners_;
  std::uint64_t mcb_bytes_ = 0, otto_bytes_ = 0;
};

}  // namespace hwemu::devices
