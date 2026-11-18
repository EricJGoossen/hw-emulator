#pragma once

// One sim interface over Achilles and MuJoCo (and a fake for tests): a
// backend is chosen by SimOptions::backend, and everything above this line --
// the devices, the hub -- sees only backend-neutral sensor and actuator kinds.
// There are no joint or body handles: motors are commanded as a motor driver
// would be (current or voltage), and positions come back through sensors.
//
// Every value here is what the sim's sensor reports, its error included: the
// backends model all error, and nothing above them adds any.

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "hwemu/display/display.hpp"

namespace YAML {
class Node;
}

namespace hwemu::sim {

enum class Backend { kAchilles, kMujoco, kFake };
enum class DisplayMode { kHeadless, kWindow };

Backend ParseBackend(std::string_view name);  // throws std::invalid_argument
std::string_view Name(Backend backend);

struct SimOptions {
  Backend backend = Backend::kAchilles;
  DisplayMode display = DisplayMode::kHeadless;
  std::string urdf;        // MuJoCo's model (URDF, or MJCF)
  std::string arow;        // Achilles's model
  std::string sim_config;  // Achilles's sim_config.yaml (gravity, ...)
  std::optional<std::uint64_t> seed;
  // MuJoCo: the actuators and sensors to create on the URDF, by name -- what
  // each devices.yaml binding names (see docs/mujoco_elements.md). A YAML map
  // name -> element spec. Unused by Achilles, whose .arow declares its own.
  std::shared_ptr<const YAML::Node> mujoco_elements;
  // Achilles: the integrator, by name (semi_implicit_euler, rk4, ...).
  std::string integrator;
};

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};

struct MotorCommand {
  enum class Mode { kCurrent, kVoltage };
  Mode mode = Mode::kCurrent;
  double value = 0.0;  // A, or V
  static MotorCommand Current(double amps) { return {Mode::kCurrent, amps}; }
  static MotorCommand Voltage(double volts) { return {Mode::kVoltage, volts}; }
};

struct MotorReading {
  double current = 0.0;  // A, in the winding
  double torque = 0.0;   // N*m at the joint
  std::optional<double> temperature;  // degC, when the backend models it
};

struct EncoderReading {
  double position = 0.0;  // rad (multi-turn) or m
  double velocity = 0.0;  // rad/s or m/s
};

struct AttitudeReading {
  double roll = 0, pitch = 0, yaw = 0;  // rad, ZYX
  Quat orientation;                     // world from sensor
};

struct Intrinsics {
  std::uint32_t width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
};

struct CameraFrame {
  std::uint32_t width = 0, height = 0;
  std::vector<std::uint8_t> rgb;  // rgb8, rows top to bottom
  std::vector<float> depth;       // m along the view axis; inf for no hit
  double capture_time = 0.0;      // sim time it was rendered at
  std::uint64_t sequence = 0;     // counts new images; 0 before the first
};

struct LidarScan {
  std::vector<float> ranges;  // m; inf for no return
  std::vector<float> angles;  // rad, counterclockwise from the sensor's +x
  float min_range = 0, max_range = 0;
  double capture_time = 0.0;
  std::uint64_t sequence = 0;  // counts new scans
};

class IMotor {
 public:
  virtual ~IMotor() = default;
  virtual void Command(const MotorCommand& command) = 0;
  virtual MotorReading Read() const = 0;
};
class IEncoder {
 public:
  virtual ~IEncoder() = default;
  virtual EncoderReading Read() const = 0;
};
class IVec3Sensor {
 public:
  virtual ~IVec3Sensor() = default;
  virtual Vec3 Read() const = 0;
};
class IAttitude {
 public:
  virtual ~IAttitude() = default;
  virtual AttitudeReading Read() const = 0;
};
class ICamera {
 public:
  virtual ~ICamera() = default;
  virtual Intrinsics GetIntrinsics() const = 0;
  // The latest image's sequence number, without copying it.
  virtual std::uint64_t Sequence() const = 0;
  // Copies the latest image into *out; false before the first.
  virtual bool Read(CameraFrame* out) const = 0;
};
class ILidar {
 public:
  virtual ~ILidar() = default;
  virtual std::uint64_t Sequence() const = 0;
  virtual bool Read(LidarScan* out) const = 0;
};
class ISwitch {
 public:
  virtual ~ISwitch() = default;
  virtual bool Read() const = 0;
};

using MotorHandle = std::shared_ptr<IMotor>;
using EncoderHandle = std::shared_ptr<IEncoder>;
using Vec3Handle = std::shared_ptr<IVec3Sensor>;
using AttitudeHandle = std::shared_ptr<IAttitude>;
using CameraHandle = std::shared_ptr<ICamera>;
using LidarHandle = std::shared_ptr<ILidar>;
using SwitchHandle = std::shared_ptr<ISwitch>;

// What a backend can be asked for by name, for listing and error messages.
struct SensorInfo {
  std::string name;
  std::string kind;  // "motor", "encoder", "gyro", "accel", "mag", "attitude",
                     // "camera", "lidar", "switch"
};

// Thrown by a lookup whose name the backend doesn't have, or has as another
// kind.
class UnknownSensor : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

class ISim {
 public:
  virtual ~ISim() = default;

  virtual Backend GetBackend() const = 0;
  // Advances the sim by dt seconds (one or more of its own steps).
  virtual void Step(double dt) = 0;
  // Sim time, s, since the start.
  virtual double Time() const = 0;

  virtual MotorHandle Motor(std::string_view name) = 0;
  virtual EncoderHandle Encoder(std::string_view name) = 0;
  virtual Vec3Handle Gyro(std::string_view name) = 0;
  virtual Vec3Handle Accel(std::string_view name) = 0;
  virtual Vec3Handle Mag(std::string_view name) = 0;
  virtual AttitudeHandle Attitude(std::string_view name) = 0;
  virtual CameraHandle Camera(std::string_view name) = 0;
  virtual LidarHandle Lidar(std::string_view name) = 0;
  virtual SwitchHandle Switch(std::string_view name) = 0;

  virtual std::vector<SensorInfo> List() const = 0;

  // Null when headless.
  virtual display::IDisplay* Display() = 0;
};

std::unique_ptr<ISim> CreateSim(const SimOptions& options);

}  // namespace hwemu::sim
