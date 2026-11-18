#pragma once

// A device emulator: a real part (a motor controller, an IMU, a lidar) as the
// firmware or the Jetson sees it on the wire, backed by a sim sensor or
// actuator kind. Each one is a pure unit conversion between its wire format
// (protocol/) and its sim kinds (sim/sim.hpp), plus the quantization the wire
// format itself imposes -- never any error of its own: the sims model that.
//
// Devices are made by type from devices.yaml (config/config.hpp):
//
//   - name: yaw_motor
//     type: dji_can_motor
//     preset: gm6020_voltage          # a params block from a presets file
//     transport: {bus: vcan0, id: 0x205}
//     bind: {motor: yaw_motor, encoder: yaw_encoder}
//     params: {reduction: 1.0}
//
// Configure() reads transport/bind/params and throws on anything missing or
// malformed; Bind() looks up the sim kinds and opens the transports; Tick()
// runs once per hub tick, after received frames have been dispatched.

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "hwemu/clock/sim_clock.hpp"
#include "hwemu/sim/sim.hpp"
#include "hwemu/transport/can_bus.hpp"
#include "hwemu/transport/frame_shm.hpp"
#include "hwemu/transport/pinbus.hpp"
#include "hwemu/transport/serial_port.hpp"

namespace hwemu::devices {

using clock::SimTime;

struct DeviceSpec {
  std::string name;
  std::string type;
  YAML::Node transport;
  YAML::Node bind;
  YAML::Node params;  // the preset's, with the device's own on top
};

// The hub's transports, handed to every device's Bind().
class Transports {
 public:
  explicit Transports(std::string runtime_dir);
  ~Transports();

  transport::ICanBus* Can(const std::string& bus) { return can_.Open(bus); }
  transport::CanBusSet& CanBuses() { return can_; }
  transport::CanRouter& Router() { return router_; }
  // Created at <runtime_dir>/pinbus.sock on first use.
  transport::PinBusServer& PinBus();
  bool HasPinBus() const { return pinbus_ != nullptr; }
  // A relative path or bare name lands under the runtime directory.
  std::string Resolve(const std::string& name) const;
  std::unique_ptr<transport::ISerialPort> OpenSerial(const std::string& spec, int baud);
  const std::string& RuntimeDir() const { return runtime_dir_; }

 private:
  std::string runtime_dir_;
  transport::CanBusSet can_;
  transport::CanRouter router_;
  std::unique_ptr<transport::PinBusServer> pinbus_;
};

class IDevice {
 public:
  virtual ~IDevice() = default;
  virtual void Configure(const DeviceSpec& spec) = 0;
  virtual void Bind(sim::ISim& sim, Transports& transports) = 0;
  virtual void Tick(const SimTime& now) = 0;
  // One line of state for the hub's status output.
  virtual std::string Status() const { return ""; }

  const std::string& Name() const { return name_; }
  const std::string& Type() const { return type_; }

 protected:
  void SetIdentity(const DeviceSpec& spec) {
    name_ = spec.name;
    type_ = spec.type;
  }

 private:
  std::string name_, type_;
};

using DeviceFactory = std::function<std::unique_ptr<IDevice>()>;

class DeviceRegistry {
 public:
  static DeviceRegistry& Instance();
  void Register(const std::string& type, DeviceFactory factory);
  // Throws std::invalid_argument for an unknown type.
  std::unique_ptr<IDevice> Make(const std::string& type) const;
  std::vector<std::string> Types() const;

 private:
  DeviceRegistry();
  std::map<std::string, DeviceFactory> factories_;
};

// Reads a YAML field, with the device and field named in any error.
class Params {
 public:
  Params(const YAML::Node& node, std::string context) : node_(node), context_(std::move(context)) {}
  bool Has(const std::string& key) const { return node_ && node_[key] && !node_[key].IsNull(); }
  double Number(const std::string& key) const;
  double Number(const std::string& key, double fallback) const;
  // Integers accept 0x.. hex.
  std::int64_t Integer(const std::string& key) const;
  std::int64_t Integer(const std::string& key, std::int64_t fallback) const;
  bool Flag(const std::string& key, bool fallback) const;
  std::string Text(const std::string& key) const;
  std::string Text(const std::string& key, const std::string& fallback) const;
  YAML::Node Node(const std::string& key) const { return node_ ? node_[key] : YAML::Node(); }

 private:
  [[noreturn]] void Fail(const std::string& key, const std::string& what) const;
  YAML::Node node_;
  std::string context_;
};

// Fires every `period` seconds of sim time (0 or less: every tick).
class Every {
 public:
  explicit Every(double period = 0.0) : period_(period) {}
  bool Due(const SimTime& now) {
    if (period_ <= 0.0) {
      return true;
    }
    if (started_ && now.seconds + 1e-9 < next_) {
      return false;
    }
    // On schedule, without drifting -- but never bursting to catch up.
    next_ = started_ ? next_ + period_ : now.seconds + period_;
    if (next_ <= now.seconds) {
      next_ = now.seconds + period_;
    }
    started_ = true;
    return true;
  }
  void SetRate(double hz) { period_ = hz > 0.0 ? 1.0 / hz : 0.0; }

 private:
  double period_;
  double next_ = 0.0;
  bool started_ = false;
};

void RegisterBuiltinDevices(DeviceRegistry& registry);

}  // namespace hwemu::devices
