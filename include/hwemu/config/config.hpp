#pragma once

// The hub's configuration, from two files (schemas/ checks both):
//
//   sim.yaml      which backend and display, the models, the clock, which
//                 sensor windows to open -- see examples/turret_testbed/sim.yaml
//   devices.yaml  the devices (devices/device.hpp) and any preset files
//
// Paths in either are relative to the file they're in. The robot's own
// copies of these live in the otto and mcb repos; this repo ships only the
// schemas, the presets (presets/) and an example.

#include <optional>
#include <string>
#include <vector>

#include "hwemu/clock/sim_clock.hpp"
#include "hwemu/devices/device.hpp"
#include "hwemu/display/display.hpp"
#include "hwemu/sim/sim.hpp"

namespace hwemu::config {

struct WindowSpec {
  std::string sensor;
  display::ViewKind view = display::ViewKind::kImage;
  std::string title;
};

struct HubConfig {
  sim::SimOptions sim;
  clock::ClockOptions clock;
  std::string runtime_dir = "/tmp/hwemu";
  std::vector<devices::DeviceSpec> devices;
  std::vector<WindowSpec> windows;
  std::optional<double> duration;  // sim seconds; run until stopped if unset
  double status_period = 0.0;      // wall seconds between status lines; 0: none
};

// Throws std::invalid_argument (or YAML::Exception) naming the file and field.
HubConfig LoadHubConfig(const std::string& sim_yaml, const std::string& devices_yaml);

// Just the devices (and their presets), into `config`.
void LoadDevices(const std::string& devices_yaml, HubConfig* config);

// The directory of built-in presets (presets/ in the source tree, or
// $HWEMU_PRESET_DIR).
std::string DefaultPresetDir();

}  // namespace hwemu::config
