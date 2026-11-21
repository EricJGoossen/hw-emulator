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

#include <array>
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
  // A camera's window that steers the camera by its keys and mouse (see
  // ISensorWindows::Show). Only for a camera on a mount of its own.
  bool fly = false;
};

// The hub's ground-truth log: a body's world pose every `period` of sim
// time, to <runtime dir>/truth.jsonl, for checking a run against what the
// firmware and otto believe. The body is named per backend (an Achilles
// joint, a MuJoCo body). Off while period is 0.
struct TruthLogSpec {
  double period = 0.0;  // sim seconds
  std::string achilles;
  std::string mujoco;
  // Sim encoders (devices.yaml bind names) whose positions each row also
  // carries, e.g. a turret joint's angle off the logged body.
  std::vector<std::string> encoders;
  // Sim gyros and accelerometers whose readings (rad/s, m/s^2, in the
  // sensor's frame, noise included) each row also carries: what a
  // firmware's attitude filter was given, to check it offline.
  std::vector<std::string> gyros;
  std::vector<std::string> accels;
};

// A scripted body the hub moves (hub/actor.hpp): around a closed loop of
// floor waypoints at a constant speed. The body is named per backend as
// ISim::SetBodyPose takes it: a MuJoCo mocap body (an attach element's), an
// Achilles welded root joint (archetype/joint).
struct ActorSpec {
  std::string name;
  std::string achilles;
  std::string mujoco;
  std::vector<std::array<double, 2>> path;  // m, world x y; the last leg returns to the first point
  double z = 0.0;                           // m, the body's height
  double speed = 1.0;                       // m/s along the path
  double spin = 0.0;                        // rad/s; 0: it faces along the path
  double turn = 0.3;                        // m: the heading eases across each waypoint over this
  double start = 0.0;                       // s of sim time it waits at path[0]
};

struct HubConfig {
  sim::SimOptions sim;
  clock::ClockOptions clock;
  std::string runtime_dir = "/tmp/hwemu";
  std::vector<devices::DeviceSpec> devices;
  // Shared libraries that register more device types (devices.yaml
  // `plugins:`), loaded before the devices are made. Each exports
  //   extern "C" void hwemu_register_devices(hwemu::devices::DeviceRegistry*);
  std::vector<std::string> plugins;
  std::vector<WindowSpec> windows;
  std::vector<ActorSpec> actors;
  std::optional<double> duration;  // sim seconds; run until stopped if unset
  double status_period = 0.0;      // wall seconds between status lines; 0: none
  // Wall seconds between window draws. A draw runs on the hub's thread, in
  // the tick loop, so drawing less often leaves more of each second to the sim.
  double draw_period = 1.0 / 60.0;
  TruthLogSpec truth_log;
};

// Throws std::invalid_argument (or YAML::Exception) naming the file and field.
HubConfig LoadHubConfig(const std::string& sim_yaml, const std::string& devices_yaml);

// Just the devices (and their presets), into `config`.
void LoadDevices(const std::string& devices_yaml, HubConfig* config);

// The directory of built-in presets (presets/ in the source tree, or
// $HWEMU_PRESET_DIR).
std::string DefaultPresetDir();

}  // namespace hwemu::config
