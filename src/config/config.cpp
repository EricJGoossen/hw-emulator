#include "hwemu/config/config.hpp"

#include <cstdlib>
#include <filesystem>
#include <map>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace hwemu::config {
namespace fs = std::filesystem;

#ifndef HWEMU_SOURCE_PRESET_DIR
#define HWEMU_SOURCE_PRESET_DIR ""
#endif

std::string DefaultPresetDir() {
  if (const char* env = std::getenv("HWEMU_PRESET_DIR"); env != nullptr && *env != '\0') {
    return env;
  }
  return HWEMU_SOURCE_PRESET_DIR;
}

namespace {

std::string RelativeTo(const std::string& file, const std::string& path) {
  if (path.empty() || fs::path(path).is_absolute()) return path;
  return (fs::path(file).parent_path() / path).lexically_normal().string();
}

YAML::Node LoadFile(const std::string& path) {
  if (!fs::exists(path)) {
    throw std::invalid_argument("no such file: " + path);
  }
  return YAML::LoadFile(path);
}

// a with b's keys on top, recursively for maps.
YAML::Node Merge(const YAML::Node& a, const YAML::Node& b) {
  if (!b || b.IsNull()) return YAML::Clone(a);
  if (!a || a.IsNull() || !a.IsMap() || !b.IsMap()) return YAML::Clone(b);
  YAML::Node out = YAML::Clone(a);
  for (const auto& kv : b) {
    const auto key = kv.first.as<std::string>();
    out[key] = Merge(a[key], kv.second);
  }
  return out;
}

void LoadPresetFile(const std::string& path, std::map<std::string, YAML::Node>* presets) {
  const YAML::Node root = LoadFile(path);
  if (!root.IsMap()) throw std::invalid_argument(path + ": a presets file is a map of name -> params");
  for (const auto& kv : root) {
    (*presets)[kv.first.as<std::string>()] = kv.second;
  }
}

}  // namespace

void LoadDevices(const std::string& devices_yaml, HubConfig* config) {
  const YAML::Node root = LoadFile(devices_yaml);
  std::map<std::string, YAML::Node> presets;
  const std::string builtin = DefaultPresetDir();
  if (!builtin.empty() && fs::is_directory(builtin)) {
    for (const auto& entry : fs::directory_iterator(builtin)) {
      if (entry.path().extension() == ".yaml") LoadPresetFile(entry.path().string(), &presets);
    }
  }
  if (const auto files = root["presets"]) {
    for (const auto& f : files) LoadPresetFile(RelativeTo(devices_yaml, f.as<std::string>()), &presets);
  }
  if (const auto plugins = root["plugins"]) {
    for (const auto& f : plugins) config->plugins.push_back(RelativeTo(devices_yaml, f.as<std::string>()));
  }

  YAML::Node elements = config->sim.mujoco_elements ? YAML::Clone(*config->sim.mujoco_elements) : YAML::Node(YAML::NodeType::Map);
  std::map<std::string, bool> names;
  for (const auto& d : root["devices"]) {
    devices::DeviceSpec spec;
    spec.name = d["name"] ? d["name"].as<std::string>() : "";
    spec.type = d["type"] ? d["type"].as<std::string>() : "";
    if (spec.name.empty() || spec.type.empty()) {
      throw std::invalid_argument(devices_yaml + ": every device needs a name and a type");
    }
    if (names[spec.name]) throw std::invalid_argument(devices_yaml + ": two devices named '" + spec.name + "'");
    names[spec.name] = true;
    spec.transport = d["transport"] ? d["transport"] : YAML::Node(YAML::NodeType::Map);
    spec.bind = d["bind"] ? d["bind"] : YAML::Node(YAML::NodeType::Map);
    YAML::Node params = d["params"] ? d["params"] : YAML::Node(YAML::NodeType::Map);
    if (d["preset"]) {
      const auto preset = d["preset"].as<std::string>();
      auto it = presets.find(preset);
      if (it == presets.end()) {
        throw std::invalid_argument(devices_yaml + ": device '" + spec.name + "': no preset '" + preset + "'");
      }
      params = Merge(it->second, params);
    }
    spec.params = params;
    // MuJoCo elements this device's bindings name, collected for the backend.
    if (spec.bind["mujoco"]) {
      for (const auto& kv : spec.bind["mujoco"]) elements[kv.first.as<std::string>()] = kv.second;
    }
    config->devices.push_back(spec);
  }
  config->sim.mujoco_elements = std::make_shared<YAML::Node>(elements);
}

HubConfig LoadHubConfig(const std::string& sim_yaml, const std::string& devices_yaml) {
  HubConfig config;
  const YAML::Node root = LoadFile(sim_yaml);
  if (root["backend"]) config.sim.backend = sim::ParseBackend(root["backend"].as<std::string>());
  if (root["display"]) {
    const auto mode = root["display"].as<std::string>();
    if (mode == "window") {
      config.sim.display = sim::DisplayMode::kWindow;
    } else if (mode == "headless") {
      config.sim.display = sim::DisplayMode::kHeadless;
    } else {
      throw std::invalid_argument(sim_yaml + ": display must be headless or window");
    }
  }
  if (root["seed"]) config.sim.seed = root["seed"].as<std::uint64_t>();
  if (const auto models = root["models"]) {
    if (models["urdf"]) config.sim.urdf = RelativeTo(sim_yaml, models["urdf"].as<std::string>());
    if (models["arow"]) config.sim.arow = RelativeTo(sim_yaml, models["arow"].as<std::string>());
    if (models["achilles_config"]) {
      config.sim.sim_config = RelativeTo(sim_yaml, models["achilles_config"].as<std::string>());
    }
  }
  if (const auto achilles = root["achilles"]) {
    if (achilles["integrator"]) config.sim.integrator = achilles["integrator"].as<std::string>();
  }
  if (const auto mujoco = root["mujoco"]) {
    if (mujoco["elements"]) {
      // An attach element's file, relative to this file.
      YAML::Node elements = YAML::Clone(mujoco["elements"]);
      for (auto kv : elements) {
        if (kv.second["type"] && kv.second["type"].as<std::string>() == "attach" && kv.second["file"]) {
          kv.second["file"] = RelativeTo(sim_yaml, kv.second["file"].as<std::string>());
        }
      }
      config.sim.mujoco_elements = std::make_shared<YAML::Node>(elements);
    }
  }
  if (root["runtime_dir"]) config.runtime_dir = root["runtime_dir"].as<std::string>();
  config.clock.shm_name = "hwemu_clock";
  if (const auto c = root["clock"]) {
    if (c["mode"]) config.clock.mode = clock::ParseClockMode(c["mode"].as<std::string>());
    if (c["dt"]) config.clock.dt = c["dt"].as<double>();
    if (c["rate"]) config.clock.rate = c["rate"].as<double>();
    if (c["order"]) config.clock.order = clock::ParseLockstepOrder(c["order"].as<std::string>());
    if (c["expected"]) config.clock.expected = c["expected"].as<std::vector<std::string>>();
    if (c["ack_timeout"]) config.clock.ack_timeout = c["ack_timeout"].as<double>();
    if (c["shm"]) config.clock.shm_name = c["shm"].as<std::string>();
    if (c["socket"]) config.clock.lockstep_socket = c["socket"].as<std::string>();
  }
  if (config.clock.mode == clock::ClockMode::kLockstep && config.clock.lockstep_socket.empty()) {
    config.clock.lockstep_socket = (fs::path(config.runtime_dir) / "clock.sock").string();
  }
  if (root["duration"]) config.duration = root["duration"].as<double>();
  if (root["status_period"]) config.status_period = root["status_period"].as<double>();
  if (root["draw_period"]) config.draw_period = root["draw_period"].as<double>();
  if (const auto t = root["truth_log"]) {
    config.truth_log.period = t["period"] ? t["period"].as<double>() : 0.01;
    if (const auto b = t["body"]) {
      if (b["achilles"]) config.truth_log.achilles = b["achilles"].as<std::string>();
      if (b["mujoco"]) config.truth_log.mujoco = b["mujoco"].as<std::string>();
    }
    for (const auto& e : t["encoders"]) config.truth_log.encoders.push_back(e.as<std::string>());
    for (const auto& g : t["gyros"]) config.truth_log.gyros.push_back(g.as<std::string>());
    for (const auto& a : t["accels"]) config.truth_log.accels.push_back(a.as<std::string>());
  }
  for (const auto& a : root["actors"]) {
    ActorSpec spec;
    spec.name = a["name"].as<std::string>();
    if (const auto b = a["body"]) {
      if (b["achilles"]) spec.achilles = b["achilles"].as<std::string>();
      if (b["mujoco"]) spec.mujoco = b["mujoco"].as<std::string>();
    }
    for (const auto& p : a["path"]) {
      const auto xy = p.as<std::vector<double>>();
      if (xy.size() != 2) throw std::invalid_argument(sim_yaml + ": actor '" + spec.name + "': each path point is [x, y]");
      spec.path.push_back({xy[0], xy[1]});
    }
    if (a["z"]) spec.z = a["z"].as<double>();
    if (a["speed"]) spec.speed = a["speed"].as<double>();
    if (a["spin"]) spec.spin = a["spin"].as<double>();
    if (a["turn"]) spec.turn = a["turn"].as<double>();
    if (a["start"]) spec.start = a["start"].as<double>();
    config.actors.push_back(spec);
  }
  for (const auto& w : root["windows"]) {
    WindowSpec spec;
    spec.sensor = w["sensor"].as<std::string>();
    spec.view = w["view"] ? display::ParseViewKind(w["view"].as<std::string>()) : display::ViewKind::kImage;
    spec.title = w["title"] ? w["title"].as<std::string>() : spec.sensor;
    spec.fly = w["fly"] && w["fly"].as<bool>();
    config.windows.push_back(spec);
  }
  if (!devices_yaml.empty()) {
    LoadDevices(devices_yaml, &config);
  }
  return config;
}

}  // namespace hwemu::config
