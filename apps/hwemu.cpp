// hwemu: the hub process. Runs the sim, the devices, the links and the clock.
//
//   hwemu --sim sim.yaml --devices devices.yaml [--backend achilles|mujoco|fake]
//         [--headless | --window] [--seconds N] [--lockstep] [--status S]
//         [--runtime-dir DIR] [--list-devices]
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "hwemu/config/config.hpp"
#include "hwemu/devices/builtin.hpp"
#include "hwemu/gaps.hpp"
#include "hwemu/hub/hub.hpp"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int /*signal*/) { g_stop = true; }

void Usage() {
  std::cerr << "usage: hwemu --sim sim.yaml --devices devices.yaml [--backend achilles|mujoco|fake]\n"
               "             [--headless|--window] [--seconds N] [--lockstep] [--status SECONDS]\n"
               "             [--runtime-dir DIR] [--list-devices]\n"
               "             lockstep: [--expect NAME,NAME] [--order broadcast|sequential]\n"
               "                       [--ack-timeout SECONDS (0: wait forever)] [--clock-shm NAME]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string sim_yaml, devices_yaml, backend, runtime_dir;
  std::optional<double> seconds, status, ack_timeout;
  std::optional<std::string> expect, order, clock_shm;
  bool headless = false, window = false, lockstep = false, list = false;
  const std::vector<std::string> args(argv + 1, argv + argc);
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= args.size()) {
        std::cerr << a << " needs a value\n";
        std::exit(2);
      }
      return args[++i];
    };
    if (a == "--sim") sim_yaml = value();
    else if (a == "--devices") devices_yaml = value();
    else if (a == "--backend") backend = value();
    else if (a == "--headless") headless = true;
    else if (a == "--window") window = true;
    else if (a == "--seconds") seconds = std::stod(value());
    else if (a == "--status") status = std::stod(value());
    else if (a == "--lockstep") lockstep = true;
    else if (a == "--runtime-dir") runtime_dir = value();
    else if (a == "--list-devices") list = true;
    else if (a == "--expect") expect = value();
    else if (a == "--order") order = value();
    else if (a == "--ack-timeout") ack_timeout = std::stod(value());
    else if (a == "--clock-shm") clock_shm = value();
    else if (a == "-h" || a == "--help") {
      Usage();
      return 0;
    } else {
      std::cerr << "unknown argument " << a << "\n";
      Usage();
      return 2;
    }
  }
  if (list) {
    for (const auto& t : hwemu::devices::DeviceRegistry::Instance().Types()) std::cout << t << "\n";
    return 0;
  }
  if (sim_yaml.empty()) {
    Usage();
    return 2;
  }

  try {
    hwemu::config::HubConfig config = hwemu::config::LoadHubConfig(sim_yaml, devices_yaml);
    if (!backend.empty()) config.sim.backend = hwemu::sim::ParseBackend(backend);
    if (headless) config.sim.display = hwemu::sim::DisplayMode::kHeadless;
    if (window) config.sim.display = hwemu::sim::DisplayMode::kWindow;
    if (seconds) config.duration = *seconds;
    if (status) config.status_period = *status;
    if (!runtime_dir.empty()) config.runtime_dir = runtime_dir;
    if (lockstep) {
      config.clock.mode = hwemu::clock::ClockMode::kLockstep;
      if (config.clock.lockstep_socket.empty()) config.clock.lockstep_socket = config.runtime_dir + "/clock.sock";
    }
    if (expect) {
      config.clock.expected.clear();
      std::stringstream names(*expect);
      for (std::string n; std::getline(names, n, ',');) {
        if (!n.empty()) config.clock.expected.push_back(n);
      }
    }
    if (order) config.clock.order = hwemu::clock::ParseLockstepOrder(*order);
    if (ack_timeout) config.clock.ack_timeout = *ack_timeout;
    if (clock_shm) config.clock.shm_name = *clock_shm;

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    hwemu::hub::Hub hub(config);
    hub.Start();
    std::cerr << "[hwemu] backend " << hwemu::sim::Name(config.sim.backend) << ", "
              << (config.sim.display == hwemu::sim::DisplayMode::kWindow ? "window" : "headless") << ", "
              << hub.Devices().size() << " devices, runtime dir " << config.runtime_dir << "\n";
    for (const auto& d : hub.Devices()) {
      if (auto* link = dynamic_cast<hwemu::devices::VisionLink*>(d.get())) {
        std::cerr << "[hwemu] " << d->Name() << ": MCB opens " << link->McbPath() << ", otto opens "
                  << link->OttoPath() << "\n";
      } else if (auto* lidar = dynamic_cast<hwemu::devices::Ld19Lidar*>(d.get())) {
        std::cerr << "[hwemu] " << d->Name() << ": otto opens " << lidar->PortPath() << "\n";
      }
    }
    if (hub.GetTransports().HasPinBus()) {
      std::cerr << "[hwemu] pinbus at " << hub.GetTransports().PinBus().Path() << "\n";
    }
    hub.Run(&g_stop);
    std::cerr << hub.StatusReport();
    const auto gaps = hwemu::gaps::All();
    std::cerr << "[hwemu] " << gaps.size() << " sim gap(s) this run (" << config.runtime_dir << "/gaps/hub.jsonl)\n";
    for (const auto& g : gaps) std::cerr << "  - " << g.source << ": " << g.message << "\n";
  } catch (const std::exception& e) {
    std::cerr << "hwemu: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
