#pragma once

// The hub: the sim, the devices, the links and the clock in one loop.
//
//   per tick:  wait for the clock -> dispatch received CAN frames
//              -> every device's Tick -> the display's writes -> sim.Step(dt)
//              -> advance the clock (lockstep: hand participants the tick)
//   per frame: the display's Draw, at most ~60 times a wall second

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hwemu/clock/sim_clock.hpp"
#include "hwemu/config/config.hpp"
#include "hwemu/devices/device.hpp"
#include "hwemu/sim/sim.hpp"

namespace hwemu::hub {

class Hub {
 public:
  // Makes the sim from config.sim unless one is given (a FakeSim, in tests).
  explicit Hub(config::HubConfig config, std::unique_ptr<sim::ISim> sim = nullptr);
  ~Hub();

  // Binds every device and opens the configured windows. Throws on any
  // device that can't bind.
  void Start();
  // One tick; false once it's time to stop (duration reached, display
  // closed, or Stop()).
  bool Tick();
  // Ticks until Tick() says stop or *stop turns true.
  void Run(const std::atomic<bool>* stop = nullptr);
  void Stop();

  sim::ISim& Sim() { return *sim_; }
  clock::SimClock& Clock() { return *clock_; }
  devices::Transports& GetTransports() { return *transports_; }
  devices::IDevice* Device(const std::string& name);
  const std::vector<std::unique_ptr<devices::IDevice>>& Devices() const { return devices_; }
  std::string StatusReport() const;

 private:
  void LoadPlugin(const std::string& path);
  config::HubConfig config_;
  std::unique_ptr<sim::ISim> sim_;
  std::unique_ptr<clock::SimClock> clock_;
  std::unique_ptr<devices::Transports> transports_;
  std::vector<std::unique_ptr<devices::IDevice>> devices_;
  std::atomic<bool> stop_{false};
  bool started_ = false;
  // The truth log (TruthLogSpec): its file, body and next sim time.
  std::unique_ptr<std::ofstream> truth_;
  std::string truth_body_;
  double next_truth_ = 0.0;
  void LogTruth(double time);
  std::chrono::steady_clock::time_point last_draw_, last_status_;
};

}  // namespace hwemu::hub
