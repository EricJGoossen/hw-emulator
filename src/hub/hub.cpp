#include "hwemu/hub/hub.hpp"

#include <iostream>
#include <sstream>

namespace hwemu::hub {

Hub::Hub(config::HubConfig config, std::unique_ptr<sim::ISim> sim)
    : config_(std::move(config)), sim_(std::move(sim)) {
  if (!sim_) {
    sim_ = sim::CreateSim(config_.sim);
  }
  clock_ = std::make_unique<clock::SimClock>(config_.clock);
  transports_ = std::make_unique<devices::Transports>(config_.runtime_dir);
  for (const auto& spec : config_.devices) {
    auto device = devices::DeviceRegistry::Instance().Make(spec.type);
    device->Configure(spec);
    devices_.push_back(std::move(device));
  }
}

Hub::~Hub() = default;

void Hub::Start() {
  for (auto& d : devices_) {
    try {
      d->Bind(*sim_, *transports_);
    } catch (const std::exception& e) {
      throw std::runtime_error(d->Type() + " '" + d->Name() + "': " + e.what());
    }
  }
  if (auto* display = sim_->Display()) {
    display->Open();
    if (auto* windows = display->SensorWindows()) {
      for (const auto& w : config_.windows) {
        windows->Show(w.sensor, w.view, w.title);
      }
    } else if (!config_.windows.empty()) {
      std::cerr << "[hub] this backend's display has no per-sensor windows; ignoring `windows`\n";
    }
  }
  last_draw_ = last_status_ = std::chrono::steady_clock::now();
  started_ = true;
}

bool Hub::Tick() {
  if (!started_) Start();
  if (stop_ || !clock_->WaitForNextTick()) return false;
  const clock::SimTime now = clock_->Now();
  transports_->Router().Poll();
  for (auto& d : devices_) d->Tick(now);
  auto* display = sim_->Display();
  if (display != nullptr) display->ApplyWrites();
  if (display == nullptr || !display->Paused()) {
    sim_->Step(config_.clock.dt);
  }
  clock_->Advance();

  const auto wall = std::chrono::steady_clock::now();
  if (display != nullptr && wall - last_draw_ >= std::chrono::milliseconds(16)) {
    last_draw_ = wall;
    if (!display->Draw()) {
      std::cerr << "[hub] display closed\n";
      return false;
    }
  }
  if (config_.status_period > 0.0 &&
      wall - last_status_ >= std::chrono::duration<double>(config_.status_period)) {
    last_status_ = wall;
    std::cerr << StatusReport();
  }
  return !(config_.duration && clock_->Now().seconds >= *config_.duration - 1e-9);
}

void Hub::Run(const std::atomic<bool>* stop) {
  while ((stop == nullptr || !*stop) && Tick()) {
  }
}

void Hub::Stop() {
  stop_ = true;
  clock_->Stop();
}

devices::IDevice* Hub::Device(const std::string& name) {
  for (auto& d : devices_) {
    if (d->Name() == name) return d.get();
  }
  return nullptr;
}

std::string Hub::StatusReport() const {
  std::ostringstream s;
  s << "[hub] t=" << clock_->Now().seconds << "s tick=" << clock_->Now().tick << "\n";
  for (const auto& d : devices_) {
    const std::string status = d->Status();
    if (!status.empty()) s << "  " << d->Name() << " (" << d->Type() << "): " << status << "\n";
  }
  return s.str();
}

}  // namespace hwemu::hub
