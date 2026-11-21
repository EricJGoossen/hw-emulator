#include "hwemu/hub/hub.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

#include <thread>

#include <dlfcn.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "hwemu/gaps.hpp"

namespace hwemu::hub {

Hub::Hub(config::HubConfig config, std::unique_ptr<sim::ISim> sim)
    : config_(std::move(config)), sim_(std::move(sim)) {
  if (!sim_) {
    sim_ = sim::CreateSim(config_.sim);
  }
  clock_ = std::make_unique<clock::SimClock>(config_.clock);
  transports_ = std::make_unique<devices::Transports>(config_.runtime_dir);
  gaps::Configure(config_.runtime_dir, "hub");
  for (const auto& plugin : config_.plugins) LoadPlugin(plugin);
  for (const auto& spec : config_.devices) {
    auto device = devices::DeviceRegistry::Instance().Make(spec.type);
    device->Configure(spec);
    devices_.push_back(std::move(device));
  }
}

Hub::~Hub() {
  std::error_code ignored;
  std::filesystem::remove(std::filesystem::path(config_.runtime_dir) / "ready", ignored);
}

void Hub::LoadPlugin(const std::string& path) {
  // RTLD_GLOBAL: a plugin's devices may share types with another plugin.
  void* handle = ::dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (handle == nullptr) throw std::runtime_error("plugin " + path + ": " + ::dlerror());
  using Register = void (*)(devices::DeviceRegistry*);
  auto reg = reinterpret_cast<Register>(::dlsym(handle, "hwemu_register_devices"));
  if (reg == nullptr) {
    throw std::runtime_error("plugin " + path + " has no extern \"C\" hwemu_register_devices(DeviceRegistry*)");
  }
  reg(&devices::DeviceRegistry::Instance());
}

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
        windows->Show(w.sensor, w.view, w.title, w.fly);
      }
    } else if (!config_.windows.empty()) {
      gaps::Report("display", "sensor-windows",
                   "this backend's display has no per-sensor windows; sim.yaml `windows` not shown");
    }
  }
  // The pinbus always listens, so a firmware's GPIO and ADC reads are
  // answered (or reported as gaps) even with no device using it.
  transports_->PinBus();
  last_draw_ = last_status_ = std::chrono::steady_clock::now();
  if (config_.truth_log.period > 0.0) {
    truth_body_ = config_.sim.backend == sim::Backend::kMujoco ? config_.truth_log.mujoco : config_.truth_log.achilles;
    (void)sim_->BodyPose(truth_body_);  // an unknown body fails here, not mid-run
    for (const auto& name : config_.truth_log.encoders) truth_encoders_.push_back(sim_->Encoder(name));
    truth_ = std::make_unique<std::ofstream>(std::filesystem::path(config_.runtime_dir) / "truth.jsonl");
  }
  started_ = true;
  // Everything is listening: a launcher waiting on this file can start the
  // firmware and otto.
  std::ofstream(std::filesystem::path(config_.runtime_dir) / "ready") << ::getpid() << "\n";
}

bool Hub::Tick() {
  if (!started_) Start();
  if (stop_ || !clock_->WaitForNextTick()) return false;
  const clock::SimTime now = clock_->Now();
  gaps::SetTime(now.seconds);
  transports_->Router().Poll();
  for (auto& d : devices_) d->Tick(now);
  auto* display = sim_->Display();
  if (display != nullptr) display->ApplyWrites();
  if (display == nullptr || !display->Paused()) {
    sim_->Step(config_.clock.dt);
  }
  clock_->Advance();
  if (truth_ && clock_->Now().seconds >= next_truth_ - 1e-9) LogTruth(clock_->Now().seconds);

  const auto wall = std::chrono::steady_clock::now();
  if (display != nullptr && wall - last_draw_ >= std::chrono::duration<double>(config_.draw_period)) {
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

void Hub::LogTruth(double time) {
  next_truth_ = time + config_.truth_log.period;
  const sim::Pose p = sim_->BodyPose(truth_body_);
  const auto& q = p.orientation;
  const double yaw = std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
  char line[256];
  std::snprintf(line, sizeof(line), "{\"t\":%.4f,\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,\"yaw\":%.6f,\"q\":[%.6f,%.6f,%.6f,%.6f]",
                time, p.position.x, p.position.y, p.position.z, yaw, q.w, q.x, q.y, q.z);
  *truth_ << line;
  if (!truth_encoders_.empty()) {
    *truth_ << ",\"encoders\":{";
    for (std::size_t i = 0; i < truth_encoders_.size(); ++i) {
      std::snprintf(line, sizeof(line), "%s\"%s\":%.6f", i == 0 ? "" : ",", config_.truth_log.encoders[i].c_str(),
                    truth_encoders_[i]->Read().position);
      *truth_ << line;
    }
    *truth_ << "}";
  }
  *truth_ << "}\n";
}

void Hub::Run(const std::atomic<bool>* stop) {
  // The clock can block inside a tick (lockstep: waiting for an expected
  // participant to join or ack) where it never sees `stop`; this passes a
  // stop on to it, so a signal or the Python Hub's stop() always ends Run.
  std::atomic<bool> done{false};
  std::thread watcher;
  if (stop != nullptr) {
    watcher = std::thread([&] {
      while (!done && !*stop) std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (*stop) clock_->Stop();
    });
  }
  while ((stop == nullptr || !*stop) && Tick()) {
  }
  done = true;
  if (watcher.joinable()) watcher.join();
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
