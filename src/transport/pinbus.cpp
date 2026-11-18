#include "hwemu/transport/pinbus.hpp"

#include <cstring>

#include "hwemu/gaps.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hwemu/transport/uds_wire.hpp"

namespace hwemu::transport {

PinBusServer::PinBusServer(const std::string& path) : path_(path) {
  const std::filesystem::path p(path);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path());
  }
  listen_fd_ = wire::Listen(path);
  if (listen_fd_ < 0) {
    throw std::runtime_error("pinbus: can't listen at " + path + ": " + std::strerror(errno));
  }
  if (::pipe(wake_) != 0) {
    throw std::runtime_error("pinbus: pipe failed");
  }
  thread_ = std::thread([this] { Run(); });
}

PinBusServer::~PinBusServer() {
  stop_ = true;
  const char c = 0;
  (void)::write(wake_[1], &c, 1);
  if (thread_.joinable()) thread_.join();
  for (int fd : clients_) ::close(fd);
  if (listen_fd_ >= 0) ::close(listen_fd_);
  ::close(wake_[0]);
  ::close(wake_[1]);
  ::unlink(path_.c_str());
}

void PinBusServer::RegisterSpi(const std::string& device, SpiHandler handler) {
  std::lock_guard<std::mutex> l(mutex_);
  spi_[device] = std::move(handler);
}

void PinBusServer::SetInput(const std::string& pin, bool level) {
  bool changed = false;
  {
    std::lock_guard<std::mutex> l(mutex_);
    auto it = inputs_.find(pin);
    changed = it == inputs_.end() || it->second != level;
    inputs_[pin] = level;
  }
  if (changed) {
    Notify(pin, level);
  }
}

bool PinBusServer::Output(const std::string& pin) const {
  std::lock_guard<std::mutex> l(mutex_);
  auto it = outputs_.find(pin);
  return it != outputs_.end() && it->second;
}

void PinBusServer::OnOutput(OutputHandler handler) {
  std::lock_guard<std::mutex> l(mutex_);
  output_handlers_.push_back(std::move(handler));
}

std::size_t PinBusServer::Clients() const {
  std::lock_guard<std::mutex> l(mutex_);
  return clients_.size();
}

void PinBusServer::Notify(const std::string& pin, bool level) {
  std::set<int> targets;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (auto it = subscribers_.find(pin); it != subscribers_.end()) targets = it->second;
  }
  wire::Message event{pinbus::kGpioEvent, 0, 0, pin, {static_cast<std::uint8_t>(level ? 1 : 0)}};
  for (int fd : targets) {
    wire::Send(fd, event);
  }
}

void PinBusServer::RegisterSpiOnBus(const std::string& device, const std::string& bus, const std::string& cs_pin) {
  std::lock_guard<std::mutex> l(mutex_);
  spi_buses_[bus].push_back({device, cs_pin});
}

void PinBusServer::SetAnalog(const std::string& pin, std::uint16_t counts) {
  std::lock_guard<std::mutex> l(mutex_);
  analog_[pin] = counts;
}

void PinBusServer::OnPwm(const std::string& pin, PwmHandler handler) {
  std::lock_guard<std::mutex> l(mutex_);
  pwm_handlers_[pin] = std::move(handler);
}

std::pair<float, std::uint32_t> PinBusServer::Pwm(const std::string& pin) const {
  std::lock_guard<std::mutex> l(mutex_);
  auto it = pwm_.find(pin);
  return it == pwm_.end() ? std::pair<float, std::uint32_t>{0.0F, 0} : it->second;
}

void PinBusServer::AcceptOutput(const std::string& pin) {
  std::lock_guard<std::mutex> l(mutex_);
  accepted_outputs_.insert(pin);
}

void PinBusServer::Run() {
  while (!stop_) {
    std::vector<pollfd> fds;
    fds.push_back({wake_[0], POLLIN, 0});
    fds.push_back({listen_fd_, POLLIN, 0});
    {
      std::lock_guard<std::mutex> l(mutex_);
      for (int fd : clients_) fds.push_back({fd, POLLIN, 0});
    }
    if (::poll(fds.data(), fds.size(), 200) <= 0) {
      continue;
    }
    if ((fds[1].revents & POLLIN) != 0) {
      const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
      if (fd >= 0) {
        std::lock_guard<std::mutex> l(mutex_);
        clients_.insert(fd);
      }
    }
    for (std::size_t i = 2; i < fds.size(); ++i) {
      if ((fds[i].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
        Handle(fds[i].fd);
      }
    }
  }
}

void PinBusServer::Handle(int fd) {
  auto request = wire::Receive(fd);
  if (!request) {
    std::lock_guard<std::mutex> l(mutex_);
    clients_.erase(fd);
    for (auto& [_, subs] : subscribers_) subs.erase(fd);
    ::close(fd);
    return;
  }
  wire::Message reply{static_cast<std::uint8_t>(request->type | pinbus::kReply), 0, request->request_id,
                      request->name, {}};
  switch (request->type) {
    case pinbus::kHello:
      break;
    case pinbus::kSpiTransfer: {
      SpiHandler handler;
      std::string gap_key, gap;
      bool routed = false;
      std::vector<std::uint8_t> transaction;
      {
        std::lock_guard<std::mutex> l(mutex_);
        std::string device = request->name;
        if (device.rfind("bus:", 0) == 0) {
          const std::string bus = device.substr(4);
          std::vector<std::string> selected;
          if (auto b = spi_buses_.find(bus); b != spi_buses_.end()) {
            for (const auto& d : b->second) {
              auto cs = outputs_.find(d.cs_pin);
              if (cs != outputs_.end() && !cs->second) selected.push_back(d.device);
            }
          }
          if (selected.size() == 1) {
            device = selected[0];
            routed = true;
          } else {
            gap_key = "spi:" + bus + (selected.empty() ? ":none" : ":many");
            gap = "SPI transfer on bus '" + bus + "' with " +
                  (selected.empty() ? std::string("no chip select low") : "more than one chip select low") +
                  "; read back 0xFF";
            device.clear();
          }
        }
        if (auto it = spi_.find(device); it != spi_.end()) {
          handler = it->second;
          if (routed) {
            auto& open = open_transactions_[device];
            open.insert(open.end(), request->payload.begin(), request->payload.end());
            transaction = open;
          }
        } else if (gap.empty()) {
          gap_key = "spi:" + request->name;
          gap = "SPI transfer to '" + request->name + "', which no device models; read back 0xFF";
        }
      }
      if (handler && routed) {
        // The whole transaction so far; this transfer's share is its tail.
        auto all = handler(transaction);
        all.resize(transaction.size(), 0);
        reply.payload.assign(all.end() - static_cast<std::ptrdiff_t>(request->payload.size()), all.end());
        ++transactions_;
      } else if (handler) {
        reply.payload = handler(request->payload);
        reply.payload.resize(request->payload.size(), 0);
        ++transactions_;
      } else {
        gaps::Report("pinbus", gap_key, gap);
        reply.status = 1;
        reply.payload.assign(request->payload.size(), 0xFF);
      }
      break;
    }
    case pinbus::kPwmWrite: {
      float duty = 0;
      std::uint32_t hz = 0;
      if (request->payload.size() >= 8) {
        std::memcpy(&duty, request->payload.data(), 4);
        std::memcpy(&hz, request->payload.data() + 4, 4);
      }
      PwmHandler handler;
      bool accepted = false;
      {
        std::lock_guard<std::mutex> l(mutex_);
        pwm_[request->name] = {duty, hz};
        if (auto it = pwm_handlers_.find(request->name); it != pwm_handlers_.end()) handler = it->second;
        accepted = accepted_outputs_.count(request->name) != 0;
      }
      if (handler) {
        handler(request->name, duty, hz);
      } else if (!accepted) {
        gaps::Report("pinbus", "pwm:" + request->name,
                     "PWM written on pin '" + request->name + "', which nothing in the sim models");
      }
      break;
    }
    case pinbus::kAdcRead: {
      bool known = false;
      std::uint16_t counts = 0;
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (auto it = analog_.find(request->name); it != analog_.end()) {
          known = true;
          counts = it->second;
        }
      }
      if (!known) {
        reply.status = 1;
        gaps::Report("pinbus", "adc:" + request->name,
                     "ADC read on pin '" + request->name + "', which nothing in the sim drives; read 0");
      }
      reply.payload = {static_cast<std::uint8_t>(counts & 0xFF), static_cast<std::uint8_t>(counts >> 8)};
      break;
    }
    case pinbus::kGpioWrite: {
      const bool level = !request->payload.empty() && request->payload[0] != 0;
      std::vector<OutputHandler> handlers;
      bool modeled = false;
      {
        std::lock_guard<std::mutex> l(mutex_);
        outputs_[request->name] = level;
        handlers = output_handlers_;
        modeled = accepted_outputs_.count(request->name) != 0;
        for (const auto& [bus, devices] : spi_buses_) {
          for (const auto& d : devices) {
            if (d.cs_pin != request->name) continue;
            modeled = true;
            open_transactions_[d.device].clear();  // either edge starts or ends a transaction
          }
        }
      }
      for (auto& h : handlers) h(request->name, level);
      if (!modeled && handlers.empty()) {
        gaps::Report("pinbus", "gpio-out:" + request->name,
                     "GPIO output '" + request->name + "' written, but nothing in the sim reacts to it");
      }
      break;
    }
    case pinbus::kGpioRead: {
      std::lock_guard<std::mutex> l(mutex_);
      if (auto it = inputs_.find(request->name); it != inputs_.end()) {
        reply.payload = {static_cast<std::uint8_t>(it->second ? 1 : 0)};
      } else if (auto out = outputs_.find(request->name); out != outputs_.end()) {
        reply.payload = {static_cast<std::uint8_t>(out->second ? 1 : 0)};
      } else {
        reply.status = 1;
        reply.payload = {0};
        gaps::Report("pinbus", "gpio-in:" + request->name,
                     "GPIO input '" + request->name + "' read, but nothing in the sim drives it; read low");
      }
      break;
    }
    case pinbus::kGpioSubscribe: {
      std::lock_guard<std::mutex> l(mutex_);
      subscribers_[request->name].insert(fd);
      break;
    }
    default:
      reply.status = 2;
      break;
  }
  wire::Send(fd, reply);
}

}  // namespace hwemu::transport
