#include "hwemu/transport/pinbus.hpp"

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
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (auto it = spi_.find(request->name); it != spi_.end()) handler = it->second;
      }
      if (handler) {
        reply.payload = handler(request->payload);
        reply.payload.resize(request->payload.size(), 0);
        ++transactions_;
      } else {
        reply.status = 1;
        reply.payload.assign(request->payload.size(), 0xFF);
      }
      break;
    }
    case pinbus::kGpioWrite: {
      const bool level = !request->payload.empty() && request->payload[0] != 0;
      std::vector<OutputHandler> handlers;
      {
        std::lock_guard<std::mutex> l(mutex_);
        outputs_[request->name] = level;
        handlers = output_handlers_;
      }
      for (auto& h : handlers) h(request->name, level);
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
