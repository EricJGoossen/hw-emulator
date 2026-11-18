#include "hwemu/transport/can_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hwemu/gaps.hpp"
#include "hwemu/transport/can_wire.hpp"
#include "hwemu/transport/uds_wire.hpp"

namespace hwemu::transport {

// ---- UDS ------------------------------------------------------------------
UdsCanBus::UdsCanBus(std::string name, std::string path) : name_(std::move(name)), path_(std::move(path)) {
  listen_fd_ = wire::Listen(path_);
  if (listen_fd_ < 0) {
    throw std::runtime_error("CAN bus '" + name_ + "': can't listen at " + path_ + ": " + std::strerror(errno));
  }
  ::fcntl(listen_fd_, F_SETFL, ::fcntl(listen_fd_, F_GETFL) | O_NONBLOCK);
}

UdsCanBus::~UdsCanBus() {
  for (int fd : clients_) ::close(fd);
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    ::unlink(path_.c_str());
  }
}

void UdsCanBus::Accept() {
  for (;;) {
    const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) return;
    clients_.push_back(fd);
  }
}

bool UdsCanBus::Send(const CanFrame& frame) {
  Accept();
  const auto p = can_wire::Pack(frame.id, frame.extended, frame.rtr, frame.data.data(), frame.len);
  bool any = false;
  for (auto it = clients_.begin(); it != clients_.end();) {
    const ssize_t n = ::send(*it, p.data(), p.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ::close(*it);
      it = clients_.erase(it);
      continue;
    }
    any = any || n == static_cast<ssize_t>(p.size());
    ++it;
  }
  return any;
}

bool UdsCanBus::Receive(CanFrame* frame) {
  if (pending_.empty()) {
    Accept();
    std::uint8_t buf[64];
    for (std::size_t c = 0; c < clients_.size();) {
      const ssize_t n = ::recv(clients_[c], buf, sizeof(buf), MSG_DONTWAIT);
      if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
        ::close(clients_[c]);
        clients_.erase(clients_.begin() + static_cast<std::ptrdiff_t>(c));
        continue;
      }
      if (n < 0) {
        ++c;
        continue;
      }
      can_wire::Unpacked u;
      if (!can_wire::Unpack(buf, static_cast<std::size_t>(n), &u)) continue;
      CanFrame f;
      f.id = u.id;
      f.extended = u.extended;
      f.rtr = u.rtr;
      f.len = u.len;
      f.data = u.data;
      pending_.push_back(f);
      // A bus: every other node hears it too.
      for (std::size_t o = 0; o < clients_.size(); ++o) {
        if (o != c) ::send(clients_[o], buf, static_cast<std::size_t>(n), MSG_NOSIGNAL | MSG_DONTWAIT);
      }
      // Keep reading this client: frames arrive in bursts.
    }
  }
  if (pending_.empty()) return false;
  *frame = pending_.front();
  pending_.pop_front();
  return true;
}

// ---- loopback -------------------------------------------------------------
std::pair<std::shared_ptr<LoopbackCanBus>, std::shared_ptr<LoopbackCanBus>> LoopbackCanBus::Pair(
    const std::string& name
) {
  auto a_to_b = std::make_shared<Queue>();
  auto b_to_a = std::make_shared<Queue>();
  std::shared_ptr<LoopbackCanBus> a(new LoopbackCanBus(name, b_to_a, a_to_b));
  std::shared_ptr<LoopbackCanBus> b(new LoopbackCanBus(name + ":peer", a_to_b, b_to_a));
  return {a, b};
}

bool LoopbackCanBus::Send(const CanFrame& frame) {
  std::lock_guard<std::mutex> l(out_->m);
  out_->frames.push_back(frame);
  return true;
}

bool LoopbackCanBus::Receive(CanFrame* frame) {
  std::lock_guard<std::mutex> l(in_->m);
  if (in_->frames.empty()) {
    return false;
  }
  *frame = in_->frames.front();
  in_->frames.pop_front();
  return true;
}

// ---- bus set --------------------------------------------------------------
ICanBus* CanBusSet::Open(const std::string& spec) {
  if (auto it = buses_.find(spec); it != buses_.end()) {
    return it->second.get();
  }
  std::shared_ptr<ICanBus> bus;
  constexpr std::string_view kLoopback = "loopback:";
  if (spec.rfind(kLoopback, 0) == 0) {
    auto [ours, theirs] = LoopbackCanBus::Pair(spec);
    peers_[spec.substr(kLoopback.size())] = theirs;
    bus = ours;
  } else {
    bus = std::make_shared<UdsCanBus>(spec, socket_dir_ + "/" + spec + ".sock");
  }
  buses_[spec] = bus;
  return bus.get();
}

ICanBus* CanBusSet::Find(const std::string& spec) const {
  auto it = buses_.find(spec);
  return it == buses_.end() ? nullptr : it->second.get();
}

std::shared_ptr<LoopbackCanBus> CanBusSet::Peer(const std::string& name) {
  auto it = peers_.find(name);
  return it == peers_.end() ? nullptr : it->second;
}

// ---- router ---------------------------------------------------------------
void CanRouter::Track(ICanBus* bus) {
  if (std::find(buses_.begin(), buses_.end(), bus) == buses_.end()) {
    buses_.push_back(bus);
  }
}

void CanRouter::Subscribe(ICanBus* bus, std::uint32_t id, bool extended, Handler handler) {
  Track(bus);
  handlers_[Key{bus, id, extended}].push_back(std::move(handler));
}

void CanRouter::SubscribeAll(ICanBus* bus, Handler handler) {
  Track(bus);
  taps_[bus].push_back(std::move(handler));
}

std::size_t CanRouter::Poll() {
  std::size_t count = 0;
  CanFrame frame;
  for (ICanBus* bus : buses_) {
    // Bounded, so a flood on one bus can't stall the tick.
    for (int i = 0; i < 4096 && bus->Receive(&frame); ++i) {
      ++count;
      if (auto it = taps_.find(bus); it != taps_.end()) {
        for (auto& h : it->second) h(frame);
      }
      bool tapped = taps_.count(bus) != 0;
      if (auto it = handlers_.find(Key{bus, frame.id, frame.extended}); it != handlers_.end()) {
        for (auto& h : it->second) h(frame);
      } else if (!tapped) {
        char id[16];
        std::snprintf(id, sizeof(id), frame.extended ? "0x%08X" : "0x%03X", frame.id);
        gaps::Report("can", bus->Name() + ":" + id,
                     "CAN frame " + std::string(id) + " on " + bus->Name() + " that no device consumes (dropped)");
      }
    }
  }
  return count;
}

}  // namespace hwemu::transport
