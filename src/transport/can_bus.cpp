#include "hwemu/transport/can_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace hwemu::transport {

// ---- SocketCAN ------------------------------------------------------------
#ifdef __linux__
SocketCanBus::SocketCanBus(const std::string& ifname) : ifname_(ifname) {
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error("SocketCAN: socket(): " + std::string(std::strerror(errno)));
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    throw std::runtime_error(
        "SocketCAN: no interface '" + ifname + "' (" + err + "); bring it up with scripts/vcan.sh"
    );
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    throw std::runtime_error("SocketCAN: bind " + ifname + ": " + err);
  }
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);
}

SocketCanBus::~SocketCanBus() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool SocketCanBus::Send(const CanFrame& frame) {
  can_frame raw{};
  raw.can_id = frame.id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  if (frame.extended) raw.can_id |= CAN_EFF_FLAG;
  if (frame.rtr) raw.can_id |= CAN_RTR_FLAG;
  raw.can_dlc = std::min<std::uint8_t>(frame.len, 8);
  std::memcpy(raw.data, frame.data.data(), raw.can_dlc);
  return ::write(fd_, &raw, sizeof(raw)) == static_cast<ssize_t>(sizeof(raw));
}

bool SocketCanBus::Receive(CanFrame* frame) {
  can_frame raw{};
  const ssize_t n = ::read(fd_, &raw, sizeof(raw));
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    return false;
  }
  frame->extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame->rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
  frame->id = raw.can_id & (frame->extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame->len = raw.can_dlc;
  frame->data.fill(0);
  std::memcpy(frame->data.data(), raw.data, std::min<std::uint8_t>(raw.can_dlc, 8));
  return true;
}
#else
SocketCanBus::SocketCanBus(const std::string& ifname) : ifname_(ifname) {
  throw std::runtime_error("SocketCAN is Linux-only");
}
SocketCanBus::~SocketCanBus() = default;
bool SocketCanBus::Send(const CanFrame&) { return false; }
bool SocketCanBus::Receive(CanFrame*) { return false; }
#endif

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
    bus = std::make_shared<SocketCanBus>(spec);
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
      if (auto it = handlers_.find(Key{bus, frame.id, frame.extended}); it != handlers_.end()) {
        for (auto& h : it->second) h(frame);
      }
    }
  }
  return count;
}

}  // namespace hwemu::transport
