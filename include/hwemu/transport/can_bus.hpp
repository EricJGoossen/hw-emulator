#pragma once

// CAN buses the hub's devices talk on.
//
//   SocketCanBus   a Linux SocketCAN interface -- vcan0 for the hosted MCB,
//                  or a real USB-CAN adapter (can0) for hardware in the loop.
//   LoopbackCanBus an in-process pair of endpoints, for tests and CI where
//                  vcan can't be loaded: what one end sends the other receives.
//
// CanRouter fans received frames out to the devices that asked for their ids.

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hwemu/protocol/can_frame.hpp"

namespace hwemu::transport {

using protocol::CanFrame;

class ICanBus {
 public:
  virtual ~ICanBus() = default;
  virtual bool Send(const CanFrame& frame) = 0;
  // Non-blocking: false if nothing is waiting.
  virtual bool Receive(CanFrame* frame) = 0;
  virtual std::string Name() const = 0;
};

class SocketCanBus final : public ICanBus {
 public:
  // Throws std::runtime_error if the interface can't be opened.
  explicit SocketCanBus(const std::string& ifname);
  ~SocketCanBus() override;
  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  bool Send(const CanFrame& frame) override;
  bool Receive(CanFrame* frame) override;
  std::string Name() const override { return ifname_; }
  int Fd() const { return fd_; }

 private:
  std::string ifname_;
  int fd_ = -1;
};

class LoopbackCanBus final : public ICanBus {
 public:
  // Two connected endpoints: a frame Send()t on one is Receive()d on the other.
  static std::pair<std::shared_ptr<LoopbackCanBus>, std::shared_ptr<LoopbackCanBus>> Pair(
      const std::string& name = "loopback"
  );

  bool Send(const CanFrame& frame) override;
  bool Receive(CanFrame* frame) override;
  std::string Name() const override { return name_; }

 private:
  struct Queue {
    std::mutex m;
    std::deque<CanFrame> frames;
  };
  LoopbackCanBus(std::string name, std::shared_ptr<Queue> in, std::shared_ptr<Queue> out)
      : name_(std::move(name)), in_(std::move(in)), out_(std::move(out)) {}
  std::string name_;
  std::shared_ptr<Queue> in_, out_;
};

// Opens "vcan0"/"can0"-style names as SocketCAN, and "loopback:<name>" as one
// end of a named in-process loopback pair (the other end is Peer(name)).
class CanBusSet {
 public:
  ICanBus* Open(const std::string& spec);
  ICanBus* Find(const std::string& spec) const;
  // The far end of loopback:<name>, for a test standing in for the firmware.
  std::shared_ptr<LoopbackCanBus> Peer(const std::string& name);
  const std::map<std::string, std::shared_ptr<ICanBus>>& All() const { return buses_; }

 private:
  std::map<std::string, std::shared_ptr<ICanBus>> buses_;
  std::map<std::string, std::shared_ptr<LoopbackCanBus>> peers_;
};

class CanRouter {
 public:
  using Handler = std::function<void(const CanFrame&)>;
  // Calls `handler` for every frame on `bus` with this id and frame format.
  void Subscribe(ICanBus* bus, std::uint32_t id, bool extended, Handler handler);
  // Calls `handler` for every frame on `bus` (a tap).
  void SubscribeAll(ICanBus* bus, Handler handler);
  // Drains every bus that has a subscriber; returns frames dispatched.
  std::size_t Poll();

 private:
  struct Key {
    ICanBus* bus;
    std::uint32_t id;
    bool extended;
    auto operator<=>(const Key&) const = default;
  };
  std::map<Key, std::vector<Handler>> handlers_;
  std::map<ICanBus*, std::vector<Handler>> taps_;
  std::vector<ICanBus*> buses_;
  void Track(ICanBus* bus);
};

}  // namespace hwemu::transport
