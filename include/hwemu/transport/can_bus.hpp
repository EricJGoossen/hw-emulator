#pragma once

// CAN buses the hub's devices talk on.
//
//   UdsCanBus      the one CAN transport between processes: a Unix
//                  SOCK_SEQPACKET socket per bus, <runtime_dir>/<bus>.sock,
//                  that the hub serves and the hosted MCB (client/) connects
//                  to. No root, no kernel module. Every connected client is a
//                  node on the bus: a frame one sends reaches the hub and
//                  every other client, as on a real bus.
//   LoopbackCanBus an in-process pair of endpoints, for tests: what one end
//                  sends the other receives.
//
// A frame on the UDS bus is one 16-byte packet (can_wire.hpp).
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

class UdsCanBus final : public ICanBus {
 public:
  // Listens at `path`. Throws std::runtime_error if it can't.
  UdsCanBus(std::string name, std::string path);
  ~UdsCanBus() override;
  UdsCanBus(const UdsCanBus&) = delete;
  UdsCanBus& operator=(const UdsCanBus&) = delete;

  bool Send(const CanFrame& frame) override;
  bool Receive(CanFrame* frame) override;
  std::string Name() const override { return name_; }
  const std::string& Path() const { return path_; }
  std::size_t Clients() const { return clients_.size(); }

 private:
  void Accept();
  std::string name_, path_;
  int listen_fd_ = -1;
  std::vector<int> clients_;
  std::deque<CanFrame> pending_;
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

// Opens a bare name ("can1") as a UdsCanBus at <socket_dir>/<name>.sock, and
// "loopback:<name>" as one end of a named in-process loopback pair (the other
// end is Peer(name)).
class CanBusSet {
 public:
  void SetSocketDir(std::string dir) { socket_dir_ = std::move(dir); }
  ICanBus* Open(const std::string& spec);
  ICanBus* Find(const std::string& spec) const;
  // The far end of loopback:<name>, for a test standing in for the firmware.
  std::shared_ptr<LoopbackCanBus> Peer(const std::string& name);
  const std::map<std::string, std::shared_ptr<ICanBus>>& All() const { return buses_; }

 private:
  std::map<std::string, std::shared_ptr<ICanBus>> buses_;
  std::map<std::string, std::shared_ptr<LoopbackCanBus>> peers_;
  std::string socket_dir_ = "/tmp/hwemu";
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
