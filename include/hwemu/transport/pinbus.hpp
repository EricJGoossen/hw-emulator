#pragma once

// pinbus: SPI transactions and GPIO levels between the hub and the hosted
// MCB, over a Unix SOCK_SEQPACKET socket (uds_wire.hpp's framing).
//
// This is custom: Linux has spidev and the GPIO character device for real
// hardware, but nothing a hosted (simulated) firmware can bind to with a
// device model behind it, so the firmware's HAL calls client/hwemu_client.h,
// which speaks this. Everything is addressed by name -- an SPI device
// ("imu" for the main board's MPU6500, "ism330", ...) or a pin ("B12").
//
//   type  request (client -> hub)      reply (hub -> client, type | 0x80)
//   0x01  HELLO      name = client      status 0
//   0x02  SPI_XFER   name = device,     payload = rx bytes (same length);
//                    payload = tx bytes status 1: no such device
//   0x03  GPIO_WRITE name = pin,        status 0
//                    payload = [level]
//   0x04  GPIO_READ  name = pin         payload = [level]; status 1: unknown pin
//   0x05  GPIO_SUBSCRIBE name = pin     status 0, then 0x06 GPIO_EVENT
//                                       (name = pin, payload = [level]) on
//                                       each change
//
// The server answers on its own thread, so a firmware blocked on a
// transaction never waits for the sim's tick.

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace hwemu::transport {

namespace pinbus {
enum Type : std::uint8_t {
  kHello = 0x01,
  kSpiTransfer = 0x02,
  kGpioWrite = 0x03,
  kGpioRead = 0x04,
  kGpioSubscribe = 0x05,
  kGpioEvent = 0x06,
  kReply = 0x80,
};
}  // namespace pinbus

class PinBusServer {
 public:
  using SpiHandler = std::function<std::vector<std::uint8_t>(std::span<const std::uint8_t>)>;
  using OutputHandler = std::function<void(const std::string& pin, bool level)>;

  // Listens at `path` (its directory created). Throws std::runtime_error.
  explicit PinBusServer(const std::string& path);
  ~PinBusServer();
  PinBusServer(const PinBusServer&) = delete;
  PinBusServer& operator=(const PinBusServer&) = delete;

  const std::string& Path() const { return path_; }

  // A device the firmware can run transactions on; `handler` runs on the
  // server's thread.
  void RegisterSpi(const std::string& device, SpiHandler handler);
  // A pin the sim drives (a limit switch): what GPIO_READ returns, and what
  // subscribers hear about when it changes.
  void SetInput(const std::string& pin, bool level);
  // A pin the firmware drives (a laser, a solenoid): its last written level.
  bool Output(const std::string& pin) const;
  void OnOutput(OutputHandler handler);

  std::size_t Clients() const;
  std::uint64_t Transactions() const { return transactions_.load(); }

 private:
  void Run();
  void Handle(int fd);
  void Notify(const std::string& pin, bool level);

  std::string path_;
  int listen_fd_ = -1;
  int wake_[2] = {-1, -1};
  std::atomic<bool> stop_{false};
  std::thread thread_;

  mutable std::mutex mutex_;
  std::map<std::string, SpiHandler> spi_;
  std::map<std::string, bool> inputs_;
  std::map<std::string, bool> outputs_;
  std::map<std::string, std::set<int>> subscribers_;
  std::vector<OutputHandler> output_handlers_;
  std::set<int> clients_;
  std::atomic<std::uint64_t> transactions_{0};
};

}  // namespace hwemu::transport
