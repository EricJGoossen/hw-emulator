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
//   0x07  PWM_WRITE  name = pin,        status 0
//                    payload = f32 duty (0..1) | u32 frequency Hz
//   0x08  ADC_READ   name = pin         payload = u16 counts (12-bit, 0..4095);
//                                       status 1: nothing drives that pin
//
// SPI is addressed by device name, or -- as the hardware does it -- by bus:
// SPI_XFER to "bus:<bus>" goes to whichever device on that bus has its
// chip-select pin low (the firmware drives CS with GPIO_WRITE). No device
// selected, or more than one, is a sim gap and reads back 0xFF.
//
// Anything a firmware asks for that nothing here serves -- an unknown SPI
// device, a pin nobody drives, a PWM or ADC pin nothing models -- is
// reported once as a sim gap (gaps.hpp), never silently.
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
  kPwmWrite = 0x07,
  kAdcRead = 0x08,
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

  using PwmHandler = std::function<void(const std::string& pin, float duty, std::uint32_t hz)>;

  // A device the firmware can run transactions on; `handler` runs on the
  // server's thread.
  void RegisterSpi(const std::string& device, SpiHandler handler);
  // The same device, also reachable as "bus:<bus>" while `cs_pin` is low.
  void RegisterSpiOnBus(const std::string& device, const std::string& bus, const std::string& cs_pin);
  // What the firmware reads on ADC pin `pin`.
  void SetAnalog(const std::string& pin, std::uint16_t counts);
  // A PWM pin the sim models; writes to any other are a gap.
  void OnPwm(const std::string& pin, PwmHandler handler);
  // A PWM pin's last duty and frequency (0, 0 before the first write).
  std::pair<float, std::uint32_t> Pwm(const std::string& pin) const;
  // A pin the firmware drives whose writes the sim deliberately ignores
  // (a stub): written, never reported. Without this, writes to a pin with no
  // handler are a gap.
  void AcceptOutput(const std::string& pin);
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
  struct BusDevice {
    std::string device, cs_pin;
  };
  std::map<std::string, std::vector<BusDevice>> spi_buses_;
  std::map<std::string, std::uint16_t> analog_;
  std::map<std::string, PwmHandler> pwm_handlers_;
  std::map<std::string, std::pair<float, std::uint32_t>> pwm_;
  std::set<std::string> accepted_outputs_;
  std::map<std::string, bool> inputs_;
  std::map<std::string, bool> outputs_;
  std::map<std::string, std::set<int>> subscribers_;
  std::vector<OutputHandler> output_handlers_;
  std::set<int> clients_;
  std::atomic<std::uint64_t> transactions_{0};
};

}  // namespace hwemu::transport
