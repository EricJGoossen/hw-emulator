#include "hwemu_client.h"

#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "hwemu/transport/can_wire.hpp"
#include "hwemu/transport/uds_wire.hpp"

namespace wire = hwemu::transport::wire;
namespace can_wire = hwemu::transport::can_wire;

namespace {

std::string RuntimeDir() {
  const char* env = std::getenv("HWEMU_RUNTIME_DIR");
  return env != nullptr && *env != '\0' ? env : "/tmp/hwemu";
}

std::string Join(const std::string& dir, const std::string& name) { return dir + "/" + name; }

// One pinbus connection: a request id counter, and GPIO events that arrived
// while a request waited for its reply.
//
// Output writes are cached per pin: a firmware rewrites the same level or
// duty every loop iteration, and a write the pin already holds changes
// nothing in the hub, so it skips the round trip and returns the status the
// last real write got.
struct PinBusConnection {
  int fd = -1;
  std::uint32_t next_id = 1;
  std::mutex mutex;
  std::mutex cache_mutex;
  std::map<std::string, std::pair<int, int>> gpio_written;  // pin -> (level, status)
  struct Pwm {
    float duty;
    uint32_t frequency_hz;
    int status;
  };
  std::map<std::string, Pwm> pwm_written;
};

std::mutex g_connections_mutex;
std::map<int, PinBusConnection*> g_connections;
int g_next_handle = 0;

PinBusConnection* Connection(int handle) {
  std::lock_guard<std::mutex> l(g_connections_mutex);
  auto it = g_connections.find(handle);
  return it == g_connections.end() ? nullptr : it->second;
}

// Sends a request and waits for the reply with its id, skipping events.
//
// The hub answers within microseconds, but waking from a blocking receive
// costs tens of microseconds on some hosts (WSL), on every request. So this
// first polls for the reply without sleeping, for up to kSpin, and only
// then blocks.
bool Request(PinBusConnection* c, wire::Message request, wire::Message* reply) {
  constexpr auto kSpin = std::chrono::microseconds(200);
  std::lock_guard<std::mutex> l(c->mutex);
  request.request_id = c->next_id++;
  if (!wire::Send(c->fd, request)) return false;
  while (true) {
    const auto spin_until = std::chrono::steady_clock::now() + kSpin;
    pollfd p{c->fd, POLLIN, 0};
    while (::poll(&p, 1, 0) == 0 && std::chrono::steady_clock::now() < spin_until) {
    }
    auto m = wire::Receive(c->fd);
    if (!m) return false;
    if ((m->type & 0x80U) != 0 && m->request_id == request.request_id) {
      *reply = std::move(*m);
      return true;
    }
  }
}

struct ClockShm {
  const std::uint8_t* base = nullptr;
  bool tried = false;
};

}  // namespace

extern "C" {

const char* hwemu_runtime_dir(void) {
  static std::string dir = RuntimeDir();
  return dir.c_str();
}

int hwemu_port_path(const char* name, char* out, size_t out_size) {
  std::string env = "HWEMU_PORT_";
  for (const char* p = name; *p != '\0'; ++p) env += static_cast<char>(std::toupper(static_cast<unsigned char>(*p)));
  const char* override_path = std::getenv(env.c_str());
  const std::string path = override_path != nullptr ? override_path : Join(RuntimeDir(), name);
  if (out_size == 0) return -1;
  std::strncpy(out, path.c_str(), out_size - 1);
  out[out_size - 1] = '\0';
  return static_cast<int>(path.size());
}

int hwemu_serial_open(const char* path) {
  const int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios t{};
  if (::tcgetattr(fd, &t) == 0) {
    ::cfmakeraw(&t);
    ::tcsetattr(fd, TCSANOW, &t);
  }
  return fd;
}

int hwemu_can_open(const char* bus) {
  // The hub's UDS CAN bus (transport/can_bus.hpp): <runtime dir>/<bus>.sock,
  // or an absolute socket path.
  const std::string name(bus);
  const std::string path = !name.empty() && name[0] == '/' ? name : Join(RuntimeDir(), name + ".sock");
  const int fd = wire::Connect(path);
  if (fd < 0) return -1;
  ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
  return fd;
}

int hwemu_can_send(int fd, uint32_t id, int extended, const uint8_t* data, uint8_t len) {
  const auto p = can_wire::Pack(id, extended != 0, false, data, len);
  return ::send(fd, p.data(), p.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(p.size()) ? 0 : -1;
}

int hwemu_can_recv(int fd, uint32_t* id, int* extended, uint8_t* data, uint8_t* len) {
  std::uint8_t buf[64];
  const ssize_t n = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
  if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  if (n == 0) return -1;  // the hub went away
  can_wire::Unpacked u;
  if (!can_wire::Unpack(buf, static_cast<std::size_t>(n), &u)) return 0;
  *id = u.id;
  *extended = u.extended ? 1 : 0;
  *len = u.len;
  std::memcpy(data, u.data.data(), u.len);
  return 1;
}

void hwemu_can_close(int fd) { ::close(fd); }

int hwemu_pinbus_connect(const char* path, const char* client_name) {
  const std::string p = path != nullptr ? path : Join(RuntimeDir(), "pinbus.sock");
  const int fd = wire::Connect(p);
  if (fd < 0) return -1;
  auto* c = new PinBusConnection;
  c->fd = fd;
  wire::Message reply;
  if (!Request(c, {0x01, 0, 0, client_name != nullptr ? client_name : "mcb", {}}, &reply)) {
    ::close(fd);
    delete c;
    return -1;
  }
  std::lock_guard<std::mutex> l(g_connections_mutex);
  const int handle = g_next_handle++;
  g_connections[handle] = c;
  return handle;
}

void hwemu_pinbus_close(int handle) {
  PinBusConnection* c = nullptr;
  {
    std::lock_guard<std::mutex> l(g_connections_mutex);
    auto it = g_connections.find(handle);
    if (it == g_connections.end()) return;
    c = it->second;
    g_connections.erase(it);
  }
  ::close(c->fd);
  delete c;
}

int hwemu_spi_transfer(int handle, const char* device, const uint8_t* tx, uint8_t* rx, uint16_t len) {
  std::memset(rx, 0xFF, len);
  PinBusConnection* c = Connection(handle);
  if (c == nullptr) return -1;
  wire::Message reply;
  if (!Request(c, {0x02, 0, 0, device, std::vector<std::uint8_t>(tx, tx + len)}, &reply)) return -1;
  if (reply.status != 0) return -1;
  std::memcpy(rx, reply.payload.data(), std::min<std::size_t>(len, reply.payload.size()));
  return 0;
}

int hwemu_gpio_read(int handle, const char* pin) {
  PinBusConnection* c = Connection(handle);
  if (c == nullptr) return -1;
  wire::Message reply;
  if (!Request(c, {0x04, 0, 0, pin, {}}, &reply) || reply.status != 0 || reply.payload.empty()) return -1;
  return reply.payload[0] != 0 ? 1 : 0;
}

int hwemu_gpio_write(int handle, const char* pin, int level) {
  PinBusConnection* c = Connection(handle);
  if (c == nullptr) return -1;
  const int bit = level != 0 ? 1 : 0;
  {
    std::lock_guard<std::mutex> l(c->cache_mutex);
    auto it = c->gpio_written.find(pin);
    if (it != c->gpio_written.end() && it->second.first == bit) return it->second.second;
  }
  wire::Message reply;
  if (!Request(c, {0x03, 0, 0, pin, {static_cast<std::uint8_t>(bit)}}, &reply)) return -1;
  const int status = reply.status == 0 ? 0 : -1;
  std::lock_guard<std::mutex> l(c->cache_mutex);
  c->gpio_written[pin] = {bit, status};
  return status;
}

int hwemu_pwm_write(int handle, const char* pin, float duty, uint32_t frequency_hz) {
  PinBusConnection* c = Connection(handle);
  if (c == nullptr) return -1;
  {
    std::lock_guard<std::mutex> l(c->cache_mutex);
    auto it = c->pwm_written.find(pin);
    if (it != c->pwm_written.end() && it->second.duty == duty && it->second.frequency_hz == frequency_hz) {
      return it->second.status;
    }
  }
  std::vector<std::uint8_t> payload(8);
  std::memcpy(payload.data(), &duty, 4);
  std::memcpy(payload.data() + 4, &frequency_hz, 4);
  wire::Message reply;
  if (!Request(c, {0x07, 0, 0, pin, payload}, &reply)) return -1;
  const int status = reply.status == 0 ? 0 : -1;
  std::lock_guard<std::mutex> l(c->cache_mutex);
  c->pwm_written[pin] = {duty, frequency_hz, status};
  return status;
}

int hwemu_adc_read(int handle, const char* pin) {
  PinBusConnection* c = Connection(handle);
  if (c == nullptr) return -1;
  wire::Message reply;
  if (!Request(c, {0x08, 0, 0, pin, {}}, &reply) || reply.payload.size() < 2) return -1;
  // A pin nothing drives reads 0 (the hub reports that gap itself).
  return reply.payload[0] | (reply.payload[1] << 8);
}

void hwemu_report_gap(const char* source, const char* key, const char* message) {
  static std::mutex m;
  static std::set<std::string> seen;
  std::lock_guard<std::mutex> l(m);
  if (!seen.insert(std::string(source) + '\x1f' + key).second) return;
  std::fprintf(stderr, "[hwemu GAP] %s: %s\n", source, message);
  const char* process = std::getenv("HWEMU_PROCESS");
  const std::string dir = Join(RuntimeDir(), "gaps");
  ::mkdir(dir.c_str(), 0755);
  const std::string path = Join(dir, std::string(process != nullptr && *process != '\0' ? process : "client") + ".jsonl");
  if (FILE* f = std::fopen(path.c_str(), "a")) {
    auto json = [](const char* text) {
      std::string out = "\"";
      for (const char* p = text; *p != '\0'; ++p) {
        if (*p == '"' || *p == '\\') out += '\\';
        if (static_cast<unsigned char>(*p) >= 0x20) out += *p;
      }
      return out + "\"";
    };
    std::fprintf(f, "{\"process\":%s,\"source\":%s,\"key\":%s,\"message\":%s,\"time\":%.6f}\n",
                 json(process != nullptr ? process : "client").c_str(), json(source).c_str(), json(key).c_str(),
                 json(message).c_str(), static_cast<double>(hwemu_clock_micros()) * 1e-6);
    std::fclose(f);
  }
}

uint64_t hwemu_clock_micros(void) {
  static std::mutex m;
  static ClockShm shm;
  static const auto start = std::chrono::steady_clock::now();
  static auto next_try = start;
  static bool warned = false;
  {
    std::lock_guard<std::mutex> l(m);
    // Keep looking (at most every 100 ms) until the hub's clock appears.
    const auto now = std::chrono::steady_clock::now();
    if (shm.base == nullptr && now >= next_try) {
      next_try = now + std::chrono::milliseconds(100);
      const char* env = std::getenv("HWEMU_CLOCK_SHM");
      const std::string name = std::string("/") + (env != nullptr ? env : "hwemu_clock");
      const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
      if (fd >= 0) {
        void* p = ::mmap(nullptr, 64, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p != MAP_FAILED && std::memcmp(p, "HWEMUCK1", 8) == 0) shm.base = static_cast<const std::uint8_t*>(p);
      }
      if (shm.base == nullptr && !warned) {
        warned = true;
        // Not through hwemu_report_gap: it stamps reports with this clock.
        std::fprintf(stderr, "[hwemu GAP] clock: no hub clock (/dev/shm%s); using wall time until it appears\n",
                     name.c_str());
      }
    }
  }
  if (shm.base != nullptr) {
    for (int attempt = 0; attempt < 16; ++attempt) {
      auto lock = std::atomic_ref<std::uint64_t>(*const_cast<std::uint64_t*>(
          reinterpret_cast<const std::uint64_t*>(shm.base + 40)));
      const std::uint64_t before = lock.load(std::memory_order_acquire);
      if ((before & 1U) != 0) continue;
      std::uint64_t micros = 0;
      std::memcpy(&micros, shm.base + 8, 8);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (lock.load(std::memory_order_acquire) == before) return micros;
    }
  }
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count());
}

int hwemu_lockstep_join(const char* socket_path, const char* name) {
  const std::string p = socket_path != nullptr ? socket_path : Join(RuntimeDir(), "clock.sock");
  const int fd = wire::Connect(p);
  if (fd < 0) return -1;
  if (!wire::Send(fd, {0x01, 0, 0, name != nullptr ? name : "participant", {}})) {
    ::close(fd);
    return -1;
  }
  auto welcome = wire::Receive(fd);
  if (!welcome || welcome->type != 0x81) {
    ::close(fd);
    return -1;
  }
  return fd;
}

int hwemu_lockstep_wait(int handle, uint64_t* tick, uint64_t* micros) {
  // The next tick usually arrives within a millisecond (the hub's step);
  // waking from a blocking receive costs tens of microseconds on some hosts
  // (WSL), every tick. So poll without sleeping for up to kSpin first.
  constexpr auto kSpin = std::chrono::milliseconds(1);
  while (true) {
    const auto spin_until = std::chrono::steady_clock::now() + kSpin;
    pollfd p{handle, POLLIN, 0};
    while (::poll(&p, 1, 0) == 0 && std::chrono::steady_clock::now() < spin_until) {
    }
    auto m = wire::Receive(handle);
    if (!m) return -1;
    if (m->type == 0x02 && m->payload.size() >= 16) {
      std::memcpy(tick, m->payload.data(), 8);
      std::memcpy(micros, m->payload.data() + 8, 8);
      return 0;
    }
  }
}

int hwemu_lockstep_ack(int handle, uint64_t tick) {
  std::vector<std::uint8_t> p(8);
  std::memcpy(p.data(), &tick, 8);
  return wire::Send(handle, {0x03, 0, 0, "", p}) ? 0 : -1;
}

void hwemu_lockstep_leave(int handle) {
  wire::Send(handle, {0x04, 0, 0, "", {}});
  ::close(handle);
}

}  // extern "C"
