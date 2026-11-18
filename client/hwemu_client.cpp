#include "hwemu_client.h"

#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <string>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "hwemu/transport/uds_wire.hpp"

namespace wire = hwemu::transport::wire;

namespace {

std::string RuntimeDir() {
  const char* env = std::getenv("HWEMU_RUNTIME_DIR");
  return env != nullptr && *env != '\0' ? env : "/tmp/hwemu";
}

std::string Join(const std::string& dir, const std::string& name) { return dir + "/" + name; }

// One pinbus connection: a request id counter, and GPIO events that arrived
// while a request waited for its reply.
struct PinBusConnection {
  int fd = -1;
  std::uint32_t next_id = 1;
  std::mutex mutex;
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
bool Request(PinBusConnection* c, wire::Message request, wire::Message* reply) {
  std::lock_guard<std::mutex> l(c->mutex);
  request.request_id = c->next_id++;
  if (!wire::Send(c->fd, request)) return false;
  while (true) {
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

int hwemu_can_open(const char* ifname) {
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return -1;
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    return -1;
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
  return fd;
}

int hwemu_can_send(int fd, uint32_t id, int extended, const uint8_t* data, uint8_t len) {
  can_frame f{};
  f.can_id = extended != 0 ? ((id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
  f.can_dlc = len > 8 ? 8 : len;
  if (f.can_dlc > 0) std::memcpy(f.data, data, f.can_dlc);
  return ::write(fd, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f)) ? 0 : -1;
}

int hwemu_can_recv(int fd, uint32_t* id, int* extended, uint8_t* data, uint8_t* len) {
  can_frame f{};
  const ssize_t n = ::read(fd, &f, sizeof(f));
  if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
  if (n != static_cast<ssize_t>(sizeof(f))) return 0;
  *extended = (f.can_id & CAN_EFF_FLAG) != 0 ? 1 : 0;
  *id = f.can_id & (*extended != 0 ? CAN_EFF_MASK : CAN_SFF_MASK);
  *len = f.can_dlc;
  std::memcpy(data, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
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
  wire::Message reply;
  if (!Request(c, {0x03, 0, 0, pin, {static_cast<std::uint8_t>(level != 0 ? 1 : 0)}}, &reply)) return -1;
  return reply.status == 0 ? 0 : -1;
}

uint64_t hwemu_clock_micros(void) {
  static std::mutex m;
  static ClockShm shm;
  static const auto start = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> l(m);
    if (shm.base == nullptr && !shm.tried) {
      shm.tried = true;
      const char* env = std::getenv("HWEMU_CLOCK_SHM");
      const std::string name = std::string("/") + (env != nullptr ? env : "hwemu_clock");
      const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
      if (fd >= 0) {
        void* p = ::mmap(nullptr, 64, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (p != MAP_FAILED && std::memcmp(p, "HWEMUCK1", 8) == 0) shm.base = static_cast<const std::uint8_t*>(p);
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
  while (true) {
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
