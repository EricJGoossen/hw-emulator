#pragma once

// The one message format both custom Unix-socket protocols here share --
// pinbus (SPI and GPIO for the hosted MCB) and lockstep (the sim clock) --
// one message per SOCK_SEQPACKET packet, little-endian:
//
//   u8 type | u8 status | u16 name length | u32 request id | u32 payload length
//   | name | payload
//
// Header-only, so the client library (client/) shares it without linking
// the hub.

#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace hwemu::transport::wire {

inline constexpr std::size_t kHeaderSize = 12;
inline constexpr std::size_t kMaxMessage = 65536;

struct Message {
  std::uint8_t type = 0;
  std::uint8_t status = 0;
  std::uint32_t request_id = 0;
  std::string name;
  std::vector<std::uint8_t> payload;
};

inline std::vector<std::uint8_t> Pack(const Message& m) {
  std::vector<std::uint8_t> out(kHeaderSize + m.name.size() + m.payload.size());
  out[0] = m.type;
  out[1] = m.status;
  const auto nl = static_cast<std::uint16_t>(m.name.size());
  out[2] = static_cast<std::uint8_t>(nl & 0xFF);
  out[3] = static_cast<std::uint8_t>(nl >> 8);
  for (int i = 0; i < 4; ++i) out[4 + i] = static_cast<std::uint8_t>((m.request_id >> (8 * i)) & 0xFF);
  const auto pl = static_cast<std::uint32_t>(m.payload.size());
  for (int i = 0; i < 4; ++i) out[8 + i] = static_cast<std::uint8_t>((pl >> (8 * i)) & 0xFF);
  std::memcpy(out.data() + kHeaderSize, m.name.data(), m.name.size());
  if (!m.payload.empty()) {
    std::memcpy(out.data() + kHeaderSize + m.name.size(), m.payload.data(), m.payload.size());
  }
  return out;
}

inline std::optional<Message> Unpack(std::span<const std::uint8_t> b) {
  if (b.size() < kHeaderSize) return std::nullopt;
  Message m;
  m.type = b[0];
  m.status = b[1];
  const std::size_t nl = b[2] | (b[3] << 8);
  m.request_id = b[4] | (b[5] << 8) | (b[6] << 16) | (static_cast<std::uint32_t>(b[7]) << 24);
  const std::size_t pl = b[8] | (b[9] << 8) | (b[10] << 16) | (static_cast<std::size_t>(b[11]) << 24);
  if (kHeaderSize + nl + pl != b.size()) return std::nullopt;
  m.name.assign(reinterpret_cast<const char*>(b.data() + kHeaderSize), nl);
  m.payload.assign(b.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + nl), b.end());
  return m;
}

inline bool Send(int fd, const Message& m) {
  const auto bytes = Pack(m);
  return ::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(bytes.size());
}

// Blocking unless the socket is non-blocking; nullopt on EOF, error or a
// malformed packet.
inline std::optional<Message> Receive(int fd) {
  std::vector<std::uint8_t> buf(kMaxMessage);
  const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
  if (n <= 0) return std::nullopt;
  return Unpack(std::span(buf).first(static_cast<std::size_t>(n)));
}

inline int Connect(const std::string& path) {
  const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (fd < 0) return -1;
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

// A listening socket at `path` (any stale one removed first); -1 on failure.
inline int Listen(const std::string& path) {
  const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
  if (fd < 0) return -1;
  ::unlink(path.c_str());
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(fd, 16) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace hwemu::transport::wire
