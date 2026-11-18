#pragma once

// One CAN frame on the UDS bus (transport/can_bus.hpp): a 16-byte
// SOCK_SEQPACKET packet, little-endian, shared by the hub and the client
// library (header-only):
//
//   u32 id | flags in its top bits (bit 31 extended, bit 30 rtr)
//   u8 len (0..8) | 3 pad | 8 data

#include <array>
#include <cstdint>
#include <cstring>

namespace hwemu::transport::can_wire {

inline constexpr std::size_t kPacketSize = 16;
inline constexpr std::uint32_t kExtendedFlag = 1U << 31;
inline constexpr std::uint32_t kRtrFlag = 1U << 30;
inline constexpr std::uint32_t kIdMask = 0x1FFFFFFF;

inline std::array<std::uint8_t, kPacketSize> Pack(std::uint32_t id, bool extended, bool rtr,
                                                 const std::uint8_t* data, std::uint8_t len) {
  std::array<std::uint8_t, kPacketSize> p{};
  std::uint32_t word = (id & kIdMask) | (extended ? kExtendedFlag : 0) | (rtr ? kRtrFlag : 0);
  for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(word >> (8 * i));
  p[4] = len > 8 ? 8 : len;
  if (data != nullptr) std::memcpy(p.data() + 8, data, p[4]);
  return p;
}

struct Unpacked {
  std::uint32_t id;
  bool extended, rtr;
  std::uint8_t len;
  std::array<std::uint8_t, 8> data;
};

inline bool Unpack(const std::uint8_t* p, std::size_t n, Unpacked* out) {
  if (n != kPacketSize) return false;
  std::uint32_t word = 0;
  for (int i = 0; i < 4; ++i) word |= static_cast<std::uint32_t>(p[i]) << (8 * i);
  out->id = word & kIdMask;
  out->extended = (word & kExtendedFlag) != 0;
  out->rtr = (word & kRtrFlag) != 0;
  out->len = p[4] > 8 ? 8 : p[4];
  out->data.fill(0);
  std::memcpy(out->data.data(), p + 8, out->len);
  return true;
}

}  // namespace hwemu::transport::can_wire
