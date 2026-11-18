#include "hwemu/protocol/ld19.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

#include "hwemu/protocol/bytes.hpp"
#include "hwemu/protocol/crc.hpp"

namespace hwemu::protocol::ld19 {

std::array<std::uint8_t, kPacketSize> Encode(const Packet& p) {
  std::array<std::uint8_t, kPacketSize> out{};
  out[0] = kHeader;
  out[1] = kVersion;
  PutU16Le(&out[2], p.speed_dps);
  PutU16Le(&out[4], p.start_angle_cdeg);
  std::size_t at = 6;
  for (const Point& pt : p.points) {
    PutU16Le(&out[at], pt.distance_mm);
    out[at + 2] = pt.intensity;
    at += 3;
  }
  PutU16Le(&out[at], p.end_angle_cdeg);
  PutU16Le(&out[at + 2], p.timestamp_ms);
  out[kPacketSize - 1] = Crc8Ld19(std::span(out).first(kPacketSize - 1));
  return out;
}

std::optional<Packet> Decode(std::span<const std::uint8_t> b) {
  if (b.size() < kPacketSize || b[0] != kHeader || b[1] != kVersion) {
    return std::nullopt;
  }
  if (Crc8Ld19(b.first(kPacketSize - 1)) != b[kPacketSize - 1]) {
    return std::nullopt;
  }
  Packet p;
  p.speed_dps = GetU16Le(&b[2]);
  p.start_angle_cdeg = GetU16Le(&b[4]);
  std::size_t at = 6;
  for (Point& pt : p.points) {
    pt.distance_mm = GetU16Le(&b[at]);
    pt.intensity = b[at + 2];
    at += 3;
  }
  p.end_angle_cdeg = GetU16Le(&b[at]);
  p.timestamp_ms = GetU16Le(&b[at + 2]);
  return p;
}

std::vector<Packet> PacketsForScan(
    std::span<const float> ranges,
    std::span<const float> angles,
    double scan_rate_hz,
    std::uint64_t time_ms,
    std::uint8_t intensity,
    double min_range,
    double max_range
) {
  const std::size_t n = std::min(ranges.size(), angles.size());
  std::vector<double> cw(n);
  for (std::size_t i = 0; i < n; ++i) {
    double deg = -static_cast<double>(angles[i]) * 180.0 / std::numbers::pi;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) {
      deg += 360.0;
    }
    cw[i] = deg;
  }
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return cw[a] < cw[b]; });

  const auto speed = static_cast<std::uint16_t>(std::lround(std::clamp(scan_rate_hz * 360.0, 0.0, 65535.0)));
  const auto stamp = static_cast<std::uint16_t>(time_ms % kTimestampRolloverMs);
  auto cdeg = [](double deg) {
    return static_cast<std::uint16_t>(static_cast<long>(std::lround(deg * 100.0)) % 36000);
  };

  std::vector<Packet> packets;
  for (std::size_t first = 0; first + kPointsPerPacket <= n; first += kPointsPerPacket) {
    Packet p;
    p.speed_dps = speed;
    p.timestamp_ms = stamp;
    p.start_angle_cdeg = cdeg(cw[order[first]]);
    p.end_angle_cdeg = cdeg(cw[order[first + kPointsPerPacket - 1]]);
    for (std::size_t k = 0; k < kPointsPerPacket; ++k) {
      const float r = ranges[order[first + k]];
      if (std::isfinite(r) && r >= min_range && r <= max_range) {
        p.points[k].distance_mm = static_cast<std::uint16_t>(std::lround(std::min(r * 1000.0, 65535.0)));
        p.points[k].intensity = intensity;
      }
    }
    packets.push_back(p);
  }
  return packets;
}

}  // namespace hwemu::protocol::ld19
