#pragma once

// The LD19 lidar's 47-byte measurement packet, as project-otto's LD19LiDAR
// parses it (little-endian):
//
//   0x54 | 0x2C | speed (u16, deg/s) | start angle (u16, 0.01 deg)
//   | 12 x (distance u16 mm, intensity u8) | end angle (u16, 0.01 deg)
//   | timestamp (u16 ms, wraps at 30000) | CRC8 (poly 0x4D, init 0)
//
// Angles grow clockwise seen from above (the LD19 spins clockwise): a beam
// at counterclockwise angle a in the sensor frame is at (-a mod 360) here.

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace hwemu::protocol::ld19 {

inline constexpr std::uint8_t kHeader = 0x54;
inline constexpr std::uint8_t kVersion = 0x2C;
inline constexpr std::size_t kPointsPerPacket = 12;
inline constexpr std::size_t kPacketSize = 47;
inline constexpr std::uint16_t kTimestampRolloverMs = 30000;
inline constexpr int kBaudRate = 230400;

struct Point {
  std::uint16_t distance_mm = 0;
  std::uint8_t intensity = 0;
  friend bool operator==(const Point&, const Point&) = default;
};

struct Packet {
  std::uint16_t speed_dps = 0;
  std::uint16_t start_angle_cdeg = 0;
  std::array<Point, kPointsPerPacket> points{};
  std::uint16_t end_angle_cdeg = 0;
  std::uint16_t timestamp_ms = 0;
  friend bool operator==(const Packet&, const Packet&) = default;
};

std::array<std::uint8_t, kPacketSize> Encode(const Packet& packet);
std::optional<Packet> Decode(std::span<const std::uint8_t> bytes);

// One full scan as packets: `ranges` (m; inf or <= 0 for no return) at
// counterclockwise sensor-frame `angles` (rad), sorted into the LD19's
// clockwise order and cut into packets of 12. A range under `min_range` or
// over `max_range` is sent as 0 mm, intensity 0, as the LD19 reports a miss.
std::vector<Packet> PacketsForScan(
    std::span<const float> ranges,
    std::span<const float> angles,
    double scan_rate_hz,
    std::uint64_t time_ms,
    std::uint8_t intensity = 200,
    double min_range = 0.02,
    double max_range = 12.0
);

}  // namespace hwemu::protocol::ld19
