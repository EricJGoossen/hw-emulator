#pragma once

// DJI serial framing, as the MCB's DJISerial and project-otto's Transceiver
// both use it on the vision link (and the referee system uses it on its own):
//
//   0xA5 | data length (u16 LE) | seq (u8) | CRC8(first 4 bytes)
//        | cmd id (u16 LE) | data | CRC16(everything before it, LE)
//
// CRC8 starts at 0xFF and CRC16 at 0xFFFF (see crc.hpp). The parser is
// streaming: bytes go in as they arrive, whole frames come out, and a bad
// header or checksum costs one byte of resync, the way both ends recover.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <vector>

namespace hwemu::protocol {

inline constexpr std::uint8_t kDjiSof = 0xA5;
inline constexpr std::size_t kDjiHeaderSize = 5;  // SOF, len, seq, crc8
inline constexpr std::size_t kDjiCmdSize = 2;
inline constexpr std::size_t kDjiCrc16Size = 2;
inline constexpr std::size_t kDjiOverhead = kDjiHeaderSize + kDjiCmdSize + kDjiCrc16Size;

struct DjiFrame {
  std::uint8_t seq = 0;
  std::uint16_t cmd = 0;
  std::vector<std::uint8_t> data;
};

std::vector<std::uint8_t> EncodeDjiFrame(
    std::uint16_t cmd, std::span<const std::uint8_t> data, std::uint8_t seq = 0
);

class DjiFrameParser {
 public:
  struct Stats {
    std::uint64_t frames = 0;
    std::uint64_t header_crc_errors = 0;
    std::uint64_t frame_crc_errors = 0;
    std::uint64_t discarded_bytes = 0;
  };

  // Frames longer than this (data bytes) are treated as a bad header.
  explicit DjiFrameParser(std::size_t max_data = 4096) : max_data_(max_data) {}

  void Feed(std::span<const std::uint8_t> bytes);
  // The next complete, checked frame, if one is buffered.
  std::optional<DjiFrame> Next();
  const Stats& GetStats() const { return stats_; }
  std::size_t Buffered() const { return buffer_.size(); }

 private:
  void DropOne();

  std::deque<std::uint8_t> buffer_;
  std::size_t max_data_;
  Stats stats_;
};

}  // namespace hwemu::protocol
