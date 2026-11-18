#include "hwemu/protocol/dji_serial.hpp"

#include <algorithm>
#include <array>

#include "hwemu/protocol/bytes.hpp"
#include "hwemu/protocol/crc.hpp"

namespace hwemu::protocol {

std::vector<std::uint8_t> EncodeDjiFrame(
    std::uint16_t cmd, std::span<const std::uint8_t> data, std::uint8_t seq
) {
  std::vector<std::uint8_t> out(kDjiOverhead + data.size());
  out[0] = kDjiSof;
  PutU16Le(&out[1], static_cast<std::uint16_t>(data.size()));
  out[3] = seq;
  out[4] = Crc8Dji(std::span(out).first(4));
  PutU16Le(&out[5], cmd);
  std::copy(data.begin(), data.end(), out.begin() + kDjiHeaderSize + kDjiCmdSize);
  const std::size_t body_end = out.size() - kDjiCrc16Size;
  PutU16Le(&out[body_end], Crc16Dji(std::span(out).first(body_end)));
  return out;
}

void DjiFrameParser::Feed(std::span<const std::uint8_t> bytes) {
  buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
}

void DjiFrameParser::DropOne() {
  buffer_.pop_front();
  ++stats_.discarded_bytes;
}

std::optional<DjiFrame> DjiFrameParser::Next() {
  while (!buffer_.empty()) {
    if (buffer_.front() != kDjiSof) {
      DropOne();
      continue;
    }
    if (buffer_.size() < kDjiHeaderSize) {
      return std::nullopt;
    }
    std::array<std::uint8_t, kDjiHeaderSize> header{};
    std::copy_n(buffer_.begin(), kDjiHeaderSize, header.begin());
    if (Crc8Dji(std::span(header).first(4)) != header[4]) {
      ++stats_.header_crc_errors;
      DropOne();
      continue;
    }
    const std::size_t length = GetU16Le(&header[1]);
    if (length > max_data_) {
      ++stats_.header_crc_errors;
      DropOne();
      continue;
    }
    const std::size_t total = kDjiOverhead + length;
    if (buffer_.size() < total) {
      return std::nullopt;
    }
    std::vector<std::uint8_t> frame(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    const std::size_t body_end = total - kDjiCrc16Size;
    if (Crc16Dji(std::span(frame).first(body_end)) != GetU16Le(&frame[body_end])) {
      ++stats_.frame_crc_errors;
      DropOne();
      continue;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    ++stats_.frames;
    DjiFrame out;
    out.seq = frame[3];
    out.cmd = GetU16Le(&frame[5]);
    out.data.assign(frame.begin() + kDjiHeaderSize + kDjiCmdSize, frame.begin() + static_cast<std::ptrdiff_t>(body_end));
    return out;
  }
  return std::nullopt;
}

}  // namespace hwemu::protocol
