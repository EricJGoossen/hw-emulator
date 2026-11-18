#pragma once

// Little- and big-endian field access on byte buffers: every codec here
// reads and writes wire fields through these, never through a reinterpret
// cast, so nothing depends on the host's byte order or alignment.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace hwemu::protocol {

inline void PutU16Le(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v & 0xFFU);
  p[1] = static_cast<std::uint8_t>(v >> 8U);
}
inline void PutU16Be(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v >> 8U);
  p[1] = static_cast<std::uint8_t>(v & 0xFFU);
}
inline void PutU32Le(std::uint8_t* p, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU);
  }
}
inline void PutU32Be(std::uint8_t* p, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p[i] = static_cast<std::uint8_t>((v >> (8 * (3 - i))) & 0xFFU);
  }
}
inline void PutU64Le(std::uint8_t* p, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    p[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU);
  }
}
inline void PutI16Le(std::uint8_t* p, std::int16_t v) {
  PutU16Le(p, static_cast<std::uint16_t>(v));
}
inline void PutI16Be(std::uint8_t* p, std::int16_t v) {
  PutU16Be(p, static_cast<std::uint16_t>(v));
}
inline void PutI32Be(std::uint8_t* p, std::int32_t v) {
  PutU32Be(p, static_cast<std::uint32_t>(v));
}
inline void PutF32Le(std::uint8_t* p, float v) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  PutU32Le(p, bits);
}

inline std::uint16_t GetU16Le(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8U));
}
inline std::uint16_t GetU16Be(const std::uint8_t* p) {
  return static_cast<std::uint16_t>((p[0] << 8U) | p[1]);
}
inline std::uint32_t GetU32Le(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (int i = 3; i >= 0; --i) {
    v = (v << 8U) | p[i];
  }
  return v;
}
inline std::uint32_t GetU32Be(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    v = (v << 8U) | p[i];
  }
  return v;
}
inline std::uint64_t GetU64Le(const std::uint8_t* p) {
  std::uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8U) | p[i];
  }
  return v;
}
inline std::int16_t GetI16Le(const std::uint8_t* p) {
  return static_cast<std::int16_t>(GetU16Le(p));
}
inline std::int16_t GetI16Be(const std::uint8_t* p) {
  return static_cast<std::int16_t>(GetU16Be(p));
}
inline std::int32_t GetI32Be(const std::uint8_t* p) {
  return static_cast<std::int32_t>(GetU32Be(p));
}
inline float GetF32Le(const std::uint8_t* p) {
  const std::uint32_t bits = GetU32Le(p);
  float v = 0.0F;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

// Appends fields to a growing little-endian buffer, the way struct.pack("<...")
// builds one.
class LeWriter {
 public:
  LeWriter& U8(std::uint8_t v) {
    bytes_.push_back(v);
    return *this;
  }
  LeWriter& U16(std::uint16_t v) {
    Grow(2);
    PutU16Le(End(2), v);
    return *this;
  }
  LeWriter& U32(std::uint32_t v) {
    Grow(4);
    PutU32Le(End(4), v);
    return *this;
  }
  LeWriter& U64(std::uint64_t v) {
    Grow(8);
    PutU64Le(End(8), v);
    return *this;
  }
  LeWriter& F32(float v) {
    Grow(4);
    PutF32Le(End(4), v);
    return *this;
  }
  LeWriter& Bytes(std::span<const std::uint8_t> v) {
    bytes_.insert(bytes_.end(), v.begin(), v.end());
    return *this;
  }
  std::vector<std::uint8_t> Take() { return std::move(bytes_); }

 private:
  void Grow(std::size_t n) { bytes_.resize(bytes_.size() + n); }
  std::uint8_t* End(std::size_t n) { return bytes_.data() + bytes_.size() - n; }
  std::vector<std::uint8_t> bytes_;
};

// Reads fields off a little-endian buffer; Ok() turns false (and every read
// returns 0) once a read runs past the end.
class LeReader {
 public:
  explicit LeReader(std::span<const std::uint8_t> bytes) : bytes_(bytes) {}
  std::uint8_t U8() { return Need(1) ? bytes_[pos_++] : 0; }
  std::uint16_t U16() { return Need(2) ? Advance(GetU16Le(At()), 2) : 0; }
  std::uint32_t U32() { return Need(4) ? Advance(GetU32Le(At()), 4) : 0; }
  std::uint64_t U64() { return Need(8) ? Advance(GetU64Le(At()), 8) : 0; }
  float F32() { return Need(4) ? Advance(GetF32Le(At()), 4) : 0.0F; }
  std::span<const std::uint8_t> Bytes(std::size_t n) {
    if (!Need(n)) {
      return {};
    }
    auto out = bytes_.subspan(pos_, n);
    pos_ += n;
    return out;
  }
  bool Ok() const { return ok_; }
  std::size_t Remaining() const { return bytes_.size() - pos_; }

 private:
  bool Need(std::size_t n) {
    if (!ok_ || pos_ + n > bytes_.size()) {
      ok_ = false;
      return false;
    }
    return true;
  }
  const std::uint8_t* At() const { return bytes_.data() + pos_; }
  template <typename T>
  T Advance(T v, std::size_t n) {
    pos_ += n;
    return v;
  }
  std::span<const std::uint8_t> bytes_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

}  // namespace hwemu::protocol
