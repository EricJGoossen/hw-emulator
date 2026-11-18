#pragma once

// A lock-free ring of frames in POSIX shared memory (/dev/shm/<name>): one
// writer (the hub) and any number of readers (otto's pyrealsense2 shim, a
// viewer), in different processes. Each slot is a seqlock: the writer marks
// it odd while writing and even when done, and a reader keeps a copy only if
// the mark was the same even value before and after it read. A reader that
// falls behind just sees the latest frame; nothing blocks the writer.
//
// Layout, little-endian, so a reader in any language can map it
// (python/hwemu/frame_shm.py does, with nothing compiled):
//
//   header (64 B):  "HWEMUFR1" | u32 version (1) | u32 slot_count
//                   | u64 meta_capacity | u64 payload_capacity
//                   | u64 latest (sequence of the newest whole frame, 0: none)
//   frame s in slot s % slot_count, at 64 + slot * (64 + meta_capacity + payload_capacity):
//                   u64 lock (2s+1 writing frame s, 2s+2 done) | u64 sequence
//                   | u32 meta_size | u32 payload_size | f64 timestamp
//                   | (pad to 64) | meta | payload

#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <vector>

namespace hwemu::transport {

inline constexpr std::size_t kShmHeaderSize = 64;
inline constexpr std::size_t kShmSlotHeaderSize = 64;

class FrameShmWriter {
 public:
  // Creates (or replaces) /dev/shm/<name>; unlinked on destruction.
  FrameShmWriter(const std::string& name, std::uint32_t slots, std::size_t meta_capacity, std::size_t payload_capacity);
  ~FrameShmWriter();
  FrameShmWriter(const FrameShmWriter&) = delete;
  FrameShmWriter& operator=(const FrameShmWriter&) = delete;

  // Publishes one frame: `meta`, then each part of the payload back to back.
  // Returns its sequence number (1, 2, ...). Throws std::length_error past
  // the capacities.
  std::uint64_t Write(
      std::span<const std::uint8_t> meta,
      std::initializer_list<std::span<const std::uint8_t>> payload,
      double timestamp
  );
  const std::string& Name() const { return name_; }

 private:
  std::string name_;
  int fd_ = -1;
  std::uint8_t* base_ = nullptr;
  std::size_t size_ = 0;
  std::uint32_t slots_ = 0;
  std::size_t meta_capacity_ = 0, payload_capacity_ = 0;
  std::uint64_t sequence_ = 0;
};

class FrameShmReader {
 public:
  struct Frame {
    std::uint64_t sequence = 0;
    double timestamp = 0.0;
    std::vector<std::uint8_t> meta;
    std::vector<std::uint8_t> payload;
  };

  // Throws std::runtime_error if /dev/shm/<name> doesn't exist yet.
  explicit FrameShmReader(const std::string& name);
  ~FrameShmReader();
  FrameShmReader(const FrameShmReader&) = delete;
  FrameShmReader& operator=(const FrameShmReader&) = delete;

  std::uint64_t Latest() const;
  // The newest frame newer than `after`, if there is one and it reads whole.
  bool ReadLatest(Frame* out, std::uint64_t after = 0) const;

 private:
  int fd_ = -1;
  std::uint8_t* base_ = nullptr;
  std::size_t size_ = 0;
  std::uint32_t slots_ = 0;
  std::size_t meta_capacity_ = 0, payload_capacity_ = 0;
};

}  // namespace hwemu::transport
