#include "hwemu/transport/frame_shm.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "hwemu/protocol/bytes.hpp"

namespace hwemu::transport {
namespace {

constexpr char kMagic[8] = {'H', 'W', 'E', 'M', 'U', 'F', 'R', '1'};

std::string ShmPath(const std::string& name) { return name.empty() || name[0] == '/' ? name : "/" + name; }

std::atomic_ref<std::uint64_t> Atomic(std::uint8_t* p) {
  return std::atomic_ref<std::uint64_t>(*reinterpret_cast<std::uint64_t*>(p));
}

std::size_t SlotStride(std::size_t meta, std::size_t payload) { return kShmSlotHeaderSize + meta + payload; }

}  // namespace

FrameShmWriter::FrameShmWriter(
    const std::string& name, std::uint32_t slots, std::size_t meta_capacity, std::size_t payload_capacity
)
    : name_(ShmPath(name)), slots_(slots), meta_capacity_((meta_capacity + 7) & ~std::size_t{7}),
      payload_capacity_((payload_capacity + 7) & ~std::size_t{7}) {
  if (slots_ == 0) {
    throw std::invalid_argument("FrameShm: needs at least one slot");
  }
  ::shm_unlink(name_.c_str());
  fd_ = ::shm_open(name_.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
  if (fd_ < 0) {
    throw std::runtime_error("FrameShm: shm_open " + name_ + ": " + std::strerror(errno));
  }
  owned_ = OwnedName(OwnedName::ShmPath(name_));
  ::fchmod(fd_, 0666);
  size_ = kShmHeaderSize + (slots_ * SlotStride(meta_capacity_, payload_capacity_));
  if (::ftruncate(fd_, static_cast<off_t>(size_)) != 0) {
    throw std::runtime_error("FrameShm: ftruncate: " + std::string(std::strerror(errno)));
  }
  void* p = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    throw std::runtime_error("FrameShm: mmap: " + std::string(std::strerror(errno)));
  }
  base_ = static_cast<std::uint8_t*>(p);
  std::memset(base_, 0, kShmHeaderSize);
  std::memcpy(base_, kMagic, 8);
  protocol::PutU32Le(base_ + 8, 1);
  protocol::PutU32Le(base_ + 12, slots_);
  protocol::PutU64Le(base_ + 16, meta_capacity_);
  protocol::PutU64Le(base_ + 24, payload_capacity_);
  Atomic(base_ + 32).store(0, std::memory_order_release);
}

FrameShmWriter::~FrameShmWriter() {
  if (base_ != nullptr) ::munmap(base_, size_);
  if (fd_ >= 0) ::close(fd_);
  owned_.Remove();
}

std::uint64_t FrameShmWriter::Write(
    std::span<const std::uint8_t> meta,
    std::initializer_list<std::span<const std::uint8_t>> payload,
    double timestamp
) {
  std::size_t payload_size = 0;
  for (const auto& part : payload) payload_size += part.size();
  if (meta.size() > meta_capacity_ || payload_size > payload_capacity_) {
    throw std::length_error("FrameShm: frame larger than the ring's slots");
  }
  const std::uint64_t seq = ++sequence_;
  std::uint8_t* slot = base_ + kShmHeaderSize + ((seq % slots_) * SlotStride(meta_capacity_, payload_capacity_));
  auto lock = Atomic(slot);
  lock.store((2 * seq) + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_release);
  protocol::PutU64Le(slot + 8, seq);
  protocol::PutU32Le(slot + 16, static_cast<std::uint32_t>(meta.size()));
  protocol::PutU32Le(slot + 20, static_cast<std::uint32_t>(payload_size));
  std::memcpy(slot + 24, &timestamp, sizeof(timestamp));
  std::uint8_t* at = slot + kShmSlotHeaderSize;
  std::memcpy(at, meta.data(), meta.size());
  at += meta_capacity_;
  for (const auto& part : payload) {
    std::memcpy(at, part.data(), part.size());
    at += part.size();
  }
  std::atomic_thread_fence(std::memory_order_release);
  lock.store((2 * seq) + 2, std::memory_order_release);
  Atomic(base_ + 32).store(seq, std::memory_order_release);
  return seq;
}

FrameShmReader::FrameShmReader(const std::string& name) {
  const std::string path = ShmPath(name);
  fd_ = ::shm_open(path.c_str(), O_RDONLY, 0);
  if (fd_ < 0) {
    throw std::runtime_error("FrameShm: no ring " + path + " (is the hub running?)");
  }
  struct stat st {};
  ::fstat(fd_, &st);
  size_ = static_cast<std::size_t>(st.st_size);
  void* p = ::mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    throw std::runtime_error("FrameShm: mmap failed");
  }
  base_ = static_cast<std::uint8_t*>(p);
  if (size_ < kShmHeaderSize || std::memcmp(base_, kMagic, 8) != 0) {
    throw std::runtime_error("FrameShm: " + path + " isn't a frame ring");
  }
  slots_ = protocol::GetU32Le(base_ + 12);
  meta_capacity_ = protocol::GetU64Le(base_ + 16);
  payload_capacity_ = protocol::GetU64Le(base_ + 24);
}

FrameShmReader::~FrameShmReader() {
  if (base_ != nullptr) ::munmap(base_, size_);
  if (fd_ >= 0) ::close(fd_);
}

std::uint64_t FrameShmReader::Latest() const {
  return Atomic(base_ + 32).load(std::memory_order_acquire);
}

bool FrameShmReader::ReadLatest(Frame* out, std::uint64_t after) const {
  for (int attempt = 0; attempt < 8; ++attempt) {
    const std::uint64_t seq = Latest();
    if (seq == 0 || seq <= after) {
      return false;
    }
    std::uint8_t* slot = base_ + kShmHeaderSize + ((seq % slots_) * SlotStride(meta_capacity_, payload_capacity_));
    auto lock = Atomic(slot);
    const std::uint64_t before = lock.load(std::memory_order_acquire);
    if (before != (2 * seq) + 2) {
      continue;
    }
    const std::uint32_t meta_size = protocol::GetU32Le(slot + 16);
    const std::uint32_t payload_size = protocol::GetU32Le(slot + 20);
    if (meta_size > meta_capacity_ || payload_size > payload_capacity_) {
      continue;
    }
    std::memcpy(&out->timestamp, slot + 24, sizeof(double));
    const std::uint8_t* at = slot + kShmSlotHeaderSize;
    out->meta.assign(at, at + meta_size);
    at += meta_capacity_;
    out->payload.assign(at, at + payload_size);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (lock.load(std::memory_order_acquire) == before) {
      out->sequence = seq;
      return true;
    }
  }
  return false;
}

}  // namespace hwemu::transport
