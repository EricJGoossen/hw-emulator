#include "hwemu/clock/sim_clock.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "hwemu/protocol/bytes.hpp"
#include "hwemu/transport/uds_wire.hpp"

namespace hwemu::clock {
namespace wire = transport::wire;

namespace lockstep {
enum Type : std::uint8_t { kJoin = 0x01, kTick = 0x02, kAck = 0x03, kLeave = 0x04, kWelcome = 0x81 };
}

ClockMode ParseClockMode(const std::string& name) {
  if (name == "realtime") return ClockMode::kRealtime;
  if (name == "lockstep") return ClockMode::kLockstep;
  throw std::invalid_argument("clock mode must be realtime or lockstep, not '" + name + "'");
}

LockstepOrder ParseLockstepOrder(const std::string& name) {
  if (name == "broadcast") return LockstepOrder::kBroadcast;
  if (name == "sequential") return LockstepOrder::kSequential;
  throw std::invalid_argument("lockstep order must be broadcast or sequential, not '" + name + "'");
}

struct SimClock::Impl {
  struct Participant {
    int fd = -1;
    std::string name;
  };

  // Shared-memory publication.
  int shm_fd = -1;
  std::uint8_t* shm = nullptr;
  std::string shm_name;

  // Wall pacing.
  std::chrono::steady_clock::time_point start;

  // Lockstep.
  int listen_fd = -1;
  std::vector<Participant> participants;  // in tick order
  std::atomic<bool> stop{false};
  mutable std::mutex mutex;

  void Publish(const SimTime& t, std::uint64_t dt_us) {
    if (shm == nullptr) return;
    auto lock = std::atomic_ref<std::uint64_t>(*reinterpret_cast<std::uint64_t*>(shm + 40));
    const std::uint64_t v = lock.load();
    lock.store(v + 1, std::memory_order_release);
    protocol::PutU64Le(shm + 8, t.micros);
    protocol::PutU64Le(shm + 16, t.tick);
    std::memcpy(shm + 24, &t.seconds, sizeof(double));
    protocol::PutU64Le(shm + 32, dt_us);
    lock.store(v + 2, std::memory_order_release);
  }

  static std::vector<std::uint8_t> TickPayload(const SimTime& t, std::uint64_t dt_us) {
    std::vector<std::uint8_t> p(24);
    protocol::PutU64Le(p.data(), t.tick);
    protocol::PutU64Le(p.data() + 8, t.micros);
    protocol::PutU64Le(p.data() + 16, dt_us);
    return p;
  }

  // Accepts pending joins (non-blocking unless `block_ms` > 0).
  void AcceptJoins(const SimTime& now, std::uint64_t dt_us, const std::vector<std::string>& expected, int block_ms) {
    if (listen_fd < 0) return;
    pollfd pfd{listen_fd, POLLIN, 0};
    while (::poll(&pfd, 1, block_ms) > 0 && (pfd.revents & POLLIN) != 0) {
      const int fd = ::accept4(listen_fd, nullptr, nullptr, SOCK_CLOEXEC);
      if (fd < 0) break;
      pollfd cfd{fd, POLLIN, 0};
      std::optional<wire::Message> join;
      if (::poll(&cfd, 1, 2000) > 0) join = wire::Receive(fd);
      if (!join || join->type != lockstep::kJoin) {
        ::close(fd);
        continue;
      }
      wire::Send(fd, {lockstep::kWelcome, 0, 0, join->name, TickPayload(now, dt_us)});
      std::lock_guard<std::mutex> l(mutex);
      participants.push_back({fd, join->name});
      // Expected participants tick in the order they're listed.
      if (!expected.empty()) {
        std::stable_sort(participants.begin(), participants.end(), [&](const auto& a, const auto& b) {
          auto rank = [&](const std::string& n) {
            auto it = std::find(expected.begin(), expected.end(), n);
            return it == expected.end() ? expected.size() : static_cast<std::size_t>(it - expected.begin());
          };
          return rank(a.name) < rank(b.name);
        });
      }
      std::cerr << "[clock] lockstep participant joined: " << join->name << "\n";
      block_ms = 0;
    }
  }

  bool AllExpectedJoined(const std::vector<std::string>& expected) const {
    std::lock_guard<std::mutex> l(mutex);
    for (const auto& name : expected) {
      if (std::none_of(participants.begin(), participants.end(), [&](const auto& p) { return p.name == name; })) {
        return false;
      }
    }
    return true;
  }

  // Waits for participant `p` to ack `tick`; false (and drops it) on timeout
  // or hang-up.
  bool AwaitAck(Participant& p, std::uint64_t tick, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (!stop) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
      if (left.count() <= 0) {
        std::cerr << "[clock] participant " << p.name << " missed tick " << tick << "; dropping it\n";
        return false;
      }
      pollfd pfd{p.fd, POLLIN, 0};
      if (::poll(&pfd, 1, static_cast<int>(std::min<long>(left.count(), 100))) <= 0) continue;
      auto m = wire::Receive(p.fd);
      if (!m || m->type == lockstep::kLeave) {
        std::cerr << "[clock] participant " << p.name << " left\n";
        return false;
      }
      if (m->type == lockstep::kAck && m->payload.size() >= 8 && protocol::GetU64Le(m->payload.data()) >= tick) {
        return true;
      }
    }
    return false;
  }
};

SimClock::SimClock(const ClockOptions& options) : options_(options), impl_(std::make_unique<Impl>()) {
  if (!(options_.dt > 0.0)) {
    throw std::invalid_argument("clock dt must be positive");
  }
  now_.dt = options_.dt;
  impl_->start = std::chrono::steady_clock::now();
  if (!options_.shm_name.empty()) {
    impl_->shm_name = "/" + options_.shm_name;
    ::shm_unlink(impl_->shm_name.c_str());
    impl_->shm_fd = ::shm_open(impl_->shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (impl_->shm_fd >= 0 && ::ftruncate(impl_->shm_fd, 64) == 0) {
      ::fchmod(impl_->shm_fd, 0666);
      void* p = ::mmap(nullptr, 64, PROT_READ | PROT_WRITE, MAP_SHARED, impl_->shm_fd, 0);
      if (p != MAP_FAILED) {
        impl_->shm = static_cast<std::uint8_t*>(p);
        std::memset(impl_->shm, 0, 64);
        std::memcpy(impl_->shm, "HWEMUCK1", 8);
      }
    }
  }
  if (options_.mode == ClockMode::kLockstep && !options_.lockstep_socket.empty()) {
    const std::filesystem::path p(options_.lockstep_socket);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    impl_->listen_fd = wire::Listen(options_.lockstep_socket);
    if (impl_->listen_fd < 0) {
      throw std::runtime_error("lockstep: can't listen at " + options_.lockstep_socket);
    }
  }
  impl_->Publish(now_, static_cast<std::uint64_t>(std::llround(options_.dt * 1e6)));
}

SimClock::~SimClock() {
  Stop();
  for (auto& p : impl_->participants) ::close(p.fd);
  if (impl_->listen_fd >= 0) {
    ::close(impl_->listen_fd);
    ::unlink(options_.lockstep_socket.c_str());
  }
  if (impl_->shm != nullptr) ::munmap(impl_->shm, 64);
  if (impl_->shm_fd >= 0) {
    ::close(impl_->shm_fd);
    ::shm_unlink(impl_->shm_name.c_str());
  }
}

void SimClock::Stop() { impl_->stop = true; }

std::vector<std::string> SimClock::Participants() const {
  std::lock_guard<std::mutex> l(impl_->mutex);
  std::vector<std::string> out;
  for (const auto& p : impl_->participants) out.push_back(p.name);
  return out;
}

bool SimClock::WaitForNextTick() {
  const auto dt_us = static_cast<std::uint64_t>(std::llround(options_.dt * 1e6));
  if (options_.mode == ClockMode::kRealtime) {
    const double target = (now_.seconds + options_.dt) / std::max(options_.rate, 1e-9);
    const auto when = impl_->start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                         std::chrono::duration<double>(target));
    // If far behind (a stall), re-anchor rather than burst to catch up.
    const auto now = std::chrono::steady_clock::now();
    if (now - when > std::chrono::milliseconds(250)) {
      impl_->start = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(now_.seconds / std::max(options_.rate, 1e-9)));
      return !impl_->stop;
    }
    std::this_thread::sleep_until(when);
    return !impl_->stop;
  }

  // Lockstep: first, everyone expected must be here.
  impl_->AcceptJoins(now_, dt_us, options_.expected, 0);
  while (!impl_->stop && !impl_->AllExpectedJoined(options_.expected)) {
    impl_->AcceptJoins(now_, dt_us, options_.expected, 200);
  }
  if (impl_->stop) return false;
  // Ticks after the first: everyone acks the tick they were handed.
  if (now_.tick > 0) {
    std::lock_guard<std::mutex> l(impl_->mutex);
    auto& ps = impl_->participants;
    for (auto it = ps.begin(); it != ps.end();) {
      // In sequential order, Advance() already waited on all but the last.
      if (options_.order == LockstepOrder::kSequential && std::next(it) != ps.end()) {
        ++it;
        continue;
      }
      if (!impl_->AwaitAck(*it, now_.tick, options_.ack_timeout)) {
        ::close(it->fd);
        it = ps.erase(it);
      } else {
        ++it;
      }
    }
  }
  return !impl_->stop;
}

void SimClock::Advance() {
  ++now_.tick;
  now_.seconds = static_cast<double>(now_.tick) * options_.dt;
  now_.micros = static_cast<std::uint64_t>(std::llround(now_.seconds * 1e6));
  const auto dt_us = static_cast<std::uint64_t>(std::llround(options_.dt * 1e6));
  impl_->Publish(now_, dt_us);
  if (options_.mode != ClockMode::kLockstep) return;
  std::lock_guard<std::mutex> l(impl_->mutex);
  auto& ps = impl_->participants;
  const auto payload = Impl::TickPayload(now_, dt_us);
  for (auto it = ps.begin(); it != ps.end();) {
    if (!wire::Send(it->fd, {lockstep::kTick, 0, 0, "", payload})) {
      ::close(it->fd);
      it = ps.erase(it);
      continue;
    }
    // Sequential: each in turn -- wait for this one before handing the next
    // its tick (the last one's ack is collected in WaitForNextTick).
    if (options_.order == LockstepOrder::kSequential && std::next(it) != ps.end()) {
      if (!impl_->AwaitAck(*it, now_.tick, options_.ack_timeout)) {
        ::close(it->fd);
        it = ps.erase(it);
        continue;
      }
    }
    ++it;
  }
}

}  // namespace hwemu::clock
