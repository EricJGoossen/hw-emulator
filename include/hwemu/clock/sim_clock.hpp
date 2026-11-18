#pragma once

// The hub's clock: every device timestamp comes from it -- including the
// MCB's microsecond counter, which is a 64-bit count of sim microseconds, so
// it is monotonic and never wraps.
//
//   realtime  ticks paced to the wall clock (times `rate`).
//   lockstep  a tick runs only once every participant has acked the last.
//             Participants opt in by connecting to the lockstep socket; with
//             none connected (and none expected), ticks run back to back, as
//             fast as the sim goes.
//
// Lockstep protocol (uds_wire.hpp framing, SOCK_SEQPACKET):
//   participant -> hub  0x01 JOIN  name = participant
//   hub -> participant  0x81 WELCOME payload = u64 tick | u64 micros | u64 dt_us
//   hub -> participant  0x02 TICK  payload = u64 tick | u64 micros | u64 dt_us:
//                       "run up to `micros`, then ack"
//   participant -> hub  0x03 ACK   payload = u64 tick
//   participant -> hub  0x04 LEAVE
// In `broadcast` order every participant gets TICK at once; in `sequential`
// order each in turn, the next only after the last acked -- one process at a
// time, in join order (or `expected` order).
//
// The clock also publishes itself to shared memory (/dev/shm/hwemu_clock by
// default), so anything -- lockstepped or not -- can read sim time:
//   "HWEMUCK1" | u64 micros | u64 tick | f64 seconds | u64 dt_us  (seqlock
//   via an odd/even u64 at offset 40)

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hwemu::clock {

enum class ClockMode { kRealtime, kLockstep };
enum class LockstepOrder { kBroadcast, kSequential };

struct SimTime {
  std::uint64_t tick = 0;
  std::uint64_t micros = 0;
  double seconds = 0.0;
  double dt = 0.0;
};

struct ClockOptions {
  ClockMode mode = ClockMode::kRealtime;
  double dt = 0.001;
  double rate = 1.0;  // realtime only: sim seconds per wall second
  std::string lockstep_socket;  // empty: no lockstep server
  LockstepOrder order = LockstepOrder::kBroadcast;
  std::vector<std::string> expected;  // lockstep waits for these to join first
  double ack_timeout = 5.0;           // s; a participant that misses it is dropped; <= 0: never
  std::string shm_name = "hwemu_clock";  // empty: don't publish
};

class SimClock {
 public:
  explicit SimClock(const ClockOptions& options);
  ~SimClock();
  SimClock(const SimClock&) = delete;
  SimClock& operator=(const SimClock&) = delete;

  // The tick about to run (or that just ran, after Advance()).
  SimTime Now() const { return now_; }
  std::uint64_t Micros() const { return now_.micros; }
  const ClockOptions& Options() const { return options_; }

  // Blocks until the next tick may start: paced to the wall clock, or until
  // every lockstep participant has acked the current one. False if Stop()
  // was called while waiting.
  bool WaitForNextTick();
  // Moves time forward one dt, publishes it, and (lockstep) hands the new
  // tick to the participants.
  void Advance();
  void Stop();

  std::vector<std::string> Participants() const;

  struct Impl;

 private:
  ClockOptions options_;
  SimTime now_;
  std::unique_ptr<Impl> impl_;
};

ClockMode ParseClockMode(const std::string& name);
LockstepOrder ParseLockstepOrder(const std::string& name);

}  // namespace hwemu::clock
