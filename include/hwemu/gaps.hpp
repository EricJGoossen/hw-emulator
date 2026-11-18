#pragma once

// Sim gaps: everything a run touches that the sim doesn't model -- a stub, a
// CAN id nothing consumes, an SPI transfer with no chip selected, a PWM pin
// nothing reads. Never silent: each gap is printed once, when it first
// happens, as
//
//   [hwemu GAP] <source>: <message>
//
// and appended once to <runtime_dir>/gaps/<process>.jsonl as
//
//   {"process": ..., "source": ..., "key": ..., "message": ..., "time": <sim s>}
//
// Every process of a run writes its own file (the hub here; the hosted MCB
// and otto through client/hwemu_client.h and python/hwemu/gaps.py), and a
// run's report is all of them together. `key` identifies a gap for
// deduplication and for known-gap lists; `message` is for people.

#include <string>
#include <string_view>
#include <vector>

namespace hwemu::gaps {

struct Gap {
  std::string source, key, message;
  double time = 0.0;
};

// Where reports go: <runtime_dir>/gaps/<process>.jsonl. Until called,
// reports are printed but not written.
void Configure(const std::string& runtime_dir, const std::string& process = "hub");
// Sim time stamped on reports from now on.
void SetTime(double seconds);
// Records a gap: printed and written the first time `source`/`key` is seen.
void Report(std::string_view source, std::string_view key, std::string_view message);
// Every gap recorded so far, in order.
std::vector<Gap> All();
// Forgets everything (tests).
void Reset();

}  // namespace hwemu::gaps
