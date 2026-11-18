#include "hwemu/gaps.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>

namespace hwemu::gaps {
namespace {

struct State {
  std::mutex mutex;
  std::string path, process = "hub";
  double time = 0.0;
  std::set<std::string> seen;
  std::vector<Gap> gaps;
};

State& S() {
  static State s;
  return s;
}

std::string Json(std::string_view text) {
  std::string out = "\"";
  for (const char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

}  // namespace

void Configure(const std::string& runtime_dir, const std::string& process) {
  auto& s = S();
  std::lock_guard<std::mutex> l(s.mutex);
  const auto dir = std::filesystem::path(runtime_dir) / "gaps";
  std::filesystem::create_directories(dir);
  s.process = process;
  s.path = (dir / (process + ".jsonl")).string();
  std::ofstream(s.path, std::ios::trunc);  // a fresh report per run
  for (const auto& g : s.gaps) {           // keep anything reported before
    std::ofstream(s.path, std::ios::app) << "{\"process\":" << Json(s.process) << ",\"source\":" << Json(g.source)
                                         << ",\"key\":" << Json(g.key) << ",\"message\":" << Json(g.message)
                                         << ",\"time\":" << g.time << "}\n";
  }
}

void SetTime(double seconds) {
  auto& s = S();
  std::lock_guard<std::mutex> l(s.mutex);
  s.time = seconds;
}

void Report(std::string_view source, std::string_view key, std::string_view message) {
  auto& s = S();
  std::lock_guard<std::mutex> l(s.mutex);
  std::string id(source);
  id += '\x1f';
  id += key;
  if (!s.seen.insert(id).second) return;
  Gap g{std::string(source), std::string(key), std::string(message), s.time};
  std::cerr << "[hwemu GAP] " << g.source << ": " << g.message << std::endl;
  if (!s.path.empty()) {
    std::ofstream(s.path, std::ios::app) << "{\"process\":" << Json(s.process) << ",\"source\":" << Json(g.source)
                                         << ",\"key\":" << Json(g.key) << ",\"message\":" << Json(g.message)
                                         << ",\"time\":" << g.time << "}\n";
  }
  s.gaps.push_back(std::move(g));
}

std::vector<Gap> All() {
  auto& s = S();
  std::lock_guard<std::mutex> l(s.mutex);
  return s.gaps;
}

void Reset() {
  auto& s = S();
  std::lock_guard<std::mutex> l(s.mutex);
  s.seen.clear();
  s.gaps.clear();
}

}  // namespace hwemu::gaps
