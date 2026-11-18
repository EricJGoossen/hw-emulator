#pragma once

// Golden byte strings, generated from project-otto's own code by
// tools/make_golden.py.

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

inline std::vector<std::uint8_t> Golden(const std::string& name) {
  std::ifstream in(std::string(HWEMU_GOLDEN_DIR) + "/" + name + ".hex");
  if (!in) throw std::runtime_error("no golden file " + name);
  std::string hex;
  in >> hex;
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}
