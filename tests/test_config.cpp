// sim.yaml keys the hub reads that no backend test covers: the window draw
// period and a window's `fly`.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "hwemu/config/config.hpp"

namespace {

std::string WriteSimYaml(const std::string& name, const std::string& body) {
  const auto dir = std::filesystem::temp_directory_path() / ("hwemu_config_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  const auto path = dir / "sim.yaml";
  std::ofstream(path) << "backend: fake\ndisplay: headless\n" << body;
  return path.string();
}

}  // namespace

TEST(HubConfig, WindowsDefaultToNotFlying) {
  const auto config = hwemu::config::LoadHubConfig(WriteSimYaml("nofly", "windows:\n  - {sensor: cam}\n"), "");
  ASSERT_EQ(config.windows.size(), 1U);
  EXPECT_FALSE(config.windows[0].fly);
  EXPECT_EQ(config.windows[0].title, "cam");
}

TEST(HubConfig, AWindowCanFlyItsCamera) {
  const auto config =
      hwemu::config::LoadHubConfig(WriteSimYaml("fly", "windows:\n  - {sensor: cam, view: image, fly: true}\n"), "");
  ASSERT_EQ(config.windows.size(), 1U);
  EXPECT_TRUE(config.windows[0].fly);
}

TEST(HubConfig, DrawPeriodDefaultsToSixtyHertzAndCanBeSet) {
  EXPECT_DOUBLE_EQ(hwemu::config::LoadHubConfig(WriteSimYaml("draw0", ""), "").draw_period, 1.0 / 60.0);
  EXPECT_DOUBLE_EQ(hwemu::config::LoadHubConfig(WriteSimYaml("draw1", "draw_period: 0.05\n"), "").draw_period, 0.05);
}
