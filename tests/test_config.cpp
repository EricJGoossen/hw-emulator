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

TEST(HubConfig, TruthLogIsOffUnlessAskedAndNamesABodyPerBackend) {
  EXPECT_EQ(hwemu::config::LoadHubConfig(WriteSimYaml("truth0", ""), "").truth_log.period, 0.0);
  const auto config = hwemu::config::LoadHubConfig(
      WriteSimYaml("truth1", "truth_log: {period: 0.02, body: {achilles: sentry/floating_base, mujoco: base_footprint}}\n"),
      "");
  EXPECT_DOUBLE_EQ(config.truth_log.period, 0.02);
  EXPECT_EQ(config.truth_log.achilles, "sentry/floating_base");
  EXPECT_EQ(config.truth_log.mujoco, "base_footprint");
  EXPECT_TRUE(config.truth_log.encoders.empty());
}

TEST(HubConfig, ActorsAreReadWithTheirDefaults) {
  const auto config = hwemu::config::LoadHubConfig(
      WriteSimYaml("actors", "actors:\n"
                             "  - {name: enemy, body: {achilles: enemy/base, mujoco: enemy/base}, path: [[1, 2], [3, 4]],"
                             " speed: 1.5}\n"
                             "mujoco:\n  elements:\n    enemy: {type: attach, file: enemy.urdf}\n"),
      "");
  ASSERT_EQ(config.actors.size(), 1U);
  const auto& a = config.actors[0];
  EXPECT_EQ(a.name, "enemy");
  EXPECT_EQ(a.achilles, "enemy/base");
  ASSERT_EQ(a.path.size(), 2U);
  EXPECT_DOUBLE_EQ(a.path[1][0], 3.0);
  EXPECT_DOUBLE_EQ(a.speed, 1.5);
  EXPECT_DOUBLE_EQ(a.spin, 0.0);
  EXPECT_DOUBLE_EQ(a.z, 0.0);
  // An attach element's file is relative to the sim.yaml naming it.
  const auto file = (*config.sim.mujoco_elements)["enemy"]["file"].as<std::string>();
  EXPECT_TRUE(std::filesystem::path(file).is_absolute()) << file;
  EXPECT_EQ(std::filesystem::path(file).filename(), "enemy.urdf");
  EXPECT_THROW((void)hwemu::config::LoadHubConfig(
                   WriteSimYaml("actors_bad", "actors:\n  - {name: e, body: {}, path: [[1, 2, 3]]}\n"), ""),
               std::invalid_argument);
}

TEST(HubConfig, TruthLogCanCarryEncodersAndImus) {
  const auto config = hwemu::config::LoadHubConfig(
      WriteSimYaml("truth2", "truth_log: {encoders: [major_yaw_encoder, pitch], gyros: [g], accels: [a, b]}\n"), "");
  EXPECT_EQ(config.truth_log.encoders, (std::vector<std::string>{"major_yaw_encoder", "pitch"}));
  EXPECT_EQ(config.truth_log.gyros, (std::vector<std::string>{"g"}));
  EXPECT_EQ(config.truth_log.accels, (std::vector<std::string>{"a", "b"}));
}
