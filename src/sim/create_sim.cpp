#include <stdexcept>

#include "hwemu/sim/fake_sim.hpp"
#include "hwemu/sim/sim.hpp"

namespace hwemu::sim {

#ifdef HWEMU_WITH_ACHILLES
std::unique_ptr<ISim> CreateAchillesSim(const SimOptions& options);
#endif
#ifdef HWEMU_WITH_MUJOCO
std::unique_ptr<ISim> CreateMujocoSim(const SimOptions& options);
#endif

Backend ParseBackend(std::string_view name) {
  if (name == "achilles") return Backend::kAchilles;
  if (name == "mujoco") return Backend::kMujoco;
  if (name == "fake") return Backend::kFake;
  throw std::invalid_argument("backend must be achilles, mujoco or fake, not '" + std::string(name) + "'");
}

std::string_view Name(Backend backend) {
  switch (backend) {
    case Backend::kAchilles: return "achilles";
    case Backend::kMujoco: return "mujoco";
    case Backend::kFake: return "fake";
  }
  return "unknown";
}

std::unique_ptr<ISim> CreateSim(const SimOptions& options) {
  switch (options.backend) {
    case Backend::kFake:
      return std::make_unique<FakeSim>();
    case Backend::kAchilles:
#ifdef HWEMU_WITH_ACHILLES
      return CreateAchillesSim(options);
#else
      throw std::runtime_error("hw-emulator was built without Achilles (HWEMU_WITH_ACHILLES=OFF)");
#endif
    case Backend::kMujoco:
#ifdef HWEMU_WITH_MUJOCO
      return CreateMujocoSim(options);
#else
      throw std::runtime_error("hw-emulator was built without MuJoCo (HWEMU_WITH_MUJOCO=OFF)");
#endif
  }
  throw std::invalid_argument("unknown backend");
}

}  // namespace hwemu::sim

namespace hwemu::display {

ViewKind ParseViewKind(std::string_view name) {
  if (name == "image") return ViewKind::kImage;
  if (name == "depth") return ViewKind::kDepth;
  if (name == "lidar") return ViewKind::kLidar;
  if (name == "plot") return ViewKind::kPlot;
  throw std::invalid_argument("view must be image, depth, lidar or plot, not '" + std::string(name) + "'");
}

}  // namespace hwemu::display
