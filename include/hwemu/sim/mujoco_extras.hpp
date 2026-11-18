#pragma once

// The MuJoCo backend's native objects, for a caller that needs more than the
// shared contract. Hold `mutex` while touching the model or data: the sim
// steps them, and the display (if open) reads them, under it. Only with
// HWEMU_WITH_MUJOCO; nullopt for any other backend.

#ifdef HWEMU_WITH_MUJOCO

#include <mutex>
#include <optional>

#include <mujoco/mujoco.h>

#include "hwemu/sim/sim.hpp"

namespace hwemu::sim {

struct MujocoNative {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::recursive_mutex* mutex = nullptr;
};

std::optional<MujocoNative> MujocoExtras(ISim& sim);

}  // namespace hwemu::sim

#endif
