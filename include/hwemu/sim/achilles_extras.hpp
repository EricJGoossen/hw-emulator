#pragma once

// The Achilles backend's native objects, for a caller that needs more than the
// shared contract: its own displays and controllers attached to the renderer
// (any achilles::display::DisplayLike / ControllerLike), or its handles read
// directly. Only with HWEMU_WITH_ACHILLES; null for any other backend.

#ifdef HWEMU_WITH_ACHILLES

#include "hwemu/sim/sim.hpp"
#include "interface/simulation.hpp"

#ifdef HWEMU_WITH_DISPLAY
#include "display/renderer.hpp"
#endif

namespace hwemu::sim {

struct AchillesNative {
  achilles::interface::Simulation* simulation = nullptr;
  // Null when headless, or before the display is opened.
#ifdef HWEMU_WITH_DISPLAY
  achilles::display::Renderer* renderer = nullptr;
#endif
};

// The backend's native objects, or nullopt if `sim` isn't an Achilles sim.
std::optional<AchillesNative> AchillesExtras(ISim& sim);

}  // namespace hwemu::sim

#endif
