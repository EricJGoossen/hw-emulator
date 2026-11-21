#pragma once

// The hub's actors (sim.yaml `actors`, config::ActorSpec): scripted bodies,
// e.g. an opponent robot, that the hub places every tick (ISim::SetBodyPose)
// rather than the physics moving them.

#include "hwemu/config/config.hpp"
#include "hwemu/sim/sim.hpp"

namespace hwemu::hub {

// Where `actor` is at sim time `t`: at path[0] until its start, then around
// its closed loop at its speed, facing along the path (the heading eased
// across each waypoint over its `turn` distance) or, with a spin, turning
// at that rate. Throws std::invalid_argument for a path of fewer than two
// distinct points or a speed that isn't positive.
sim::Pose ActorPose(const config::ActorSpec& actor, double t);

}  // namespace hwemu::hub
