#include "hwemu/hub/actor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace hwemu::hub {
namespace {

double Wrap(double a) { return std::remainder(a, 2.0 * std::numbers::pi); }

// a + w * (the shortest turn from a to b).
double Ease(double a, double b, double w) { return a + (w * Wrap(b - a)); }

}  // namespace

sim::Pose ActorPose(const config::ActorSpec& actor, double t) {
  const auto& path = actor.path;
  const std::size_t n = path.size();
  if (n < 2) throw std::invalid_argument("actor '" + actor.name + "': its path needs at least two points");
  if (!(actor.speed > 0.0)) throw std::invalid_argument("actor '" + actor.name + "': its speed must be positive");

  // The loop's legs: leg i runs from path[i] to path[i + 1 mod n].
  std::vector<double> length(n), heading(n);
  double loop = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const auto& a = path[i];
    const auto& b = path[(i + 1) % n];
    length[i] = std::hypot(b[0] - a[0], b[1] - a[1]);
    heading[i] = std::atan2(b[1] - a[1], b[0] - a[0]);
    loop += length[i];
  }
  if (!(loop > 0.0)) throw std::invalid_argument("actor '" + actor.name + "': its path has no length");

  double s = std::fmod(actor.speed * std::max(0.0, t - actor.start), loop);
  std::size_t i = 0;
  while (s > length[i] && i + 1 < n) s -= length[i++];
  // A zero-length leg has no heading of its own: it takes the previous one.
  const auto leg_heading = [&](std::size_t k) {
    for (std::size_t m = 0; m < n && length[k] == 0.0; ++m) k = (k + n - 1) % n;
    return heading[k];
  };
  const double u = length[i] == 0.0 ? 0.0 : s / length[i];
  const auto& a = path[i];
  const auto& b = path[(i + 1) % n];

  double yaw = 0.0;
  if (actor.spin != 0.0) {
    yaw = Wrap(actor.spin * std::max(0.0, t - actor.start));
  } else {
    // Halfway through the turn at each waypoint, eased over `turn` metres
    // either side of it (at most half of each leg).
    const double h = leg_heading(i);
    const double ease = std::min(actor.turn, 0.5 * length[i]);
    if (ease > 0.0 && s < ease) {
      yaw = Ease(leg_heading((i + n - 1) % n), h, 0.5 + (0.5 * s / ease));
    } else if (ease > 0.0 && length[i] - s < ease) {
      yaw = Ease(h, leg_heading((i + 1) % n), 0.5 * (1.0 - ((length[i] - s) / ease)));
    } else {
      yaw = h;
    }
  }

  sim::Pose pose;
  pose.position = {a[0] + (u * (b[0] - a[0])), a[1] + (u * (b[1] - a[1])), actor.z};
  pose.orientation = {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  return pose;
}

}  // namespace hwemu::hub
