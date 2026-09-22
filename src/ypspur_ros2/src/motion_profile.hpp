#pragma once
#include <cmath>
#include <stdexcept>

namespace icart_motion
{
inline void validate_limit(double requested, double ceiling)
{
  if (!std::isfinite(requested) || !std::isfinite(ceiling) ||
      requested <= 0.0 || ceiling <= 0.0 || requested > ceiling + 1e-9) {
    throw std::runtime_error("Motion limit is invalid or exceeds coordinator parameter ceiling");
  }
}

struct LinearCommand { double target; double acceleration; };

// YPSpur uses one slew rate for both directions. Reverse only after the
// coordinator's smoothed reference reaches zero; otherwise the braking rate
// would also accelerate the robot in the opposite direction.
inline LinearCommand select_linear_command(
  double reference, double target, double acceleration, double deceleration)
{
  if (!std::isfinite(reference) || !std::isfinite(target)) {
    return {0.0, deceleration};
  }
  if (reference * target < 0.0) {
    return {0.0, deceleration};
  }
  return {target, std::abs(target) < std::abs(reference) || target == 0.0 ?
    deceleration : acceleration};
}
}  // namespace icart_motion
