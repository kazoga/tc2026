#include "motion_profile.hpp"
#include <cassert>
#include <algorithm>
#include <limits>
int main()
{
  using icart_motion::select_linear_command;
  using icart_motion::validate_limit;
  auto c = select_linear_command(0., 1., .7, 1.5);
  assert(c.target == 1. && c.acceleration == .7);
  for (double sign : {-1., 1.}) {
    c = select_linear_command(sign, sign*.2, .7, 1.5);
    assert(c.acceleration == 1.5 && c.target == sign*.2);
    c = select_linear_command(sign, -sign, .7, 1.5);
    assert(c.target == 0. && c.acceleration == 1.5);
    double reference = sign, distance = 0.;
    for (int i=0; i<100 && reference != 0.; ++i) {
      c = select_linear_command(reference, 0., .7, 1.5);
      assert(c.acceleration == 1.5);
      distance += std::abs(reference)*.015;
      reference = sign*std::max(0., std::abs(reference)-c.acceleration*.015);
    }
    // Discrete coordinator cycle, versus continuous 1/(2*1.5).
    assert(distance <= 1./3.+.015);
    assert(reference == 0.);
    c = select_linear_command(reference, -sign, .7, 1.5);
    assert(c.acceleration == .7 && c.target == -sign);
  }
  c = select_linear_command(.5, std::numeric_limits<double>::quiet_NaN(), .7, 1.5);
  assert(c.target == 0. && c.acceleration == 1.5);
  validate_limit(1.5, 1.5);
  for (double requested : {0., -1., 1.51, std::numeric_limits<double>::infinity()}) {
    bool rejected=false;
    try { validate_limit(requested, 1.5); } catch (const std::runtime_error &) { rejected=true; }
    assert(rejected);
  }
}
