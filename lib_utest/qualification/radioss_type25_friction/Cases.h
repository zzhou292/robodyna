// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25Friction.h"
#include <vector>
namespace type25_friction_test {
namespace n = tlfea::contact::radioss_type25;
struct Case {
  n::ResolvedNormalConfig normal_config;
  n::FrictionControls controls;
  n::NativeFrictionCoefficients coefficients;
  n::NativeFrictionInput input;
  n::NativeFrictionHistory history;
};
inline void RefreshNormalVelocity(Case& c) {
  c.input.normal.normal_velocity = tl::math::fixed3::Dot(c.input.normal_axis, c.input.relative_velocity);
}
inline Case Basic() {
  Case c;
  c.normal_config.engine = {0, 0, 0};
  c.controls = {2, 10, 0, 1, 0, 0, 1.};
  c.coefficients = {0.1, {0., 0., 0., 0., 0.1, -0.001}};
  c.input.normal = {0.002, 400., -20., 1e-5, 0., 0.002,
      {0.004, 0.002, 0.008, 0.001}, {0.25, 0.25, 0.25, 0.25}, 0.};
  c.input.normal_axis = {0, 0, 1}; c.input.relative_velocity = {5, -3, -20};
  c.input.main_vertices[0] = {0, 0, 0}; c.input.main_vertices[1] = {10, 0, 0};
  c.input.main_vertices[2] = {10, 10, 0}; c.input.main_vertices[3] = {0, 10, 0};
  c.input.dt12 = 0.5e-5;
  return c;
}
inline Case Float32BoundaryNormal() {
  auto c = Basic();
  // Source-supported REAL*4 bisector promotion, not a captured Yaris value.
  // Binary32 bits0x3f3504f3, exactly11863283/16777216 after promotion.
  const double value = static_cast<double>(0x1.6a09e6p-1f);
  c.input.normal_axis = {value, value, 0.};
  RefreshNormalVelocity(c);
  return c;
}
} // namespace type25_friction_test
#include "ActualInput.h"
namespace type25_friction_test {
inline std::vector<Case> Cases() {
  std::vector<Case> rows;
  for (double speed : {0., 1., 5., 100., 2000., -2000.})
    for (double old_force : {0., 0.1, -0.1}) for (bool initial : {true, false})
      for (double damping : {0., 0.05}) {
        auto c = Basic();
        c.input.relative_velocity.x = speed;
        c.history.previous_force = {old_force, 0.02, 0.01};
        c.history.normal = {0.001, 300., 0.0005, 200., -0.01};
        c.input.normal.time = initial ? 0. : 1e-4;
        c.normal_config.damping_factor = damping;
        RefreshNormalVelocity(c); rows.push_back(c);
      }
  rows.push_back(ActualForceInput());
  rows.push_back(Float32BoundaryNormal());
  auto zero = Basic(); zero.input.normal_axis = {}; RefreshNormalVelocity(zero); rows.push_back(zero);
  return rows;
}
} // namespace type25_friction_test
