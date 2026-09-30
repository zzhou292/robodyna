// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <vector>
namespace type25_normal_test {
struct Case {
  normal::ResolvedNormalConfig config;
  normal::NativeNormalInput input;
  normal::NativeNormalHistory history;
};
inline Case Basic() {
  Case c;
  c.config.engine = {0, 0, 0};
  c.input = {0.002, 400., -20., 1e-5, 0., 0.002,
             {0.004, 0.002, 0.008, 0.001}, {0.25, 0.25, 0.25, 0.25}, 0.};
  return c;
}
inline std::vector<Case> Cases() {
  std::vector<Case> result;
  for (int k : {0, 1}) for (int m : {0, 1, 2}) for (int n : {0, 1})
    for (double damping : {0., 0.05}) for (double viscous : {0., 0.01})
      for (double velocity : {-20., 0., 20.}) for (bool initial : {false, true}) {
        auto c = Basic();
        c.config.engine = {k, m, n}; c.config.damping_factor = damping;
        c.input.normal_velocity = velocity; c.input.friction_viscosity = viscous;
        c.input.time = initial ? 0. : 1e-4;
        c.history = {0.001, 300., 0.0005, 200., -0.01};
        result.push_back(c);
      }
  return result;
}
} // namespace type25_normal_test
