// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
namespace type25_normal_test {
template<class Units> inline std::array<double, 18> Fields(const normal::NormalResult<Units>& r) {
  return {{r.history.previous_penetration, r.history.previous_stiffness,
    r.history.staged_penetration, r.history.staged_stiffness, r.history.damping_half_force,
    r.weights[0], r.weights[1], r.weights[2], r.weights[3], r.force_stiffness,
    r.stability_stiffness, r.normal_force, r.elastic_energy, r.damping_force,
    r.damping_work, r.damping_coefficient, r.separate_elastic_stiffness,
    r.separate_friction_damping}};
}
template<class Units> inline void Same(const normal::NormalResult<Units>& a,
    const normal::NormalResult<Units>& b, bool exact = false) {
  ASSERT_EQ(a.terms_valid, b.terms_valid);
  const auto first = Fields(a), second = Fields(b);
  for (std::size_t i = 0; i < first.size(); ++i) {
    SCOPED_TRACE(i);
    ASSERT_TRUE(std::isfinite(first[i])); ASSERT_TRUE(std::isfinite(second[i]));
    if (exact) {
      std::uint64_t x, y;
      std::memcpy(&x, &first[i], sizeof(x)); std::memcpy(&y, &second[i], sizeof(y));
      EXPECT_EQ(x, y);
    } else {
      // Fixed before execution. Native sqrt/division may differ by a few ulps.
      // No unit-sized absolute tolerance hides small force/history errors.
      const double scale = std::max({std::abs(first[i]), std::abs(second[i]), 1e-300});
      EXPECT_NEAR(first[i], second[i], 32 * std::numeric_limits<double>::epsilon() * scale);
    }
  }
}
inline normal::NativeNormalResult Sentinel() {
  normal::NativeNormalResult r;
  r.history = {11, 12, 13, 14, 15};
  for (unsigned i = 0; i < 4; ++i) r.weights[i] = 16 + i;
  r.force_stiffness = 20; r.stability_stiffness = 21; r.normal_force = 22;
  r.elastic_energy = 23; r.damping_force = 24; r.damping_work = 25;
  r.damping_coefficient = 26; r.separate_elastic_stiffness = 27;
  r.separate_friction_damping = 28; r.terms_valid = true;
  return r;
}
} // namespace type25_normal_test
