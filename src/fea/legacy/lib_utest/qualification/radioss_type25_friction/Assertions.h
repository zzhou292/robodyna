// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include "../radioss_type25_normal/Assertions.h"
#include <array>
namespace type25_friction_test {
template<class U> inline std::array<double, 43> Fields(const n::FrictionResult<U>& r) {
  const auto& h = r.history;
  const auto normal = type25_normal_test::Fields(r.normal);
  std::array<double, 43> fields{};
  std::copy(normal.begin(), normal.end(), fields.begin());
  const double tail[]{h.normal.previous_penetration, h.normal.previous_stiffness,
    h.normal.staged_penetration, h.normal.staged_stiffness, h.normal.damping_half_force,
    h.previous_force.x, h.previous_force.y, h.previous_force.z,
    h.staged_force.x, h.staged_force.y, h.staged_force.z,
    r.tangent_predictor.x, r.tangent_predictor.y, r.tangent_predictor.z,
    r.tangent_force.x, r.tangent_force.y, r.tangent_force.z,
    r.native_resultant.x, r.native_resultant.y, r.native_resultant.z,
    r.coefficient, r.limiter, r.contact_area, r.pressure, r.friction_work};
  std::copy(std::begin(tail), std::end(tail), fields.begin() + 18);
  return fields;
}
template<class U> inline void Same(const n::FrictionResult<U>& a,
    const n::FrictionResult<U>& b, bool exact = false) {
  ASSERT_EQ(a.normal.terms_valid, b.normal.terms_valid);
  ASSERT_EQ(a.contact_active, b.contact_active);
  const auto first = Fields(a), second = Fields(b);
  for (std::size_t i = 0; i < first.size(); ++i) {
    SCOPED_TRACE(i); ASSERT_TRUE(std::isfinite(first[i])); ASSERT_TRUE(std::isfinite(second[i]));
    if (exact) {
      std::uint64_t x, y; std::memcpy(&x, &first[i], sizeof(x)); std::memcpy(&y, &second[i], sizeof(y));
      EXPECT_EQ(x, y);
    } else {
      const auto scale = std::max({std::abs(first[i]), std::abs(second[i]), 1e-300});
      EXPECT_NEAR(first[i], second[i], 64 * std::numeric_limits<double>::epsilon() * scale);
    }
  }
}
template<class U> inline n::FrictionResult<U> Sentinel() {
  n::FrictionResult<U> result;
  result.normal.normal_force = 101; result.history.previous_force = {102, 103, 104};
  result.tangent_predictor = {105, 106, 107}; result.tangent_force = {108, 109, 110};
  result.native_resultant = {111, 112, 113}; result.coefficient = 114;
  result.limiter = 115; result.contact_area = 116; result.pressure = 117;
  result.friction_work = 118; result.contact_active = true;
  return result;
}
} // namespace type25_friction_test
