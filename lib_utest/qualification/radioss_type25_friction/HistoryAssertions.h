// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Assertions.h"
namespace type25_friction_test {
inline n::NativeContactRow Row() {
  n::NativeContactRow row;
  row.history.normal = {0.001, 300., 0.002, 400., -0.01};
  row.history.previous_force = {0.01, -0.02, 0.03};
  row.history.staged_force = {0.04, -0.05, 0.06};
  row.penetration_auxiliary = -0.17; row.penetration_offset = 0.001;
  row.irtlm[0] = 7; row.irtlm[1] = -1; row.irtlm[2] = 1; row.irtlm[3] = 1;
  row.selection_metric[0] = 0.1; row.selection_metric[1] = 0.2;
  return row;
}
inline std::array<double, 15> RowFields(const n::NativeContactRow& row) {
  const auto& h = row.history;
  return {{h.normal.previous_penetration, h.normal.previous_stiffness,
    h.normal.staged_penetration, h.normal.staged_stiffness, h.normal.damping_half_force,
    h.previous_force.x, h.previous_force.y, h.previous_force.z,
    h.staged_force.x, h.staged_force.y, h.staged_force.z,
    row.penetration_auxiliary, row.penetration_offset, row.selection_metric[0], row.selection_metric[1]}};
}
inline void SameRow(const n::NativeContactRow& a, const n::NativeContactRow& b, bool exact = true) {
  for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(a.irtlm[i], b.irtlm[i]);
  const auto x = RowFields(a), y = RowFields(b);
  for (unsigned i = 0; i < x.size(); ++i) {
    std::uint64_t first, second;
    std::memcpy(&first, &x[i], sizeof(first)); std::memcpy(&second, &y[i], sizeof(second));
    if (exact) { EXPECT_EQ(first, second) << i; }
    else {
      const auto scale = std::max({std::abs(x[i]), std::abs(y[i]), 1e-300});
      EXPECT_NEAR(x[i], y[i], 64 * std::numeric_limits<double>::epsilon() * scale) << i;
    }
  }
}
} // namespace type25_friction_test
