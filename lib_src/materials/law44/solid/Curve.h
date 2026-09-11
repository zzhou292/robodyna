// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Types.h"
#include "lib_src/materials/detail/VinterValue.h"

namespace tl::material::law44::solid::detail {
TL_LAW44_SOLID_HD inline bool CurveShape(Curve c) noexcept {
  return c.plastic_strain && c.yield_stress_pa && c.count >= 2 && c.count <= 1024;
}
TL_LAW44_SOLID_HD inline bool CurveValid(Curve c) noexcept {
  if (!CurveShape(c) || c.plastic_strain[0] != 0) return false;
  for (std::uint32_t i = 0; i < c.count; ++i) {
    if (!tl::math::Finite(c.plastic_strain[i]) ||
        !tl::math::Finite(c.yield_stress_pa[i]) || c.yield_stress_pa[i] <= 0 ||
        (i && (c.plastic_strain[i] <= c.plastic_strain[i - 1] ||
               c.yield_stress_pa[i] < c.yield_stress_pa[i - 1]))) return false;
    if (i) {
      double value = 0, slope = 0;
      if (!tl::material::detail::VinterSegmentValue(
              c, i - 1, c.plastic_strain[i], value, slope)) return false;
    }
  }
  return true;
}
// Native VINTER moves only forward and preserves its incoming segment at knots.
// The final segment extrapolates. Preparation owns complete curve validation.
TL_LAW44_SOLID_HD inline bool CurveValue(Curve c, double query,
    std::uint32_t& cursor, double& value, double& slope) noexcept {
  if (!CurveShape(c) || cursor >= c.count - 1 || !tl::math::Finite(query)) return false;
  auto next = cursor;
  while (next < c.count - 2 && query > c.plastic_strain[next + 1]) ++next;
  if (!tl::material::detail::VinterSegmentValue(c, next, query, value, slope)) return false;
  cursor = next;
  return true;
}
}  // namespace tl::material::law44::solid::detail
