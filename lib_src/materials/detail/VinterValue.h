// SPDX-License-Identifier: AGPL-3.0-or-later
// VINTER interpolation adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Neutral borrowed-curve arithmetic; declaration/domain checks belong to callers.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>
#if defined(__CUDACC__)
#define TL_VINTER_HD __host__ __device__
#else
#define TL_VINTER_HD
#endif
namespace tl::material::detail {
template<class Curve>
TL_VINTER_HD inline bool VinterValue(Curve c, double x,
                                    double& value, double& slope) noexcept {
  // Strict X > next knot selects the native left segment at an exact knot.
  std::uint32_t low = 0, high = c.count - 1;
  while (high - low > 1) {
    const auto middle = low + (high - low) / 2;
    if (x > c.plastic_strain[middle]) low = middle;
    else high = middle;
  }
  const double width = c.plastic_strain[low + 1] - c.plastic_strain[low];
  slope = (c.yield_stress_pa[low + 1] - c.yield_stress_pa[low]) / width;
  value = c.yield_stress_pa[low] + slope * (x - c.plastic_strain[low]);
  return width > 0 && tl::math::Finite(slope) && slope >= 0 &&
         tl::math::Finite(value) && value > 0;
}
}  // namespace tl::material::detail
#undef TL_VINTER_HD
