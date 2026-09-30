// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include <algorithm>
#include <cmath>
namespace tlfea::contact::radioss_type25::search_startup::detail {
using Vector = tl::math::Vec3;
inline unsigned Corners(const startup::Main& main) noexcept {
  return main.nodes[2] == main.nodes[3] ? 3u : 4u;
}
inline double Distance(Vector a, Vector b) noexcept {
  const double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
  return std::sqrt(x*x + y*y + z*z);
}
struct Box { Vector minimum, maximum; };
inline Box Bounds(const Vector* x, const startup::Main& main) noexcept {
  Box result{x[main.nodes[0]], x[main.nodes[0]]};
  for (unsigned k = 1; k < 4; ++k) {
    const auto v = x[main.nodes[k]];
    result.minimum.x = std::min(result.minimum.x, v.x);
    result.minimum.y = std::min(result.minimum.y, v.y);
    result.minimum.z = std::min(result.minimum.z, v.z);
    result.maximum.x = std::max(result.maximum.x, v.x);
    result.maximum.y = std::max(result.maximum.y, v.y);
    result.maximum.z = std::max(result.maximum.z, v.z);
  }
  return result;
}
inline bool Finite(Box b) noexcept {
  return std::isfinite(b.minimum.x) && std::isfinite(b.minimum.y) && std::isfinite(b.minimum.z) &&
      std::isfinite(b.maximum.x) && std::isfinite(b.maximum.y) && std::isfinite(b.maximum.z);
}
inline bool Inside(Vector p, Box b) noexcept {
  return p.x >= b.minimum.x && p.x <= b.maximum.x &&
      p.y >= b.minimum.y && p.y <= b.maximum.y &&
      p.z >= b.minimum.z && p.z <= b.maximum.z;
}
Report Margin(const Input&, const Vector*, double multiplier, double gap,
    Limits, double& mean, double& margin) noexcept;
Report Extent(const Input&, const Vector*, double* primary, double& maximum) noexcept;
} // namespace tlfea::contact::radioss_type25::search_startup::detail
