// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ExactPredicates.h"
#include "ExactPredicateKernel.h"

namespace tlfea::contact::fixed_triangle_features::exact {
Sign Orient2D(Vec3 a, Vec3 b, Vec3 c, int dropped_axis) noexcept {
  return detail::EvaluateOrient2D<detail::Storage::Adaptive>(a, b, c, dropped_axis);
}
Sign Orient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d) noexcept {
  return detail::EvaluateOrient3D<detail::Storage::Adaptive>(a, b, c, d);
}
Sign DirectedTriangle(Vec3 a, Vec3 b, Vec3 c, Vec3 direction) noexcept {
  return detail::EvaluateDirectedTriangle<detail::Storage::Adaptive>(a, b, c, direction);
}
bool ClosestStratum(Vec3 point, const Vec3 (&triangle)[3],
                    ClosestTriangleStratum* output) noexcept {
  return detail::EvaluateClosestStratum<detail::Storage::Adaptive>(point, triangle, output);
}
}  // namespace tlfea::contact::fixed_triangle_features::exact
