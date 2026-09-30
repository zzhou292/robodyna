// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../Q4ContactBounds.h"
#include <algorithm>

namespace tlfea::contact::contact_facets {
namespace {
double Component(Vec3 value, unsigned component) noexcept {
  return component == 0 ? value.x : component == 1 ? value.y : value.z;
}
bool CrossTermBound(const Vec3* points, double& upper) noexcept {
  double magnitude[3]{};
  for (unsigned axis = 0; axis < 3; ++axis) {
    q4_bounds::Interval a, b, difference;
    if (!q4_bounds::Difference(Component(points[0], axis), Component(points[1], axis), &a) ||
        !q4_bounds::Difference(Component(points[2], axis), Component(points[3], axis), &b) ||
        !q4_bounds::Add(a, b, &difference)) return false;
    magnitude[axis] = std::max(::fabs(difference.lower), ::fabs(difference.upper));
  }
  return mass_detail::UpperNorm({magnitude[0], magnitude[1], magnitude[2]}, &upper);
}
bool VertexRoundoff(const Vec3* points, unsigned arity, const Vertex& vertex, double& upper) noexcept {
  double width[3]{};
  for (unsigned axis = 0; axis < 3; ++axis) {
    q4_bounds::Interval sum{};
    for (unsigned i = 0; i < arity; ++i) {
      const double value = Component(points[i], axis);
      q4_bounds::Interval term;
      if (!q4_bounds::Scale({value, value}, vertex.weights[i], &term) || !q4_bounds::Add(sum, term, &sum)) return false;
    }
    // The interval encloses both the exact weighted sum and the source-order
    // represented Add/Scale result. Its full width safely bounds their distance.
    q4_bounds::Interval difference;
    if (!q4_bounds::Difference(sum.upper, sum.lower, &difference)) return false;
    width[axis] = difference.upper;
  }
  return mass_detail::UpperNorm({width[0], width[1], width[2]}, &upper);
}
}
Status MeasureApproximation(const SelfContactSurfaceParent& parent, unsigned level,
    const Vertex* vertices, std::size_t count, VectorView view, FacetApproximationBound* output) noexcept {
  Vec3 points[4];
  for (unsigned i = 0; i < parent.arity; ++i) {
    const auto node = parent.arity == 4 ? parent.q4.nodes[i] : parent.t3.nodes[i];
    if (node >= view.node_count) return Status::kOutOfRange;
    points[i] = view.at(node);
    if (!IsFinite(points[i])) return Status::kInvalidArgument;
  }
  FacetApproximationBound next;
  if (parent.arity == 4) {
    double cross_term = 0;
    const double factor = ::ldexp(1., -2 - 2 * static_cast<int>(level));
    if (!CrossTermBound(points, cross_term) ||
        !mass_detail::UpperProduct(cross_term, factor, &next.bilinear_error_upper_m)) return Status::kNonFiniteResult;
  }
  for (std::size_t i = 0; i < count; ++i) {
    double upper = 0;
    if (!VertexRoundoff(points, parent.arity, vertices[i], upper)) return Status::kNonFiniteResult;
    next.vertex_roundoff_upper_m = std::max(next.vertex_roundoff_upper_m, upper);
  }
  if (!mass_detail::UpperSum(next.bilinear_error_upper_m, next.vertex_roundoff_upper_m,
      &next.total_error_upper_m)) return Status::kNonFiniteResult;
  *output = next;
  return Status::kOk;
}
} // namespace tlfea::contact::contact_facets
