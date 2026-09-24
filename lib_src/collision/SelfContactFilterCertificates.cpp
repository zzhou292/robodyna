// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SelfContactFilterCertificates.h"
#include "self_contact_filters/PrismQualification.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlfea::contact {
namespace {

double Down(double value) noexcept {
  return std::nextafter(
      value, -std::numeric_limits<double>::infinity());
}

double Up(double value) noexcept {
  return std::nextafter(
      value, std::numeric_limits<double>::infinity());
}

struct Interval {
  double lower = 0;
  double upper = 0;
};

bool Finite(const CurrentFixedTriangle& triangle) noexcept {
  for (const auto point : triangle.vertices)
    if (!IsFinite(point))
      return false;
  return true;
}

bool SameGeometry(
    const CurrentFixedTriangle& first,
    const CurrentFixedTriangle& second) noexcept {
  for (unsigned vertex = 0; vertex < 3; ++vertex) {
    const auto a = first.vertices[vertex];
    const auto b = second.vertices[vertex];
    if (a.x != b.x || a.y != b.y || a.z != b.z)
      return false;
  }
  return true;
}

bool ProductInterval(double a, double b, Interval* output) noexcept {
  const double value = a * b;
  if (!std::isfinite(value))
    return false;
  *output = {Down(value), Up(value)};
  return std::isfinite(output->lower) &&
      std::isfinite(output->upper);
}

bool AddInterval(Interval a, Interval b, Interval* output) noexcept {
  const double lower = Down(a.lower + b.lower);
  const double upper = Up(a.upper + b.upper);
  if (!std::isfinite(lower) || !std::isfinite(upper))
    return false;
  *output = {lower, upper};
  return true;
}

bool DotInterval(Vec3 point, Vec3 axis, Interval* output) noexcept {
  Interval x, y, z, sum;
  return ProductInterval(point.x, axis.x, &x) &&
      ProductInterval(point.y, axis.y, &y) &&
      ProductInterval(point.z, axis.z, &z) &&
      AddInterval(x, y, &sum) &&
      AddInterval(sum, z, output);
}

bool ProjectionBounds(
    const CurrentFixedTriangle& base,
    const CurrentFixedTriangle& current, Vec3 axis,
    Interval* output) noexcept {
  bool first = true;
  Interval next;
  const CurrentFixedTriangle* endpoints[2]{&base, &current};
  const unsigned endpoint_count = SameGeometry(base, current) ? 1 : 2;
  for (unsigned endpoint = 0; endpoint < endpoint_count; ++endpoint) {
    const auto* triangle = endpoints[endpoint];
    for (const auto point : triangle->vertices) {
      Interval projection;
      if (!DotInterval(point, axis, &projection))
        return false;
      if (first) {
        next = projection;
        first = false;
      } else {
        next.lower = std::min(next.lower, projection.lower);
        next.upper = std::max(next.upper, projection.upper);
      }
    }
  }
  *output = next;
  return true;
}

Vec3 EdgeAxis(
    const CurrentFixedTriangle& triangle, unsigned edge) noexcept {
  const auto& first = triangle.vertices[edge];
  const auto& second = triangle.vertices[(edge + 1) % 3];
  return {
      second.x - first.x,
      second.y - first.y,
      second.z - first.z};
}

Vec3 CrossAxis(Vec3 first, Vec3 second) noexcept {
  return {
      first.y * second.z - first.z * second.y,
      first.z * second.x - first.x * second.z,
      first.x * second.y - first.y * second.x};
}

Vec3 Difference(Vec3 first, Vec3 second) noexcept {
  return {
      first.x - second.x,
      first.y - second.y,
      first.z - second.z};
}

Vec3 FaceAxis(const CurrentFixedTriangle& triangle) noexcept {
  const Vec3 second = Difference(
      triangle.vertices[2], triangle.vertices[0]);
  return CrossAxis(EdgeAxis(triangle, 0), second);
}

Vec3 VertexEdgeAxis(
    Vec3 vertex, const CurrentFixedTriangle& triangle,
    unsigned edge) noexcept {
  const auto edge_axis = EdgeAxis(triangle, edge);
  const auto vertex_from_start = Difference(
      vertex, triangle.vertices[edge]);
  // e x ((v - e0) x e) is division-free, perpendicular to the line, and
  // points along the represented line-to-vertex displacement.
  return CrossAxis(
      edge_axis, CrossAxis(vertex_from_start, edge_axis));
}

bool InflateProjection(
    double thickness, double norm_l1, Interval* interval) noexcept {
  const double margin = Up(thickness * norm_l1);
  if (!std::isfinite(margin))
    return false;
  const double lower = Down(interval->lower - margin);
  const double upper = Up(interval->upper + margin);
  if (!std::isfinite(lower) || !std::isfinite(upper))
    return false;
  *interval = {lower, upper};
  return true;
}

template <bool observe>
bool AxisSeparates(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current,
    double first_thickness, double second_thickness,
    Vec3 axis, self_contact_filters::PrismCounts* counts) noexcept {
  if constexpr (observe) ++counts->axis_tests;
  if (!IsFinite(axis))
    return false;
  const double norm_l1 = Up(Up(
      std::fabs(axis.x) + std::fabs(axis.y)) +
      std::fabs(axis.z));
  if (!std::isfinite(norm_l1) || !(norm_l1 > 0))
    return false;
  Interval first, second;
  if (!ProjectionBounds(first_base, first_current, axis, &first) ||
      !ProjectionBounds(second_base, second_current, axis, &second))
    return false;
  if (!InflateProjection(first_thickness, norm_l1, &first) ||
      !InflateProjection(second_thickness, norm_l1, &second))
    return false;
  return first.upper < second.lower ||
      second.upper < first.lower;
}

double Component(Vec3 value, unsigned axis) noexcept {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

bool InflatedFacetBoundsSeparated(
    const CurrentFixedTriangle& first, double first_thickness,
    const CurrentFixedTriangle& second, double second_thickness) noexcept {
  const double infinity = std::numeric_limits<double>::infinity();
  for (unsigned axis = 0; axis < 3; ++axis) {
    double first_lower = Component(first.vertices[0], axis);
    double first_upper = first_lower;
    double second_lower = Component(second.vertices[0], axis);
    double second_upper = second_lower;
    for (unsigned vertex = 1; vertex < 3; ++vertex) {
      const auto a = Component(first.vertices[vertex], axis);
      const auto b = Component(second.vertices[vertex], axis);
      first_lower = std::min(first_lower, a);
      first_upper = std::max(first_upper, a);
      second_lower = std::min(second_lower, b);
      second_upper = std::max(second_upper, b);
    }
    first_lower = std::nextafter(
        first_lower - first_thickness, -infinity);
    first_upper = std::nextafter(
        first_upper + first_thickness, infinity);
    second_lower = std::nextafter(
        second_lower - second_thickness, -infinity);
    second_upper = std::nextafter(
        second_upper + second_thickness, infinity);
    if (first_upper < second_lower ||
        second_upper < first_lower)
      return true;
  }
  return false;
}

// This is a necessary-no-separator check for the existing endpoint HULLS,
// not a contact/topology certificate. Even cross-time coincidence prevents
// any strictly disjoint projection hulls. All geometry was validated first;
// numeric equality intentionally includes signed zero and ignores source IDs.
template <bool observe>
bool EndpointHullsShareVertex(
    const CurrentFixedTriangle& first_base, const CurrentFixedTriangle& first_current,
    const CurrentFixedTriangle& second_base, const CurrentFixedTriangle& second_current,
    self_contact_filters::PrismCounts* counts) noexcept {
  const CurrentFixedTriangle* first[]{&first_base, &first_current};
  const CurrentFixedTriangle* second[]{&second_base, &second_current};
  for (const auto* a : first) for (const auto* b : second)
    for (const auto p : a->vertices) for (const auto q : b->vertices) {
      if constexpr (observe) ++counts->coordinate_tests;
      if (p.x == q.x && p.y == q.y && p.z == q.z) {
        if constexpr (observe) counts->hull_coincidence = true;
        return true;
      }
    }
  return false;
}

template <bool reject_coincident_hulls, bool observe>
bool CertifiedLinearFacetPrismSeparationImpl(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit,
    SelfContactFacetPrismSeparationAxis* separated_axis,
    bool* valid, self_contact_filters::PrismCounts* counts) noexcept {
  if (!valid)
    return false;
  if (separated_axis)
    *separated_axis = SelfContactFacetPrismSeparationAxis::None;
  *valid = static_cast<std::uint8_t>(axis_limit) <=
          static_cast<std::uint8_t>(
              SelfContactFacetPrismAxisLimit::VertexVertex) &&
      std::isfinite(first_thickness) && first_thickness > 0 &&
      std::isfinite(second_thickness) && second_thickness > 0 &&
      Finite(first_base) && Finite(first_current) &&
      Finite(second_base) && Finite(second_current);
  if (!*valid)
    return false;
  if constexpr (reject_coincident_hulls)
    if (EndpointHullsShareVertex<observe>(first_base, first_current,
                                       second_base, second_current, counts))
      return false;
  // For LinearNodalV1, every vertex projection lies in the hull of its two
  // endpoint projections. The intervals enclose all rounded dot operations,
  // and each half-thickness*|axis|_1 overbounds its Euclidean projection.
  // Any finite nonzero represented axis is itself a valid hyperplane, even
  // when rounded edge construction differs from the mathematical edge. Thus
  // a strict interval gap certifies the complete swept pair. Equality,
  // degenerate axes, and nonfinite/overflowed arithmetic remain unresolved.
  const Vec3 axes[4]{
      FaceAxis(first_base), FaceAxis(first_current),
      FaceAxis(second_base), FaceAxis(second_current)};
  for (const auto axis : axes) {
    if (AxisSeparates<observe>(
            first_base, first_current, second_base, second_current,
            first_thickness, second_thickness, axis, counts)) {
      if (separated_axis)
        *separated_axis =
            SelfContactFacetPrismSeparationAxis::FaceNormal;
      return true;
    }
  }
  if (axis_limit == SelfContactFacetPrismAxisLimit::FaceNormal)
    return false;

  // Test every 3x3 edge cross-edge family for all four represented endpoint
  // state combinations. Endpoint projection hulls, not endpoint chords,
  // bound both linear swept triangular prisms.
  const CurrentFixedTriangle* first_states[2]{
      &first_base, &first_current};
  const CurrentFixedTriangle* second_states[2]{
      &second_base, &second_current};
  const unsigned first_state_count =
      SameGeometry(first_base, first_current) ? 1 : 2;
  const unsigned second_state_count =
      SameGeometry(second_base, second_current) ? 1 : 2;
  for (unsigned first_state_index = 0;
       first_state_index < first_state_count; ++first_state_index) {
    const auto* first_state = first_states[first_state_index];
    for (unsigned second_state_index = 0;
         second_state_index < second_state_count; ++second_state_index) {
      const auto* second_state = second_states[second_state_index];
      for (unsigned first_edge = 0; first_edge < 3; ++first_edge) {
        const auto first_axis = EdgeAxis(*first_state, first_edge);
        for (unsigned second_edge = 0; second_edge < 3; ++second_edge) {
          const auto axis = CrossAxis(
              first_axis, EdgeAxis(*second_state, second_edge));
          if (AxisSeparates<observe>(
                  first_base, first_current,
                  second_base, second_current,
                  first_thickness, second_thickness, axis, counts)) {
            if (separated_axis)
              *separated_axis =
                  SelfContactFacetPrismSeparationAxis::EdgeCross;
            return true;
          }
        }
      }
    }
  }
  if (axis_limit == SelfContactFacetPrismAxisLimit::EdgeCross)
    return false;

  // Closest vertex-edge directions are not generally triangle SAT axes after
  // physical thickness inflation. Construct all represented perpendicular
  // point-line directions at every endpoint-state combination. AxisSeparates
  // still projects both complete endpoint triangles, so no generated axis is
  // assumed to remain a closest-feature direction during linear motion.
  for (unsigned first_state_index = 0;
       first_state_index < first_state_count; ++first_state_index) {
    const auto* first_state = first_states[first_state_index];
    for (unsigned second_state_index = 0;
         second_state_index < second_state_count; ++second_state_index) {
      const auto* second_state = second_states[second_state_index];
      for (unsigned vertex = 0; vertex < 3; ++vertex) {
        for (unsigned edge = 0; edge < 3; ++edge) {
          const auto first_vertex_axis = VertexEdgeAxis(
              first_state->vertices[vertex], *second_state, edge);
          if (AxisSeparates<observe>(
                  first_base, first_current,
                  second_base, second_current,
                  first_thickness, second_thickness,
                  first_vertex_axis, counts)) {
            if (separated_axis)
              *separated_axis =
                  SelfContactFacetPrismSeparationAxis::VertexEdge;
            return true;
          }
          const auto second_vertex_axis = VertexEdgeAxis(
              second_state->vertices[vertex], *first_state, edge);
          if (AxisSeparates<observe>(
                  first_base, first_current,
                  second_base, second_current,
                  first_thickness, second_thickness,
                  second_vertex_axis, counts)) {
            if (separated_axis)
              *separated_axis =
                  SelfContactFacetPrismSeparationAxis::VertexEdge;
            return true;
          }
        }
      }
    }
  }
  if (axis_limit == SelfContactFacetPrismAxisLimit::VertexEdge)
    return false;

  // A represented vertex difference supplies the remaining closest
  // vertex-vertex candidate direction. As above, every fixed candidate axis
  // is tested against endpoint projection hulls for the whole linear sweep.
  for (unsigned first_state_index = 0;
       first_state_index < first_state_count; ++first_state_index) {
    const auto* first_state = first_states[first_state_index];
    for (unsigned second_state_index = 0;
         second_state_index < second_state_count; ++second_state_index) {
      const auto* second_state = second_states[second_state_index];
      for (const auto first_vertex : first_state->vertices) {
        for (const auto second_vertex : second_state->vertices) {
          const auto axis = Difference(first_vertex, second_vertex);
          if (AxisSeparates<observe>(
                  first_base, first_current,
                  second_base, second_current,
                  first_thickness, second_thickness, axis, counts)) {
            if (separated_axis)
              *separated_axis =
                  SelfContactFacetPrismSeparationAxis::VertexVertex;
            return true;
          }
        }
      }
    }
  }
  return false;
}

}  // namespace

bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit,
    SelfContactFacetPrismSeparationAxis* separated_axis,
    bool* valid) noexcept {
  return CertifiedLinearFacetPrismSeparationImpl<true, false>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, separated_axis, valid, nullptr);
}

self_contact_filters::PrismComparison self_contact_filters::ComparePrismHullCoincidence(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    SelfContactFacetPrismAxisLimit axis_limit) noexcept {
  PrismComparison result;
  result.original.separated = CertifiedLinearFacetPrismSeparationImpl<false, true>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, &result.original.axis, &result.original.valid, &result.original.counts);
  result.current.separated = CertifiedLinearFacetPrismSeparationImpl<true, true>(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      axis_limit, &result.current.axis, &result.current.valid, &result.current.counts);
  return result;
}

bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    bool include_edge_axes,
    SelfContactFacetPrismSeparationAxis* separated_axis,
    bool* valid) noexcept {
  return CertifiedLinearFacetPrismSeparation(
      first_base, first_current, first_thickness,
      second_base, second_current, second_thickness,
      include_edge_axes
          ? SelfContactFacetPrismAxisLimit::EdgeCross
          : SelfContactFacetPrismAxisLimit::FaceNormal,
      separated_axis, valid);
}

SelfContactFacetFilterResult ClassifyAcceptedFacetPair(
    const CurrentFixedTriangle& first, double first_thickness,
    std::uint32_t first_complete_rigid_group,
    const CurrentFixedTriangle& second, double second_thickness,
    std::uint32_t second_complete_rigid_group) noexcept {
  if (!std::isfinite(first_thickness) || !(first_thickness > 0) ||
      !std::isfinite(second_thickness) || !(second_thickness > 0) ||
      !Finite(first) || !Finite(second))
    return {SelfContactFacetFilterStatus::InvalidInput,
            SelfContactFacetFilterCategory::ExactRemaining};
  if (first_complete_rigid_group != UINT32_MAX &&
      first_complete_rigid_group == second_complete_rigid_group)
    return {SelfContactFacetFilterStatus::Ok,
            SelfContactFacetFilterCategory::ExcludedSameRigidGroup};
  if (InflatedFacetBoundsSeparated(
          first, first_thickness, second, second_thickness))
    return {SelfContactFacetFilterStatus::Ok,
            SelfContactFacetFilterCategory::CoordinateAabbSeparated};

  SelfContactFacetPrismSeparationAxis axis =
      SelfContactFacetPrismSeparationAxis::None;
  bool valid = false;
  const bool separated = CertifiedLinearFacetPrismSeparation(
      first, first, first_thickness,
      second, second, second_thickness,
      SelfContactFacetPrismAxisLimit::VertexVertex,
      &axis, &valid);
  if (!valid)
    return {SelfContactFacetFilterStatus::InvalidInput,
            SelfContactFacetFilterCategory::ExactRemaining};
  if (!separated)
    return {SelfContactFacetFilterStatus::Ok,
            SelfContactFacetFilterCategory::ExactRemaining};
  SelfContactFacetFilterCategory category =
      SelfContactFacetFilterCategory::ExactRemaining;
  switch (axis) {
    case SelfContactFacetPrismSeparationAxis::FaceNormal:
      category = SelfContactFacetFilterCategory::FaceAxisSeparated;
      break;
    case SelfContactFacetPrismSeparationAxis::EdgeCross:
      category = SelfContactFacetFilterCategory::EdgeCrossAxisSeparated;
      break;
    case SelfContactFacetPrismSeparationAxis::VertexEdge:
      category = SelfContactFacetFilterCategory::VertexEdgeAxisSeparated;
      break;
    case SelfContactFacetPrismSeparationAxis::VertexVertex:
      category = SelfContactFacetFilterCategory::VertexVertexAxisSeparated;
      break;
    case SelfContactFacetPrismSeparationAxis::None:
      break;
  }
  return {SelfContactFacetFilterStatus::Ok, category};
}

}  // namespace tlfea::contact
