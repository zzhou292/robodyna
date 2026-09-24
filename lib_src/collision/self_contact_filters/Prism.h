// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arithmetic.h"

namespace tlfea::contact::self_contact_filters::detail {
// This is a necessary-no-separator check for the existing endpoint HULLS,
// not a contact/topology certificate. Even cross-time coincidence prevents
// any strictly disjoint projection hulls. All geometry was validated first;
// numeric equality intentionally includes signed zero and ignores source IDs.
template <bool observe, class Triangle>
TL_SURFACE_HD inline bool EndpointHullsShareVertex(
    const Triangle& first_base, const Triangle& first_current,
    const Triangle& second_base, const Triangle& second_current,
    self_contact_filters::PrismCounts* counts) noexcept {
  const Triangle* first[]{&first_base, &first_current};
  const Triangle* second[]{&second_base, &second_current};
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

template <bool reject_coincident_hulls, bool observe, class Triangle>
TL_SURFACE_HD inline bool CertifiedLinearFacetPrismSeparationImpl(
    const Triangle& first_base,
    const Triangle& first_current, double first_thickness,
    const Triangle& second_base,
    const Triangle& second_current, double second_thickness,
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
      ScalarFinite(first_thickness) && first_thickness > 0 &&
      ScalarFinite(second_thickness) && second_thickness > 0 &&
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
  const Triangle* first_states[2]{
      &first_base, &first_current};
  const Triangle* second_states[2]{
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

template <class Triangle>
TL_SURFACE_HD inline SelfContactFacetFilterResult ClassifyAcceptedFacetPairImpl(
    const Triangle& first, double first_thickness,
    std::uint32_t first_complete_rigid_group,
    const Triangle& second, double second_thickness,
    std::uint32_t second_complete_rigid_group) noexcept {
  if (!ScalarFinite(first_thickness) || !(first_thickness > 0) ||
      !ScalarFinite(second_thickness) || !(second_thickness > 0) ||
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
  const bool separated = CertifiedLinearFacetPrismSeparationImpl<true, false>(
      first, first, first_thickness,
      second, second, second_thickness,
      SelfContactFacetPrismAxisLimit::VertexVertex,
      &axis, &valid, nullptr);
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

}  // namespace tlfea::contact::self_contact_filters::detail
