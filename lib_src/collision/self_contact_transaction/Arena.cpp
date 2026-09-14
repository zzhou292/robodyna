// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tlfea::contact::self_contact_transaction {
namespace {

bool Product(std::size_t a, std::size_t b, std::size_t* output) noexcept {
  if (!output || (a && b > SIZE_MAX / a)) return false;
  *output = a * b;
  return true;
}

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

Vec3 FaceAxis(const CurrentFixedTriangle& triangle) noexcept {
  const Vec3 second{
      triangle.vertices[2].x - triangle.vertices[0].x,
      triangle.vertices[2].y - triangle.vertices[0].y,
      triangle.vertices[2].z - triangle.vertices[0].z};
  return CrossAxis(EdgeAxis(triangle, 0), second);
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

bool AxisSeparates(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current,
    double first_thickness, double second_thickness,
    Vec3 axis) noexcept {
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

}  // namespace

PairMotionAction ClassifyCandidatePairMotion(
    const MotionSupport& first,
    const SelfContactSweptParentBounds& first_bounds,
    const MotionSupport& second,
    const SelfContactSweptParentBounds& second_bounds) noexcept {
  if (first.motion == SelfContactFacetMotion::CompleteRigidGroup &&
      second.motion == SelfContactFacetMotion::CompleteRigidGroup &&
      first.complete_rigid_group != UINT32_MAX &&
      first.complete_rigid_group == second.complete_rigid_group)
    return PairMotionAction::ExcludedSameRigidGroup;
  const bool separated =
      first_bounds.upper.x < second_bounds.lower.x ||
      second_bounds.upper.x < first_bounds.lower.x ||
      first_bounds.upper.y < second_bounds.lower.y ||
      second_bounds.upper.y < first_bounds.lower.y ||
      first_bounds.upper.z < second_bounds.lower.z ||
      second_bounds.upper.z < first_bounds.lower.z;
  if (first.motion == SelfContactFacetMotion::LinearNodalV1 &&
      second.motion == SelfContactFacetMotion::LinearNodalV1)
    return separated ? PairMotionAction::CertifiedLinearSeparation
                     : PairMotionAction::LinearNodalV1;
  return separated ? PairMotionAction::CertifiedRigidArcSeparation
                   : PairMotionAction::UnsupportedRigidArc;
}

bool CertifiedLinearFacetPrismSeparation(
    const CurrentFixedTriangle& first_base,
    const CurrentFixedTriangle& first_current, double first_thickness,
    const CurrentFixedTriangle& second_base,
    const CurrentFixedTriangle& second_current, double second_thickness,
    bool include_edge_axes,
    FacetPrismSeparationAxis* separated_axis,
    bool* valid) noexcept {
  if (!valid)
    return false;
  if (separated_axis)
    *separated_axis = FacetPrismSeparationAxis::None;
  *valid = std::isfinite(first_thickness) && first_thickness > 0 &&
      std::isfinite(second_thickness) && second_thickness > 0 &&
      Finite(first_base) && Finite(first_current) &&
      Finite(second_base) && Finite(second_current);
  if (!*valid)
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
    if (AxisSeparates(
            first_base, first_current, second_base, second_current,
            first_thickness, second_thickness, axis)) {
      if (separated_axis)
        *separated_axis = FacetPrismSeparationAxis::FaceNormal;
      return true;
    }
  }
  if (!include_edge_axes)
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
          if (AxisSeparates(
                  first_base, first_current,
                  second_base, second_current,
                  first_thickness, second_thickness, axis)) {
            if (separated_axis)
              *separated_axis = FacetPrismSeparationAxis::EdgeCross;
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool MakeLayout(std::size_t nodes, std::size_t surface_parents,
                std::size_t parents, std::size_t facets,
                std::size_t rigid_groups,
                std::size_t broadphase_pair_capacity,
                std::size_t pair_chunk_capacity,
                std::size_t event_capacity,
                std::size_t event_ledger_capacity,
                std::size_t event_hash_capacity,
                std::size_t policy_outcome_capacity,
                std::size_t max_bytes,
                Layout& output) noexcept {
  if (!nodes || !surface_parents || !parents || !facets ||
      !broadphase_pair_capacity || !pair_chunk_capacity ||
      !event_capacity || !event_ledger_capacity ||
      !event_hash_capacity || event_capacity > event_ledger_capacity ||
      event_ledger_capacity > UINT32_MAX ||
      nodes > UINT32_MAX ||
      surface_parents > UINT32_MAX || parents >= UINT32_MAX ||
      facets > UINT32_MAX || pair_chunk_capacity > UINT32_MAX)
    return false;
  std::size_t vector_values = 0;
  std::size_t identity_references = 0;
  std::size_t chunk_paths = 0;
  std::size_t chunk_events = 0;
  if (!Product(nodes, 3, &vector_values) ||
      !Product(facets, 3, &identity_references) ||
      !Product(pair_chunk_capacity, 2, &chunk_paths) ||
      !Product(pair_chunk_capacity, 15, &chunk_events) ||
      identity_references > UINT32_MAX)
    return false;
  tl::util::BoundedArenaLayout builder(max_bytes);
  Layout next;
  if (!builder.Append<double>(vector_values, next.accepted_positions) ||
      !builder.Append<double>(vector_values, next.accepted_velocities) ||
      !builder.Append<double>(vector_values, next.prepared_positions) ||
      !builder.Append<double>(vector_values, next.prepared_velocities) ||
      !builder.Append<tl::fea::NodalRigidGroupSnapshot>(
          rigid_groups, next.accepted_rigid_groups) ||
      !builder.Append<tl::fea::NodalRigidGroupSnapshot>(
          rigid_groups, next.prepared_rigid_groups) ||
      !builder.Append<std::uint32_t>(
          nodes, next.node_rigid_groups) ||
      !builder.Append<std::uint32_t>(
          surface_parents, next.surface_to_active) ||
      !builder.Append<std::uint32_t>(
          parents + 1, next.parent_facet_offsets) ||
      !builder.Append<FixedContactFacet>(
          facets, next.facet_descriptors) ||
      !builder.Append<MotionSupport>(
          parents, next.parent_motion) ||
      !builder.Append<MotionSupport>(
          facets, next.facet_motion) ||
      !builder.Append<std::uint32_t>(facets, next.triangle_order) ||
      !builder.Append<std::uint32_t>(
          identity_references, next.vertex_identity_order) ||
      !builder.Append<std::uint32_t>(
          identity_references, next.edge_identity_order) ||
      !builder.Append<CurrentFixedTriangle>(
          facets, next.accepted_triangles) ||
      !builder.Append<CurrentFixedTriangle>(
          facets, next.prepared_triangles) ||
      !builder.Append<SelfContactPairKey>(
          broadphase_pair_capacity, next.broadphase_pairs) ||
      !builder.Append<FacetPairCursor>(
          broadphase_pair_capacity, next.facet_pair_cursors) ||
      !builder.Append<std::uint32_t>(
          broadphase_pair_capacity, next.facet_pair_heap) ||
      !builder.Append<FixedTrianglePair>(
          pair_chunk_capacity, next.facet_pair_chunk) ||
      !builder.Append<RepresentedTrianglePath>(
          chunk_paths, next.chunk_paths) ||
      !builder.Append<RepresentedTrianglePair>(
          pair_chunk_capacity, next.chunk_represented_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_chunk_capacity, next.chunk_canonical_pairs) ||
      !builder.Append<RepresentedIntervalPairKey>(
          pair_chunk_capacity, next.chunk_raw_canonical_pairs) ||
      !builder.Append<PairMotionAction>(
          pair_chunk_capacity, next.chunk_motion_actions) ||
      !builder.Append<RepresentedIntervalResult>(
          pair_chunk_capacity, next.chunk_crossings) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          pair_chunk_capacity, next.chunk_validated_outcomes) ||
      !builder.Append<SelfContactForceEvent>(
          chunk_events, next.chunk_events) ||
      !builder.Append<AcceptedEventCertificate>(
          chunk_events, next.chunk_certificates) ||
      !builder.Append<SelfContactSweptParentBounds>(
          surface_parents, next.swept_parent_bounds) ||
      !builder.Append<SelfContactSweptParentBounds>(
          facets, next.swept_facet_bounds) ||
      !builder.Append<SelfContactForceEvent>(
          event_capacity, next.accepted_events) ||
      !builder.Append<AcceptedEventCertificate>(
          event_ledger_capacity, next.accepted_certificates) ||
      !builder.Append<std::uint32_t>(
          event_hash_capacity, next.accepted_event_hash) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          pair_chunk_capacity, next.chunk_policy_outcomes) ||
      !builder.Append<SelfContactCandidatePolicyOutcome>(
          policy_outcome_capacity, next.policy_outcomes))
    return false;
  next.bytes = builder.bytes();
  output = next;
  return true;
}

Buffers Bind(void* base, const Layout& layout) noexcept {
  using tl::util::ArenaPointer;
  return {
      ArenaPointer<double>(base, layout.accepted_positions),
      ArenaPointer<double>(base, layout.accepted_velocities),
      ArenaPointer<double>(base, layout.prepared_positions),
      ArenaPointer<double>(base, layout.prepared_velocities),
      ArenaPointer<tl::fea::NodalRigidGroupSnapshot>(
          base, layout.accepted_rigid_groups),
      ArenaPointer<tl::fea::NodalRigidGroupSnapshot>(
          base, layout.prepared_rigid_groups),
      ArenaPointer<std::uint32_t>(base, layout.node_rigid_groups),
      ArenaPointer<std::uint32_t>(base, layout.surface_to_active),
      ArenaPointer<std::uint32_t>(base, layout.parent_facet_offsets),
      ArenaPointer<FixedContactFacet>(base, layout.facet_descriptors),
      ArenaPointer<MotionSupport>(base, layout.parent_motion),
      ArenaPointer<MotionSupport>(base, layout.facet_motion),
      ArenaPointer<std::uint32_t>(base, layout.triangle_order),
      ArenaPointer<std::uint32_t>(base, layout.vertex_identity_order),
      ArenaPointer<std::uint32_t>(base, layout.edge_identity_order),
      ArenaPointer<CurrentFixedTriangle>(base, layout.accepted_triangles),
      ArenaPointer<CurrentFixedTriangle>(base, layout.prepared_triangles),
      ArenaPointer<SelfContactPairKey>(
          base, layout.broadphase_pairs),
      ArenaPointer<FacetPairCursor>(base, layout.facet_pair_cursors),
      ArenaPointer<std::uint32_t>(base, layout.facet_pair_heap),
      ArenaPointer<FixedTrianglePair>(base, layout.facet_pair_chunk),
      ArenaPointer<RepresentedTrianglePath>(base, layout.chunk_paths),
      ArenaPointer<RepresentedTrianglePair>(
          base, layout.chunk_represented_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(
          base, layout.chunk_canonical_pairs),
      ArenaPointer<RepresentedIntervalPairKey>(
          base, layout.chunk_raw_canonical_pairs),
      ArenaPointer<PairMotionAction>(
          base, layout.chunk_motion_actions),
      ArenaPointer<RepresentedIntervalResult>(
          base, layout.chunk_crossings),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.chunk_validated_outcomes),
      ArenaPointer<SelfContactForceEvent>(base, layout.chunk_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.chunk_certificates),
      ArenaPointer<SelfContactSweptParentBounds>(
          base, layout.swept_parent_bounds),
      ArenaPointer<SelfContactSweptParentBounds>(
          base, layout.swept_facet_bounds),
      ArenaPointer<SelfContactForceEvent>(base, layout.accepted_events),
      ArenaPointer<AcceptedEventCertificate>(
          base, layout.accepted_certificates),
      ArenaPointer<std::uint32_t>(base, layout.accepted_event_hash),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.chunk_policy_outcomes),
      ArenaPointer<SelfContactCandidatePolicyOutcome>(
          base, layout.policy_outcomes)};
}

}  // namespace tlfea::contact::self_contact_transaction
