// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SelfContactSurfaceTypes.h"
#include "weighted_surface/Mapping.h"

namespace tlfea::contact {
enum class FixedContactFacetProfile { WeightedReferenceSurfaceV1 };
struct FixedContactFacetConfig {
  FixedContactFacetProfile profile = FixedContactFacetProfile::WeightedReferenceSurfaceV1;
  unsigned level = 0; // Immutable n=1<<level; only levels 0,1,2 are admitted.
};
enum class FixedContactFacetStatus { Ok, AlreadyInitialized, InvalidInput, ResourceLimit, OutOfRange };
struct FixedContactFacetReport {
  FixedContactFacetStatus status = FixedContactFacetStatus::Ok;
  const char* message = "OK";
};
struct FixedContactFacetLimits {
  std::size_t max_parents = 2048, max_facets = 65536, max_host_bytes = 64u << 20;
  static constexpr FixedContactFacetLimits Vehicle() noexcept {
    return {524288, 16777216, 2ull << 30};
  }
};
struct FixedContactFacetForecast {
  std::size_t parents = 0, facets = 0;
  std::size_t q4_template_vertices = 0, t3_template_vertices = 0;
  std::size_t q4_template_facets = 0, t3_template_facets = 0;
  std::size_t template_arena_bytes = 0, retained_source_bytes = 0;
  std::size_t scratch_payload_bytes = 0; // Fixed startup/query staging, no parent-sized arrays.
  std::size_t owned_payload_bytes = 0, startup_payload_bytes = 0;
};
struct FixedContactFacetPreflight {
  FixedContactFacetReport report;
  FixedContactFacetForecast forecast;
};
enum class FacetVertexKind : std::uint8_t { SourceVertex, SourceEdge, ParentInterior };
// Source-edge position=(1-numerator/denominator)*first + fraction*second.
// Endpoints are source-NID sorted and the dyadic fraction is reduced. Interior
// keys use parent EID plus level/grid coordinates. No coordinate welding.
struct FacetVertexKey {
  std::uint64_t source_instance_id = 0, first = 0, second = 0;
  FacetVertexKind kind = FacetVertexKind::SourceVertex;
  unsigned numerator = 0, denominator = 1, level = 0, grid_i = 0, grid_j = 0;
};
struct FacetEdgeKey {
  FacetVertexKey endpoints[2];
  std::uint64_t parent_eid = 0; // Zero only for a physical parent boundary.
  bool parent_boundary = false;
};
struct FixedContactFacet {
  tl::fea::ShellPlasticityParentInput source;
  tl::fea::ShellSectionLaw law = tl::fea::ShellSectionLaw::Unspecified;
  std::uint64_t source_instance_id = 0;
  std::size_t parent_index = SIZE_MAX;
  unsigned level = 0, local_facet = 0, material_points = 0;
  double reference_half_thickness_m = 0;
  WeightedSurfacePoint vertices[3];
  FacetVertexKey vertex_keys[3];
  FacetEdgeKey edge_keys[3];
};
struct FacetApproximationBound {
  // Exact native polynomial -> ideal fixed facets, then represented vertex
  // construction. These are geometric error bounds, never physical radii.
  double bilinear_error_upper_m = 0;
  double vertex_roundoff_upper_m = 0;
  double total_error_upper_m = 0;
};
struct FacetApproximationSummary {
  std::size_t parents = 0;
  std::size_t positive_bilinear_parents = 0;
  std::size_t positive_vertex_roundoff_parents = 0;
  double maximum_bilinear_error_upper_m = 0;
  double maximum_vertex_roundoff_upper_m = 0;
  double maximum_total_error_upper_m = 0;
  std::size_t maximum_bilinear_parent = SIZE_MAX;
  std::size_t maximum_vertex_roundoff_parent = SIZE_MAX;
  std::size_t maximum_total_parent = SIZE_MAX;
};
} // namespace tlfea::contact
