// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Q4SurfaceMapping.h"
#include "../elements/ShellBatchPlasticityBinding.h"

namespace tlfea::contact {
// Geometry declaration only: current bilinear/linear midsurface plus a constant
// reference half-thickness distance. No original contact-card parity is claimed.
enum class SelfContactSurfaceProfile { FrictionlessReferenceThicknessShellSubsetV1 };
enum class SelfContactSurfaceStatus {
  Ok, AlreadyInitialized, InvalidInput, ResourceLimit, IdentityMismatch,
  DuplicateParent, UnsupportedReferencePlane
};
struct SelfContactSurfaceReport {
  SelfContactSurfaceStatus status = SelfContactSurfaceStatus::Ok;
  std::size_t selection = SIZE_MAX;
  const char* message = "OK";
};
struct SelfContactParentSelection {
  std::size_t catalog_row = SIZE_MAX;
  tl::fea::ShellBindingFamily family = tl::fea::ShellBindingFamily::None;
  std::size_t family_index = SIZE_MAX;
  std::uint64_t source_parent_id = 0, source_part_id = 0;
};
struct SelfContactSurfaceInput {
  const SelfContactParentSelection* parents = nullptr;
  std::size_t parent_count = 0;
  SelfContactSurfaceProfile profile = SelfContactSurfaceProfile::FrictionlessReferenceThicknessShellSubsetV1;
};
struct SelfContactSurfaceLimits {
  std::size_t max_parents = 2048, max_nodes = 2048;
  std::size_t max_vertices = 2048, max_edges = 8192;
  std::size_t max_host_bytes = 64u << 20;
  static constexpr SelfContactSurfaceLimits Vehicle() noexcept {
    return {524288, 524288, 524288, 2097152, 2ull << 30};
  }
};
struct SelfContactSurfaceForecast {
  std::size_t parent_count = 0, corner_capacity = 0;
  std::size_t vertex_capacity = 0, edge_capacity = 0;
  std::size_t arena_bytes = 0; // Exact owned allocation, including unused tails.
  std::size_t retained_source_bytes = 0; // Complete physical backing, charged once here.
  std::size_t owned_payload_bytes = 0;
  std::size_t validation_scratch_bytes = 0;
  // Conservative complete payload bound: includes source-index scratch although
  // it retires before the owned arena is allocated. Not a process RSS forecast.
  std::size_t startup_payload_bytes = 0;
};
struct SelfContactSurfacePreflight {
  SelfContactSurfaceReport report;
  SelfContactSurfaceForecast forecast;
};
struct SelfContactSurfaceParent {
  tl::fea::ShellPlasticityParentInput source;
  tl::fea::ShellSectionLaw law = tl::fea::ShellSectionLaw::Unspecified;
  unsigned material_points = 0, arity = 0;
  double reference_half_thickness_m = 0;
  // Only the arity-selected record is active. Both maps have zero geometric
  // offset; reference thickness above is NOT a director displacement.
  SurfaceQ4 q4;
  SurfaceTriangle t3;
  std::uint32_t vertices[4]{}, edges[4]{};
};
struct SelfContactVertex {
  std::uint64_t source_node_id = 0;
  std::uint32_t domain_node = 0, use_offset = 0, use_count = 0;
};
struct SelfContactEdge {
  std::uint64_t source_node_ids[2]{}; // Ascending source IDs, never coordinate deduplication.
  std::uint32_t vertices[2]{}, use_offset = 0, use_count = 0;
};
struct SelfContactVertexUse {
  std::uint64_t source_node_id = 0, source_parent_id = 0;
  std::uint32_t domain_node = 0, parent = 0, local = 0;
};
struct SelfContactEdgeUse {
  std::uint64_t source_node_ids[2]{}, source_parent_id = 0;
  std::uint32_t parent = 0, local = 0;
};
// Invalid indices never mean that a candidate pair is admissible.
enum class TopologicalIncidence { Invalid, Disjoint, Incident };
} // namespace tlfea::contact
