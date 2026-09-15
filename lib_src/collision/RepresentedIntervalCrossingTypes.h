// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedContactFacetTypes.h"

#include <cstddef>
#include <cstdint>

namespace tlfea::contact {

// This motion declaration is part of the certificate.  Only the exact real
// chord between the two represented binary64 endpoints is admitted by V1.
// RigidArc and Nonlinear are named unsupported paths; their endpoints must
// never be silently substituted for a linear trajectory.
enum class RepresentedMotion : std::uint8_t {
  LinearNodalV1,
  RigidArc,
  Nonlinear,
};

struct RepresentedTrianglePathKey {
  std::uint64_t source_instance_id = 0;
  std::uint64_t parent_eid = 0;
  unsigned level = 0;
  unsigned local_facet = 0;
};

struct RepresentedVertexPath {
  FacetVertexKey key;
  Vec3 endpoint[2];
};

// edge_keys[i] connects vertices[i] and vertices[(i+1)%3].  The edge key
// itself remains in the canonical orientation supplied by the fixed-facet
// binding.  Vertex order and winding are not identity.
struct RepresentedTrianglePath {
  RepresentedTrianglePathKey key;
  RepresentedMotion motion = RepresentedMotion::LinearNodalV1;
  RepresentedVertexPath vertices[3];
  FacetEdgeKey edge_keys[3];
};

struct RepresentedTrianglePair {
  std::uint32_t first = 0;
  std::uint32_t second = 0;
};

struct RepresentedIntervalPairKey {
  RepresentedTrianglePathKey paths[2];
};

enum class RepresentedFeatureKind : std::uint8_t {
  None,
  VertexFace,
  EdgeEdge,
  TriangleIntersection,
};

// Only fields selected by kind participate in identity.  VertexFace uses
// vertex/face; EdgeEdge uses edges.  TriangleIntersection is keyed by pair.
struct RepresentedFeaturePathKey {
  RepresentedFeatureKind kind = RepresentedFeatureKind::None;
  FacetVertexKey vertex;
  RepresentedTrianglePathKey face;
  FacetEdgeKey edges[2];
};

enum class RepresentedIntervalClassification : std::uint8_t {
  CertifiedSeparated,
  CertifiedCrossingContact,
  Unresolved,
};

enum class RepresentedIntervalReason : std::uint8_t {
  None,
  UnsupportedMotion,
  DegenerateGeometry,
  WorkExhausted,
  ExactArithmeticRange,
};

enum class RepresentedIntersectionGeometry : std::uint8_t {
  None,
  Transverse,
  Coplanar,
  PersistentPhysicalContact,
};

struct RepresentedIntervalResult {
  RepresentedIntervalPairKey key;
  RepresentedFeaturePathKey feature;
  RepresentedIntervalClassification classification =
      RepresentedIntervalClassification::Unresolved;
  RepresentedIntervalReason reason =
      RepresentedIntervalReason::WorkExhausted;
  RepresentedIntersectionGeometry geometry =
      RepresentedIntersectionGeometry::None;
  // Exact dyadic witness_time_numerator / 2^witness_time_depth.  It is
  // meaningful only for CertifiedCrossingContact.
  std::uint64_t witness_time_numerator = 0;
  unsigned witness_time_depth = 0;
  std::size_t work = 0;
};

enum class RepresentedIntervalStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  IdentityMismatch,
  ResourceLimit,
};

struct RepresentedIntervalReport {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::Ok;
  std::size_t input_path = SIZE_MAX;
  std::size_t input_pair = SIZE_MAX;
  std::size_t input_paths = 0;
  std::size_t input_pairs = 0;
  std::size_t unique_pairs = 0;
  std::size_t certified_separated = 0;
  std::size_t certified_crossing_contact = 0;
  std::size_t unresolved = 0;
  std::size_t work = 0;
  const char* message = "OK";
};

inline constexpr unsigned RepresentedIntervalMaximumWorkerCount = 8;

struct RepresentedIntervalLimits {
  std::size_t max_paths = 2048;
  std::size_t max_input_pairs = 4096;
  std::size_t max_results = 4096;
  std::size_t max_work_per_pair = 4095;
  std::size_t max_total_work = 1u << 20;
  unsigned max_depth = 20;
  std::size_t max_host_bytes = 64u << 20;
  // Total persistent startup worker threads. The caller validates, stages and
  // canonically reduces pair results; it is not counted as a worker.
  unsigned worker_count = 1;
};

struct RepresentedIntervalForecast {
  std::size_t path_index_capacity = 0;
  std::size_t pair_capacity = 0;
  std::size_t result_capacity = 0;
  std::size_t vertex_ledger_capacity = 0;
  // Per-worker capacity. dfs_frame_bytes sums this arena over all workers.
  std::size_t dfs_frame_capacity = 0;
  std::size_t path_index_bytes = 0;
  std::size_t pair_bytes = 0;
  std::size_t result_bytes = 0;
  std::size_t vertex_ledger_bytes = 0;
  std::size_t dfs_frame_bytes = 0;
  // Sum of one fixed ExactScratch object per worker.
  std::size_t exact_scratch_bytes = 0;
  std::size_t owned_host_bytes = 0;
  // Appended worker diagnostics preserve the original forecast prefix.
  std::size_t pair_status_capacity = 0;
  std::size_t pair_status_bytes = 0;
  unsigned worker_count = 0;
  std::size_t worker_metadata_bytes = 0;
  // Includes every explicitly mapped worker stack and its guard page.
  std::size_t worker_stack_bytes = 0;
  // Workers, exact scratch and stacks persist after Initialize.
  std::size_t startup_host_bytes = 0;
};

struct RepresentedIntervalPreflight {
  RepresentedIntervalReport report;
  RepresentedIntervalForecast forecast;
};

struct RepresentedIntervalResultView {
  const RepresentedIntervalResult* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
};

}  // namespace tlfea::contact
