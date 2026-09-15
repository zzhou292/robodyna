// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedContactFacetTypes.h"

#include <cstddef>
#include <cstdint>
#include <new>

namespace tlfea::contact {

// Stable identity of one physical contact facet.  It is independent of input
// array order and current coordinates.
struct FixedTriangleKey {
  std::uint64_t source_instance_id = 0;
  std::uint64_t parent_eid = 0;
  unsigned level = 0;
  unsigned local_facet = 0;
};

// One fixed physical facet evaluated at current nodal positions.  Canonical
// vertex/edge keys come from FixedContactFacetBinding; coordinates are never
// welded and therefore preserve coincident independent layers.
struct CurrentFixedTriangle {
  FixedTriangleKey key;
  Vec3 vertices[3];
  FacetVertexKey vertex_keys[3];
  FacetEdgeKey edge_keys[3];
};

struct FixedTrianglePair {
  std::uint32_t first = 0;
  std::uint32_t second = 0;
};

// Optional local-incidence mask for one exact triangle pair.  Bit one means
// that the corresponding task is omitted.  Slots are defined in canonical
// FixedTriangleKey order: 2*v is canonical-first vertex v -> second face,
// 2*v+1 is second vertex v -> first face, and 6+3*ea+eb is first edge ea
// against second edge eb.  Bit 15 is outside the fixed task roster.
struct FixedTriangleFeatureTaskMask {
  std::uint16_t local_tasks = 0;
};

constexpr unsigned FixedTriangleVertexFaceTaskSlot(
    unsigned source_side, unsigned local_vertex) noexcept {
  return 2 * local_vertex + source_side;
}

constexpr unsigned FixedTriangleEdgeEdgeTaskSlot(
    unsigned first_local_edge, unsigned second_local_edge) noexcept {
  return 6 + 3 * first_local_edge + second_local_edge;
}

constexpr std::uint16_t FixedTriangleFeatureTaskBit(
    unsigned slot) noexcept {
  return slot < 15 ? static_cast<std::uint16_t>(1u << slot) : 0;
}

constexpr std::uint16_t FixedTriangleFeatureTaskBits = 0x7fffu;

enum class FixedTriangleCandidateKind : std::uint8_t {
  VertexFace,
  EdgeEdge,
};

enum class FixedTriangleStratumKind : std::uint8_t {
  Vertex,
  Edge,
  Face,
};

// Immutable closest stratum on the target triangle.  Exactly one member is
// meaningful according to kind.  A boundary vertex/edge is shared across
// incident fixed facets, while a face-interior stratum is facet-specific.
struct FixedTriangleStratumKey {
  FixedTriangleStratumKind kind = FixedTriangleStratumKind::Vertex;
  union {
    FacetVertexKey vertex;
    FacetEdgeKey edge;
    FixedTriangleKey face;
  };
  FixedTriangleStratumKey() noexcept : vertex{} {}
  void SetVertex(const FacetVertexKey& value) noexcept {
    new (&vertex) FacetVertexKey(value);
    kind = FixedTriangleStratumKind::Vertex;
  }
  void SetEdge(const FacetEdgeKey& value) noexcept {
    new (&edge) FacetEdgeKey(value);
    kind = FixedTriangleStratumKind::Edge;
  }
  void SetFace(const FixedTriangleKey& value) noexcept {
    new (&face) FixedTriangleKey(value);
    kind = FixedTriangleStratumKind::Face;
  }
};

struct FixedTriangleVertexFaceKey {
  FacetVertexKey vertex;
  FixedTriangleStratumKey target;
};

struct FixedTriangleEdgeEdgeKey {
  FacetEdgeKey edges[2];
};

// Canonical geometric identity.  Producing triangle/local ordinals are not
// part of this key, so seam repetitions cannot become duplicate force/energy
// events in later consumers.
struct FixedTriangleFeatureKey {
  FixedTriangleCandidateKind kind = FixedTriangleCandidateKind::VertexFace;
  union {
    FixedTriangleVertexFaceKey vertex_face;
    FixedTriangleEdgeEdgeKey edge_edge;
  };
  FixedTriangleFeatureKey() noexcept : vertex_face{} {}
  void SetEdgeEdge() noexcept {
    new (&edge_edge) FixedTriangleEdgeEdgeKey{};
    kind = FixedTriangleCandidateKind::EdgeEdge;
  }
};

// Geometry only.  Active parent-use ownership, thickness, area, stiffness and
// force mapping are intentionally absent.  For VertexFace, points[0] is the
// canonical vertex and points[1] is its closest face point.  For EdgeEdge,
// edges and points are in canonical edge-key order.  Edge parameters use each
// FacetEdgeKey's canonical endpoint order.  A VertexFace candidate targeting
// an Edge publishes that target parameter in edge_parameters[0].
struct FixedTriangleFeatureCandidate {
  FixedTriangleFeatureKey key;
  // Deterministically selected producing facet/task provenance.  This is a
  // value, not geometric identity.
  FixedTriangleKey triangles[2];
  unsigned local_features[2] = {0, 0};
  Vec3 points[2];
  double face_weights[3] = {1, 0, 0};
  double edge_parameters[2] = {0, 0};
  double distance_m = 0;
  // Upper bound on the binary64 distance between the unconstrained rounded
  // closest-point evaluation and the deterministic point represented by the
  // published weights/parameters. Zero means the representations coincide.
  double representation_error_m = 0;
};

enum class FixedTriangleIntersectionKind : std::uint8_t {
  Transverse,
  CoplanarTouch,
  CoplanarOverlap,
};

// Only topology local to the two consumed facets can suppress an intersection.
// None means that the intersection needs a later explicit admission/rejection
// policy.  Same PID, same parent, coordinate coincidence, body and tie guesses
// are not represented and therefore cannot cause an exclusion here.
// SharedVertexOnly means the complete exact intersection set is the shared
// canonical vertex.  This includes a transverse plane cut along an incident
// edge when that edge leaves the other triangle's exact vertex tangent cone;
// any positive-length or positive-area overlap remains unexcluded.
enum class FixedTriangleLocalExclusion : std::uint8_t {
  None,
  IdenticalFace,
  SharedVertexOnly,
  SharedEdgeOnly,
};

struct FixedTriangleIntersection {
  FixedTriangleKey triangles[2];
  FixedTriangleIntersectionKind kind =
      FixedTriangleIntersectionKind::Transverse;
  FixedTriangleLocalExclusion local_exclusion =
      FixedTriangleLocalExclusion::None;
};

inline bool RequiresIntersectionAdmission(
    const FixedTriangleIntersection& value) noexcept {
  return value.local_exclusion == FixedTriangleLocalExclusion::None;
}

enum class FixedTriangleDiscoveryStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  OutOfRange,
  DegenerateTriangle,
  NonFiniteResult,
  ResourceLimit,
  IdentityMismatch,
};

enum class FixedTriangleArithmeticReason : std::uint8_t {
  None,
  TriangleValidation,
  IntersectionPredicate,
  ClosestPoint,
  ExactClosestStratum,
  EdgeClosestPoint,
  EdgeInteriorRepresentation,
  FaceInteriorRepresentation,
  SegmentClosestPoints,
};

struct FixedTriangleDiscoveryReport {
  FixedTriangleDiscoveryStatus status = FixedTriangleDiscoveryStatus::Ok;
  std::size_t input_pair = SIZE_MAX;
  std::size_t input_task = SIZE_MAX;
  FixedTriangleArithmeticReason arithmetic_reason =
      FixedTriangleArithmeticReason::None;
  // Executed feature tasks. Discover executes exactly six directed VF and
  // nine EE tasks per valid pair. DiscoverMasked omits authenticated local
  // incidence before geometry and keeps this as an alias of
  // exact_executed_tasks.
  std::size_t feature_tasks = 0;
  std::size_t triangle_references = 0;
  std::size_t triangles = 0;
  std::size_t vertex_references = 0;
  std::size_t vertices = 0;
  std::size_t edge_references = 0;
  std::size_t edges = 0;
  std::size_t raw_feature_candidates = 0;
  std::size_t feature_candidates = 0;
  std::size_t raw_intersections = 0;
  std::size_t intersections = 0;
  const char* message = "OK";
  // Appended diagnostics retain the original report prefix.  Discover has
  // potential==exact_executed and local_masked==0.  DiscoverMasked partitions
  // potential tasks into caller-authenticated local omissions and executions
  // for a successful complete query.
  std::size_t potential_tasks = 0;
  std::size_t local_masked_tasks = 0;
  std::size_t exact_executed_tasks = 0;
};

inline constexpr unsigned FixedTriangleFeatureMaximumWorkerCount = 8;

struct FixedTriangleFeatureLimits {
  std::size_t max_input_pairs = 4096;
  std::size_t max_triangle_references = 2 * 4096;
  std::size_t max_vertex_references = 6 * 4096;
  std::size_t max_edge_references = 6 * 4096;
  std::size_t max_raw_feature_candidates = 15 * 4096;
  std::size_t max_feature_candidates = 15 * 4096;
  std::size_t max_raw_intersections = 4096;
  std::size_t max_intersections = 4096;
  std::size_t max_host_bytes = 64u << 20;
  // Total persistent startup worker threads. The caller only stages and
  // canonically reduces pair results; it is not counted as a worker.
  unsigned worker_count = 1;
};

struct FixedTriangleFeatureForecast {
  std::size_t triangle_ledger_capacity = 0;
  std::size_t vertex_ledger_capacity = 0;
  std::size_t edge_ledger_capacity = 0;
  std::size_t raw_feature_capacity = 0;
  std::size_t feature_publication_capacity = 0;
  std::size_t pair_intersection_capacity = 0;
  std::size_t raw_intersection_capacity = 0;
  std::size_t intersection_publication_capacity = 0;
  std::size_t owned_host_bytes = 0;
  // Appended worker diagnostics preserve the original forecast prefix.
  std::size_t pair_status_capacity = 0;
  std::size_t pair_status_bytes = 0;
  unsigned worker_count = 0;
  std::size_t worker_metadata_bytes = 0;
  // Includes each explicitly mapped worker stack and its guard page.
  std::size_t worker_stack_bytes = 0;
  // Workers and their stacks persist, so this equals owned_host_bytes.
  std::size_t startup_host_bytes = 0;
};

struct FixedTriangleFeaturePreflight {
  FixedTriangleDiscoveryReport report;
  FixedTriangleFeatureForecast forecast;
};

struct FixedTriangleFeatureView {
  const FixedTriangleFeatureCandidate* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
};

struct FixedTriangleIntersectionView {
  const FixedTriangleIntersection* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
};

}  // namespace tlfea::contact
