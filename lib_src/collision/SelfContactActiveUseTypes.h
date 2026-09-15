// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "FixedContactFacetTypes.h"
#include "Q4ParametricContact.h"
#include "lib_src/constraints/NodalRigidAssemblyBinding.h"
#include "lib_src/constraints/tied_shell/runtime/CinStageTypes.h"

namespace tlfea::contact {
inline constexpr const char* SymmetricDirectedVertexDualReferenceV1 =
    "SymmetricDirectedVertexDualReferenceV1";
inline constexpr const char*
    SymmetricDirectedVertexAndEdgePointDualReferenceV2 =
        "SymmetricDirectedVertexAndEdgePointDualReferenceV2";

enum class SelfContactActiveUsePolicy {
  SymmetricDirectedVertexDualReferenceV1,
  SymmetricDirectedVertexAndEdgePointDualReferenceV2
};
enum class SelfContactActiveUseStatus {
  Ok, AlreadyInitialized, InvalidInput, ResourceLimit, IdentityMismatch,
  ReferenceFailure, Unrepresentable
};
struct SelfContactActiveUseReport {
  SelfContactActiveUseStatus status = SelfContactActiveUseStatus::Ok;
  std::size_t parent = SIZE_MAX, feature = SIZE_MAX;
  const char* message = "OK";
};
struct SelfContactCinWitnessSource {
  const tl::constraints::tied_shell::TiedCinAttachmentModel* model = nullptr;
  const tl::constraints::tied_shell::cin::WitnessRange* ranges = nullptr;
  const tl::constraints::tied_shell::cin::ActiveWitness* witnesses = nullptr;
  std::size_t range_count = 0, witness_count = 0;
};
struct SelfContactActiveUseSource {
  // Null means that this isolated source contains no rigid groups. When
  // supplied, this must be the actual NodalRigidAssemblyBinding over S0's
  // complete domain; source-kind labels or shell PIDs are never substitutes.
  const tl::fea::NodalRigidAssemblyBinding* rigid = nullptr;
  // All-null/zero means no CIN. Nonempty input must be the complete admitted
  // model/range/witness roster. The arrays are copied during initialization.
  SelfContactCinWitnessSource cin;
};
struct SelfContactActiveUseLimits {
  std::size_t max_parents = 2048, max_facets = 65536;
  std::size_t max_vertices = 65536, max_edges = 131072;
  std::size_t max_vertex_uses = 65536, max_edge_uses = 131072;
  std::size_t max_nodes = 2048, max_cin_rows = 2048, max_cin_witnesses = 8192;
  std::size_t max_host_bytes = 256u << 20;
  static constexpr SelfContactActiveUseLimits Vehicle() noexcept {
    return {524288, 16777216, 16777216, 33554432, 16777216, 33554432,
            524288, 65536, 262144, 16ull << 30};
  }
};
struct SelfContactActiveUseCounts {
  std::size_t parents = 0, facets = 0, vertices = 0, edges = 0;
  std::size_t vertex_uses = 0, edge_uses = 0;
};
struct SelfContactActiveUseForecast : SelfContactActiveUseCounts {
  std::size_t node_roles = 0, cin_rows = 0, cin_witnesses = 0;
  std::size_t arena_bytes = 0;
  std::size_t retained_facet_bytes = 0, retained_rigid_bytes = 0;
  std::size_t retained_cin_bytes = 0;
  // Immutable domain-node -> source-ordered CIN row index.
  std::size_t cin_index_bytes = 0;
  // Transient uint32 source-order indexes used to canonicalize feature uses.
  // This allocation retires before the immutable binding is published.
  std::size_t startup_index_bytes = 0;
  std::size_t owned_payload_bytes = 0, startup_payload_bytes = 0;
};
struct SelfContactActiveUsePreflight {
  SelfContactActiveUseReport report;
  SelfContactActiveUseForecast forecast;
};

enum class SelfContactReferenceAreaModel : std::uint8_t {
  T3CertifiedNativeArea,
  Q4CenterAreaContactModel
};
struct SelfContactParentUse {
  std::size_t surface_parent = SIZE_MAX;
  tl::fea::ShellPlasticityParentInput source;
  unsigned arity = 0, level = 0;
  std::uint32_t nodes[4]{};
  std::uint32_t facet_offset = 0, facet_count = 0;
  double reference_half_thickness_m = 0;
  Q4CertifiedIntegral reference_area_m2;
  SelfContactReferenceAreaModel area_model =
      SelfContactReferenceAreaModel::T3CertifiedNativeArea;
};
struct SelfContactFacetUse {
  std::uint32_t parent = 0, local_facet = 0;
  std::uint32_t vertex_features[3]{}, edge_features[3]{};
  std::uint32_t vertex_uses[3]{}, edge_uses[3]{};
};
struct SelfContactVertexFeature {
  FacetVertexKey key;
  std::uint32_t use_offset = 0, use_count = 0;
};
struct SelfContactEdgeFeature {
  FacetEdgeKey key;
  std::uint32_t vertices[2]{};
  std::uint32_t use_offset = 0, use_count = 0;
};

enum class SelfContactSupportStatus : std::uint8_t {
  AdmittedOrdinary,
  AdmittedCinMaster,
  AdmittedPartialOrMixedRigid,
  CompleteRigidGroup,
  UnsupportedCinSecondary
};
struct SelfContactSupportClassification {
  SelfContactSupportStatus status = SelfContactSupportStatus::AdmittedOrdinary;
  std::size_t complete_rigid_group = SIZE_MAX;
  std::uint8_t nonzero_slots = 0, rigid_slots = 0, cin_master_slots = 0;
};
struct SelfContactFacetVertexUse {
  std::uint32_t feature = 0, parent = 0, facet_valence = 0;
  FacetVertexKey key;
  WeightedSurfacePoint point;
  SelfContactSupportClassification support;
  Q4CertifiedIntegral dual_area_m2;
  Q4CertifiedIntegral directed_vf_area_m2;
};
struct SelfContactFacetEdgeUse {
  std::uint32_t feature = 0, parent = 0, facet_valence = 0;
  FacetEdgeKey key;
  WeightedSurfacePoint endpoints[2];
  SelfContactSupportClassification endpoint_support[2];
  // Exact parent-local vertex-use authority at the canonical edge endpoints.
  // These are directed half-dual areas, not line area or mass-scaled weights.
  Q4CertifiedIntegral directed_endpoint_dual_area_m2[2];
};

// The two arrays are aligned with SelfContactActiveUseBinding::parents().
// current may equal base for an accepted query. A 0->1 transition rejects;
// this value view owns no accepted history or second clock.
struct SelfContactActivityView {
  const std::uint8_t* base = nullptr;
  const std::uint8_t* current = nullptr;
  std::size_t parent_count = 0;
};
struct SelfContactResolvedVertexUse {
  SelfContactFacetVertexUse use;
  bool active = false;
  double reference_half_thickness_m = 0;
  Q4CertifiedIntegral reference_area_m2;
  Q4CertifiedIntegral dual_area_m2;
  Q4CertifiedIntegral directed_vf_area_m2;
};
struct SelfContactResolvedEdgeUse {
  SelfContactFacetEdgeUse use;
  bool active = false;
  double reference_half_thickness_m = 0;
  Q4CertifiedIntegral reference_area_m2;
  Q4CertifiedIntegral directed_endpoint_dual_area_m2[2];
};

enum class SelfContactTiedStatus : std::uint8_t {
  NotRelated,
  PartialOrUnauthenticatedLocalSupportNotExcluded,
  // Startup rows/witnesses are necessary but insufficient. Exclusion still
  // requires runtime owner-authenticated witness activity, active master
  // participation, and authoritative no-release state.
  CompleteLocalSupportNeedsRuntimeActivity
};
enum class SelfContactPairKind : std::uint8_t { VertexFace, EdgeEdge };
enum class SelfContactEdgeEdgeCase : std::uint8_t {
  StrictInteriorInteriorMinimum,
  BoundaryVertexEdgeMinimum,
  EdgeEdgeOnlyPenetrationOrCrossing,
  UnresolvedGeometricTie,
  CoplanarOverlap,
  ZeroDistance
};
enum class SelfContactPairStatus : std::uint8_t {
  InactiveParent,
  ExcludedLocalIncidence,
  // Nonincident uses of one physical parent require an authenticated current
  // regularity/geometry policy; canonical reference topology is insufficient.
  SameParentNeedsCurrentRegularity,
  UnsupportedCinSecondary,
  ExcludedSameRigidGroup,
  UnresolvedTiedSupportNotExcluded,
  AdmittedVertexFace,
  // No line area or topology-only VF coverage is inferred. A runtime owner may
  // cover the exact EE event only with independently authenticated geometry.
  UnadmittedEdgeEdgeForceArea,
  AdmittedEdgeEdge,
  // Added only by the current-regularity receipt consumer.  The active-use
  // classifier itself never emits this decision.
  ExcludedRegularOwnParent
};
struct SelfContactPairClassification {
  const void* binding_identity = nullptr;
  // Exact activity input identity used by the active-use query.  It permits a
  // later geometry receipt to reject a pair from another activity view even
  // when the byte values happen to agree.
  const std::uint8_t* activity_base_identity = nullptr;
  const std::uint8_t* activity_current_identity = nullptr;
  std::size_t activity_parent_count = 0;
  SelfContactPairKind kind = SelfContactPairKind::VertexFace;
  SelfContactEdgeEdgeCase edge_edge_case =
      SelfContactEdgeEdgeCase::StrictInteriorInteriorMinimum;
  SelfContactPairStatus status = SelfContactPairStatus::InactiveParent;
  SelfContactTiedStatus tied = SelfContactTiedStatus::NotRelated;
  SelfContactSupportClassification endpoint_support[2];
  std::uint32_t parent[2]{UINT32_MAX, UINT32_MAX};
  // VF: canonical vertex feature, facet-use ordinal. EE: two edge features.
  std::uint32_t feature[2]{UINT32_MAX, UINT32_MAX};
  bool active[2]{}, local_incidence = false, excluded = false;
  double reference_half_thickness_m[2]{};
  // VF: one directed vertex area. EE: the symmetric sum of the two
  // authenticated directed edge-point areas.
  Q4CertifiedIntegral candidate_directed_area_m2;
  Q4CertifiedIntegral admitted_force_area_m2;
};
} // namespace tlfea::contact
