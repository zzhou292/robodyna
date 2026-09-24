// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureDiscovery.h"
#include "RepresentedIntervalCrossing.h"
#include "SelfContactBroadphase.h"
#include "SelfContactCurrentRegularity.h"
#include "SelfContactForceAssembly.h"
#include "SelfContactPhysicalActivityTypes.h"
#include "self_contact_filters/Types.h"
#include "RepresentedIntervalCrossingGpu.h"
#include "lib_src/elements/ShellBatchPublication.h"

#include <cstddef>
#include <cstdint>

namespace tlfea::contact {

namespace self_contact_transaction {
class QualificationAccess;
}

enum class SelfContactTransactionStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  ResourceLimit,
  IdentityMismatch,
  ActivityFailure,
  UnsupportedActivity,
  UnsupportedMotion,
  BroadphaseFailure,
  ForceFailure,
  OwnerFailure,
  PublicationFailure,
  RegularityFailure,
  DiscoveryFailure,
  CrossingFailure,
  UnresolvedCandidate,
  CandidateRejected,
  FacetFilterFailure,
};

// Value-only initialization route, not proof that any GPU query executed.
// Cuda remains the initialization mode after discard or a later device error.
enum class SelfContactFacetFilterInitialization : std::uint8_t {
  NotInitialized,
  Disabled,
  Cuda,
  UnsupportedHostArithmetic,
};

// A candidate chunk query may fail before its original serial geometry fold.
// This is nonphysical failure context, never an offending-pair certificate.
enum class SelfContactFacetFilterFailureScope : std::uint8_t {
  None, CandidateChunkBeforeFold,
};

enum class SelfContactTransactionCountKind : std::uint8_t {
  None,
  ExactAcceptedEvents,
  AcceptedEventsLowerBound,
};

enum class SelfContactFacetMotion : std::uint8_t {
  LinearNodalV1,
  CompleteRigidGroup,
  PartialOrMixedRigid,
};

struct SelfContactRigidGroupIdentity {
  std::size_t binding_group = SIZE_MAX;
  tl::fea::RigidBindingSourceKind source_kind =
      tl::fea::RigidBindingSourceKind::NodalGroup;
  std::uint64_t source_group_id = 0;
  std::uint64_t source_node_set_id = 0;
};

struct SelfContactFacetMotionIdentity {
  FixedTriangleKey facet;
  std::size_t active_parent = SIZE_MAX;
  SelfContactFacetMotion motion =
      SelfContactFacetMotion::LinearNodalV1;
  std::size_t rigid_group_count = 0;
  SelfContactRigidGroupIdentity rigid_groups[4];
};

// Scalar diagnostics for a failed native crossing call. These fields describe
// that call's admitted canonical prefix; they are not complete chunk work,
// a geometry certificate, or authority to publish a candidate.
struct SelfContactCrossingDiagnostics {
  bool available = false;
  std::size_t batch_pair_offset = 0;
  std::size_t prior_batch_work = 0;
  std::size_t input_path = SIZE_MAX;
  std::size_t input_pair = SIZE_MAX;
  std::size_t input_paths = 0;
  std::size_t input_pairs = 0;
  std::size_t unique_pairs = 0;
  std::size_t certified_separated = 0;
  std::size_t certified_crossing_contact = 0;
  std::size_t unresolved = 0;
  std::size_t admitted_work = 0;
  std::size_t total_work_limit = 0;
  std::size_t rejected_pair_work = 0;
};

struct SelfContactTransactionReport {
  SelfContactTransactionStatus status = SelfContactTransactionStatus::Ok;
  std::size_t candidate = SIZE_MAX;
  std::size_t pair = SIZE_MAX;
  // Resource-limit event counts are either the complete exact requirement or
  // a typed lower bound when the compact identity census itself was exhausted.
  SelfContactTransactionCountKind count_kind =
      SelfContactTransactionCountKind::None;
  // Populated for an exact facet-pair motion failure.  Partial/mixed support
  // can name up to the four actual groups present in one shell-parent map.
  SelfContactFacetMotionIdentity offending_motion[2];
  Vec3 offending_quadratic_lower[2][3];
  Vec3 offending_quadratic_upper[2][3];
  SelfContactSweptParentBounds offending_swept_bounds[2];
  double offending_half_thickness_m[2]{};
  double offending_feature_distance_m = 0;
  double offending_edge_parameters[2] = {0, 0};
  SelfContactForceStatus force_status = SelfContactForceStatus::Ok;
  SelfContactPhysicalActivityStatus activity_status =
      SelfContactPhysicalActivityStatus::Ok;
  SelfContactBroadphaseStatus broadphase_status =
      SelfContactBroadphaseStatus::Ok;
  SelfContactCurrentRegularityStatus regularity_status =
      SelfContactCurrentRegularityStatus::Ok;
  FixedTriangleDiscoveryStatus discovery_status =
      FixedTriangleDiscoveryStatus::Ok;
  std::size_t discovery_task = SIZE_MAX;
  FixedTriangleArithmeticReason discovery_reason =
      FixedTriangleArithmeticReason::None;
  RepresentedIntervalStatus crossing_status =
      RepresentedIntervalStatus::Ok;
  RepresentedIntervalReason crossing_reason =
      RepresentedIntervalReason::None;
  std::size_t nonlinear_subdivision_work = 0;
  unsigned nonlinear_subdivision_depth = 0;
  bool nonlinear_subdivision_work_exhausted = false;
  bool nonlinear_subdivision_depth_exhausted = false;
  tl::fea::ShellPublicationStatus publication_status =
      tl::fea::ShellPublicationStatus::Success;
  tl::fea::NodalStatus owner_status = tl::fea::NodalStatus::Ok;
  const char* message = "OK";
  SelfContactCrossingDiagnostics crossing_diagnostics;
  self_contact_filters::Status filter_status = self_contact_filters::Status::Ok;
  SelfContactFacetFilterFailureScope filter_scope = SelfContactFacetFilterFailureScope::None;
  std::size_t filter_chunk_begin = SIZE_MAX, filter_chunk_pairs = 0;
  // Numerical execution failure context only. It never identifies a physical
  // collision by itself or changes native per-slice work/publication semantics.
  RepresentedIntervalDeviceStatus crossing_device_status = RepresentedIntervalDeviceStatus::NotInvoked;
  std::size_t crossing_fault_cohort_begin = SIZE_MAX, crossing_fault_cohort_count = 0;
  std::size_t crossing_fault_pair_ordinal = SIZE_MAX;
};

enum class SelfContactTransactionNonlocalPolicy : std::uint8_t {
  AcceptedVertexFaceOnlyRejectIntersectionAndEdgeV1,
  AcceptedSymmetricVfEeRejectIntersectionV2,
};

struct SelfContactTransactionConfig {
  SelfContactForceConfig force;
  // Immutable nonzero identity used by the fixed SelfContact roster slot.
  std::uint64_t source_id = 0;
  // Canonical broadphase sweep axis. Pair and event publication remains
  // independent of this choice.
  unsigned broadphase_axis = 0;
  SelfContactTransactionNonlocalPolicy nonlocal_policy =
      SelfContactTransactionNonlocalPolicy::
          AcceptedSymmetricVfEeRejectIntersectionV2;
  // Optional host substage observation. Never participates in physical identity.
  bool enable_diagnostics = false;
  // Optional equivalent numerical backend; default CPU path stays selected.
  bool enable_cuda_facet_filters = false;
  bool enable_cuda_native_crossing = false;
  unsigned native_crossing_device_workers = 128;
  // Explicit numerical lookahead capacity; zero preserves per-slice execution.
  std::size_t native_crossing_numeric_cohort_pairs = 0;
};

struct SelfContactTransactionLimits {
  SelfContactPhysicalActivityLimits activity;
  SelfContactForceLimits force;
  SelfContactBroadphaseLimits broadphase;
  FixedTriangleFeatureLimits accepted_discovery;
  FixedTriangleFeatureLimits candidate_discovery;
  SelfContactCurrentRegularityLimits regularity;
  RepresentedIntervalLimits crossing;
  tl::fea::ShellPhysicalScratchParticipationLimits participation;
  std::size_t max_candidate_triangles = 4096;
  // Complete parent keys remain resident, but facet pairs and their exact
  // geometry are materialized only in chunks. max_candidate_pairs is the
  // complete facet-pair census cap, not an allocation shape.
  std::size_t max_candidate_pairs = 65536;
  std::size_t max_facet_pair_chunk = 1024;
  // Full certificates and force events never exceed this profile cap or the
  // configured force capacity.
  std::size_t max_global_events = 4096;
  // The compact exact identity census is independent of the full event cap.
  std::size_t max_event_identity_census = 4096;
  std::size_t max_event_hash_slots = 8192;
  // Zero folds all outcomes into the complete summary without retaining an
  // addressable per-pair publication.
  std::size_t max_policy_outcomes = 4096;
  std::size_t max_stream_crossing_work = 1u << 20;
  std::size_t max_nonlinear_subdivision_work_per_pair = 4095;
  std::size_t max_nonlinear_subdivision_work_per_chunk = 1u << 20;
  std::size_t max_stream_nonlinear_subdivision_work = 1u << 24;
  unsigned max_nonlinear_subdivision_depth = 20;
  std::size_t max_host_bytes = 512u << 20;
  std::size_t max_device_bytes = 64u << 20;
  std::size_t max_startup_host_bytes = 1u << 30;

  struct ExactCensus {
    std::size_t nodes = 0;
    std::size_t surface_parents = 0;
    std::size_t selected_parents = 0;
    std::size_t maximum_family_parents = 0;
    std::size_t facets = 0;
    std::size_t parent_pairs = 0;
    std::size_t facet_pairs = 0;
    std::size_t accepted_events = 0;
  };

  // Generic vehicle-scale construction. Counts and caps come from a prior
  // exact census; there are deliberately no model-specific constants here.
  static SelfContactTransactionLimits Vehicle(
      const ExactCensus&, std::size_t facet_pair_chunk,
      std::size_t event_ledger_capacity,
      std::size_t event_hash_slots,
      std::size_t policy_outcome_capacity,
      std::size_t crossing_work_per_pair,
      std::size_t crossing_work_per_chunk,
      std::size_t crossing_work_complete,
      unsigned crossing_depth,
      std::size_t max_host_bytes,
      std::size_t max_device_bytes,
      std::size_t max_startup_host_bytes,
      unsigned discovery_worker_count = 1,
      unsigned crossing_worker_count = 1,
      // Zero preserves the former behavior: the full event cap is also the
      // compact identity-census cap.
      std::size_t event_identity_census = 0) noexcept;
};

struct SelfContactTransactionForecast {
  SelfContactPhysicalActivityForecast activity;
  SelfContactForceForecast force;
  SelfContactBroadphaseForecast broadphase;
  FixedTriangleFeatureForecast accepted_discovery;
  FixedTriangleFeatureForecast candidate_discovery;
  SelfContactCurrentRegularityForecast regularity;
  RepresentedIntervalForecast crossing;
  tl::fea::ShellPhysicalScratchParticipationForecast participation;
  std::size_t surface_parent_map_capacity = 0;
  std::size_t parent_facet_offset_count = 0;
  std::size_t facet_descriptor_capacity = 0;
  std::size_t accepted_snapshot_values = 0;
  std::size_t prepared_snapshot_values = 0;
  std::size_t broadphase_pair_capacity = 0;
  std::size_t candidate_triangle_capacity = 0;
  std::size_t complete_facet_pair_capacity = 0;
  std::size_t candidate_pair_capacity = 0;
  std::size_t facet_pair_chunk_capacity = 0;
  std::size_t parent_pair_cursor_capacity = 0;
  std::size_t accepted_event_identity_census_capacity = 0;
  std::size_t event_identity_hash_capacity = 0;
  // Compatibility aliases for the full certificate ledger and identity hash.
  std::size_t accepted_event_ledger_capacity = 0;
  std::size_t event_hash_capacity = 0;
  std::size_t rigid_group_snapshot_capacity = 0;
  std::size_t node_rigid_group_capacity = 0;
  std::size_t parent_motion_capacity = 0;
  std::size_t facet_motion_capacity = 0;
  std::size_t facet_quadratic_capacity = 0;
  std::size_t swept_parent_bound_capacity = 0;
  std::size_t swept_facet_bound_capacity = 0;
  std::size_t candidate_crossing_capacity = 0;
  std::size_t accepted_event_capacity = 0;
  std::size_t accepted_certificate_capacity = 0;
  std::size_t policy_outcome_capacity = 0;
  std::size_t policy_chunk_capacity = 0;
  std::size_t complete_crossing_work_capacity = 0;
  std::size_t crossing_work_per_pair = 0;
  unsigned crossing_depth = 0;
  std::size_t nonlinear_subdivision_work_per_pair = 0;
  std::size_t nonlinear_subdivision_work_per_chunk = 0;
  std::size_t complete_nonlinear_subdivision_work_capacity = 0;
  unsigned nonlinear_subdivision_depth = 0;
  std::size_t broadphase_pair_readback_bytes = 0;
  std::size_t streaming_cursor_bytes = 0;
  std::size_t streaming_heap_bytes = 0;
  std::size_t candidate_arena_bytes = 0;
  std::size_t shared_backing_discount_bytes = 0;
  std::size_t owned_host_bytes = 0;
  std::size_t startup_host_bytes = 0;
  std::size_t device_bytes = 0;
  std::size_t device_allocations = 0;
  // One caller-owned fixed 15-bit mask per materialized facet-pair slot.
  std::size_t feature_task_mask_capacity = 0;
  std::size_t feature_task_mask_bytes = 0;
  // Raw-only work-admitted crossing calls retain the complete discovery
  // cohort. Their separate private result staging is fully charged to arena
  // storage; no device allocation or source/feature grouping changes.
  std::size_t raw_crossing_result_capacity = 0;
  std::size_t raw_crossing_result_bytes = 0;
  std::size_t crossing_batch_pair_capacity = 0;
};

struct SelfContactTransactionPreflight {
  SelfContactTransactionReport report;
  SelfContactTransactionForecast forecast;
};

struct SelfContactTransactionAllocationInfo {
  SelfContactPhysicalActivityAllocationInfo activity;
  tl::fea::NodalAllocationInfo device;
};

enum class SelfContactCandidateDisposition : std::uint8_t {
  CertifiedSeparated,
  ExcludedSameRigidGroup,
  ExcludedLocalIntersection,
  RepresentedByAcceptedVertexFace,
  RepresentedByAcceptedEdgeEdge,
};

// Read-only result of the transaction's fixed fail-closed policy. The
// accepted_event ordinal is meaningful for represented VF and EE crossings.
struct SelfContactCandidatePolicyOutcome {
  RepresentedIntervalPairKey pair;
  SelfContactCandidateDisposition disposition =
      SelfContactCandidateDisposition::CertifiedSeparated;
  std::size_t accepted_event = SIZE_MAX;
  std::uint64_t source_order = UINT64_MAX;
};

struct SelfContactCandidatePolicyView {
  const SelfContactCandidatePolicyOutcome* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
};

struct SelfContactCandidatePolicySummary {
  std::size_t outcomes = 0;
  std::size_t certified_separated = 0;
  std::size_t excluded_same_rigid_group = 0;
  std::size_t excluded_local_intersection = 0;
  std::size_t represented_by_accepted_vf = 0;
  std::size_t represented_by_accepted_ee = 0;
  // Motion-filter accounting proves which candidate pairs bypassed exact
  // represented-interval work. The motion/excluded/exact counts partition
  // outcomes; the axis counts refine the motion-certified subset.
  std::size_t motion_certified_linear_separated = 0;
  std::size_t axis_certified_linear_separated = 0;
  std::size_t edge_axis_certified_linear_separated = 0;
  std::size_t motion_excluded_same_rigid_group = 0;
  std::size_t exact_crossing_pairs = 0;
  std::size_t exact_crossing_work = 0;
  // FNV-1a over canonical pair identity, disposition and accepted source
  // order. It is a deterministic completeness diagnostic, not authority.
  std::uint64_t digest = 1469598103934665603ull;
  bool complete = false;
  bool detailed_publication = false;
  // Appended disjoint refinements preserve every original member offset.
  std::size_t vertex_edge_axis_separated = 0;
  std::size_t vertex_vertex_axis_separated = 0;
  std::size_t motion_certified_nonlinear_separated = 0;
  std::size_t nonlinear_subdivision_pairs = 0;
  std::size_t nonlinear_subdivision_work = 0;
  std::size_t nonlinear_subdivision_unresolved = 0;
  std::size_t nonlinear_subdivision_work_exhausted = 0;
  std::size_t nonlinear_subdivision_depth_exhausted = 0;
  std::size_t motion_certified_nonlinear_accepted_coverage = 0;
  // Exact same-rigid support or continuously proved local topology. Final
  // policy outcomes retain their distinct rigid/local dispositions.
  std::size_t motion_certified_nonlinear_exact_exclusion = 0;
  std::size_t linear_policy_coverage_pairs = 0;
  std::size_t linear_policy_coverage_work = 0;
  std::size_t linear_policy_certified_separated = 0;
  std::size_t linear_policy_accepted_coverage = 0;
  // Includes continuously proved local topology, as above.
  std::size_t linear_policy_exact_exclusion = 0;
  std::size_t linear_policy_potential_contact = 0;
  std::size_t linear_policy_work_exhausted = 0;
  std::size_t linear_policy_depth_exhausted = 0;
  std::size_t linear_policy_missing_accepted_owner = 0;
  std::size_t linear_policy_owner_ambiguity = 0;
  std::size_t linear_policy_possible_geometric_crossing = 0;
  std::size_t linear_policy_unresolved = 0;
};

class SelfContactTransaction;

class SelfContactAcceptedAssemblyReceipt {
 public:
  SelfContactAcceptedAssemblyReceipt() noexcept = default;
  // Diagnostic shape only. SealCandidate additionally authenticates the
  // embedded force receipt against its live assembler startup identity.
  bool valid() const noexcept {
    return transaction_ != nullptr && owner_ != nullptr && source_id_ != 0 &&
        attempt_ != 0 && force_.prepared() && activity_.valid();
  }
  const SelfContactForceDiagnostics& diagnostics() const noexcept {
    return force_.diagnostics();
  }
  std::size_t broadphase_pairs() const noexcept {
    return broadphase_pairs_;
  }
  std::size_t facet_pairs() const noexcept { return facet_pairs_; }
  std::size_t discovered_features() const noexcept {
    return discovered_features_;
  }
  std::size_t potential_tasks() const noexcept {
    return potential_tasks_;
  }
  std::size_t local_masked_tasks() const noexcept {
    return local_masked_tasks_;
  }
  std::size_t exact_executed_tasks() const noexcept {
    return exact_executed_tasks_;
  }

 private:
  friend class SelfContactTransaction;
  friend class self_contact_transaction::QualificationAccess;
  const SelfContactTransaction* transaction_ = nullptr;
  const tl::fea::FENodalState* owner_ = nullptr;
  const void* active_use_identity_ = nullptr;
  std::uint64_t source_id_ = 0;
  std::uint64_t configuration_id_ = 0;
  std::uint64_t qualification_id_ = 0;
  std::uint64_t owner_id_ = 0;
  std::uint64_t base_epoch_ = 0;
  std::uint64_t attempt_ = 0;
  std::size_t broadphase_pairs_ = 0;
  std::size_t facet_pairs_ = 0;
  std::size_t discovered_features_ = 0;
  std::size_t potential_tasks_ = 0;
  std::size_t local_masked_tasks_ = 0;
  std::size_t exact_executed_tasks_ = 0;
  SelfContactAcceptedActivityReceipt activity_;
  SelfContactForceAssemblyReceipt force_;
};

// The only public candidate-completion authority. The physical typed receipt
// remains private and can be borrowed only as the fixed roster shape.
class SelfContactTransactionReceipt {
 public:
  SelfContactTransactionReceipt() noexcept = default;
  bool valid() const noexcept {
    return transaction_ != nullptr && owner_ != nullptr && source_id_ != 0 &&
        attempt_ != 0 && participation_.valid() && activity_.valid();
  }
  std::uint64_t source_id() const noexcept { return source_id_; }
  std::uint64_t regularity_generation() const noexcept {
    return regularity_generation_;
  }
  std::size_t broadphase_pairs() const noexcept {
    return broadphase_pairs_;
  }
  std::size_t facet_pairs() const noexcept { return facet_pairs_; }
  std::size_t potential_tasks() const noexcept {
    return potential_tasks_;
  }
  std::size_t local_masked_tasks() const noexcept {
    return local_masked_tasks_;
  }
  std::size_t exact_executed_tasks() const noexcept {
    return exact_executed_tasks_;
  }
  std::size_t policy_outcomes() const noexcept {
    return policy_outcomes_;
  }
  const SelfContactCandidatePolicySummary& policy_summary() const noexcept {
    return policy_summary_;
  }
  std::size_t active_parents() const noexcept {
    return active_parents_;
  }
  std::size_t removing_parents() const noexcept {
    return removing_parents_;
  }
  std::size_t skipped_parents() const noexcept {
    return skipped_parents_;
  }
  tl::fea::ShellPhysicalScratchReceiptRoster scratch_receipts()
      const noexcept {
    return valid()
        ? tl::fea::ShellPhysicalScratchReceiptRoster{nullptr, &participation_}
        : tl::fea::ShellPhysicalScratchReceiptRoster{};
  }

 private:
  friend class SelfContactTransaction;
  const SelfContactTransaction* transaction_ = nullptr;
  const tl::fea::FENodalState* owner_ = nullptr;
  const void* active_use_identity_ = nullptr;
  std::uint64_t source_id_ = 0;
  std::uint64_t configuration_id_ = 0;
  std::uint64_t qualification_id_ = 0;
  std::uint64_t owner_id_ = 0;
  std::uint64_t base_epoch_ = 0;
  std::uint64_t attempt_ = 0;
  std::uint64_t regularity_generation_ = 0;
  std::size_t broadphase_pairs_ = 0;
  std::size_t facet_pairs_ = 0;
  std::size_t potential_tasks_ = 0;
  std::size_t local_masked_tasks_ = 0;
  std::size_t exact_executed_tasks_ = 0;
  std::size_t policy_outcomes_ = 0;
  SelfContactCandidatePolicySummary policy_summary_;
  std::size_t active_parents_ = 0;
  std::size_t removing_parents_ = 0;
  std::size_t skipped_parents_ = 0;
  SelfContactPreparedActivityReceipt activity_;
  tl::fea::ShellPhysicalScratchParticipationReceipt participation_;
};

}  // namespace tlfea::contact
