// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureDiscovery.h"
#include "RepresentedIntervalCrossing.h"
#include "SelfContactBroadphase.h"
#include "SelfContactCurrentRegularity.h"
#include "SelfContactForceAssembly.h"
#include "lib_src/elements/ShellBatchPublication.h"

#include <cstddef>
#include <cstdint>

namespace tlfea::contact {

enum class SelfContactTransactionStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  ResourceLimit,
  IdentityMismatch,
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
};

struct SelfContactTransactionReport {
  SelfContactTransactionStatus status = SelfContactTransactionStatus::Ok;
  std::size_t candidate = SIZE_MAX;
  std::size_t pair = SIZE_MAX;
  SelfContactForceStatus force_status = SelfContactForceStatus::Ok;
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
  tl::fea::ShellPublicationStatus publication_status =
      tl::fea::ShellPublicationStatus::Success;
  tl::fea::NodalStatus owner_status = tl::fea::NodalStatus::Ok;
  const char* message = "OK";
};

enum class SelfContactTransactionActivityPolicy : std::uint8_t {
  RequireAllSelectedParentsActiveV1,
};

enum class SelfContactTransactionNonlocalPolicy : std::uint8_t {
  AcceptedVertexFaceOnlyRejectIntersectionAndEdgeV1,
};

struct SelfContactTransactionConfig {
  SelfContactForceConfig force;
  // Immutable nonzero identity used by the fixed SelfContact roster slot.
  std::uint64_t source_id = 0;
  // Canonical broadphase sweep axis. Pair and event publication remains
  // independent of this choice.
  unsigned broadphase_axis = 0;
  SelfContactTransactionActivityPolicy activity_policy =
      SelfContactTransactionActivityPolicy::
          RequireAllSelectedParentsActiveV1;
  SelfContactTransactionNonlocalPolicy nonlocal_policy =
      SelfContactTransactionNonlocalPolicy::
          AcceptedVertexFaceOnlyRejectIntersectionAndEdgeV1;
};

struct SelfContactTransactionLimits {
  SelfContactForceLimits force;
  SelfContactBroadphaseLimits broadphase;
  FixedTriangleFeatureLimits accepted_discovery;
  FixedTriangleFeatureLimits candidate_discovery;
  SelfContactCurrentRegularityLimits regularity;
  RepresentedIntervalLimits crossing;
  tl::fea::ShellPhysicalScratchParticipationLimits participation;
  std::size_t max_candidate_triangles = 4096;
  std::size_t max_candidate_pairs = 4096;
  std::size_t max_host_bytes = 128u << 20;
  std::size_t max_device_bytes = 64u << 20;
  std::size_t max_startup_host_bytes = 512u << 20;
};

struct SelfContactTransactionForecast {
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
  std::size_t parent_activity_bytes = 0;
  std::size_t accepted_snapshot_values = 0;
  std::size_t prepared_snapshot_values = 0;
  std::size_t broadphase_pair_capacity = 0;
  std::size_t candidate_triangle_capacity = 0;
  std::size_t candidate_pair_capacity = 0;
  std::size_t accepted_event_capacity = 0;
  std::size_t accepted_certificate_capacity = 0;
  std::size_t policy_outcome_capacity = 0;
  std::size_t candidate_arena_bytes = 0;
  std::size_t owned_host_bytes = 0;
  std::size_t startup_host_bytes = 0;
  std::size_t device_bytes = 0;
  std::size_t device_allocations = 0;
};

struct SelfContactTransactionPreflight {
  SelfContactTransactionReport report;
  SelfContactTransactionForecast forecast;
};

enum class SelfContactCandidateDisposition : std::uint8_t {
  CertifiedSeparated,
  ExcludedLocalIntersection,
  RepresentedByAcceptedVertexFace,
};

// Read-only result of the transaction's fixed fail-closed policy. The
// accepted_event ordinal is meaningful only for represented VF crossings.
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

class SelfContactTransaction;

class SelfContactAcceptedAssemblyReceipt {
 public:
  SelfContactAcceptedAssemblyReceipt() noexcept = default;
  // Diagnostic shape only. SealCandidate additionally authenticates the
  // embedded force receipt against its live assembler startup identity.
  bool valid() const noexcept {
    return transaction_ != nullptr && owner_ != nullptr && source_id_ != 0 &&
        attempt_ != 0 && force_.prepared();
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
  std::size_t broadphase_pairs_ = 0;
  std::size_t facet_pairs_ = 0;
  std::size_t discovered_features_ = 0;
  SelfContactForceAssemblyReceipt force_;
};

// The only public candidate-completion authority. The physical typed receipt
// remains private and can be borrowed only as the fixed roster shape.
class SelfContactTransactionReceipt {
 public:
  SelfContactTransactionReceipt() noexcept = default;
  bool valid() const noexcept {
    return transaction_ != nullptr && owner_ != nullptr && source_id_ != 0 &&
        attempt_ != 0 && participation_.valid();
  }
  std::uint64_t source_id() const noexcept { return source_id_; }
  std::uint64_t regularity_generation() const noexcept {
    return regularity_generation_;
  }
  std::size_t broadphase_pairs() const noexcept {
    return broadphase_pairs_;
  }
  std::size_t facet_pairs() const noexcept { return facet_pairs_; }
  std::size_t policy_outcomes() const noexcept {
    return policy_outcomes_;
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
  std::size_t policy_outcomes_ = 0;
  tl::fea::ShellPhysicalScratchParticipationReceipt participation_;
};

}  // namespace tlfea::contact
