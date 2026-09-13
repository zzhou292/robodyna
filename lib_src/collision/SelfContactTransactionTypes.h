// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "FixedTriangleFeatureDiscovery.h"
#include "RepresentedIntervalCrossing.h"
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
  SelfContactCurrentRegularityStatus regularity_status =
      SelfContactCurrentRegularityStatus::Ok;
  FixedTriangleDiscoveryStatus discovery_status =
      FixedTriangleDiscoveryStatus::Ok;
  RepresentedIntervalStatus crossing_status =
      RepresentedIntervalStatus::Ok;
  tl::fea::ShellPublicationStatus publication_status =
      tl::fea::ShellPublicationStatus::Success;
  tl::fea::NodalStatus owner_status = tl::fea::NodalStatus::Ok;
  const char* message = "OK";
};

struct SelfContactTransactionConfig {
  SelfContactForceConfig force;
  // Immutable nonzero identity used by the fixed SelfContact roster slot.
  std::uint64_t source_id = 0;
};

struct SelfContactTransactionLimits {
  SelfContactForceLimits force;
  tl::fea::ShellPhysicalScratchParticipationLimits participation;
  std::size_t max_candidate_triangles = 4096;
  std::size_t max_candidate_pairs = 4096;
  std::size_t max_host_bytes = 128u << 20;
  std::size_t max_device_bytes = 64u << 20;
  std::size_t max_startup_host_bytes = 512u << 20;
};

struct SelfContactTransactionForecast {
  SelfContactForceForecast force;
  tl::fea::ShellPhysicalScratchParticipationForecast participation;
  std::size_t parent_activity_bytes = 0;
  std::size_t accepted_snapshot_values = 0;
  std::size_t prepared_snapshot_values = 0;
  std::size_t candidate_triangle_capacity = 0;
  std::size_t candidate_pair_capacity = 0;
  std::size_t accepted_event_identity_capacity = 0;
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

// Exact immutable facet declarations. Geometry is rebuilt by the transaction
// from its private accepted/prepared owner readbacks.
struct SelfContactCandidateTriangle {
  std::uint32_t active_use_parent = UINT32_MAX;
  std::uint32_t local_facet = UINT32_MAX;
};

enum class SelfContactCrossingDisposition : std::uint8_t {
  RepresentedByAcceptedVertexFace,
  RejectCandidate,
};

// Exact pair identity. A decision is mandatory for every nonlocal
// CertifiedCrossingContact result.
struct SelfContactCrossingDecision {
  RepresentedIntervalPairKey pair;
  SelfContactCrossingDisposition disposition =
      SelfContactCrossingDisposition::RejectCandidate;
};

struct SelfContactCandidateEvidence {
  SelfContactCurrentRegularity* regularity = nullptr;
  FixedTriangleFeatureDiscovery* discovery = nullptr;
  RepresentedIntervalCrossing* crossing = nullptr;
  const SelfContactCandidateTriangle* triangles = nullptr;
  std::size_t triangle_count = 0;
  const FixedTrianglePair* pairs = nullptr;
  std::size_t pair_count = 0;
  const SelfContactCrossingDecision* decisions = nullptr;
  std::size_t decision_count = 0;
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
  tl::fea::ShellPhysicalScratchParticipationReceipt participation_;
};

}  // namespace tlfea::contact
