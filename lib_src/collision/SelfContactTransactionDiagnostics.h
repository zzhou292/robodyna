// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {

// Coordinator host elapsed time, including existing worker/device waits.
// These are diagnostic scopes, never a solver clock or GPU kernel timings.
// Filtering: accepted pair/mask filtering; candidate prism/nonlinear root pass.
// Residual: candidate residual/persistent/nonlinear coverage and path packing.
// Policy: candidate intersection/edge policy, linear coverage and publication.
enum class SelfContactDiagnosticStage : std::uint8_t {
  Setup, Filtering, Discovery, EventAssembly, ForceAssembly,
  Residual, NativeCrossing, Policy, Finalization, Count
};
inline constexpr std::size_t SelfContactDiagnosticStageCount =
    static_cast<std::size_t>(SelfContactDiagnosticStage::Count);

struct SelfContactDiagnosticCounter {
  std::uint64_t calls = 0;
  // Scope ended by failure/exception, rather than the next stage or success.
  std::uint64_t failures = 0;
  std::uint64_t valid_samples = 0;
  std::uint64_t wall_ns = 0;
  std::uint64_t maximum_ns = 0;
};

// Sums of fields already returned by discovery, including failed calls.
// Failed reports can be partial; failures and counts_complete make this clear.
struct SelfContactDiscoveryDiagnostics {
  std::uint64_t calls = 0, failures = 0;
  std::uint64_t triangle_references = 0, triangles = 0;
  std::uint64_t vertex_references = 0, vertices = 0;
  std::uint64_t edge_references = 0, edges = 0;
  std::uint64_t raw_feature_candidates = 0, feature_candidates = 0;
  std::uint64_t raw_intersections = 0, intersections = 0;
  std::uint64_t potential_tasks = 0, local_masked_tasks = 0;
  std::uint64_t exact_executed_tasks = 0;
};

struct SelfContactAttemptDiagnostics {
  bool enabled = false, entered = false, finished = false, succeeded = false;
  // Stamp is the requested owner/base epoch/attempt until ordinary mechanics
  // authentication succeeds. This flag is descriptive, never authority.
  bool authenticated = false;
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  bool counter_saturated = false, counts_complete = true;
  std::uint64_t clock_failures = 0, backward_samples = 0;
  // Empty stages have calls==0. A stage duration is fully available only when
  // calls==valid_samples and counters have not saturated. Never treat missing
  // timing samples as zero-duration observations.
  std::array<SelfContactDiagnosticCounter, SelfContactDiagnosticStageCount> stages{};
  SelfContactDiscoveryDiagnostics discovery;
  std::uint64_t native_batches = 0;
  // Actual native slice inputs, including a failing invoked slice, excluding
  // residual/nonlinear bypasses. Distinct from policy exact_crossing_pairs.
  std::uint64_t native_submitted_pairs = 0;
  // Admitted native report work only; excludes later linear policy coverage.
  std::uint64_t native_work = 0;
};

// Value-only last-attempt observation. Discard retains it for failure analysis;
// the next AssembleAccepted resets both snapshots. No receipt/digest/state
// authority depends on this record. The owner is externally nonconcurrent.
struct SelfContactTransactionDiagnostics {
  SelfContactAttemptDiagnostics accepted, candidate;
};

}  // namespace tlfea::contact
