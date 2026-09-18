// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../RepresentedIntervalCrossing.h"

namespace tlfea::contact {
struct SelfContactCrossingDiagnostics;
}

namespace tlfea::contact::self_contact_transaction {

// Bound each native invocation by its worst-case per-pair work. A native
// limit smaller than one pair's worst case still permits a one-pair attempt;
// the unchanged native admission rule decides whether its actual work fits.
constexpr std::size_t RawCrossingBatchPairCapacity(
    std::size_t pair_capacity, std::size_t work_per_pair,
    std::size_t work_per_call) noexcept {
  if (!pair_capacity || !work_per_pair || !work_per_call) return 0;
  const auto admitted = work_per_call / work_per_pair;
  const auto count = admitted ? admitted : std::size_t{1};
  return count < pair_capacity ? count : pair_capacity;
}

struct CrossingBatchReport {
  RepresentedIntervalStatus status = RepresentedIntervalStatus::Ok;
  const char* message = "OK";
  // Index in the complete caller pair roster, including adapter failures.
  std::size_t input_pair = SIZE_MAX;
  // Verbatim last native report; meaningful only when native_called is true.
  RepresentedIntervalReport native_report;
  bool native_called = false;
  std::size_t batch_offset = 0;
  std::size_t prior_work = 0;
  std::size_t completed_pairs = 0;
  std::size_t completed_batches = 0;
  // Private staging is exposed only after the complete roster succeeds.
  RepresentedIntervalResultView results;
};

// Internal adapter for an already canonical, strictly increasing pair roster.
// Discovery cohorts, feature/seam ownership, and nonlinear policy grouping are
// unchanged. EVERY native call receives ALL original paths, including unused
// paths, so cross-batch source/trajectory contradictions remain visible.
// Native pair work, witnesses, and identities are copied without modification.
//
// scratch is a separate caller-owned arena, bounded private attempt storage,
// never a borrowed native buffer (including an expired native view), disjoint from the
// inputs and native publication. It may be overwritten on failure; no complete
// result view is returned then. The transaction remains the sole publication
// owner. No allocation, threads, or partial physical publication occurs here.
CrossingBatchReport CertifyCrossingBatches(
    RepresentedIntervalCrossing& crossing,
    const RepresentedTrianglePath* paths, std::size_t path_count,
    const RepresentedTrianglePair* pairs, std::size_t pair_count,
    std::size_t batch_pair_capacity,
    RepresentedIntervalResult* scratch,
    std::size_t scratch_capacity) noexcept;

SelfContactCrossingDiagnostics CrossingBatchDiagnostics(
    const CrossingBatchReport&) noexcept;

}  // namespace tlfea::contact::self_contact_transaction
