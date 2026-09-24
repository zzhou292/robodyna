// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransactionDiagnostics.h"
#include "../FixedTriangleFeatureTypes.h"
#include "CrossingBatch.h"

#include <algorithm>
#include <cerrno>
#include <ctime>

namespace tlfea::contact::self_contact_transaction {

// Reuses robo-dyna benchmarks/stage_timing/StageTimer's per-instance monotonic
// clock, errno preservation, valid-sample accounting and saturation pattern.
// Kept private here to avoid a TL -> app dependency or a new app host dependency.
// Only qualification installs another reader; production uses this clock.
struct DiagnosticClock {
  using Read = bool (*)(void*, std::uint64_t*) noexcept;
  Read read = nullptr;
  void* context = nullptr;
};

inline bool DiagnosticNanoseconds(void*, std::uint64_t* output) noexcept {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0 || value.tv_sec < 0 ||
      value.tv_nsec < 0 || value.tv_nsec >= 1000000000) return false;
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  const auto fraction = static_cast<std::uint64_t>(value.tv_nsec);
  if (seconds > (UINT64_MAX - fraction) / 1000000000) return false;
  *output = seconds * 1000000000 + fraction;
  return true;
}

// One serialized coordinator, nonoverlapping coarse stages. No allocations,
// CUDA calls, physical work accounting, callbacks into mechanics or exceptions.
// The destructor closes the last scope through every return/exception, after
// normal rollback if one occurred. Discard must not clear the referenced record.
class DiagnosticAttempt {
 public:
  DiagnosticAttempt(SelfContactAttemptDiagnostics& snapshot, bool enabled,
                    std::uint64_t owner_id, std::uint64_t base_epoch,
                    std::uint64_t attempt, DiagnosticClock clock = {}) noexcept
      : snapshot_(snapshot), clock_{clock.read ? clock.read : DiagnosticNanoseconds,
                                  clock.context} {
    snapshot_ = {};
    if (!enabled) return;
    snapshot_.enabled = snapshot_.entered = true;
    snapshot_.owner_id = owner_id;
    snapshot_.base_epoch = base_epoch;
    snapshot_.attempt = attempt;
    Start(SelfContactDiagnosticStage::Setup);
  }
  ~DiagnosticAttempt() noexcept {
    if (!snapshot_.enabled) return;
    Finish(!snapshot_.succeeded);
    snapshot_.finished = true;
    if (!snapshot_.succeeded) snapshot_.counts_complete = false;
  }
  DiagnosticAttempt(const DiagnosticAttempt&) = delete;
  DiagnosticAttempt& operator=(const DiagnosticAttempt&) = delete;

  void Authenticate() noexcept {
    if (snapshot_.enabled) snapshot_.authenticated = true;
  }
  void Success() noexcept {
    if (snapshot_.enabled) snapshot_.succeeded = true;
  }
  void Stage(SelfContactDiagnosticStage stage) noexcept {
    if (!snapshot_.enabled) return;
    Finish(false);
    Start(stage);
  }
  void Discovery(const FixedTriangleDiscoveryReport& report) noexcept {
    if (!snapshot_.enabled) return;
    auto& counts = snapshot_.discovery;
    Add(counts.calls, 1);
    if (report.status != FixedTriangleDiscoveryStatus::Ok) {
      Add(counts.failures, 1);
      snapshot_.counts_complete = false;
    }
    Add(counts.triangle_references, report.triangle_references);
    Add(counts.triangles, report.triangles);
    Add(counts.vertex_references, report.vertex_references);
    Add(counts.vertices, report.vertices);
    Add(counts.edge_references, report.edge_references);
    Add(counts.edges, report.edges);
    Add(counts.raw_feature_candidates, report.raw_feature_candidates);
    Add(counts.feature_candidates, report.feature_candidates);
    Add(counts.raw_intersections, report.raw_intersections);
    Add(counts.intersections, report.intersections);
    Add(counts.potential_tasks, report.potential_tasks);
    Add(counts.local_masked_tasks, report.local_masked_tasks);
    Add(counts.exact_executed_tasks, report.exact_executed_tasks);
  }
  void Crossing(const CrossingBatchReport& report, std::size_t requested_pairs,
                std::size_t batch_capacity) noexcept {
    if (!snapshot_.enabled) return;
    Add(snapshot_.native_batches, report.completed_batches);
    Add(snapshot_.native_submitted_pairs, report.completed_pairs);
    if (report.status != RepresentedIntervalStatus::Ok) {
      snapshot_.counts_complete = false;
      // A failed adapter can have returned before the native entry. Otherwise
      // completed_* excludes its last invoked (possibly failing) slice.
      if (report.native_called) {
        Add(snapshot_.native_batches, 1);
        if (report.batch_offset <= requested_pairs)
          Add(snapshot_.native_submitted_pairs,
              std::min(batch_capacity, requested_pairs - report.batch_offset));
      }
    }
    if (report.native_called) {
      Add(snapshot_.native_work, report.prior_work);
      Add(snapshot_.native_work, report.native_report.work);
    }
  }

 private:
  void Add(std::uint64_t& target, std::uint64_t value) noexcept {
    if (value > UINT64_MAX - target) {
      target = UINT64_MAX;
      snapshot_.counter_saturated = true;
      snapshot_.counts_complete = false;
    } else target += value;
  }
  bool Read(std::uint64_t& value) noexcept {
    const int saved_errno = errno;
    const bool valid = clock_.read(clock_.context, &value);
    errno = saved_errno;
    if (!valid) Add(snapshot_.clock_failures, 1);
    return valid;
  }
  void Start(SelfContactDiagnosticStage stage) noexcept {
    stage_ = stage;
    start_valid_ = Read(start_);
  }
  void Finish(bool failed) noexcept {
    std::uint64_t end = 0;
    const bool end_valid = Read(end);
    bool valid = start_valid_ && end_valid;
    if (valid && end < start_) {
      Add(snapshot_.backward_samples, 1);
      valid = false;
    }
    auto& counter = snapshot_.stages[static_cast<std::size_t>(stage_)];
    Add(counter.calls, 1);
    if (failed) Add(counter.failures, 1);
    if (valid) {
      Add(counter.valid_samples, 1);
      Add(counter.wall_ns, end - start_);
      counter.maximum_ns = std::max(counter.maximum_ns, end - start_);
    }
  }

  SelfContactAttemptDiagnostics& snapshot_;
  const DiagnosticClock clock_;
  SelfContactDiagnosticStage stage_ = SelfContactDiagnosticStage::Setup;
  std::uint64_t start_ = 0;
  bool start_valid_ = false;
};

}  // namespace tlfea::contact::self_contact_transaction
