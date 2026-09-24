// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SelfContactTransactionDiagnostics.h"
#include "../DiagnosticClock.h"
#include "../FixedTriangleFeatureTypes.h"
#include "CrossingBatch.h"
#include "../RepresentedIntervalCrossingGpu.h"

#include <algorithm>
#include <cerrno>
#include <ctime>

namespace tlfea::contact::self_contact_transaction {

using DiagnosticClock = diagnostic::Clock;
using diagnostic::Nanoseconds;
inline constexpr auto DiagnosticNanoseconds = Nanoseconds;

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
  void Discovery(const FixedTriangleDiscoveryReport& report,
                 const FixedTriangleDiscoveryDiagnostics& child = {}) noexcept {
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
    if (child.enabled && child.finished) {
      auto& total = counts.timing;
      const auto& timing = child.timing;
      total.counter_saturated |= timing.counter_saturated;
      // Use a child-specific saturation flag as well as the enclosing attempt.
      const auto add = [&](std::uint64_t& target, std::uint64_t value) {
        if (value > UINT64_MAX - target) total.counter_saturated = true;
        Add(target, value);
      };
      add(counts.timed_calls, 1);
      add(total.clock_failures, timing.clock_failures);
      add(total.backward_samples, timing.backward_samples);
      for (std::size_t i = 0; i < total.stages.size(); ++i) {
        auto& dst = total.stages[i];
        const auto& src = timing.stages[i];
        add(dst.calls, src.calls); add(dst.failures, src.failures);
        add(dst.valid_samples, src.valid_samples); add(dst.wall_ns, src.wall_ns);
        dst.maximum_ns = std::max(dst.maximum_ns, src.maximum_ns);
      }
    }
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
  void CrossingDevice(const RepresentedIntervalDeviceReport& report) noexcept {
    if (!snapshot_.enabled || report.status == RepresentedIntervalDeviceStatus::NotInvoked) return;
    auto& device = snapshot_.native_device;
    Add(device.calls, 1);
    Add(device.admitted_pairs, report.device_pairs);
    Add(device.consumed_pairs, report.consumed_device_pairs);
    Add(device.host_pairs, report.host_pairs);
    Add(device.launches, report.batches);
    Add(device.scene_uploads, report.scene_uploads);
    Add(device.numeric_cohorts, report.numeric_cohorts);
    if (report.status != RepresentedIntervalDeviceStatus::Ok) {
      Add(device.failures, 1);
      snapshot_.counts_complete = false;
      device.last_fault_cohort_begin = report.fault_cohort_begin;
      device.last_fault_cohort_count = report.fault_cohort_count;
      device.last_fault_pair_ordinal = report.fault_pair_ordinal;
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
