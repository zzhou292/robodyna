// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../DiagnosticClock.h"
#include "../FixedTriangleDiscoveryDiagnostics.h"
#include "../FixedTriangleFeatureTypes.h"
#include <algorithm>
#include <cerrno>

namespace tlfea::contact::fixed_triangle_features {
// Coarse scopes only. Workers never touch the diagnostic record. Disabled
// calls read no clock; failed/backward samples cannot affect geometry/status.
class DiscoveryTiming {
 public:
  DiscoveryTiming(FixedTriangleDiscoveryDiagnostics& snapshot, bool enabled,
                  const FixedTriangleDiscoveryReport& report,
                  diagnostic::Clock clock = {}) noexcept
      : snapshot_(snapshot), report_(report),
        clock_{clock.read ? clock.read : diagnostic::Nanoseconds, clock.context} {
    snapshot_ = {};
    if (!enabled) return;
    snapshot_.enabled = true;
    Start(FixedTriangleDiscoveryStage::InputLedger);
  }
  ~DiscoveryTiming() noexcept {
    if (!snapshot_.enabled) return;
    snapshot_.succeeded = report_.status == FixedTriangleDiscoveryStatus::Ok;
    Finish(!snapshot_.succeeded);
    snapshot_.finished = true;
  }
  DiscoveryTiming(const DiscoveryTiming&) = delete;
  DiscoveryTiming& operator=(const DiscoveryTiming&) = delete;
  void Stage(FixedTriangleDiscoveryStage stage) noexcept {
    if (!snapshot_.enabled) return;
    Finish(false);
    Start(stage);
  }
 private:
  void Add(std::uint64_t& target, std::uint64_t value) noexcept {
    if (value > UINT64_MAX - target) {
      target = UINT64_MAX;
      snapshot_.timing.counter_saturated = true;
    } else target += value;
  }
  bool Read(std::uint64_t& value) noexcept {
    const int saved = errno;
    const bool valid = clock_.read(clock_.context, &value);
    errno = saved;
    if (!valid) Add(snapshot_.timing.clock_failures, 1);
    return valid;
  }
  void Start(FixedTriangleDiscoveryStage stage) noexcept {
    stage_ = stage;
    start_valid_ = Read(start_);
  }
  void Finish(bool failed) noexcept {
    std::uint64_t end = 0;
    const bool end_valid = Read(end);
    bool valid = start_valid_ && end_valid;
    if (valid && end < start_) {
      Add(snapshot_.timing.backward_samples, 1);
      valid = false;
    }
    auto& counter = snapshot_.timing.stages[static_cast<std::size_t>(stage_)];
    Add(counter.calls, 1);
    if (failed) Add(counter.failures, 1);
    if (valid) {
      Add(counter.valid_samples, 1);
      Add(counter.wall_ns, end - start_);
      counter.maximum_ns = std::max(counter.maximum_ns, end - start_);
    }
  }
  FixedTriangleDiscoveryDiagnostics& snapshot_;
  const FixedTriangleDiscoveryReport& report_;
  const diagnostic::Clock clock_;
  FixedTriangleDiscoveryStage stage_ = FixedTriangleDiscoveryStage::InputLedger;
  std::uint64_t start_ = 0;
  bool start_valid_ = false;
};
}  // namespace tlfea::contact::fixed_triangle_features
