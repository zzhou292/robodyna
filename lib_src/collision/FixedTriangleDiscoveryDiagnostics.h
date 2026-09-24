// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace tlfea::contact {
// Disjoint coordinator elapsed scopes, including worker wake/join in Geometry.
// Never a physical clock, geometry receipt or GPU kernel timer.
enum class FixedTriangleDiscoveryStage : std::uint8_t {
  InputLedger, InputSort, TaskPreparation, Geometry, ResultFold, OutputSort,
  Publication, Count
};
inline constexpr std::size_t FixedTriangleDiscoveryStageCount =
    static_cast<std::size_t>(FixedTriangleDiscoveryStage::Count);
struct FixedTriangleDiscoveryCounter {
  std::uint64_t calls = 0, failures = 0, valid_samples = 0;
  std::uint64_t wall_ns = 0, maximum_ns = 0;
};
struct FixedTriangleDiscoveryTimings {
  bool counter_saturated = false;
  std::uint64_t clock_failures = 0, backward_samples = 0;
  std::array<FixedTriangleDiscoveryCounter, FixedTriangleDiscoveryStageCount> stages{};
};
// A last-call value. Failures retain only the measured prefix; publication is
// independently preserved. Concurrent rejected entry never changes this record.
struct FixedTriangleDiscoveryDiagnostics {
  bool enabled = false, finished = false, succeeded = false;
  FixedTriangleDiscoveryTimings timing;
};
}  // namespace tlfea::contact
