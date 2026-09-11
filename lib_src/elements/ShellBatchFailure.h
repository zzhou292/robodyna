#pragma once
#include "sections/ShellLayeredJ2Failure.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tl::fea {
// Orthogonal to LAW1/LAW44. The reserved TAB1 value is rejected by this profile.
enum class ShellFailurePolicy : std::uint32_t {
  None = 0,
  ConstantAllPoints = 1,
  Tab1AnyPoint = 2
};
struct ShellBatchFailureState {
  ShellFailurePolicy policy = ShellFailurePolicy::None;
  sections::ConstantFailureHistory point[3]{};
  sections::ShellFailureForcePoint current_force_point[3]{};
  bool active = true; // Completed native OFF: never the caller's pending 0.8.
};
struct ShellBatchFailureLimits {
  std::size_t max_parents = 1024;
  std::size_t max_device_bytes = 1024 * 1024;
  std::size_t max_host_bytes = 16 * 1024 * 1024;
  static constexpr ShellBatchFailureLimits Vehicle() noexcept {
    return {524288, 256ULL * 1024 * 1024, 512ULL * 1024 * 1024};
  }
};
static_assert(std::is_trivially_copyable_v<ShellBatchFailureState>);
} // namespace tl::fea
