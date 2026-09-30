#pragma once
#include "sections/ShellLayeredJ2Failure.h"
#include "sections/ShellLayeredTab1.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tl::fea {
// Orthogonal to the complete collection's LAW1/LAW44 material declaration.
enum class ShellFailurePolicy : std::uint32_t {
  None = 0,
  ConstantAllPoints = 1,
  Tab1AnyPoint = 2
};
struct ShellBatchFailureState {
  TL_SHELL_SECTION_HD ShellBatchFailureState() noexcept = default;
  TL_SHELL_SECTION_HD ShellFailurePolicy policy() const noexcept { return policy_; }
  TL_SHELL_SECTION_HD const sections::ConstantFailureHistory* constant_points() const noexcept {
    return policy_ == ShellFailurePolicy::ConstantAllPoints ? value_.constant.point : nullptr;
  }
  TL_SHELL_SECTION_HD sections::ConstantFailureHistory* constant_points() noexcept {
    return policy_ == ShellFailurePolicy::ConstantAllPoints ? value_.constant.point : nullptr;
  }
  TL_SHELL_SECTION_HD const material::failure::Tab1ConstantFailureHistory* tab1_points() const noexcept {
    return policy_ == ShellFailurePolicy::Tab1AnyPoint ? value_.tab1.point : nullptr;
  }
  TL_SHELL_SECTION_HD material::failure::Tab1ConstantFailureHistory* tab1_points() noexcept {
    return policy_ == ShellFailurePolicy::Tab1AnyPoint ? value_.tab1.point : nullptr;
  }
  TL_SHELL_SECTION_HD static ShellBatchFailureState Constant() noexcept {
    return {ShellFailurePolicy::ConstantAllPoints, Payload(ConstantPoints{})};
  }
  TL_SHELL_SECTION_HD static ShellBatchFailureState Tab1() noexcept {
    return {ShellFailurePolicy::Tab1AnyPoint, Payload(Tab1Points{})};
  }
  sections::ShellFailureForcePoint current_force_point[3]{};
  bool active = true; // Completed native OFF: never the caller's pending 0.8.
 private:
  struct ConstantPoints { sections::ConstantFailureHistory point[3]{}; };
  struct Tab1Points { material::failure::Tab1ConstantFailureHistory point[3]{}; };
  union Payload {
    ConstantPoints constant;
    Tab1Points tab1;
    TL_SHELL_SECTION_HD Payload() noexcept : constant{} {}
    TL_SHELL_SECTION_HD explicit Payload(const ConstantPoints& value) noexcept : constant(value) {}
    TL_SHELL_SECTION_HD explicit Payload(const Tab1Points& value) noexcept : tab1(value) {}
  };
  TL_SHELL_SECTION_HD ShellBatchFailureState(ShellFailurePolicy policy, Payload value) noexcept
      : policy_(policy), value_(value) {}
  ShellFailurePolicy policy_ = ShellFailurePolicy::None;
  Payload value_;
};
// A trivial union copy starts the corresponding member lifetime in its destination.
// Only the tagged active fields are values; padding/inactive bytes are not a
// successful readback, equality, serialization or identity contract. None has no
// exposed point payload. Named factories initialize the active member explicitly.
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
