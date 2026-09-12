#pragma once
#include "../ShellBatchFailure.h"
#include "../ShellBatchPlasticity.h"
#if defined(__CUDACC__)
#define TL_RESIDENT_FAILURE_HD __host__ __device__
#else
#define TL_RESIDENT_FAILURE_HD
#endif

namespace tl::fea::shell_batch_plasticity_detail {
// Check copied bool representations before evaluating them as bool values.
inline bool ValidFailureEncoding(const ShellBatchFailureState& value) noexcept {
  static_assert(sizeof(bool) == 1, "Readback flag encoding is one byte");
  if (*reinterpret_cast<const unsigned char*>(&value.active) > 1) return false;
  if (value.policy() == ShellFailurePolicy::None) return true;
  if (const auto* points = value.constant_points()) {
    for (unsigned p = 0; p < 3; ++p) {
      if (*reinterpret_cast<const unsigned char*>(&points[p].point_active) > 1) return false;
    }
    return true;
  }
  if (const auto* points = value.tab1_points()) {
    for (unsigned p = 0; p < 3; ++p) {
      if (*reinterpret_cast<const unsigned char*>(&points[p].point_active) > 1) return false;
    }
    return true;
  }
  return false;
}

// Internal conversion precondition: the corresponding immutable policy has
// already been matched by ValidFailureState; no alternate payload is read.
TL_RESIDENT_FAILURE_HD inline sections::ShellLayeredJ2FailureHistory FailureHistory(
    const ShellBatchSectionState& section, const ShellBatchFailureState& sidecar) noexcept {
  sections::ShellLayeredJ2FailureHistory out;
  out.saved = section.history;
  out.element_active = sidecar.active;
  for (unsigned p = 0; p < 3; ++p) {
    out.failure[p] = sidecar.constant_points()[p];
    out.current_force_point[p] = sidecar.current_force_point[p];
  }
  return out;
}
TL_RESIDENT_FAILURE_HD inline ShellBatchFailureState FailureState(
    const sections::ShellLayeredJ2FailureHistory& history) noexcept {
  auto out = ShellBatchFailureState::Constant();
  out.active = history.element_active;
  for (unsigned p = 0; p < 3; ++p) {
    out.constant_points()[p] = history.failure[p];
    out.current_force_point[p] = history.current_force_point[p];
  }
  return out;
}

TL_RESIDENT_FAILURE_HD inline sections::ShellLayeredTab1History Tab1FailureHistory(
    const ShellBatchSectionState& section, const ShellBatchFailureState& sidecar) noexcept {
  sections::ShellLayeredTab1History out;
  out.saved = section.history;
  out.element_active = sidecar.active;
  for (unsigned p = 0; p < 3; ++p) {
    out.failure[p] = sidecar.tab1_points()[p];
    out.current_force_point[p] = sidecar.current_force_point[p];
  }
  return out;
}
TL_RESIDENT_FAILURE_HD inline ShellBatchFailureState FailureState(
    const sections::ShellLayeredTab1History& history) noexcept {
  auto out = ShellBatchFailureState::Tab1();
  out.active = history.element_active;
  for (unsigned p = 0; p < 3; ++p) {
    out.tab1_points()[p] = history.failure[p];
    out.current_force_point[p] = history.current_force_point[p];
  }
  return out;
}

TL_RESIDENT_FAILURE_HD inline bool ValidFailureState(const ShellBatchFailureState& value,
    ShellFailurePolicy expected, const ShellBatchSectionState* section, double time) noexcept {
  if (value.policy() != expected || !tl::math::Finite(time) || time < 0) return false;
  if (expected == ShellFailurePolicy::None) {
    if (!value.active) return false;
    for (unsigned p = 0; p < 3; ++p) {
      for (double x : value.current_force_point[p].stress) {
        if (x != 0) return false;
      }
    }
    return true;
  }
  if (!section) return false;
  if (expected == ShellFailurePolicy::Tab1AnyPoint) {
    for (unsigned p = 0; p < 3; ++p) {
      const auto& point = value.tab1_points()[p];
      if (!material::failure::ValidTab1ConstantHistory(point) || point.failure_time_s > time ||
          (point.point_active && point.failure_time_s != 0) ||
          point.maximum_damage != ::fmin(1., point.damage)) return false;
    }
    return sections::layered_tab1_detail::ValidHistory(Tab1FailureHistory(*section, value));
  }
  if (expected != ShellFailurePolicy::ConstantAllPoints) return false;
  for (unsigned p = 0; p < 3; ++p) {
    const auto& point = value.constant_points()[p];
    if (!tl::math::Finite(point.damage) || point.damage < 0 || point.damage > 1 ||
        !tl::math::Finite(point.failure_time_s) || point.failure_time_s < 0 || point.failure_time_s > time ||
        (point.point_active && (point.damage >= 1 || point.failure_time_s != 0)) ||
        (!point.point_active && point.damage != 1)) {
      return false;
    }
  }
  return sections::layered_j2_failure_detail::ValidHistory(FailureHistory(*section, value));
}
} // namespace tl::fea::shell_batch_plasticity_detail
#undef TL_RESIDENT_FAILURE_HD
