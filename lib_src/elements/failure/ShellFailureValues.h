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
  for (const auto& point : value.point) {
    if (*reinterpret_cast<const unsigned char*>(&point.point_active) > 1) return false;
  }
  return true;
}

TL_RESIDENT_FAILURE_HD inline sections::ShellLayeredJ2FailureHistory FailureHistory(
    const ShellBatchSectionState& section, const ShellBatchFailureState& sidecar) noexcept {
  sections::ShellLayeredJ2FailureHistory out;
  out.saved = section.history;
  out.element_active = sidecar.active;
  for (unsigned p = 0; p < 3; ++p) {
    out.failure[p] = sidecar.point[p];
    out.current_force_point[p] = sidecar.current_force_point[p];
  }
  return out;
}
TL_RESIDENT_FAILURE_HD inline ShellBatchFailureState FailureState(
    const sections::ShellLayeredJ2FailureHistory& history) noexcept {
  ShellBatchFailureState out;
  out.policy = ShellFailurePolicy::ConstantAllPoints;
  out.active = history.element_active;
  for (unsigned p = 0; p < 3; ++p) {
    out.point[p] = history.failure[p];
    out.current_force_point[p] = history.current_force_point[p];
  }
  return out;
}

TL_RESIDENT_FAILURE_HD inline bool ValidFailureState(const ShellBatchFailureState& value,
    ShellFailurePolicy expected, const ShellBatchSectionState* section, double time) noexcept {
  if (value.policy != expected || !tl::math::Finite(time) || time < 0) return false;
  if (expected == ShellFailurePolicy::None) {
    if (!value.active) return false;
    for (unsigned p = 0; p < 3; ++p) {
      if (!value.point[p].point_active || value.point[p].damage != 0 || value.point[p].failure_time_s != 0) {
        return false;
      }
      for (double x : value.current_force_point[p].stress) {
        if (x != 0) return false;
      }
    }
    return true;
  }
  if (expected != ShellFailurePolicy::ConstantAllPoints || !section) return false;
  for (const auto& point : value.point) {
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
