// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Type13Math.h"
#include "../../ShellBatchStartup.h"

namespace tl::fea::type13::batch_detail {
// Validate complete readback before exposing any caller range. This checks
// typed availability and cached-field consistency, not a recurrence replay.
inline bool ValidResult(const Property& property, const Evaluation& value) noexcept {
  const auto& frame = value.native_frame;
  if (!detail::ValidHistory(value.native_history) ||
      !tl::math::fixed3::Orthonormal(frame.axes) ||
      !tl::math::fixed3::Orthonormal(frame.midpoint_axes) ||
      !detail::Positive(frame.length) || !detail::Positive(frame.midpoint_length) ||
      !detail::Positive(value.stability.critical_dt_s) ||
      !detail::Positive(value.stability.translation_stiffness_N_per_m) ||
      !detail::Positive(value.stability.rotation_stiffness_Nm_per_rad) ||
      (value.newly_failed && value.native_history.active) ||
      !shell_startup_detail::SameVector(value.native_history.transverse_axis,
                                        tl::math::fixed3::Column(frame.axes, 1))) {
    return false;
  }
  auto derived = value;
  if (detail::ConvertSIValues(property, derived) != Status::Success) {
    return false;
  }
  using shell_startup_detail::SameBits;
  using shell_startup_detail::SameVector;
  if (!SameVector(value.local_force_N, derived.local_force_N) ||
      !SameVector(value.local_couple_Nm, derived.local_couple_Nm) ||
      !SameBits(value.total_signed_work_J, derived.total_signed_work_J)) {
    return false;
  }
  for (unsigned local = 0; local < 2; ++local) {
    if (!SameVector(value.endpoints[local].force_N, derived.endpoints[local].force_N) ||
        !SameVector(value.endpoints[local].couple_Nm, derived.endpoints[local].couple_Nm)) {
      return false;
    }
  }
  for (unsigned k = 0; k < ChannelCount; ++k) {
    if (!SameBits(value.signed_work_J[k], derived.signed_work_J[k])) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea::type13::batch_detail
