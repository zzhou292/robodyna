// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/constraints/tied_shell/runtime/CinForceStage.h"
#include "lib_src/constraints/tied_shell/TiedPatchMotion.h"

namespace tl::constraints::tied_shell::cin_recovery_frozen {
using cin::StageView;
using cin::MotionTrial;
using cin::StageReport;
using cin::StageStatus;
namespace detail { using cin::detail::ReadXyz; }
namespace detail {
TL_TIED_PATCH_HD inline void WriteXyz(double* field, std::uint32_t node, Vec3 value) noexcept {
  field[3*node] = value.x;
  field[3*node+1] = value.y;
  field[3*node+2] = value.z;
}
}
// Uses force-stage DPARA and offsets, after the common owner's master kick.
// This operation does not integrate positions or restore secondary M/J.
TL_TIED_PATCH_HD inline StageReport RecoverMotionTrial(StageView model, MotionTrial trial) noexcept {
  if (!model.rows || !trial.patches || !trial.velocity_xyz || !trial.angular_velocity_xyz ||
      !trial.acceleration_xyz || !trial.angular_acceleration_xyz) return {StageStatus::InvalidInput};
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
    const auto row = model.rows[r];
    MasterMotion masters;
    for (unsigned slot = 0; slot < 4; ++slot) {
      masters.velocity[slot] = detail::ReadXyz(trial.velocity_xyz, row.masters[slot]);
      masters.acceleration[slot] = detail::ReadXyz(trial.acceleration_xyz, row.masters[slot]);
    }
    SecondaryMotion recovered;
    if (RecoverMotion(trial.patches[r], masters, recovered) != Status::Success) {
      return {StageStatus::NonfiniteResult, r, row.secondary};
    }
    detail::WriteXyz(trial.velocity_xyz, row.secondary, recovered.velocity);
    detail::WriteXyz(trial.angular_velocity_xyz, row.secondary, recovered.angular_velocity);
    detail::WriteXyz(trial.acceleration_xyz, row.secondary, recovered.acceleration);
    detail::WriteXyz(trial.angular_acceleration_xyz, row.secondary, recovered.angular_acceleration);
  }
  return {};
}
} // namespace tl::constraints::tied_shell::cin
