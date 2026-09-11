// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../T3BatchDiagnostics.h"
#include "Result.h"

namespace tl::fea::t3::mapped {
// Assembly has completed on this stream. Its typed transient parent/node
// records are available until the next assembly, without a lifetime overlay.
TL_T3_HD inline void PrepareDiagnosticParent(const batch_detail::Model& model,
    const ForceTrial& result, Status status, const ShellSectionLaw* roles, std::size_t parent,
    const NodalPreparedView& view, AssemblyParent& scratch) noexcept {
  scratch.status = BatchStatus::Success;
  // Failed element evaluation can leave an unwritten trial. Its status always
  // precedes result validation, so do not inspect that packet here.
  if (status != Status::kSuccess || !roles) return;
  if (!ValidResult(model.element[parent].reference, result, view.proposed_time,
      view.kinematics.base_epoch + 1, roles[parent] == ShellSectionLaw::RigidSkin)) {
    scratch.status = BatchStatus::NonfiniteResult;
  }
}
TL_T3_HD inline void PrepareDiagnosticNode(const batch_detail::Model& model,
    const NodalPreparedView& view, std::size_t node, AssemblyNode& scratch) noexcept {
  using namespace shell_batch_fields;
  const auto dx = Difference(ReadVector(view.kinematics.position_xyz, node), model.initial_position[node]);
  scratch.value[0] = ::hypot(::hypot(dx.x, dx.y), dx.z);
  scratch.touched = false;
  if (!tl::math::Finite(scratch.value[0])) return;
  const auto* q = view.kinematics.orientation_wxyz + 4 * node;
  scratch.touched = tl::math::UnitQuaternion({q[0], q[1], q[2], q[3]});
}
} // namespace tl::fea::t3::mapped
