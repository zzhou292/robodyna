// SPDX-License-Identifier: AGPL-3.0-or-later
// I25IRTLM non-adhesion local-row phase and I25MAINF contact-loss reset.
// OpenRadioss Copyright(C)2026 Siemens, pinned a62b27e6.
#pragma once
#include "IsotropicResponse.h"
#include <cstdint>
namespace tlfea::contact::radioss_type25 {
namespace history_detail {
TL_MATH_HOST_DEVICE inline bool Valid(const NativeContactRow& row) {
  return friction_detail::Valid(row.history) && tl::math::Finite(row.penetration_auxiliary) &&
      tl::math::Finite(row.penetration_offset) && tl::math::Finite(row.selection_metric[0]) &&
      tl::math::Finite(row.selection_metric[1]);
}
TL_MATH_HOST_DEVICE inline void ClearHistory(NativeContactRow& row) {
  row.history = {}; row.penetration_auxiliary = 0; row.penetration_offset = 0;
}
TL_MATH_HOST_DEVICE inline void ClearMarker(NativeContactRow& row) {
  for (auto& value : row.irtlm) value = 0;
}
} // namespace history_detail
// A row operation, not MPI candidate publication. The caller supplies current
// secondary stiffness and the authentic prior main segment's stiffness.
TL_MATH_HOST_DEVICE inline NormalStatus BeginNativeHistory(const NativeContactRow& old,
    HistoryPhaseInput input, HistoryPhaseResult* output) {
  if (!output || !history_detail::Valid(old) || input.local_processor <= 0 ||
      !normal_detail::Nonnegative(input.secondary_stiffness) ||
      !normal_detail::Nonnegative(input.main_stiffness)) return NormalStatus::InvalidInput;
  HistoryPhaseResult next; next.row = old;
  auto& row = next.row;
  if (old.irtlm[0] > 0) {
    if (input.secondary_stiffness == 0) history_detail::ClearMarker(row);
    else if (old.irtlm[3] == input.local_processor) {
      if (old.irtlm[2] <= 0) return NormalStatus::InvalidInput;
      if (input.main_stiffness == 0) {
        row.irtlm[0] = 0; row.irtlm[1] = 0; row.irtlm[2] = -1; row.irtlm[3] = 0;
        row.selection_metric[0] = native_constant::ep20; row.selection_metric[1] = native_constant::ep20;
        history_detail::ClearHistory(row);
      } else {
        next.retained_candidate = true;
        row.history.previous_force = row.history.staged_force;
        row.history.staged_force = {};
        row.history.normal.previous_penetration = row.history.normal.staged_penetration;
        row.history.normal.staged_penetration = 0;
        row.history.normal.previous_stiffness = row.history.normal.staged_stiffness;
        row.history.normal.staged_stiffness = 0;
        row.selection_metric[0] = native_constant::ep20; row.selection_metric[1] = native_constant::ep20;
      }
    } else {
      history_detail::ClearHistory(row);
      row.selection_metric[0] = native_constant::ep20; row.selection_metric[1] = native_constant::ep20;
    }
  } else {
    row.history.normal.damping_half_force = 0;
    row.penetration_auxiliary = 0;
    row.selection_metric[0] = -native_constant::ep20; row.selection_metric[1] = native_constant::ep20;
  }
  *output = next;
  return NormalStatus::Ok;
}
// Run after native contact classification, before force evaluation. Penetration
// zero alone is NOT the source's contact-loss/reset predicate. TIME_S is retained.
TL_MATH_HOST_DEVICE inline NormalStatus EndNativeContact(const NativeContactRow& old,
    NativeContactRow* output) {
  if (!output || !history_detail::Valid(old)) return NormalStatus::InvalidInput;
  auto next = old;
  const bool leave = old.irtlm[1] < 0 && (-static_cast<std::int64_t>(old.irtlm[1])) % 5 == 0;
  if (old.irtlm[0] > 0 && (old.selection_metric[0] == native_constant::ep20 || leave)) {
    history_detail::ClearMarker(next); history_detail::ClearHistory(next);
  }
  *output = next;
  return NormalStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
