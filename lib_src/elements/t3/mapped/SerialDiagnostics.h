// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../T3BatchDiagnostics.h"
#include "Result.h"
#include "../../ShellMixedSectionArenaLayout.h"

namespace tl::fea::t3::mapped {
// Full original finalizer, including its partial failed diagnostics. Both the
// legacy launch and every observer fallback use this same function.
TL_T3_HD inline void FinalizeSerial(const batch_detail::Storage& storage,
    const batch_detail::Slab& accepted, const batch_detail::Slab& trial,
    NodalPreparedView view, BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed,
    batch_detail::Control& output) noexcept {
  output = {};
  output.diagnostics = identity;
  const auto& model = storage.model;
  for (unsigned parent = 0; parent < model.config.element_count; ++parent) {
    const auto status = storage.candidate_status[parent];
    if (status != Status::kSuccess) {
      output.status = BatchStatus::ElementFailure;
      output.element = parent;
      output.element_status = status;
      return;
    }
  }
  if (model.mapped) {
    if (!mixed) {
      output.status = BatchStatus::InvalidInput;
      return;
    }
    for (unsigned parent = 0; parent < model.config.element_count; ++parent) {
      const bool skin = mixed->law[parent] == ShellSectionLaw::RigidSkin;
      if (!ValidResult(model.element[parent].reference, trial.element[parent], view.proposed_time,
          view.kinematics.base_epoch + 1, skin)) {
        output.status = BatchStatus::NonfiniteResult;
        output.element = parent;
        return;
      }
    }
  }
  if (!batch_detail::Measure(model, accepted, trial, view, output, model.mapped ? mixed->law : nullptr)) {
    output.status = BatchStatus::NonfiniteResult;
    return;
  }
  output.diagnostics.valid = true;
}
} // namespace tl::fea::t3::mapped
