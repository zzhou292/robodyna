// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CinMotionRows.h"

namespace tl::constraints::tied_shell::cin {
// Uses force-stage DPARA and offsets, after the common owner's master kick.
// This operation does not integrate positions or restore secondary M/J.
TL_TIED_PATCH_HD inline StageReport RecoverMotionTrial(StageView model, MotionTrial trial) noexcept {
  if (!detail::MotionPointersValid(model, trial)) return {StageStatus::InvalidInput};
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
    SecondaryMotion recovered;
    const auto report = detail::PrepareMotionRow(model, trial, r, recovered);
    if (!report) return report;
    detail::ApplyMotionRow(model, trial, r, recovered);
  }
  return {};
}
} // namespace tl::constraints::tied_shell::cin
