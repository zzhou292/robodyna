// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Results.h"

namespace tl::fea::beam18::batch_detail {
using State = ForceTrial;
TL_BEAM18_HD inline Result Read(const State& state) noexcept {
  Result result;
  result.history = state.proposed_history.values();
  result.stamp = state.proposed_history.stamp();
  result.geometry = state.geometry;
  result.rate = state.rate;
  result.diagnostics = state.diagnostics;
  for (unsigned p = 0; p < 4; ++p) result.point[p] = state.point[p];
  for (unsigned n = 0; n < 2; ++n) {
    result.rhs_force_n[n] = state.rhs_force_n[n];
    result.rhs_couple_nm[n] = state.rhs_couple_nm[n];
  }
  return result;
}
} // namespace tl::fea::beam18::batch_detail
