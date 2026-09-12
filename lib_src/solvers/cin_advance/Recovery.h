// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Input.h"

namespace tl::fea::cin_advance::recovery {
namespace cin = constraints::tied_shell::cin;
TL_SURFACE_HD inline cin::MotionTrial MotionView(Input input) noexcept {
  const auto nodes = input.model.node_count;
  return {input.patches, input.trial+3*nodes, input.trial+6*nodes,
      input.work+3*nodes, input.work+6*nodes};
}
TL_SURFACE_HD inline Row Prepare(Input input, std::uint32_t row) noexcept {
  Row result;
  result.valid = bool(cin::detail::PrepareMotionRow(input.model, MotionView(input), row, result.motion));
  return result;
}
TL_SURFACE_HD inline void Publish(Input input, std::uint32_t row, FailureRow first_failure) noexcept {
  if (row < first_failure)
    cin::detail::ApplyMotionRow(input.model, MotionView(input), row, input.prepared_recovery[row].motion);
}
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::recovery
