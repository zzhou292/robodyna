// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceGatherValues.h"
#include "ForceTransfers.h"

namespace tl::fea::cin_advance::force_gather {
TL_TIED_PATCH_HD inline void Resolve(const Input& input) noexcept {
  auto& summary = *input.force_gather.summary;
  const auto force = force_inputs::ForceView(input);
  double numerical_mass = 0;
  if (summary.mode == Mode::Staging && NumericalMass(input.model, force,
      input.prepared_transfers, numerical_mass)) {
    summary.numerical_mass = numerical_mass;
    summary.report = {};
    summary.mode = Mode::Publish;
    return;
  }
  // All original destinations and control are still untouched. The complete
  // serial apply owns the first error and every partly published failing row.
  // Even a successful conservative fallback must suppress staged publication.
  summary.report = force_transfers::Apply(input);
  summary.mode = Mode::SerialCompleted;
}
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::force_gather
