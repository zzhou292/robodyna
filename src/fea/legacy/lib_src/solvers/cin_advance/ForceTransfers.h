// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceInputs.h"

namespace tl::fea::cin_advance::force_transfers {
namespace cin = constraints::tied_shell::cin;
using Row = constraints::tied_shell::cin::detail::PreparedForceRow;

// Source admission proves distinct secondaries and no secondary is a master.
// Only those secondary fields, accepted x and entry IN feed the pure leaves.
// Thus earlier ordered applies cannot change another row's prepared inputs.
TL_TIED_PATCH_HD inline cin::StageReport Apply(const Input& input) noexcept {
  const auto force = force_inputs::ForceView(input);
  for (std::uint32_t row = 0; row < input.model.row_count; ++row) {
    const auto report = cin::detail::ApplyForceRow(input.model, force, row, input.prepared_transfers[row]);
    if (!report) return report;
  }
  return {};
}
cudaError_t Launch(const Input&, cudaStream_t);
} // namespace tl::fea::cin_advance::force_transfers
