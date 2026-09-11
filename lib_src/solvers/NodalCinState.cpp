// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include <algorithm>

namespace tl::fea::nodal_detail {
CinStorage::~CinStorage() { if (arena) cudaFree(arena); }
void CinStorage::InitializeState(double* state, const NodalCinStartup& input,
    const double* inverse_mass, const NodalDofConfig& dofs) const noexcept {
  auto* tail = state+state_offset;
  const auto n = layout.nodes;
  std::copy(input.mass, input.mass+n, tail);
  std::copy(input.inertia, input.inertia+n, tail+n);
  for (std::size_t i = 0; i < n; ++i) {
    tail[2*n+i] = inverse_mass[i];
    // Startup already validated this exact inverse, including dependent/fixed
    // PART dependents and explicitly absent rotations. Do not reconstruct 1/0.
    tail[3*n+i] = dofs.inverse_inertia[i];
  }
  // INIEND initializes ILEV28 SMAS/SINER from literal secondary coefficients.
  // Later force stages update only a nonzero current coefficient.
  for (std::size_t r = 0; r < rows.size(); ++r) {
    tail[4*n+r] = input.mass[rows[r].secondary];
    tail[4*n+rows.size()+r] = input.inertia[rows[r].secondary];
  }
}
} // namespace tl::fea::nodal_detail
