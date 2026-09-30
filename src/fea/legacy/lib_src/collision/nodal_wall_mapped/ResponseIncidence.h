// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ResponseTypes.h"

namespace tlfea::contact::nodal_wall_mapped::response {
// The caller supplies fresh, disjoint arena regions. Validate every borrowed
// root before writing. Offsets double as construction cursors, then are restored;
// no additional host staging or borrowed lifetime is retained.
inline bool BuildIncidence(const std::uint32_t* roots, std::size_t nodes,
    std::size_t groups, Scratch output) noexcept {
  if (!nodes || nodes > UINT32_MAX || groups > 1024 || !roots ||
      !output.offsets || !output.rows || output.node_count != nodes) return false;
  for (std::size_t i = 0; i < nodes; ++i)
    if (roots[i] != UINT32_MAX && roots[i] >= groups) return false;
  for (std::size_t g = 0; g <= groups; ++g) output.offsets[g] = 0;
  for (std::size_t i = 0; i < nodes; ++i) {
    output.rows[i] = 0;
    if (roots[i] != UINT32_MAX) ++output.offsets[roots[i]+1];
  }
  for (std::size_t g = 1; g <= groups; ++g) output.offsets[g] += output.offsets[g-1];
  for (std::size_t i = 0; i < nodes; ++i)
    if (roots[i] != UINT32_MAX) output.rows[output.offsets[roots[i]]++] = static_cast<std::uint32_t>(i);
  for (std::size_t g = groups; g > 0; --g) output.offsets[g] = output.offsets[g-1];
  output.offsets[0] = 0;
  return true;
}
} // namespace tlfea::contact::nodal_wall_mapped::response
