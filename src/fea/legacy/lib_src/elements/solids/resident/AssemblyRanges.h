// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Arena.h"
namespace tl::fea::solids::batch_detail {
TL_BRICK_HD inline bool AssemblyDisjoint(const void* a, std::size_t an,
    const void* b, std::size_t bn) noexcept {
  const auto x = reinterpret_cast<std::uintptr_t>(a), y = reinterpret_cast<std::uintptr_t>(b);
  return a && b && an <= UINTPTR_MAX-x && bn <= UINTPTR_MAX-y &&
      (x+an <= y || y+bn <= x);
}
// Borrowed owner arrays normally guarantee this. Keep malformed/overlapping
// private inputs on the exact original path rather than grant new authority.
TL_BRICK_HD inline bool CanGatherAssembly(Storage& state, NodalAssemblyView view,
    NodalCinAssemblyView cin) noexcept {
  const auto n = view.accepted.node_count;
  if (n != state.config.owner.node_count || view.forces.node_count != n ||
      n > SIZE_MAX / (3*sizeof(double))) return false;
  const void* writes[]{view.forces.force_x, view.forces.force_y,
      view.forces.force_z, cin.translational_stiffness};
  const auto bytes = n*sizeof(double);
  for (unsigned i = 0; i < 4; ++i) {
    if (!AssemblyDisjoint(writes[i], bytes, &state, state.assembly.arena_bytes) ||
        !AssemblyDisjoint(writes[i], bytes, view.accepted.position_xyz, 3*bytes) ||
        !AssemblyDisjoint(writes[i], bytes, view.accepted.velocity_xyz, 3*bytes)) return false;
    for (unsigned j = 0; j < i; ++j)
      if (!AssemblyDisjoint(writes[i], bytes, writes[j], bytes)) return false;
  }
  return state.assembly.offsets && state.assembly.incidence && state.assembly.nodes;
}
} // namespace tl::fea::solids::batch_detail
