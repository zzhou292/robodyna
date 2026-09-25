// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "DeviceFamilies.h"
namespace tl::fea::solids::batch_detail {
struct AssemblyOccurrence {
  std::size_t node = SIZE_MAX;
  tl::math::Vec3 force{};
  double stiffness = 0;
  bool distinct = true;
};
// Family, retained parent, original slot. No sorting or repeated-slot merging.
template<class Traits, bool Values>
TL_BRICK_HD inline bool ReadFamilyOccurrence(Storage& state, unsigned slab,
    std::size_t& ordinal, AssemblyOccurrence& output) noexcept {
  const auto& family = FamilyStorage<Traits>(state);
  const auto count = Traits::nodes * family.count;
  if (ordinal >= count) { ordinal -= count; return false; }
  const auto parent = ordinal / Traits::nodes;
  const auto slot = ordinal % Traits::nodes;
  const auto& nodes = family.parents[parent].domain_nodes;
  output.node = nodes[slot];
  if constexpr (Values) {
    const auto& cache = family.slab[slab][parent].cache;
    output.force = cache.rhs_force_n[slot];
    output.stiffness = cache.stiffness.translation_n_m;
    if constexpr (!std::is_same_v<Traits, Traits18Law44>)
      for (unsigned a = 0; a < Traits::nodes; ++a)
        for (unsigned b = 0; b < a; ++b)
          if (nodes[a] == nodes[b]) output.distinct = false;
  }
  return true;
}
template<bool Values>
TL_BRICK_HD inline bool ReadAssemblyOccurrence(Storage& state, unsigned slab,
    std::size_t ordinal, AssemblyOccurrence& output) noexcept {
  return ReadFamilyOccurrence<Traits18, Values>(state, slab, ordinal, output) ||
      ReadFamilyOccurrence<Traits24, Values>(state, slab, ordinal, output) ||
      ReadFamilyOccurrence<Traits6z, Values>(state, slab, ordinal, output) ||
      ReadFamilyOccurrence<Traits18Law44, Values>(state, slab, ordinal, output) ||
      ReadFamilyOccurrence<Traits18Law90, Values>(state, slab, ordinal, output);
}
} // namespace tl::fea::solids::batch_detail
