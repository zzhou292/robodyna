// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyOccurrences.h"
#include "../../../solvers/NodalForceAssembly.h"
#include "../../../solvers/NodalRepeatedStiffness.h"
namespace tl::fea::solids::batch_detail {
// All destination values stay private until every node succeeds. On failure the
// original serial traversal runs against untouched owner arrays, preserving its
// first error, partial failed-trial writes and common-owner invalidation.
TL_BRICK_HD inline bool GatherAssemblyNode(Storage& state, unsigned slab,
    std::size_t node, DeviceNodalForceView forces, const double* translation,
    AssemblyNode& output) noexcept {
  AssemblyNode next{{forces.force_x[node], forces.force_y[node], forces.force_z[node]},
      translation[node]};
  DeviceNodalForceView local{&next.force[0], &next.force[1], &next.force[2],
      nullptr, nullptr, nullptr, 1, forces.base_epoch};
  const std::size_t zero = 0;
  const auto& memory = state.assembly;
  for (auto i = memory.offsets[node]; i < memory.offsets[node + 1]; ++i) {
    AssemblyOccurrence value;
    if (i >= memory.occurrences || !ReadAssemblyOccurrence<true>(state, slab,
        memory.incidence[i], value) || value.node != node || !value.distinct ||
        AccumulateRepeatedNodalStiffness<1>(&zero, &value.stiffness,
            &next.translation, 1) != NodalForceAssemblyStatus::Success ||
        AccumulateNodalTranslationalForces<1>(&zero, &value.force, local) !=
            NodalForceAssemblyStatus::Success) return false;
  }
  output = next;
  return true;
}
} // namespace tl::fea::solids::batch_detail
