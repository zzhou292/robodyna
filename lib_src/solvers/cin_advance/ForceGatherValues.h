// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceGatherTypes.h"
#include "../../constraints/tied_shell/runtime/CinForceTransfer.h"

namespace tl::fea::cin_advance::force_gather {
TL_TIED_PATCH_HD inline bool ValidMaster(const Master& value) noexcept {
  return cin::detail::Nonnegative(value.mass) && cin::detail::Nonnegative(value.inertia) &&
      cin::detail::Nonnegative(value.stiffness) &&
      constraints::tied_shell::detail::math::Finite(constraints::tied_shell::Vec3{
          value.force[0], value.force[1], value.force[2]});
}

// Independent scalar folds begin at the actual incoming master values. Source
// row/slot order, including both copies of a repeated slot, is never changed.
// Validation occurs after all occurrences of a source row at this master.
TL_TIED_PATCH_HD inline bool GatherMaster(cin::StageView source, cin::ForceTrial trial,
    const cin::detail::PreparedForceRow* prepared, const View& view,
    std::uint32_t index, Master& output) noexcept {
  if (!prepared || index >= view.master_count) return false;
  const auto node = view.nodes[index];
  const auto begin = view.offsets[index], end = view.offsets[index + 1];
  if (node >= source.node_count || source.dependent_nodes[node] || begin >= end ||
      end > 4ull * source.row_count || (index == 0 && begin != 0) ||
      (index + 1 == view.master_count && end != 4ull * source.row_count) ||
      (index && view.nodes[index - 1] >= node)) return false;
  const auto n = source.node_count;
  Master next{{trial.loads[node], trial.loads[n + node], trial.loads[2 * n + node]},
      trial.mass[node], trial.inertia[node], trial.translational_stiffness[node]};
  std::uint32_t previous = UINT32_MAX;
  for (auto cursor = begin; cursor < end; ++cursor) {
    const auto encoded = view.incidence[cursor];
    if (encoded >= 4ull * source.row_count || (cursor != begin && encoded <= previous)) return false;
    const auto row = encoded / 4, slot = encoded % 4;
    if (source.rows[row].masters[slot] != node || !prepared[row].report) return false;
    if (cursor != begin && row != previous / 4 && !ValidMaster(next)) return false;
    const auto force = prepared[row].transferred_load.force[slot];
    const auto& coefficient = prepared[row].transferred_coefficients.master[slot];
    next.force[0] = next.force[0] + force.x;
    next.force[1] = next.force[1] + force.y;
    next.force[2] = next.force[2] + force.z;
    next.mass = next.mass + coefficient.mass;
    next.stiffness = next.stiffness + coefficient.translational_stiffness;
    next.inertia = next.inertia + coefficient.inertia;
    previous = encoded;
  }
  if (!ValidMaster(next)) return false;
  output = next;
  return true;
}

// Preserve the native add-then-subtract recurrence and every finite prefix.
// No value from a rejected prepared row may be consumed.
TL_TIED_PATCH_HD inline bool NumericalMass(cin::StageView source, cin::ForceTrial trial,
    const cin::detail::PreparedForceRow* prepared, double& output) noexcept {
  double next = *trial.numerical_mass;
  for (std::uint32_t row = 0; row < source.row_count; ++row) {
    if (!prepared[row].report) return false;
    next = next + 4 * prepared[row].transferred_coefficients.master[0].mass
        - prepared[row].secondary_mass;
    if (!constraints::tied_shell::detail::math::Finite(next)) return false;
  }
  output = next;
  return true;
}

TL_TIED_PATCH_HD inline void PublishMaster(cin::ForceTrial trial, std::uint32_t nodes,
    std::uint32_t node, const Master& value) noexcept {
  trial.loads[node] = value.force[0];
  trial.loads[nodes + node] = value.force[1];
  trial.loads[2 * nodes + node] = value.force[2];
  trial.mass[node] = value.mass;
  trial.translational_stiffness[node] = value.stiffness;
  trial.inertia[node] = value.inertia;
}

// Unique secondaries, disjoint from every master, retain conditional save
// semantics for both signs of zero. These arrays/patches are indexed by row.
TL_TIED_PATCH_HD inline void PublishSecondary(cin::StageView source, cin::ForceTrial trial,
    const cin::detail::PreparedForceRow* prepared, std::uint32_t row) noexcept {
  const auto secondary = source.rows[row].secondary;
  if (trial.mass[secondary] != 0) trial.saved_secondary_mass[row] = trial.mass[secondary];
  if (trial.inertia[secondary] != 0) trial.saved_secondary_inertia[row] = trial.inertia[secondary];
  trial.mass[secondary] = 0;
  trial.inertia[secondary] = 0;
  trial.translational_stiffness[secondary] = 1e-20;
  trial.rotational_stiffness[secondary] = 1e-20;
  trial.patches[row] = prepared[row].patch;
}
} // namespace tl::fea::cin_advance::force_gather
