// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CinStageTypes.h"
#include "../TiedPatchForce.h"

namespace tl::constraints::tied_shell::cin::detail {
TL_TIED_PATCH_HD inline Vec3 ReadXyz(const double* field, std::uint32_t node) noexcept {
  return {field[3*node], field[3*node+1], field[3*node+2]};
}
TL_TIED_PATCH_HD inline bool Nonnegative(double x) noexcept {
  return tied_shell::detail::math::Finite(x) && x >= 0;
}

// Owner-private, per-attempt leaf packet. Source topology and borrowed extents
// are authenticated before preparation. It is neither history nor admission.
struct PreparedForceRow {
  StageReport report;
  Patch patch;
  MasterLoads transferred_load;
  CoefficientTransfer transferred_coefficients;
  double secondary_mass = 0;
};

TL_TIED_PATCH_HD inline StageReport PrepareForceRow(StageView model, ForceTrial trial,
  std::uint32_t r, PreparedForceRow& output) noexcept {
  const auto n = model.node_count;
  const auto row = model.rows[r];
  const auto secondary = row.secondary;
  PatchInput geometry;
  geometry.secondary_position = detail::ReadXyz(trial.position_xyz, secondary);
  for (unsigned slot = 0; slot < 4; ++slot) {
    geometry.master_position[slot] = detail::ReadXyz(trial.position_xyz, row.masters[slot]);
  }
  Patch patch;
  if (PreparePatch(geometry, patch) != Status::Success) return {StageStatus::InvalidPatch, r, secondary};
  const SecondaryLoad load{{trial.loads[secondary], trial.loads[n+secondary], trial.loads[2*n+secondary]},
                           {trial.loads[3*n+secondary], trial.loads[4*n+secondary], trial.loads[5*n+secondary]}};
  MasterLoads transferred_load;
  if (TransferLoad(patch, load, transferred_load) != Status::Success) {
    return {StageStatus::NonfiniteResult, r, secondary};
  }
  CoefficientInput coefficients;
  coefficients.secondary = {trial.mass[secondary], trial.inertia[secondary],
    trial.translational_stiffness[secondary], trial.rotational_stiffness[secondary]};
  for (unsigned slot = 0; slot < 4; ++slot) {
    coefficients.initial_master_inertia[slot] = trial.entry_inertia[row.masters[slot]];
  }
  CoefficientTransfer transferred_coefficients;
  if (TransferCoefficients(patch, coefficients, transferred_coefficients) != Status::Success) {
    return {StageStatus::NonfiniteResult, r, secondary};
  }
  output.patch = patch;
  output.transferred_load = transferred_load;
  output.transferred_coefficients = transferred_coefficients;
  output.secondary_mass = coefficients.secondary.mass;
  return {};
}

// Every addition and post-addition check stays in native row/slot order. A
// failed leaf is observed at its original row, after every earlier row's apply.
TL_TIED_PATCH_HD inline StageReport ApplyForceRow(StageView model, ForceTrial trial,
  std::uint32_t r, const PreparedForceRow& prepared) noexcept {
  if (!prepared.report) return prepared.report;
  const auto n = model.node_count;
  const auto row = model.rows[r];
  const auto secondary = row.secondary;
  const auto& patch = prepared.patch;
  const auto& transferred_load = prepared.transferred_load;
  const auto& transferred_coefficients = prepared.transferred_coefficients;
  const auto secondary_mass = prepared.secondary_mass;
  // Native NSV order and four ordered slots, including a triangle's repeat.
  for (unsigned slot = 0; slot < 4; ++slot) {
    const auto node = row.masters[slot];
    const auto force = transferred_load.force[slot];
    trial.loads[node] = trial.loads[node] + force.x;
    trial.loads[n+node] = trial.loads[n+node] + force.y;
    trial.loads[2*n+node] = trial.loads[2*n+node] + force.z;
  }
  // Do not combine this expression into += numerical_mass_delta: native
  // adds the new four-slot mass before subtracting the source secondary M.
  *trial.numerical_mass = *trial.numerical_mass + 4*transferred_coefficients.master[0].mass
      - secondary_mass;
  for (unsigned slot = 0; slot < 4; ++slot) {
    const auto node = row.masters[slot];
    trial.mass[node] = trial.mass[node] + transferred_coefficients.master[slot].mass;
  }
  for (unsigned slot = 0; slot < 4; ++slot) {
    const auto node = row.masters[slot];
    trial.translational_stiffness[node] = trial.translational_stiffness[node]
        + transferred_coefficients.master[slot].translational_stiffness;
  }
  for (unsigned slot = 0; slot < 4; ++slot) {
    const auto node = row.masters[slot];
    trial.inertia[node] = trial.inertia[node] + transferred_coefficients.master[slot].inertia;
  }
  if (trial.mass[secondary] != 0) trial.saved_secondary_mass[r] = trial.mass[secondary];
  if (trial.inertia[secondary] != 0) trial.saved_secondary_inertia[r] = trial.inertia[secondary];
  trial.mass[secondary] = 0;
  trial.inertia[secondary] = 0;
  trial.translational_stiffness[secondary] = 1e-20;
  trial.rotational_stiffness[secondary] = 1e-20;
  trial.patches[r] = patch;
  if (!tied_shell::detail::math::Finite(*trial.numerical_mass)) {
    return {StageStatus::NonfiniteResult, r, secondary};
  }
  for (const auto node : row.masters) {
    if (!detail::Nonnegative(trial.mass[node]) || !detail::Nonnegative(trial.inertia[node]) ||
        !detail::Nonnegative(trial.translational_stiffness[node]) ||
        !tied_shell::detail::math::Finite(Vec3{trial.loads[node], trial.loads[n+node], trial.loads[2*n+node]})) {
      return {StageStatus::NonfiniteResult, r, node};
    }
  }
  return {};
}
} // namespace tl::constraints::tied_shell::cin::detail
