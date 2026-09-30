// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CinStageTypes.h"
#include "../TiedPatchForce.h"

namespace tl::constraints::tied_shell::cin {
namespace detail {
TL_TIED_PATCH_HD inline Vec3 ReadXyz(const double* field, std::uint32_t node) noexcept {
  return {field[3*node], field[3*node+1], field[3*node+2]};
}
TL_TIED_PATCH_HD inline bool Nonnegative(double x) noexcept {
  return tied_shell::detail::math::Finite(x) && x >= 0;
}
TL_TIED_PATCH_HD inline StageReport CheckForcePointers(StageView model, ForceTrial trial) noexcept {
  if (!model.rows || !model.dependent_nodes || !model.node_count || !model.row_count ||
      !trial.position_xyz || !trial.loads || !trial.mass || !trial.inertia ||
      !trial.translational_stiffness || !trial.rotational_stiffness ||
      !trial.saved_secondary_mass || !trial.saved_secondary_inertia ||
      !trial.numerical_mass || !trial.entry_inertia || !trial.patches || !trial.witness_activity) {
    return {StageStatus::InvalidInput};
  }
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceNode(StageView model, ForceTrial trial,
    std::uint32_t i) noexcept {
  if (!Nonnegative(trial.mass[i]) || !Nonnegative(trial.inertia[i]) ||
      !Nonnegative(trial.translational_stiffness[i]) || !Nonnegative(trial.rotational_stiffness[i]) ||
      !tied_shell::detail::math::Finite(ReadXyz(trial.position_xyz, i))) {
    return {StageStatus::InvalidInput, UINT32_MAX, i};
  }
  for (unsigned axis = 0; axis < 6; ++axis) {
    if (!tied_shell::detail::math::Finite(trial.loads[axis*model.node_count+i])) {
      return {StageStatus::InvalidInput, UINT32_MAX, i};
    }
  }
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceAfterNodes(StageView model, ForceTrial trial) noexcept {
  if (!tied_shell::detail::math::Finite(*trial.numerical_mass)) return {StageStatus::InvalidInput};
  for (std::uint32_t w = 0; w < model.witness_count; ++w) {
    if (model.first_witness && (model.first_witness[w] > w ||
        trial.witness_activity[w] != trial.witness_activity[model.first_witness[w]])) {
      return {StageStatus::SourceMismatch};
    }
  }
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
    const auto row = model.rows[r];
    if (row.secondary >= model.node_count || !row.witnesses.count ||
        row.witnesses.offset > model.witness_count ||
        row.witnesses.count > model.witness_count-row.witnesses.offset ||
        !Nonnegative(trial.saved_secondary_mass[r]) || !Nonnegative(trial.saved_secondary_inertia[r])) {
      return {StageStatus::InvalidInput, r};
    }
    bool active = false;
    for (std::uint32_t w = 0; w < row.witnesses.count; ++w) {
      const auto flag = trial.witness_activity[row.witnesses.offset+w];
      if (flag != 1 && flag != 2) return {StageStatus::PendingReleaseEligibility, r, row.secondary};
      active = active || flag == 1;
    }
    if (!active) return {StageStatus::PendingReleaseEligibility, r, row.secondary};
    for (const auto node : row.masters) {
      if (node >= model.node_count || node == row.secondary || model.dependent_nodes[node]) {
        return {StageStatus::ConflictingRole, r, node};
      }
    }
  }
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceInputs(StageView model, ForceTrial trial) noexcept {
  auto report = CheckForcePointers(model, trial);
  if (!report) return report;
  for (std::uint32_t node = 0; node < model.node_count; ++node) {
    report = CheckForceNode(model, trial, node);
    if (!report) return report;
  }
  return CheckForceAfterNodes(model, trial);
}
// Inputs and entry IN are complete before this strictly ordered row/slot pass.
TL_TIED_PATCH_HD inline StageReport TransferForceTrial(StageView model, ForceTrial trial) noexcept {
  const auto n = model.node_count;
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
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
        - coefficients.secondary.mass;
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
  }
  return {};
}
} // namespace detail

// Selected serial IPARIT0/IRODDL1/WEIGHT1/IDEL2=1 force phase. A failure may
// leave private trial destinations partly written; the sole nodal owner must
// discard them. No accepted input or caller publication is modified.
TL_TIED_PATCH_HD inline StageReport PrepareForceTrial(StageView model, ForceTrial trial) noexcept {
  auto report = detail::CheckForceInputs(model, trial);
  if (!report) return report;
  const auto n = model.node_count;
  // INTTI1 copies entry IN before any ordered CIN coefficient transfer.
  for (std::uint32_t i = 0; i < n; ++i) trial.entry_inertia[i] = trial.inertia[i];
  return detail::TransferForceTrial(model, trial);
}
} // namespace tl::constraints::tied_shell::cin
