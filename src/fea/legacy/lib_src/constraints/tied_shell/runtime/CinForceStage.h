// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "CinStageTypes.h"
#include "CinForceTransfer.h"

namespace tl::constraints::tied_shell::cin {
namespace detail {
TL_TIED_PATCH_HD inline StageReport CheckForcePointers(StageView model, ForceTrial trial) noexcept {
  const bool empty=model.explicitly_empty;
  if ((empty ? (model.row_count || model.witness_count || model.rows || model.first_witness ||
                 trial.saved_secondary_mass || trial.saved_secondary_inertia || trial.patches || trial.witness_activity) :
                (!model.row_count || !model.rows || !trial.saved_secondary_mass ||
                 !trial.saved_secondary_inertia || !trial.patches || !trial.witness_activity)) ||
      !model.dependent_nodes || !model.node_count ||
      !trial.position_xyz || !trial.loads || !trial.mass || !trial.inertia ||
      !trial.translational_stiffness || !trial.rotational_stiffness ||
      !trial.numerical_mass || !trial.entry_inertia) {
    return {StageStatus::InvalidInput};
  }
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceNode(StageView model, ForceTrial trial,
    std::uint32_t i) noexcept {
  if ((model.explicitly_empty && model.dependent_nodes[i]) ||
      !Nonnegative(trial.mass[i]) || !Nonnegative(trial.inertia[i]) ||
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
// Independent leaves retain the exact serial read and local failure order.
TL_TIED_PATCH_HD inline StageReport CheckForceWitness(StageView model, ForceTrial trial,
    std::uint32_t w) noexcept {
  if (model.first_witness && (model.first_witness[w] > w ||
      trial.witness_activity[w] != trial.witness_activity[model.first_witness[w]])) {
    return {StageStatus::SourceMismatch};
  }
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceRow(StageView model, ForceTrial trial,
    std::uint32_t r) noexcept {
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
  return {};
}
TL_TIED_PATCH_HD inline StageReport CheckForceAfterNodes(StageView model, ForceTrial trial) noexcept {
  if (!tied_shell::detail::math::Finite(*trial.numerical_mass)) return {StageStatus::InvalidInput};
  for (std::uint32_t w = 0; w < model.witness_count; ++w) {
    const auto report = CheckForceWitness(model, trial, w);
    if (!report) return report;
  }
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
    const auto report = CheckForceRow(model, trial, r);
    if (!report) return report;
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
  for (std::uint32_t r = 0; r < model.row_count; ++r) {
    PreparedForceRow prepared;
    prepared.report = PrepareForceRow(model, trial, r, prepared);
    const auto report = ApplyForceRow(model, trial, r, prepared);
    if (!report) return report;
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
