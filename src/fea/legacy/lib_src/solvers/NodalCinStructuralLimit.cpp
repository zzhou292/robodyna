// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FENodalStateStorage.h"
#include "cin_physical_mains/OutputRanges.h"

namespace tl::fea {
NodalReport FENodalState::CopyPreparedCinStructuralLimit(const NodalTrialToken& token,
    NodalCinStructuralLimit* output) const {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!state.cin) return {NodalStatus::InvalidInput, "Structural limiter requires CIN"};
  if (!state.Matches(token.owner_id_, token.base_epoch_, token.attempt_))
    return {NodalStatus::StaleTrial, "Structural limiter token is not the current owner attempt"};
  if (state.phase != nodal_detail::Phase::AwaitingValidation && state.phase != nodal_detail::Phase::Ready)
    return {NodalStatus::WrongPhase, "Structural limiter requires a successful prepared step"};
  const auto& control = state.host_control;
  const auto& witness = control.structural_limiter;
  if (control.status != NodalStatus::Ok || witness.epoch != state.stamp.epoch ||
      witness.attempt != state.attempt || witness.values.kind == NodalCinLimitKind::Unavailable ||
      witness.values.minimum_dt_s != control.limit.dt)
    return {NodalStatus::InvalidInput, "No captured structural limiter for this successful attempt"};
  if (!output) return {NodalStatus::InvalidInput, "Missing structural limiter output"};
  if (reinterpret_cast<std::uintptr_t>(output) % alignof(NodalCinStructuralLimit))
    return {NodalStatus::InvalidInput, "Structural limiter output is misaligned"};
  namespace ranges = cin_physical_mains;
  if (!ranges::Outside(output, sizeof(*output), &token) ||
      !ranges::Outside(output, sizeof(*output), this) ||
      !ranges::Outside(output, sizeof(*output), impl_.get()) ||
      !ranges::OutsideSources(output, sizeof(*output), *state.cin, state.rigid_groups.get(),
          state.staging, state.constraint_staging))
    return {NodalStatus::InvalidInput, "Structural limiter output overlaps owner, token or retained source"};
  const auto* domain = state.cin->source.domain();
  if (!domain || domain->node_count() != state.config.node_count)
    return {NodalStatus::InvalidOutput, "Structural limiter domain differs from owner"};
  NodalCinStructuralLimit next;
  next.values = witness.values;
  next.owner_id = state.stamp.owner_id;
  next.base_epoch = state.stamp.epoch;
  next.attempt = state.attempt;
  next.cin_qualification_id = state.cin->qualification_id;
  next.source_instance_id = domain->source_instance_id();
  next.base_time_s = state.stamp.time;
  next.owner_fixed_dt_s = state.config.fixed_dt;
  if (next.values.kind != NodalCinLimitKind::Unbounded) {
    if (next.values.node >= domain->node_count())
      return {NodalStatus::InvalidOutput, "Structural limiter node is outside actual domain"};
    next.source_node_id = domain->nodes()[next.values.node].source_id;
  }
  if (next.values.kind == NodalCinLimitKind::RigidTrace) {
    if (!state.rigid_groups || next.values.group >= state.rigid_groups->properties.size())
      return {NodalStatus::InvalidOutput, "Structural limiter group is outside actual binding"};
    const auto& group = state.rigid_groups->properties[next.values.group];
    if (next.values.mass_kg != group.mass_kg ||
        next.values.principal_inertia_kg_m2.x != group.principal.inertia.x ||
        next.values.principal_inertia_kg_m2.y != group.principal.inertia.y ||
        next.values.principal_inertia_kg_m2.z != group.principal.inertia.z)
      return {NodalStatus::InvalidOutput, "Structural limiter aggregate differs from retained source"};
    next.source_kind = group.source_kind;
    next.source_group_id = group.source_id;
    next.source_node_set_id = group.source_node_set_id;
  }
  *output = next;
  return {NodalStatus::Ok, "Actual prepared structural limiter copied"};
}
} // namespace tl::fea
