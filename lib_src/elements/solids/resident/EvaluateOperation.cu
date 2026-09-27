// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::solids {
BatchReport Batch::EvaluateCandidate(FENodalState& owner, const NodalTrialToken& token,
    const NodalPreparedView& view, BatchDiagnostics* output) {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  auto& state = *impl_;
  using trial_identity::Disjoint;
  if (reinterpret_cast<std::uintptr_t>(output) % alignof(BatchDiagnostics) ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &owner, sizeof(owner)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &view, sizeof(view)))
    return {BatchStatus::InvalidInput, "Solid diagnostic output overlaps inspected input or storage"};
  state.Discard();
  auto report = state.PendingError();
  if (!report) return report;
  if (!state.bound || !state.publication_scope)
    return {BatchStatus::NotBound, "Solid common publication claim required"};
  const auto authenticated = native_physical_coefficients::AuthenticatePrepared(owner, token,
      state.accepted_stamp, view);
  if (authenticated.status != NodalStatus::Ok) {
    if (authenticated.status == NodalStatus::DeviceFailure) state.usable = false;
    return batch_detail::NodalFailure(authenticated);
  }
  const auto& stamp = state.accepted_stamp;
  if (!batch_detail::CandidatePhase(stamp, view) || view.stream != state.stream ||
      !view.attempt || view.attempt <= state.last_candidate_attempt ||
      state.assembled_epoch != stamp.epoch || state.assembled_attempt != view.attempt)
    return {BatchStatus::StaleTrial, "Solid candidate differs from its accepted RHS interval"};
  state.last_candidate_attempt = view.attempt;
  BatchDiagnostics identity;
  identity.source_instance_id = state.model.source_instance_id();
  identity.owner_id = stamp.owner_id;
  identity.configuration_id = state.config.configuration_id;
  identity.qualification_id = state.config.qualification_id;
  identity.epoch = stamp.epoch + 1;
  identity.base_epoch = stamp.epoch;
  identity.attempt = view.attempt;
  identity.time = view.proposed_time;
  identity.base_time = view.base_time;
  identity.velocity_time = view.velocity_time;
  identity.base_velocity_time = view.base_velocity_time;
  identity.kick_dt = view.kick_dt;
  identity.phase = BatchPhase::Prepared;
  identity.has_completed_interval = true;
  identity.accepted_force_assembled = true;
  batch_detail::LaunchCandidate(state.device, state.accepted_slab, state.TrialSlab(), view, identity,state.device_header.controlled.packet_count);
  report = state.ReadControl();
  if (!report) return report;
  state.candidate_diagnostics = state.control.diagnostics;
  state.candidate_view = view;
  state.pending = true;
  *output = state.candidate_diagnostics;
  return {};
}
} // namespace tl::fea::solids
