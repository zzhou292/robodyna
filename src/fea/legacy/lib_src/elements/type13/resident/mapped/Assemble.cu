// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../OperationChecks.h"

namespace tl::fea::type13 {
BatchReport Batch::AssembleMappedAccepted(FENodalState& owner, const NodalTrialToken& token,
    const NodalAssemblyView& view) {
  if (!impl_) return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  auto& state = *impl_;
  if (!state.physical || state.physical->owner != &owner) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 requires its actual physical owner"};
  }
  state.Discard();
  auto report = state.PendingError();
  if (!report) return report;
  NodalCinAssemblyView cin;
  const auto borrowed = shell_physical_owner::BorrowAssembly(owner, token, state.accepted_stamp,
      view, state.physical->witness_count, &cin);
  if (borrowed.status != NodalStatus::Ok) {
    if (borrowed.status == NodalStatus::DeviceFailure) state.usable = false;
    // A token failure may expire this view; never mark it through stale pointers.
    owner.Discard();
    return batch_detail::NodalFailure(borrowed);
  }
  auto reject = [&](BatchReport failure) {
    batch_detail::LaunchFailure(view);
    auto marked = state.Runtime(cudaGetLastError(), "Mapped TYPE13 rejection launch failed");
    if (!marked) return marked;
    marked = state.Runtime(cudaStreamSynchronize(view.stream), "Mapped TYPE13 rejection stream failed");
    return marked ? failure : marked;
  };
  const auto& stamp = state.accepted_stamp;
  if (!batch_detail::CompleteAssembly(view, stamp) ||
      view.mass.node_count != stamp.node_count || view.mass.base_epoch != stamp.epoch ||
      !view.mass.inverse_mass || !view.mass.fixed || !view.inverse_inertia ||
      !view.translation_fixed_bits || !view.rotation_fixed ||
      (stamp.has_rotation_presence && !view.rotation_present)) {
    return reject({BatchStatus::InvalidInput, "Mapped TYPE13 assembly sources are incomplete"});
  }
  if (view.temporal_scheme != stamp.temporal_scheme || view.velocity_phase != stamp.velocity_phase ||
      view.position_time != stamp.time || view.velocity_time != stamp.velocity_time ||
      !view.attempt || view.attempt <= state.assembled_attempt ||
      (state.bound && view.stream != state.stream)) {
    return reject({BatchStatus::StaleTrial, "Mapped TYPE13 assembly phase/attempt differs"});
  }
  state.assembled_attempt = view.attempt;
  state.assembled_epoch = UINT64_MAX;
  state.stream = view.stream;
  batch_detail::LaunchAssembly(state.device, state.accepted_slab, view, cin,
                              stamp.epoch == 0, true);
  report = state.ReadControl();
  if (!report) return report;
  if (!state.bound) state.physical->initial_sources = view;
  state.bound = true;
  state.assembled_epoch = stamp.epoch;
  return {};
}
} // namespace tl::fea::type13
