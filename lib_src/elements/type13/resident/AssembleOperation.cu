// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"

namespace tl::fea::type13 {
BatchReport Batch::AssembleAccepted(FENodalState& owner, const NodalTrialToken& token,
                                     const NodalAssemblyView& view) {
  if (!impl_) {
    return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  }
  auto& state = *impl_;
  state.Discard();
  if (!state.usable) {
    return {BatchStatus::Unusable, "TYPE13 CUDA storage is poisoned"};
  }
  const auto authenticated = native_physical_coefficients::AuthenticateAccepted(
      owner, state.accepted_stamp, view);
  if (authenticated.status != NodalStatus::Ok) {
    if (authenticated.status == NodalStatus::DeviceFailure) {
      state.usable = false;
    }
    return batch_detail::NodalFailure(authenticated);
  }
  // As with other nodal contributors, the caller owns assembly destinations.
  // Authenticating retained sources never grants access to an expired view.
  auto reject = [&](BatchReport report) {
    if (view.result && view.bounds) {
      batch_detail::LaunchFailure(view);
      auto written = state.Runtime(cudaGetLastError(), "TYPE13 rejection launch failed");
      if (!written) {
        return written;
      }
      written = state.Runtime(cudaStreamSynchronize(view.stream), "TYPE13 rejection stream failed");
      if (!written) {
        return written;
      }
    }
    return report;
  };
  auto report = state.PendingError();
  if (!report) {
    return report;
  }
  const auto& stamp = state.accepted_stamp;
  if (!batch_detail::CompleteAssembly(view, stamp) || view.owner_id != stamp.owner_id ||
      view.temporal_scheme != stamp.temporal_scheme || view.velocity_phase != stamp.velocity_phase ||
      view.position_time != stamp.time || view.velocity_time != stamp.velocity_time ||
      !view.attempt || view.attempt <= state.assembled_attempt ||
      (state.bound && view.stream != state.stream)) {
    return reject({BatchStatus::StaleTrial, "TYPE13 assembly phase/attempt differs"});
  }
  NodalCinAssemblyView cin;
  // With this valid, nonoverlapping output, InvalidInput means that the owner
  // has no CIN admission. All other failures may already invalidate its view.
  const auto borrowed = owner.BorrowCinAssembly(token, &cin);
  const bool native_stiffness = state.config.assembly == BatchAssembly::CinNativeStiffness;
  if (borrowed.status != NodalStatus::Ok &&
      !(borrowed.status == NodalStatus::InvalidInput && !native_stiffness)) {
    if (borrowed.status == NodalStatus::DeviceFailure) {
      state.usable = false;
    }
    owner.Discard();
    return batch_detail::NodalFailure(borrowed);
  }
  if (borrowed.status == NodalStatus::Ok && !native_stiffness) {
    owner.Discard();
    return {BatchStatus::InvalidInput, "A CIN owner requires native TYPE13 stiffness assembly"};
  }
  if (native_stiffness) {
    if (cin.owner_id != stamp.owner_id || cin.base_epoch != stamp.epoch ||
        cin.attempt != view.attempt || cin.node_count != stamp.node_count ||
        cin.stream != view.stream || !cin.translational_stiffness || !cin.rotational_stiffness) {
      return reject({BatchStatus::StaleTrial, "TYPE13 CIN stiffness destination differs"});
    }
  }
  if (!state.bound) {
    const auto records = state.source.records();
    for (std::size_t e = 0; e < state.Count(); ++e) {
      const std::size_t nodes[2] = {records[2 * e].value.global_node,
                                    records[2 * e + 1].value.global_node};
      const auto qualified = owner.ValidateNonRigidNodes(nodes, 2);
      if (qualified.status != NodalStatus::Ok) {
        auto failure = batch_detail::NodalFailure(qualified);
        failure.element = e;
        if (qualified.status == NodalStatus::DeviceFailure) {
          state.usable = false;
          return failure;
        }
        failure.status = BatchStatus::InvalidInput;
        return reject(failure);
      }
    }
  }
  state.assembled_attempt = view.attempt;
  state.assembled_epoch = UINT64_MAX;
  state.stream = view.stream;
  batch_detail::LaunchAssembly(state.device, state.accepted_slab, view, cin, !state.bound);
  report = state.ReadControl();
  if (!report) {
    return report;
  }
  state.bound = true;
  state.assembled_epoch = stamp.epoch;
  return {};
}
} // namespace tl::fea::type13
