// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalCinStorage.h"
#include "FENodalStateStorage.h"
#include "NodalTrialIdentity.h"
#include <cmath>
#include <cstring>

namespace tl::fea {
namespace {
bool OutputsValid(NodalCinSnapshotBuffer out, std::size_t n, std::size_t r,
    const void* stamp, std::size_t stamp_bytes, const void* token = nullptr,
    std::size_t token_bytes = 0) noexcept {
  if (!out.mass || !out.inertia || !out.saved_secondary_mass || !out.saved_secondary_inertia ||
      !out.numerical_mass || !stamp || out.capacity_nodes < n || out.capacity_attachments < r) return false;
  const void* ranges[] = {out.mass, out.inertia, out.saved_secondary_mass,
      out.saved_secondary_inertia, out.numerical_mass, stamp};
  const std::size_t bytes[] = {n*sizeof(double), n*sizeof(double), r*sizeof(double),
      r*sizeof(double), sizeof(double), stamp_bytes};
  for (unsigned i = 0; i < 6; ++i) {
    if (token && !trial_identity::Disjoint(ranges[i], bytes[i], token, token_bytes)) return false;
    for (unsigned j = i+1; j < 6; ++j) {
      if (!trial_identity::Disjoint(ranges[i], bytes[i], ranges[j], bytes[j])) return false;
    }
  }
  return true;
}
void Publish(NodalCinSnapshotBuffer out, const double* tail, std::size_t n, std::size_t r) noexcept {
  std::memcpy(out.mass, tail, n*sizeof(double));
  std::memcpy(out.inertia, tail+n, n*sizeof(double));
  std::memcpy(out.saved_secondary_mass, tail+4*n, r*sizeof(double));
  std::memcpy(out.saved_secondary_inertia, tail+4*n+r, r*sizeof(double));
  *out.numerical_mass = tail[4*n+2*r];
}
}

NodalReport FENodalState::ValidateCinWitnessSource(const NodalCinWitnessSource& source) const noexcept {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  const auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!state.cin || !source.model || !source.model->prepared()) {
    return {NodalStatus::InvalidInput, "No admitted CIN source or supplied immutable model"};
  }
  const auto& cin = *state.cin;
  if (source.range_count != cin.rows.size() || source.witness_count != cin.witnesses.size()) {
    return {NodalStatus::InvalidInput, "CIN source counts differ from the admitted complete roster"};
  }
  if (!source.ranges || !source.witnesses ||
      source.model->rows().data != cin.source.rows().data ||
      !source.model->domain()->SharesStorage(*cin.source.domain())) {
    return {NodalStatus::InvalidInput, "CIN source is not the admitted immutable model/domain backing"};
  }
  for (std::size_t row = 0; row < source.range_count; ++row) {
    const auto& expected = cin.rows[row].witnesses;
    const auto& actual = source.ranges[row];
    if (actual.offset != expected.offset || actual.count != expected.count) {
      return {NodalStatus::InvalidInput, "CIN source witness range differs", std::uint32_t(row)};
    }
  }
  for (std::size_t i = 0; i < source.witness_count; ++i) {
    const auto& actual = source.witnesses[i];
    const auto& expected = cin.witnesses[i];
    bool same = actual.source_element_id == expected.source_element_id &&
        actual.native_parent_index == expected.native_parent_index && actual.family == expected.family;
    for (unsigned slot = 0; slot < 4; ++slot) same = same && actual.nodes[slot] == expected.nodes[slot];
    if (!same) return {NodalStatus::InvalidInput, "CIN source witness differs", std::uint32_t(i)};
  }
  return {NodalStatus::Ok, "Complete CIN roster matches the actual owner"};
}

NodalReport FENodalState::BorrowCinAssembly(const NodalTrialToken& token, NodalCinAssemblyView* output) {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!output || !state.cin) return {NodalStatus::InvalidInput, "Missing CIN assembly output or admission"};
  if (!state.Matches(token.owner_id_, token.base_epoch_, token.attempt_)) {
    return state.Reject(NodalStatus::StaleTrial, "CIN assembly token belongs to another attempt");
  }
  if (state.phase != nodal_detail::Phase::Assembling) {
    return state.Reject(NodalStatus::WrongPhase, "CIN inputs require the open common assembly");
  }
  if (!trial_identity::Disjoint(output, sizeof(*output), &token, sizeof(token))) {
    return {NodalStatus::InvalidInput, "CIN assembly output overlaps its token"};
  }
  const auto& cin = *state.cin;
  const NodalCinAssemblyView next{state.stamp.owner_id, state.stamp.epoch, state.attempt,
    cin.qualification_id, cin.work, cin.work+cin.layout.nodes, cin.activity,
    cin.layout.nodes, cin.layout.witnesses, state.stream};
  *output = next;
  return {NodalStatus::Ok, "Current CIN contributor view borrowed"};
}

NodalReport FENodalState::Impl::StageCinSnapshot(const double* source) {
  const auto& storage = *cin;
  const auto n = storage.layout.nodes;
  const auto r = storage.layout.attachments;
  auto* tail = staging.data()+storage.state_offset;
  auto report = Check(cudaMemcpyAsync(tail, source+storage.state_offset,
      storage.layout.state_values*sizeof(double), cudaMemcpyDeviceToHost, stream));
  if (report.status != NodalStatus::Ok) return report;
  report = Check(cudaStreamSynchronize(stream));
  if (report.status != NodalStatus::Ok) return report;
  for (std::size_t i = 0; i < 4*n+2*r; ++i) {
    if (!std::isfinite(tail[i]) || tail[i] < 0) {
      return Reject(NodalStatus::InvalidOutput, "CIN coefficient/history readback is invalid");
    }
  }
  if (!std::isfinite(tail[4*n+2*r])) {
    return Reject(NodalStatus::InvalidOutput, "CIN numerical mass readback is invalid");
  }
  const bool completed = source == trial || stamp.epoch > 0;
  for (std::size_t i = 0; i < n; ++i) {
    const bool dependent = storage.dependent[i] != 0;
    if (dependent && completed && (tail[i] != 0 || tail[n+i] != 0)) {
      return Reject(NodalStatus::InvalidOutput, "Completed CIN dependent coefficients are not zero", std::uint32_t(i));
    }
    const double inverse_mass = dependent || constraint_staging[n+i] == 7 ? 0 : 1/tail[i];
    const double inverse_inertia = dependent || constraint_staging[2*n+i] ? 0 : 1/tail[n+i];
    if (tail[2*n+i] != inverse_mass || tail[3*n+i] != inverse_inertia) {
      return Reject(NodalStatus::InvalidOutput, "CIN coefficient readback disagrees with its derived inverse", std::uint32_t(i));
    }
  }
  return {NodalStatus::Ok, "CIN snapshot staged"};
}

NodalReport FENodalState::CopyAcceptedCin(NodalCinSnapshotBuffer output, NodalStamp* stamp) {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!state.cin) return {NodalStatus::InvalidInput, "Owner has no CIN admission"};
  const auto n = state.cin->layout.nodes;
  const auto r = state.cin->layout.attachments;
  if (!OutputsValid(output, n, r, stamp, sizeof(*stamp))) {
    return {NodalStatus::InvalidInput, "CIN snapshot outputs overlap or have insufficient capacity"};
  }
  const auto report = state.StageCinSnapshot(state.accepted);
  if (report.status != NodalStatus::Ok) return report;
  Publish(output, state.staging.data()+state.cin->state_offset, n, r);
  *stamp = state.stamp;
  return {NodalStatus::Ok, "Accepted CIN snapshot copied"};
}

NodalReport FENodalState::CopyPreparedCin(const NodalTrialToken& token, NodalCinSnapshotBuffer output,
    NodalPreparedView* prepared) {
  if (!impl_) return {NodalStatus::NotInitialized, "Owner is not initialized"};
  auto& state = *impl_;
  if (!state.usable) return {NodalStatus::DeviceFailure, "CUDA owner is poisoned"};
  if (!state.cin) return {NodalStatus::InvalidInput, "Owner has no CIN admission"};
  const auto n = state.cin->layout.nodes;
  const auto r = state.cin->layout.attachments;
  if (!OutputsValid(output, n, r, prepared, sizeof(*prepared), &token, sizeof(token))) {
    return {NodalStatus::InvalidInput, "CIN candidate outputs overlap or have insufficient capacity"};
  }
  NodalPreparedView next;
  auto report = BorrowPrepared(token, &next);
  if (report.status != NodalStatus::Ok) return report;
  report = state.StageCinSnapshot(state.trial);
  if (report.status != NodalStatus::Ok) return report;
  Publish(output, state.staging.data()+state.cin->state_offset, n, r);
  *prepared = next;
  return {NodalStatus::Ok, "Prepared CIN snapshot copied"};
}
} // namespace tl::fea
