#include "T3BatchStorage.h"
#include "../failure/ShellFailureReadback.h"
#include "../ShellPreparedActivityReadback.h"
#include <algorithm>

namespace tl::fea::t3 {
BatchReport T3Batch::CopyAcceptedFailureHistory(const NodalStamp& expected,
    ShellBatchFailureState* output, std::size_t capacity, BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
  if (state.plasticity && state.plasticity->one_point_sections()) {
    return {BatchStatus::InvalidInput, "One-point failure history requires typed complete section readback"};
  }
  if (!state.plasticity || !state.plasticity->failure_sections()) {
    return {BatchStatus::InvalidInput, "No explicit failure sidecar"};
  }
  if (!state.bound) return {BatchStatus::NotBound, "Initial source binding is required"};
  if (capacity != state.config.element_count) {
    return {BatchStatus::ResourceLimit, "Failure readback requires exact complete family capacity"};
  }
  if (!batch_detail::SameStamp(expected, state.accepted_stamp)) {
    return {BatchStatus::StaleTrial, "Accepted failure endpoint identity mismatch"};
  }
  if (!diagnostics ||
      !shell_batch_plasticity_detail::FailureOutputRanges(expected, output, capacity, diagnostics, *this)) {
    return {BatchStatus::InvalidInput, "Failure readback outputs are missing or overlap inputs"};
  }
  if (!state.OutputDisjoint(output,capacity*sizeof(*output)) ||
      !state.OutputDisjoint(diagnostics,sizeof(*diagnostics))) {
    return {BatchStatus::InvalidInput,"Mapped readback output aliases retained source/state"};
  }
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      state.AcceptedSlabIndex(), state.accepted_diagnostics.time);
  if (report.status != BatchStatus::Success) return report;
  std::copy_n(state.plasticity->failure_staging(), capacity, output);
  *diagnostics = state.accepted_diagnostics;
  return {BatchStatus::Success, "OK"};
}

BatchReport T3Batch::CopyAcceptedParentActivity(const NodalStamp& expected,
    std::uint8_t* output, std::size_t capacity, BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
  if (!state.plasticity || !state.plasticity->failure_sections()) {
    return {BatchStatus::InvalidInput, "Parent activity needs the explicit failure-capable scope"};
  }
  if (!state.bound) return {BatchStatus::NotBound, "Initial source and virgin history are not bound"};
  if (capacity != state.config.element_count) {
    return {BatchStatus::ResourceLimit, "Activity readback requires exact complete family capacity"};
  }
  if (!batch_detail::SameStamp(expected, state.accepted_stamp)) {
    return {BatchStatus::StaleTrial, "Accepted parent activity endpoint differs"};
  }
  if (!diagnostics || !shell_batch_plasticity_detail::FailureOutputRanges(
      expected, output, capacity, diagnostics, *this)) {
    return {BatchStatus::InvalidInput, "Activity output is missing or overlaps inspected inputs"};
  }
  if (!state.OutputDisjoint(output,capacity*sizeof(*output)) ||
      !state.OutputDisjoint(diagnostics,sizeof(*diagnostics))) {
    return {BatchStatus::InvalidInput,"Mapped readback output aliases retained source/state"};
  }
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      state.AcceptedSlabIndex(), state.accepted_diagnostics.time);
  if (report.status != BatchStatus::Success) return report;
  if (state.plasticity->one_point_sections()) {
    const auto checked = state.ValidateOnePointReadback(state.AcceptedSlabIndex(),
        state.accepted_diagnostics.time, state.accepted_stamp.epoch);
    if (checked.status != BatchStatus::Success) return checked;
  }
  const auto* history = state.plasticity->failure_staging();
  const auto* sections = state.plasticity->section_staging();
  for (std::size_t parent = 0; parent < capacity; ++parent) {
    const auto* point = sections[parent].one_point();
    const bool active = point ? point->point.failure.history.point_active : history[parent].active;
    output[parent] = active ? 1 : 0;
  }
  *diagnostics = state.accepted_diagnostics;
  return {BatchStatus::Success, "Accepted parent activity copied"};
}

BatchReport T3Batch::CopyPreparedParentActivity(FENodalState& owner,
    const NodalTrialToken& token, const BatchDiagnostics& expected,
    std::uint8_t* output, std::size_t capacity) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
  if (!state.plasticity || !state.plasticity->failure_sections()) {
    return {BatchStatus::InvalidInput, "Prepared activity needs the explicit failure-capable scope"};
  }
  auto report = shell_activity_detail::PreparedPreflight(state, *this, owner, token,
      expected, batch_detail::SameDiagnostics(expected, state.candidate_diagnostics), output, capacity);
  if (report.status != BatchStatus::Success) return report;
  if (!shell_activity_detail::JoinedOutputDisjoint(state,output,capacity))
    return {BatchStatus::InvalidInput,"Prepared activity overlaps retained joined shell sources"};
  const auto slab = 1u - state.AcceptedSlabIndex();
  report = shell_batch_plasticity_detail::ReadFailure(state, slab, expected.time);
  if (report.status == BatchStatus::Success && state.plasticity->one_point_sections()) {
    report = state.ValidateOnePointReadback(slab, expected.time, expected.epoch);
  }
  if (report.status != BatchStatus::Success) {
    state.Discard();
    return report;
  }
  const auto* history = state.plasticity->failure_staging();
  const auto* sections = state.plasticity->section_staging();
  for (std::size_t parent = 0; parent < capacity; ++parent) {
    const auto* point = sections[parent].one_point();
    const bool active = point ? point->point.failure.history.point_active : history[parent].active;
    output[parent] = active ? 1 : 0;
  }
  return {BatchStatus::Success, "Complete prepared parent activity copied"};
}

BatchReport T3Batch::CopyPreparedFailureHistory(const BatchDiagnostics& expected,
    ShellBatchFailureState* output, std::size_t capacity) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
  if (state.plasticity && state.plasticity->one_point_sections()) {
    return {BatchStatus::InvalidInput, "One-point failure history requires typed complete section readback"};
  }
  if (!state.plasticity || !state.plasticity->failure_sections()) {
    return {BatchStatus::InvalidInput, "No explicit failure sidecar"};
  }
  if (!state.bound) return {BatchStatus::NotBound, "Initial source binding is required"};
  if (capacity != state.config.element_count) {
    return {BatchStatus::ResourceLimit, "Failure readback requires exact complete family capacity"};
  }
  if (!state.pending || !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics)) {
    return {BatchStatus::StaleTrial, "Prepared failure endpoint identity mismatch"};
  }
  if (!shell_batch_plasticity_detail::FailureOutputRanges(expected, output, capacity,
                                                        static_cast<BatchDiagnostics*>(nullptr), *this)) {
    return {BatchStatus::InvalidInput, "Failure readback output is missing or overlaps input"};
  }
  if (!state.OutputDisjoint(output,capacity*sizeof(*output))) {
    return {BatchStatus::InvalidInput,"Mapped readback output aliases retained source/state"};
  }
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      1u - state.AcceptedSlabIndex(), state.candidate_diagnostics.time);
  if (report.status != BatchStatus::Success) {
    state.Discard();
    return report;
  }
  std::copy_n(state.plasticity->failure_staging(), capacity, output);
  return {BatchStatus::Success, "OK"};
}
} // namespace tl::fea::t3
