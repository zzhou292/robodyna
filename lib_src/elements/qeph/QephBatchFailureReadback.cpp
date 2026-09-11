#include "QephBatchStorage.h"
#include "../failure/ShellFailureReadback.h"
#include <cstring>

namespace tl::fea::qeph {
BatchReport QephBatch::CopyAcceptedFailureHistory(const NodalStamp& expected,
    ShellBatchFailureState* output, std::size_t capacity, BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
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
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      state.AcceptedSlabIndex(), state.accepted_diagnostics.time);
  if (report.status != BatchStatus::Success) return report;
  std::memcpy(output, state.plasticity->failure_staging(), capacity * sizeof(*output));
  *diagnostics = state.accepted_diagnostics;
  return {BatchStatus::Success, "OK"};
}

BatchReport QephBatch::CopyPreparedFailureHistory(const BatchDiagnostics& expected,
    ShellBatchFailureState* output, std::size_t capacity) {
  if (!impl_) return {BatchStatus::NotInitialized, "Batch is not initialized"};
  auto& state = *impl_;
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
  const auto report = shell_batch_plasticity_detail::ReadFailure(state,
      1u - state.AcceptedSlabIndex(), state.candidate_diagnostics.time);
  if (report.status != BatchStatus::Success) {
    state.Discard();
    return report;
  }
  std::memcpy(output, state.plasticity->failure_staging(), capacity * sizeof(*output));
  return {BatchStatus::Success, "OK"};
}
} // namespace tl::fea::qeph
