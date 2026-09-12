#pragma once
#include "ShellFailureStorage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace tl::fea::shell_batch_plasticity_detail {
template<class Impl, class Validate>
auto ReadFailure(Impl& state, unsigned slab, double time, Validate validate) {
  auto report = state.PendingError();
  using Status = decltype(report.status);
  if (report.status != Status::Success) return report;
  const auto result = state.plasticity->ReadSections(slab, state.config.element_count, state.stream, time);
  if (result.status == SetupStatus::DeviceFailure) {
    return state.Runtime(result.cuda_status, result.message);
  }
  if (result.status != SetupStatus::Success) {
    report.status = result.status == SetupStatus::NonfiniteResult ? Status::NonfiniteResult : Status::InvalidInput;
    report.message = result.message;
  }
  return report.status==Status::Success?validate(slab):report;
}

template<class Impl> auto ReadFailure(Impl& state, unsigned slab, double time) {
  return ReadFailure(state,slab,time,
      [&state](unsigned selected) { return state.ValidateMappedSections(selected); });
}

template<class Stamp, class Value, class Diagnostics, class Batch>
bool FailureOutputRanges(const Stamp& expected, Value* output,
    std::size_t count, Diagnostics* diagnostics, const Batch& batch) noexcept {
  using trial_identity::Disjoint;
  const auto bytes = count * sizeof(Value);
  if (!Disjoint(output, bytes, &expected, sizeof(expected)) ||
      !Disjoint(output, bytes, &batch, sizeof(batch))) {
    return false;
  }
  return !diagnostics ||
      (Disjoint(output, bytes, diagnostics, sizeof(*diagnostics)) &&
       Disjoint(diagnostics, sizeof(*diagnostics), &expected, sizeof(expected)) &&
       Disjoint(diagnostics, sizeof(*diagnostics), &batch, sizeof(batch)));
}
} // namespace tl::fea::shell_batch_plasticity_detail
