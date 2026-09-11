// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ResultChecks.h"
#include <cstring>

namespace tl::fea::type13 {
BatchReport Batch::Impl::ReadResults(unsigned slab) {
  auto report = PendingError();
  if (!report) {
    return report;
  }
  if (slab > 1) {
    return {BatchStatus::InvalidInput, "Unknown TYPE13 result slab"};
  }
  report = Runtime(cudaMemcpyAsync(staging.get(), device_header.slab[slab],
      Count() * sizeof(Evaluation), cudaMemcpyDeviceToHost, stream),
      "TYPE13 complete result readback failed");
  if (!report) {
    return report;
  }
  report = Runtime(cudaStreamSynchronize(stream), "TYPE13 readback stream failed");
  if (!report) {
    return report;
  }
  const auto& model = *source.model();
  for (std::size_t e = 0; e < Count(); ++e) {
    if (!batch_detail::ValidResult(*model.property(model.connections()[e].property), staging[e])) {
      return {BatchStatus::NonfiniteResult, "TYPE13 complete cached result is inconsistent", e};
    }
  }
  return {};
}

BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected, Evaluation* output,
                                        std::size_t count, BatchDiagnostics* diagnostics) {
  if (!impl_) {
    return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  }
  auto& state = *impl_;
  if (!state.bound) {
    return {BatchStatus::NotBound, "TYPE13 initial live-source binding required"};
  }
  if (!trial_identity::SameStamp(expected, state.accepted_stamp)) {
    return {BatchStatus::StaleTrial, "TYPE13 accepted owner stamp differs"};
  }
  if (count != state.Count()) {
    return {BatchStatus::ResourceLimit, "TYPE13 readback requires exact complete count"};
  }
  const auto bytes = count * sizeof(Evaluation);
  using trial_identity::Disjoint;
  if (!state.OutputDisjoint(output, bytes) ||
      !state.OutputDisjoint(diagnostics, sizeof(*diagnostics)) ||
      !Disjoint(output, bytes, diagnostics, sizeof(*diagnostics)) ||
      !Disjoint(output, bytes, &expected, sizeof(expected)) ||
      !Disjoint(diagnostics, sizeof(*diagnostics), &expected, sizeof(expected)) ||
      !Disjoint(output, bytes, this, sizeof(*this)) ||
      !Disjoint(diagnostics, sizeof(*diagnostics), this, sizeof(*this))) {
    return {BatchStatus::InvalidInput, "TYPE13 readback ranges overlap or are missing"};
  }
  const auto report = state.ReadResults(state.accepted_slab);
  if (!report) {
    return report;
  }
  std::memcpy(output, state.staging.get(), bytes);
  *diagnostics = state.accepted_diagnostics;
  return {};
}

BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected,
                                        Evaluation* output, std::size_t count) {
  if (!impl_) {
    return {BatchStatus::NotInitialized, "TYPE13 batch is not initialized"};
  }
  auto& state = *impl_;
  if (!state.pending || !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics)) {
    return {BatchStatus::StaleTrial, "TYPE13 complete pending diagnostic identity differs"};
  }
  if (count != state.Count()) {
    return {BatchStatus::ResourceLimit, "TYPE13 readback requires exact complete count"};
  }
  const auto bytes = count * sizeof(Evaluation);
  if (!state.OutputDisjoint(output, bytes) ||
      !trial_identity::Disjoint(output, bytes, &expected, sizeof(expected)) ||
      !trial_identity::Disjoint(output, bytes, this, sizeof(*this))) {
    return {BatchStatus::InvalidInput, "TYPE13 readback ranges overlap or are missing"};
  }
  const auto report = state.ReadResults(state.TrialSlab());
  if (!report) {
    return report;
  }
  std::memcpy(output, state.staging.get(), bytes);
  return {};
}
} // namespace tl::fea::type13
