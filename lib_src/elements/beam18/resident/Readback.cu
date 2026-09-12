// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ResultChecks.h"

namespace tl::fea::beam18 {
BatchReport Batch::Impl::ReadResults(unsigned slab, const BatchDiagnostics& diagnostics) {
  auto report = PendingError();
  if (!report) return report;
  if (slab > 1) return {BatchStatus::InvalidInput, "Unknown beam result slab"};
  report = Runtime(cudaMemcpyAsync(staging.data(), device_header.slab[slab], layout.staging.bytes,
      cudaMemcpyDeviceToHost, stream), "Beam complete result readback failed");
  if (!report) return report;
  report = Runtime(cudaStreamSynchronize(stream), "Beam readback stream failed");
  if (!report) return report;
  const auto* values = util::ArenaPointer<batch_detail::State>(staging.data(), layout.staging);
  auto* curves = util::ArenaPointer<double>(device, layout.curves);
  for (std::size_t p = 0; p < model.parents().size(); ++p) {
    const auto& parent = model.parents()[p];
    const auto material = batch_detail::ExpectedMaterial(model, parent.material_index, curves);
    // Full named identity is compared without dereferencing device curve data.
    if (!batch_detail::ValidResult(parent, material, values[p], diagnostics.time, diagnostics.epoch))
      return {BatchStatus::NonfiniteResult, "Beam cached history or observation differs", p};
  }
  return {};
}
void Batch::Impl::PublishResults(ResultBuffer output) const noexcept {
  const auto* values = util::ArenaPointer<batch_detail::State>(staging.data(), layout.staging);
  for (std::size_t p = 0; p < model.parents().size(); ++p)
    output.values[p] = batch_detail::Read(values[p]);
}
BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected, ResultBuffer output,
    BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized, "Beam batch is not initialized"};
  auto& state = *impl_;
  if (!state.bound) return {BatchStatus::NotBound, "Beam publication claim required"};
  if (!trial_identity::SameStamp(expected, state.accepted_stamp))
    return {BatchStatus::StaleTrial, "Beam accepted stamp differs"};
  if (reinterpret_cast<std::uintptr_t>(diagnostics) % alignof(BatchDiagnostics) ||
      !state.OutputBuffers(output, &expected, sizeof(expected), this, sizeof(*this)) ||
      !state.OutputBuffers(output, diagnostics, sizeof(*diagnostics), this, sizeof(*this)) ||
      !state.OutputDisjoint(diagnostics, sizeof(*diagnostics)) ||
      !trial_identity::Disjoint(diagnostics, sizeof(*diagnostics), &expected, sizeof(expected)) ||
      !trial_identity::Disjoint(diagnostics, sizeof(*diagnostics), this, sizeof(*this)))
    return {BatchStatus::InvalidInput, "Beam readback counts, ranges or diagnostics overlap"};
  const auto report = state.ReadResults(state.accepted_slab, state.accepted_diagnostics);
  if (!report) return report;
  state.PublishResults(output);
  *diagnostics = state.accepted_diagnostics;
  return {};
}
BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected, ResultBuffer output) {
  if (!impl_) return {BatchStatus::NotInitialized, "Beam batch is not initialized"};
  auto& state = *impl_;
  if (!state.pending || !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics))
    return {BatchStatus::StaleTrial, "Beam prepared diagnostic identity differs"};
  if (!state.OutputBuffers(output, &expected, sizeof(expected), this, sizeof(*this)))
    return {BatchStatus::InvalidInput, "Beam readback counts or ranges overlap"};
  const auto report = state.ReadResults(state.TrialSlab(), state.candidate_diagnostics);
  if (!report) return report;
  state.PublishResults(output);
  return {};
}
} // namespace tl::fea::beam18
