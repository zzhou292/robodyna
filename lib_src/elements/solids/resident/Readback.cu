// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "ResultChecks.h"
#include "MaterialUpload.h"

namespace tl::fea::solids {
namespace {
template<class Traits> BatchReport Check(const Model& model, util::HostArena& staging,
    const batch_detail::FamilyLayout& layout, util::ConstView<typename Traits::Parent> parents,
    const BatchDiagnostics& diagnostics, void* device, const batch_detail::ArenaLayout& arena) {
  const auto* values = util::ArenaPointer<batch_detail::State<Traits>>(staging.data(), layout.staging);
  for (std::size_t p = 0; p < parents.size(); ++p) {
    const auto index = parents[p].material_index;
    typename Traits::Material material;
    auto* curves = util::ArenaPointer<double>(device, arena.curves);
    if constexpr (std::is_same_v<Traits, batch_detail::Traits18>) {
      material = batch_detail::ExpectedMaterial36(model, index, curves);
    } else if constexpr (std::is_same_v<Traits, batch_detail::Traits18Law44>) {
      material = batch_detail::ExpectedMaterial44(model, index, curves);
    } else if constexpr (std::is_same_v<Traits, batch_detail::Traits18Law90>) {
      const auto report = batch_detail::ExpectedMaterial90(model, index, curves, material);
      if (!report) return report;
    } else {
      material = model.materials42()[index].value;
    }
    // Named values only; expected device curve addresses are compared, never
    // dereferenced by the host or exposed in a public result packet.
    if (!batch_detail::ValidResult(parents[p], material, values[p], diagnostics.time, diagnostics.epoch))
      return {BatchStatus::NonfiniteResult, "Solid complete cached history or observation differs",
          Traits::family, p};
  }
  return {};
}
template<class Traits> void PublishStagedResults(const util::HostArena& staging,
    const batch_detail::FamilyLayout& layout, typename Traits::Result* output) noexcept {
  const auto* values = util::ArenaPointer<batch_detail::State<Traits>>(staging.data(), layout.staging);
  for (std::size_t p = 0; p < layout.staging.count; ++p)
    output[p] = Traits::Read(values[p].history, values[p].cache);
}
} // namespace
BatchReport Batch::Impl::ReadResults(unsigned slab, const BatchDiagnostics& diagnostics) {
  auto report = PendingError();
  if (!report) return report;
  if (slab > 1) return {BatchStatus::InvalidInput, "Unknown solid result slab"};
  auto copy = [&](const auto& family, const auto& family_layout) {
    if (!family.count) return BatchReport{};
    return Runtime(cudaMemcpyAsync(static_cast<unsigned char*>(staging.data()) + family_layout.staging.offset,
        family.slab[slab], family_layout.staging.bytes, cudaMemcpyDeviceToHost, stream),
        "Solid complete typed result readback failed");
  };
  report = copy(device_header.solid18, layout.solid18);
  if (report) report = copy(device_header.solid24, layout.solid24);
  if (report) report = copy(device_header.solid6z, layout.solid6z);
  if (report) report = copy(device_header.solid18_law44, layout.solid18_law44);
  if (report) report = copy(device_header.solid18_law90, layout.solid18_law90);
  if (!report) return report;
  report = Runtime(cudaStreamSynchronize(stream), "Solid readback stream failed");
  if (!report) return report;
  report = Check<batch_detail::Traits18>(model, staging, layout.solid18, model.solid18(), diagnostics, device, layout);
  if (report) report = Check<batch_detail::Traits24>(model, staging, layout.solid24, model.solid24(), diagnostics, device, layout);
  if (report) report = Check<batch_detail::Traits6z>(model, staging, layout.solid6z, model.solid6z(), diagnostics, device, layout);
  if (report) report = Check<batch_detail::Traits18Law44>(model, staging, layout.solid18_law44, model.solid18_law44(), diagnostics, device, layout);
  if (report) report = Check<batch_detail::Traits18Law90>(model, staging, layout.solid18_law90, model.solid18_law90(), diagnostics, device, layout);
  return report;
}
void Batch::Impl::PublishResults(ResultBuffers output) const noexcept {
  PublishStagedResults<batch_detail::Traits18>(staging, layout.solid18, output.solid18);
  PublishStagedResults<batch_detail::Traits24>(staging, layout.solid24, output.solid24);
  PublishStagedResults<batch_detail::Traits6z>(staging, layout.solid6z, output.solid6z);
  PublishStagedResults<batch_detail::Traits18Law44>(staging, layout.solid18_law44, output.solid18_law44);
  PublishStagedResults<batch_detail::Traits18Law90>(staging, layout.solid18_law90, output.solid18_law90);
}
BatchReport Batch::CopyAcceptedResults(const NodalStamp& expected, ResultBuffers output,
    BatchDiagnostics* diagnostics) {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  auto& state = *impl_;
  if (!state.bound) return {BatchStatus::NotBound, "Solid common publication claim required"};
  if (!trial_identity::SameStamp(expected, state.accepted_stamp))
    return {BatchStatus::StaleTrial, "Solid accepted stamp differs"};
  if (reinterpret_cast<std::uintptr_t>(diagnostics) % alignof(BatchDiagnostics) ||
      !state.OutputBuffers(output, &expected, sizeof(expected), this, sizeof(*this)) ||
      !state.OutputBuffers(output, diagnostics, sizeof(*diagnostics), this, sizeof(*this)) ||
      !state.OutputDisjoint(diagnostics, sizeof(*diagnostics)) ||
      !trial_identity::Disjoint(diagnostics, sizeof(*diagnostics), &expected, sizeof(expected)) ||
      !trial_identity::Disjoint(diagnostics, sizeof(*diagnostics), this, sizeof(*this)))
    return {BatchStatus::InvalidInput, "Solid readback counts, ranges or diagnostics overlap"};
  const auto report = state.ReadResults(state.accepted_slab, state.accepted_diagnostics);
  if (!report) return report;
  state.PublishResults(output);
  *diagnostics = state.accepted_diagnostics;
  return {};
}
BatchReport Batch::CopyPreparedResults(const BatchDiagnostics& expected, ResultBuffers output) {
  if (!impl_) return {BatchStatus::NotInitialized, "Solid batch is not initialized"};
  auto& state = *impl_;
  if (!state.pending || !batch_detail::SameDiagnostics(expected, state.candidate_diagnostics))
    return {BatchStatus::StaleTrial, "Solid prepared diagnostic identity differs"};
  if (!state.OutputBuffers(output, &expected, sizeof(expected), this, sizeof(*this)))
    return {BatchStatus::InvalidInput, "Solid readback counts or ranges overlap"};
  const auto report = state.ReadResults(state.TrialSlab(), state.candidate_diagnostics);
  if (!report) return report;
  state.PublishResults(output);
  return {};
}
} // namespace tl::fea::solids
