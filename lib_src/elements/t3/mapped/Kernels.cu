// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblyValues.h"
#include "../T3BatchStorage.h"
#include "../../mapped_shell/Kernels.cuh"

namespace tl::fea::t3::batch_detail {
namespace {
struct AssemblyFamily {
  using Storage = batch_detail::Storage;
  using Slab = batch_detail::Slab;
  using BatchStatus = t3::BatchStatus;
  using ForceTrial = t3::ForceTrial;
  using Memory = mapped::AssemblyMemory;
  static constexpr unsigned Slots = 3;
  __device__ static mapped::AssemblyParent Prepare(const Model& model, const ForceTrial& result,
      std::size_t parent, ShellSectionLaw law, const NodalAssemblyView& view, bool initial) noexcept {
    return mapped::PrepareAssemblyParent(model, result, parent, law, view, initial);
  }
};
} // namespace
void LaunchMappedAssembly(Storage* storage, const Slab* accepted, NodalAssemblyView view,
    NodalCinAssemblyView cin, const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, bool initial) {
  mapped_shell::LaunchAssembly<AssemblyFamily>(storage, accepted, view, cin, mixed, initial);
}
} // namespace tl::fea::t3::batch_detail
