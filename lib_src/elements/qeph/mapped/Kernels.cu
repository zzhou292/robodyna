// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblyValues.h"
#include "../QephBatchStorage.h"
#include "../../mapped_shell/Kernels.cuh"

namespace tl::fea::qeph::batch_detail {
namespace {
struct AssemblyFamily {
  using Storage = batch_detail::Storage;
  using Slab = batch_detail::Slab;
  using BatchStatus = qeph::BatchStatus;
  using ForceTrial = qeph::ForceTrial;
  using Memory = mapped::AssemblyMemory;
  static constexpr unsigned Slots = 4;
  __device__ static mapped::AssemblyParent Prepare(const Model& model, const ForceTrial& result,
      std::size_t parent, ShellSectionLaw law, const NodalAssemblyView& view, bool initial,const ShellGlobalLaw1Profile* global) noexcept {
    return mapped::PrepareAssemblyParent(model, result, parent, law, view, initial,global);
  }
};
} // namespace
void LaunchMappedAssembly(Storage* storage, const Slab* accepted, NodalAssemblyView view,
    NodalCinAssemblyView cin, const shell_batch_plasticity_detail::MixedDeviceStorage* mixed, bool initial) {
  mapped_shell::LaunchAssembly<AssemblyFamily>(storage, accepted, view, cin, mixed, initial);
}
} // namespace tl::fea::qeph::batch_detail
