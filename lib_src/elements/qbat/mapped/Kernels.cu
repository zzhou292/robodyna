// SPDX-License-Identifier: AGPL-3.0-or-later
#include "AssemblyValues.h"
#include "../QbatBatchStorage.h"
#include "../../mapped_shell/Kernels.cuh"
namespace tl::fea::qbat::batch_detail {
namespace {
struct AssemblyFamily {
  using Storage=batch_detail::Storage;
  using Slab=batch_detail::Slab;
  using BatchStatus=qbat::BatchStatus;
  using Memory=mapped::AssemblyMemory;
  using Context=void;
  static constexpr unsigned Slots=4;
  __device__ static bool ValidContext(const void*) { return true; }
  __device__ static mapped::AssemblyParent Prepare(const Model& model,const BatchResult& result,
      std::size_t parent,const void*,const NodalAssemblyView& view,bool initial) {
    return mapped::PrepareAssemblyParent(model,result,parent,view,initial);
  }
  __device__ static mapped::ForceAccess Access(const Slab* accepted,const void*) {
    return {accepted->element};
  }
};
}
void LaunchMappedAssembly(Storage* storage,const Slab* accepted,NodalAssemblyView view,
    NodalCinAssemblyView cin,bool initial) {
  mapped_shell::LaunchAssemblyValues<AssemblyFamily>(storage,accepted,view,cin,nullptr,initial);
}
} // namespace tl::fea::qbat::batch_detail
