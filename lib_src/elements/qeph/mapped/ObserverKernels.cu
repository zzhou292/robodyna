// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ObserverValues.h"
#include "../../ShellMixedSectionArenaLayout.h"

namespace tl::fea::qeph::batch_detail {
namespace {
using mapped::ObserverSummary;
__device__ void ReduceBlock(ObserverSummary* values) {
  for(unsigned offset=mapped::ObserverThreads/2;offset;offset/=2) {
    __syncthreads();
    if(threadIdx.x<offset) mapped::MergeObservations(values[threadIdx.x],values[threadIdx.x+offset]);
  }
  __syncthreads();
}
__global__ void ObserveMapped(Storage* storage,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& s=*storage;
  const auto blocks=mapped::ObserverBlocks(s.model.config.element_count,s.model.config.owner.node_count);
  if(blockIdx.x>=blocks) return;
  __shared__ ObserverSummary values[mapped::ObserverThreads];
  ObserverSummary out{};
  const auto first=blockIdx.x*blockDim.x+threadIdx.x;
  const auto stride=blocks*blockDim.x;
  const auto* roles=mixed?mixed->law:nullptr;
  // The preceding same-stream preparation kernel initialized all scratch and
  // never read a failed candidate packet. No accepted state is written here.
  for(std::size_t p=first;p<s.model.config.element_count;p+=stride)
    mapped::ObserveParent(s,*accepted,*trial,view,roles,identity.accepted_force_assembled,p,out);
  for(std::size_t n=first;n<s.model.config.owner.node_count;n+=stride)
    mapped::ObserveNode(s,n,out);
  values[threadIdx.x]=out;
  ReduceBlock(values);
  if(!threadIdx.x) s.assembly.observer[blockIdx.x]=values[0];
}
__global__ void FinishMappedObservers(Storage* storage,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity,
    const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  auto& s=*storage;
  const auto blocks=mapped::ObserverBlocks(s.model.config.element_count,s.model.config.owner.node_count);
  __shared__ ObserverSummary values[mapped::ObserverThreads];
  ObserverSummary out{};
  if(!blocks) out.serial=true;
  for(unsigned i=threadIdx.x;i<blocks;i+=blockDim.x) mapped::MergeObservations(out,s.assembly.observer[i]);
  values[threadIdx.x]=out;
  ReduceBlock(values);
  if(!threadIdx.x) mapped::FinalizeObservations(s,*accepted,*trial,view,identity,
      mixed?mixed->law:nullptr,values[0],s.control);
}
}
void LaunchMappedObserverReduction(Storage* s,const Slab* a,const Slab* b,NodalPreparedView view,
    BatchDiagnostics identity,const shell_batch_plasticity_detail::MixedDeviceStorage* mixed) {
  ObserveMapped<<<mapped::ObserverMaxBlocks,mapped::ObserverThreads,0,view.stream>>>(s,a,b,view,identity,mixed);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  FinishMappedObservers<<<1,mapped::ObserverThreads,0,view.stream>>>(s,a,b,view,identity,mixed);
}
} // namespace tl::fea::qeph::batch_detail
