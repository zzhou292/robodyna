// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Measurement.h"
#include "../QbatBatchStorage.h"
namespace tl::fea::qbat::batch_detail {
namespace {
__global__ void ValidateParents(Storage* state,const Slab* trial,BatchDiagnostics identity) {
  for(std::size_t parent=blockIdx.x*blockDim.x+threadIdx.x;
      parent<state->model.config.element_count;parent+=blockDim.x*gridDim.x) {
    state->assembly.parent[parent].status=state->candidate_status[parent]==Status::kSuccess &&
        ValidResult(trial->element[parent],state->model.element[parent].material,identity.time,identity.epoch)
        ? BatchStatus::Success : BatchStatus::NonfiniteResult;
  }
}
__global__ void MaximumDisplacement(Storage* state,NodalPreparedView view) {
  __shared__ mapped::MaximumSummary values[mapped_shell::ObserverThreads];
  mapped::MaximumSummary next{0,true};
  for(std::size_t node=blockIdx.x*blockDim.x+threadIdx.x;
      node<state->model.config.owner.node_count;node+=blockDim.x*gridDim.x) {
    double magnitude=0;
    if(!NodeDisplacement(state->model,view,node,magnitude)) next.valid=false;
    else if(magnitude>next.value) next.value=magnitude;
  }
  values[threadIdx.x]=next;
  for(unsigned offset=mapped_shell::ObserverThreads/2;offset;offset/=2) {
    __syncthreads();
    if(threadIdx.x<offset) mapped::MergeMaximum(values[threadIdx.x],values[threadIdx.x+offset]);
  }
  __syncthreads();
  if(!threadIdx.x) state->assembly.maximum[blockIdx.x]=values[0];
}
__global__ void FinalizeMapped(Storage* state,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity,unsigned blocks) {
  auto& s=*state;
  s.control={}; s.control.diagnostics=identity;
  for(std::size_t parent=0;parent<s.model.config.element_count;++parent) {
    if(s.candidate_status[parent]==Status::kSuccess) continue;
    s.control.status=BatchStatus::ElementFailure;
    s.control.element=static_cast<std::uint32_t>(parent);
    s.control.element_status=s.candidate_status[parent];
    return;
  }
  if(!mapped::Measure(s,*accepted,*trial,view,s.control.diagnostics,blocks)) {
    s.control.status=BatchStatus::NonfiniteResult; return;
  }
  s.control.diagnostics.valid=true;
}
}
void LaunchMappedMeasurements(Storage* storage,const Slab* accepted,const Slab* trial,
    NodalPreparedView view,BatchDiagnostics identity,std::size_t nodes) {
  const auto blocks=mapped::MaximumBlocks(nodes);
  ValidateParents<<<256,128,0,view.stream>>>(storage,trial,identity);
  MaximumDisplacement<<<blocks,mapped_shell::ObserverThreads,0,view.stream>>>(storage,view);
  FinalizeMapped<<<1,1,0,view.stream>>>(storage,accepted,trial,view,identity,blocks);
}
} // namespace tl::fea::qbat::batch_detail
