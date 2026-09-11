// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Measure.h"

namespace tl::fea::type45::resident_detail {
namespace {
__global__ void Initialize(Storage* s) {
  for(std::size_t j=threadIdx.x;j<s->count;j+=blockDim.x) {
    s->status[j]=InitializeState(s->joints[j],s->slab[0][j]);
    if(s->status[j]==Status::Success) s->slab[1][j]=s->slab[0][j];
  }
}
__global__ void Evaluate(Storage* s,unsigned accepted,unsigned trial,NodalPreparedView view) {
  const auto first=blockIdx.x*blockDim.x+threadIdx.x;
  for(std::size_t j=first;j<s->count;j+=gridDim.x*blockDim.x) {
    const auto& row=s->joints[j];
    Interval interval;
    interval.base_time_s=view.base_time;interval.dt_s=s->config.owner.fixed_dt;
    interval.sample_index=view.kinematics.base_epoch+1;
    for(unsigned e=0;e<2;++e) {
      interval.position_m[e]=shell_batch_fields::ReadVector(view.kinematics.position_xyz,row.domain_nodes[e]);
      interval.angular_velocity_rad_s[e]=shell_batch_fields::ReadVector(view.kinematics.angular_velocity_xyz,row.domain_nodes[e]);
    }
    s->status[j]=UpdateState(row,s->slab[accepted][j],view.kinematics.base_epoch?nullptr:s->contexts+j,
        interval,s->slab[trial][j]);
  }
}
__global__ void Finalize(Storage* s,unsigned accepted,unsigned trial,NodalPreparedView view,
    BatchDiagnostics identity,bool initial) {
  s->control={};
  if(initial) {
    identity.source_instance_id=s->source_instance_id;identity.owner_id=s->config.owner.owner_id;
    identity.configuration_id=s->config.configuration_id;identity.qualification_id=s->config.qualification_id;
    identity.phase=BatchPhase::Accepted;
  }
  if(!Measure(*s,accepted,trial,initial?nullptr:&view,identity)) return;
  identity.automatic_stiffness_initialized=!initial;identity.valid=true;
  s->control.diagnostics=identity;
}
}
void LaunchInitialize(Storage* storage,cudaStream_t stream) {
  Initialize<<<1,64,0,stream>>>(storage);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Finalize<<<1,1,0,stream>>>(storage,0,0,{}, {},true);
}
void LaunchCandidate(Storage* storage,unsigned accepted,unsigned trial,NodalPreparedView view,
    BatchDiagnostics identity) {
  Evaluate<<<8,64,0,view.stream>>>(storage,accepted,trial,view);
  if(cudaPeekAtLastError()!=cudaSuccess) return;
  Finalize<<<1,1,0,view.stream>>>(storage,accepted,trial,view,identity,false);
}
} // namespace tl::fea::type45::resident_detail
