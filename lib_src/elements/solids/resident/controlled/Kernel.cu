// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../Storage.h"
#include "../DeviceFamilies.h"
#include "Capture.h"
#include "lib_src/elements/solid24/controlled_distortion/Stage.h"
#include "lib_src/elements/solid18/total_strain/controlled_distortion/Stage.h"
#include "lib_src/elements/ShellBatchFields.h"
namespace tl::fea::solids::batch_detail {
namespace {
template<class Traits> __device__ auto Interval(Storage& state,std::size_t p,NodalPreparedView view) {
  auto interval=Traits::Phase(view.base_time,state.config.owner.fixed_dt,view.kinematics.base_epoch);
  const auto& parent=FamilyStorage<Traits>(state).parents[p];
  for(unsigned n=0;n<Traits::nodes;++n)Traits::Node(interval,n,
    shell_batch_fields::ReadVector(view.kinematics.position_xyz,parent.domain_nodes[n]),
    shell_batch_fields::ReadVector(view.kinematics.velocity_xyz,parent.domain_nodes[n]));
  return interval;
}
__global__ void Packets(Storage* storage,unsigned accepted,unsigned trial,NodalPreparedView view,bool initial) {
  using namespace controlled;
  __shared__ unsigned gate;
  auto& state=*storage;auto& controls=state.controlled;
  for(std::size_t packet_index=blockIdx.x;packet_index<controls.packet_count;packet_index+=Blocks) {
    const auto& packet=controls.packets[packet_index].source;
    if(!packet.icontrol)continue; // Uniform branch; unchanged legacy kernels own these rows.
    if(threadIdx.x==0)gate=0;
    __syncthreads();
    const bool active=threadIdx.x<packet.member_count;
    std::size_t parent=0;int status=0;unsigned trigger=0;
    if(active) {
      parent=controls.members[packet.member_begin+threadIdx.x].family_index;
      auto& workspace=controls.workspace[controls.worker_begin[blockIdx.x]+threadIdx.x];
      if(packet.family==Family::Solid24) {
        auto& work=workspace.H24();const auto ref=controls.index24[parent];
        const auto* history=state.solid24.slab[accepted][parent].history.native();
        if(!initial&&!history)status=int(solid24::ForceStatus::InvalidInput);
        else status=int(initial?h24::PrepareInitial(controls.reference24[ref],state.config.startup.uniform_velocity,work.scratch):
          h24::PrepareCandidate(controls.reference24[ref],*history,Interval<Traits24>(state,parent,view),work.scratch));
        if(!status)trigger=work.scratch.activity.triggers_native_batch;
      } else {
        auto& work=workspace.Foam();const auto ref=controls.index90[parent];
        const auto* history=state.solid18_law90.slab[accepted][parent].history.native();
        if(!initial&&!history)status=int(solid18::Status::InvalidInput);
        else status=FoamStatus(initial?foam::PrepareInitial(controls.reference90[ref],state.config.startup.uniform_velocity,work.scratch):
          foam::PrepareCandidate(controls.reference90[ref],*history,Interval<Traits18Law90>(state,parent,view),work.scratch));
        if(!status)trigger=work.scratch.activity.triggers_native_batch;
      }
    }
    if(trigger)atomicOr(&gate,trigger); // Integer OR only; no force/stiffness reduction.
    __syncthreads(); // Every lane reaches the complete native-packet decision, including failures.
    if(active) {
      auto& workspace=controls.workspace[controls.worker_begin[blockIdx.x]+threadIdx.x];
      if(packet.family==Family::Solid24) {
        auto& work=workspace.H24();
        if(!status)status=int(h24::Complete(work.scratch,gate!=0,work.result));
        if(!status)status=Capture(work.result,state.solid24.slab[trial][parent]);
        state.solid24.status[parent]=status;
        if(initial&&!status)state.solid24.slab[1][parent]=state.solid24.slab[0][parent];
      } else {
        auto& work=workspace.Foam();
        if(!status)status=FoamStatus(foam::Complete(work.scratch,gate!=0,work.result));
        if(!status)status=Capture(work.result,state.solid18_law90.slab[trial][parent]);
        state.solid18_law90.status[parent]=status;
        if(initial&&!status)state.solid18_law90.slab[1][parent]=state.solid18_law90.slab[0][parent];
      }
    }
    __syncthreads(); // The bounded workspace cannot be reused by the next packet early.
  }
}
} // namespace
void LaunchControlledInitialize(Storage* storage,cudaStream_t stream) {
  Packets<<<controlled::Blocks,controlled::Threads,0,stream>>>(storage,0,0,{},true);
}
void LaunchControlledCandidate(Storage* storage,unsigned accepted,unsigned trial,NodalPreparedView view) {
  Packets<<<controlled::Blocks,controlled::Threads,0,view.stream>>>(storage,accepted,trial,view,false);
}
} // namespace tl::fea::solids::batch_detail
