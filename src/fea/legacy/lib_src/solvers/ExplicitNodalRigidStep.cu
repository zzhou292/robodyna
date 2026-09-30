// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ExplicitNodalRigidStep.h"
#include "NodalRigidGroupStorage.h"
#include "NodalNodeStep.h"
#include "NodalForceStageCaptureLayout.h"
#include "../constraints/NodalRigidGroupCandidate.h"

namespace tl::fea {
namespace {
__device__ void Fail(nodal_detail::Control* control,NodalStatus status,std::uint32_t node) {
  control->status=status; control->node=node;
  if(status==NodalStatus::StepTooLarge) control->limit.dt=0;
}
template<bool Capture=false>
__global__ void AdvanceRigid(nodal_detail::Control* control,const double* accepted,double* trial,
    const double* loads,const double* inverse,const std::uint8_t* constraints,std::uint32_t n,
    rigid::GroupDeviceView groups,rigid::StepDurations durations,double maximum_angle,
    std::uint64_t epoch,std::uint64_t attempt,const std::uint8_t* rotation_present,
    rigid::AccelerationSink sink={}) {
  if(control->status!=NodalStatus::Ok) return;
  if(control->rows.base_epoch!=epoch||control->rows.attempt!=attempt||
      !stability::IsCurrentLimit(control->rows,control->limit)||control->limit.dt<durations.drift_dt) {
    Fail(control,NodalStatus::StaleTrial,UINT32_MAX); return;
  }
  if(control->limit.has_stiffness_or_damping) {
    Fail(control,NodalStatus::MissingStepAdmission,UINT32_MAX); return;
  }
  for(std::uint32_t i=0;i<n;++i) if(!groups.member_nodes[i]) {
    const auto status=nodal_detail::AdvanceOrdinaryNode<Capture>(accepted,trial,loads,inverse,constraints,i,n,
        durations.drift_dt,durations.kick_dt,maximum_angle,sink.node,sink.node_rotation,rotation_present);
    if(status!=NodalStatus::Ok) { Fail(control,status,i); return; }
  }
  for(std::uint32_t g=0;g<groups.group_count;++g) {
    const auto result=rigid::PrepareGroupCandidate<Capture>(groups,g,accepted,trial,loads,n,durations,sink);
    if(result.status!=rigid::StepStatus::Success) {
      Fail(control,result.status==rigid::StepStatus::RotationLimit?NodalStatus::StepTooLarge:NodalStatus::InvalidOutput,result.node);
      return;
    }
    const auto range=groups.groups[g];
    for(std::uint32_t i=0;i<range.count;++i) {
      const auto node=groups.members[range.offset+i].node;
      const auto status=nodal_detail::PrepareNodeOrientation(accepted,trial,node,n,durations.drift_dt,maximum_angle,false);
      if(status!=NodalStatus::Ok) { Fail(control,status,node); return; }
    }
  }
}
} // namespace

cudaError_t FENodalState::Impl::LaunchRigidAdvance(double maximum_angle) {
  const rigid::StepDurations durations{stamp.epoch==0?0:config.fixed_dt,candidate_kick_dt,config.fixed_dt};
  if(config.capture_force_stage_accelerations) {
    const nodal_detail::ForceStageCaptureLayout layout{config.node_count,rigid_groups->info.group_count};
    AdvanceRigid<true><<<1,1,0,stream>>>(control,accepted,trial,scratch,inverse,fixed,
      static_cast<std::uint32_t>(config.node_count),rigid_groups->device,durations,maximum_angle,stamp.epoch,attempt,
      stamp.has_rotation_presence?fixed+3*config.node_count:nullptr,layout.Sink(scratch));
  } else {
    AdvanceRigid<false><<<1,1,0,stream>>>(control,accepted,trial,scratch,inverse,fixed,
      static_cast<std::uint32_t>(config.node_count),rigid_groups->device,durations,maximum_angle,stamp.epoch,attempt,
      stamp.has_rotation_presence?fixed+3*config.node_count:nullptr);
  }
  return cudaGetLastError();
}
NodalReport AdvanceStaggeredRigidGroups(FENodalState& owner,const NodalTrialToken& token,
    const NodalStaggeredHistoryAdmission& admission) {
  if(!owner.impl_) return {NodalStatus::NotInitialized,"Owner is not initialized"};
  const NodalStepAdmission declared{admission.owner_id,admission.base_epoch,admission.attempt,
    admission.maximum_dt,admission.maximum_rotation_increment,
    NodalStepAdmissionKind::RestrictedHistoryTrajectory,admission.qualification_id,0};
  return owner.impl_->AdvanceSealedNodal(token.owner_id_,token.base_epoch_,token.attempt_,declared,
    NodalTemporalScheme::StaggeredHalfKickStart,true);
}
} // namespace tl::fea
