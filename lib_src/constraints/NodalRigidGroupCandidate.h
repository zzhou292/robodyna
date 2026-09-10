// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupState.h"
#include "NodalRigidTwoMemberStep.h"
#include "NodalRigidAccelerationSink.h"

#if defined(__CUDACC__)
#define TL_RIGID_CANDIDATE_HD __host__ __device__
#else
#define TL_RIGID_CANDIDATE_HD
#endif
namespace tl::fea::rigid {
struct GroupCandidateResult { StepStatus status=StepStatus::Success; std::uint32_t node=UINT32_MAX; };
namespace candidate_detail {
TL_RIGID_CANDIDATE_HD inline Vec3 Node(const double* data,std::uint32_t node) {
  return {data[3*node],data[3*node+1],data[3*node+2]};
}
TL_RIGID_CANDIDATE_HD inline Vec3 Load(const double* data,std::uint32_t node,std::uint32_t n) {
  return {data[node],data[n+node],data[2*n+node]};
}
TL_RIGID_CANDIDATE_HD inline void WriteNode(double* data,std::uint32_t node,Vec3 value) {
  data[3*node]=value.x; data[3*node+1]=value.y; data[3*node+2]=value.z;
}
}
// Private assembly adapter for the declared x3/v3/omega3/q4/reactionF3/
// reactionC3 nodal layout followed by 18 values per group. Only trial values
// and the optional transient acceleration sink are written. A caller must
// discard the whole trial on any failure; no view
// or acceptance is published here. Immutable membership is startup-validated.
template<bool Capture=false>
TL_RIGID_CANDIDATE_HD inline GroupCandidateResult PrepareGroupCandidate(GroupDeviceView view,
    std::uint32_t group,const double* accepted,double* trial,const double* loads,
    std::uint32_t nodes,StepDurations durations,AccelerationSink sink={}) {
  using namespace candidate_detail;
  if(!view.groups||!view.members||!accepted||!trial||!loads||group>=view.group_count)
    return {StepStatus::InvalidInput};
  if constexpr(Capture)
    if(!sink.node||!sink.node_rotation||!sink.group||!sink.group_rotation) return {StepStatus::InvalidInput};
  const auto range=view.groups[group];
  if(range.count<2||range.offset>view.member_count||range.count>view.member_count-range.offset)
    return {StepStatus::InvalidInput};
  const auto offset=19*std::size_t(nodes)+GroupStateValues*group;
  const auto prior=ReadGroupState(accepted+offset);
  PrimaryStepInput input{{prior.principal_axes,range.principal_inertia},prior.center,prior.velocity,
                         prior.omega,range.mass,{},durations};
  for(std::uint32_t i=0;i<range.count;++i) {
    const auto node=view.members[range.offset+i].node;
    if(node>=nodes) return {StepStatus::InvalidInput,node};
    const auto x=Node(accepted,node),f=Load(loads,node,nodes),c=Load(loads+3*nodes,node,nodes);
    Wrench contribution;
    if(AggregateWrench(prior.center,&x,&f,&c,1,contribution)!=MathStatus::Success)
      return {StepStatus::NonfiniteResult,node};
    input.applied.force=detail::Add(input.applied.force,contribution.force);
    input.applied.couple=detail::Add(input.applied.couple,contribution.couple);
    if(!detail::Finite(input.applied.force)||!detail::Finite(input.applied.couple))
      return {StepStatus::NonfiniteResult,node};
  }
  PrimaryStepTrial primary;
  const auto first_node=view.members[range.offset].node;
  auto status=range.count==2 ? EvaluateTwoMemberPrimaryStep(input,primary) : EvaluatePrimaryStep(input,primary);
  if(status!=StepStatus::Success) return {status,first_node};
  for(std::uint32_t i=0;i<range.count;++i) {
    const auto m=view.members[range.offset+i]; const auto node=m.node;
    const MemberStepInput member{Node(accepted,node),Node(accepted+3*nodes,node),Node(accepted+6*nodes,node),
      Load(loads,node,nodes),Load(loads+3*nodes,node,nodes),m.mass,m.inertia};
    MemberStepTrial next;
    status=range.count==2 ? EvaluateTwoMemberStep(input,primary,member,view.source_length_to_m,next)
                         : EvaluateMemberStep(input,primary,member,next);
    if(status!=StepStatus::Success) return {status,node};
    WriteNode(trial,node,next.position); WriteNode(trial+3*nodes,node,next.velocity);
    WriteNode(trial+6*nodes,node,next.omega); WriteNode(trial+13*nodes,node,next.reaction_force);
    WriteNode(trial+16*nodes,node,next.reaction_couple);
    if constexpr(Capture) {
      AccelerationSink::Write(sink.node,node,next.acceleration);
      AccelerationSink::Write(sink.node_rotation,node,next.angular_acceleration);
    }
  }
  WriteGroupState(trial+offset,{primary.center,primary.velocity,primary.omega,primary.force_frame.axes});
  if constexpr(Capture) {
    AccelerationSink::Write(sink.group,group,primary.acceleration);
    AccelerationSink::Write(sink.group_rotation,group,primary.angular_acceleration);
  }
  return {};
}
} // namespace tl::fea::rigid
#undef TL_RIGID_CANDIDATE_HD
