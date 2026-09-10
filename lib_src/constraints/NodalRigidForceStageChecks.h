// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidForceStageObservationTypes.h"
#include "NodalRigidObservationMath.h"
#include <cstdint>
namespace tl::fea::rigid::force_stage_detail {
// Finite, scale-aware time comparisons; never manufacture an infinite budget.
TL_RIGID_OBSERVATION_HD inline bool SameTime(double a,double b) {
  return observation_detail::Near(a,b,16);
}
TL_RIGID_OBSERVATION_HD inline bool Phase(ForceStageObservationPhase p) {
  using namespace observation_detail;
  const auto d=p.durations;
  if(!Nonnegative(p.force_time)||!Nonnegative(p.input_velocity_time)||!Nonnegative(p.previous_frame_time)||
      !step_detail::Durations(d)) return false;
  // Scale the terms before adding: two finite durations need not have a finite sum.
  const double kick=.5*d.previous_drift_dt+.5*d.drift_dt;
  if(!Finite(kick)||!SameTime(d.kick_dt,kick)) return false;
  if(d.previous_drift_dt==0)
    return p.force_time==0&&p.input_velocity_time==0&&p.previous_frame_time==0;
  return p.force_time>p.input_velocity_time&&p.input_velocity_time>p.previous_frame_time&&
    SameTime(p.force_time,p.previous_frame_time+d.previous_drift_dt)&&
    SameTime(p.input_velocity_time,p.previous_frame_time+.5*d.previous_drift_dt);
}
TL_RIGID_OBSERVATION_HD inline bool Disjoint(const void* input,std::size_t bytes,
                                            const void* output,std::size_t output_bytes) {
  if(!input||!bytes||!output||!output_bytes) return false;
  const auto a=reinterpret_cast<std::uintptr_t>(input),b=reinterpret_cast<std::uintptr_t>(output);
  if(a>UINTPTR_MAX-bytes||b>UINTPTR_MAX-output_bytes) return false;
  return a>=b+output_bytes||b>=a+bytes;
}
TL_RIGID_OBSERVATION_HD inline bool Ranges(const GroupForceStageKineticInput& in,
                                           const GroupForceStageKineticObservation& out) {
  const auto n=in.metric.member_count;
  if(n<2||n>observation_detail::MaxMembers) return false;
  return Disjoint(&in,sizeof(in),&out,sizeof(out))&&
    Disjoint(in.metric.group,sizeof(NodalRigidGroupProperties),&out,sizeof(out))&&
    Disjoint(in.metric.members,n*sizeof(NodalRigidGroupMember),&out,sizeof(out))&&
    Disjoint(in.before_members,n*sizeof(MemberMotion),&out,sizeof(out))&&
    Disjoint(in.member_acceleration,n*sizeof(ForceStageAcceleration),&out,sizeof(out));
}
TL_RIGID_OBSERVATION_HD inline bool UniqueMembers(GroupObservationMetric metric) {
  for(std::size_t i=0;i<metric.member_count;++i) for(std::size_t j=0;j<i;++j)
    if(metric.members[i].source_node_id==metric.members[j].source_node_id||
       metric.members[i].global_node==metric.members[j].global_node) return false;
  return true;
}
// Native operation order V + A*DT05. The caller supplies actual constrained A/AR.
TL_RIGID_OBSERVATION_HD inline bool Collocate(MemberMotion before,ForceStageAcceleration a,
                                             double half_previous,MemberMotion& out) {
  if(!detail::Finite(before.velocity)||!detail::Finite(before.omega)||
     !detail::Finite(a.translation)||!detail::Finite(a.rotation)) return false;
  out={{before.velocity.x+a.translation.x*half_previous,before.velocity.y+a.translation.y*half_previous,
        before.velocity.z+a.translation.z*half_previous},
       {before.omega.x+a.rotation.x*half_previous,before.omega.y+a.rotation.y*half_previous,
        before.omega.z+a.rotation.z*half_previous}};
  return detail::Finite(out.velocity)&&detail::Finite(out.omega);
}
} // namespace tl::fea::rigid::force_stage_detail
