// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidKineticObservation.h"
namespace tl::fea::rigid {
namespace observation_detail {
struct KickSums { Sum kinetic,applied_translation,applied_rotation,reaction_translation,reaction_rotation,budget; };
TL_RIGID_OBSERVATION_HD inline ObservationReport AddKickComponent(KickSums& sums,double coefficient,
    double before,double after,double applied,double reaction,double dt,std::size_t member,unsigned dof) {
  if(!Finite(before)||!Finite(after)||!Finite(applied)||!Finite(reaction))
    return {ObservationStatus::InvalidInput,member,dof};
  const double average=.5*before+.5*after,change=after-before;
  const double impulse=coefficient*change,applied_impulse=dt*applied,reaction_impulse=dt*reaction;
  const double impulse_residual=(impulse-applied_impulse)-reaction_impulse;
  // A rounded velocity kick need not retain a sub-ULP increment. Include both
  // stored endpoint magnitudes plus load/reaction arithmetic, not work alone.
  const double impulse_scale=coefficient*(::fabs(before)+::fabs(after))+
      ::fabs(applied_impulse)+::fabs(reaction_impulse);
  const double impulse_budget=64*Epsilon*impulse_scale;
  const double kinetic=impulse*average,aw=applied_impulse*average,rw=reaction_impulse*average;
  const double budget=impulse_budget*::fabs(average)+32*Epsilon*(::fabs(kinetic)+::fabs(aw)+::fabs(rw));
  const double values[]{average,change,impulse,applied_impulse,reaction_impulse,impulse_residual,impulse_budget,kinetic,aw,rw,budget};
  for(double value:values) if(!Finite(value)) return {ObservationStatus::NonfiniteResult,member,dof};
  // Check every impulse, including zero-average-velocity reversals whose net
  // energy happens to vanish. A corrupt reaction cannot hide in global sums.
  if(::fabs(impulse_residual)>impulse_budget)
    return {ObservationStatus::KickMismatch,member,dof,impulse_residual,impulse_budget};
  auto& applied_sum=dof<3?sums.applied_translation:sums.applied_rotation;
  auto& reaction_sum=dof<3?sums.reaction_translation:sums.reaction_rotation;
  if(!sums.kinetic.Add(kinetic)||!applied_sum.Add(aw)||!reaction_sum.Add(rw)||!sums.budget.Add(budget))
    return {ObservationStatus::NonfiniteResult,member,dof};
  return {};
}
TL_RIGID_OBSERVATION_HD inline bool QuadraticDelta(Sum& sum,double coefficient,double before,double after) {
  return sum.Add(coefficient*(after-before)*(.5*before+.5*after));
}
}

// Actual kick identity, including physical member force AND couple reactions.
// Reactions are constraint work; no dissipative/material interpretation is made.
// All output remains unchanged on input, overflow or kick-identity rejection.
TL_RIGID_OBSERVATION_HD inline ObservationReport ObserveGroupKick(const GroupKickInput& input,GroupKickObservation& output) {
  using namespace observation_detail;
  if(!input.applied||!input.reaction||!input.before_members||!input.after_members||
      !Finite(input.kick_dt)||input.kick_dt<=0) return {ObservationStatus::InvalidInput};
  const auto before=input.before_phase,after=input.after_phase;
  // Scale each finite time before summing, so large absolute times cannot
  // overflow the tolerance into an unlimited admission budget.
  const double phase_budget=32*Epsilon*::fabs(after.velocity_time)+
      32*Epsilon*::fabs(before.velocity_time)+32*Epsilon*input.kick_dt;
  if(!ValidPhase(before)||!ValidPhase(after)||after.kind!=ObservationPhaseKind::StoredMidpointWithLaggedFrame||
      after.frame_time!=before.position_time||after.position_time<=before.position_time||
      after.velocity_time<=before.velocity_time||!Finite(phase_budget)||
      ::fabs((after.velocity_time-before.velocity_time)-input.kick_dt)>phase_budget)
    return {ObservationStatus::UnsupportedPhase};
  GroupKickObservation next;
  auto report=ObserveGroupKinetic({input.metric,input.before_members,input.before_group,before},next.before);
  if(!report) return report;
  report=ObserveGroupKinetic({input.metric,input.after_members,input.after_group,after},next.after);
  if(!report) return report;
  KickSums sums;
  for(std::size_t i=0;i<input.metric.member_count;++i) {
    const auto& metric=input.metric.members[i];
    for(unsigned axis=0;axis<6;++axis) {
      const bool translation=axis<3; const auto a=axis%3;
      report=AddKickComponent(sums,translation?metric.mass_kg:metric.total_inertia_kg_m2,
        Get(translation?input.before_members[i].velocity:input.before_members[i].omega,a),
        Get(translation?input.after_members[i].velocity:input.after_members[i].omega,a),
        Get(translation?input.applied[i].force:input.applied[i].couple,a),
        Get(translation?input.reaction[i].force:input.reaction[i].couple,a),input.kick_dt,i,axis);
      if(!report) return report;
    }
  }
  next.native_delta=sums.kinetic.Value();
  next.applied.translation=sums.applied_translation.Value(); next.applied.rotation=sums.applied_rotation.Value();
  next.reaction.translation=sums.reaction_translation.Value(); next.reaction.rotation=sums.reaction_rotation.Value();
  next.applied.total=next.applied.translation+next.applied.rotation;
  next.reaction.total=next.reaction.translation+next.reaction.rotation;
  const auto old_w=detail::ToLocal(input.before_group.principal_axes,input.before_group.omega);
  const auto new_w=detail::ToLocal(input.after_group.principal_axes,input.after_group.omega);
  Sum aggregate_delta;
  for(unsigned a=0;a<3;++a)
    if(!QuadraticDelta(aggregate_delta,input.metric.group->total_mass_kg,
          Get(input.before_group.velocity,a),Get(input.after_group.velocity,a))||
        !QuadraticDelta(aggregate_delta,Get(input.metric.group->principal.inertia,a),Get(old_w,a),Get(new_w,a)))
      return {ObservationStatus::NonfiniteResult};
  next.aggregate_delta=aggregate_delta.Value(); next.replacement_delta=next.aggregate_delta-next.native_delta;
  next.native_residual=(next.native_delta-next.applied.total)-next.reaction.total;
  next.effective_residual=((next.aggregate_delta-next.applied.total)-next.reaction.total)-next.replacement_delta;
  next.roundoff_budget=sums.budget.Value()+32*Epsilon*(::fabs(next.aggregate_delta)+::fabs(next.replacement_delta)+
      ::fabs(next.native_delta)+::fabs(next.applied.total)+::fabs(next.reaction.total));
  const double values[]{next.native_delta,next.aggregate_delta,next.replacement_delta,next.native_residual,
      next.effective_residual,next.roundoff_budget};
  for(double value:values) if(!Finite(value)) return {ObservationStatus::NonfiniteResult};
  const double residual=::fmax(::fabs(next.native_residual),::fabs(next.effective_residual));
  if(residual>next.roundoff_budget) return {ObservationStatus::KickMismatch,SIZE_MAX,6,residual,next.roundoff_budget};
  output=next; return {};
}
} // namespace tl::fea::rigid
