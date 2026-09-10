// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidForceStageChecks.h"
#include "NodalRigidKineticValues.h"
namespace tl::fea::rigid {
// Value-only observer. Caller authenticates source order, actual constrained
// accelerations and updated force frame from one owner packet. Raw values cannot
// prove that a supplied orthonormal frame/velocity is the correct historical one.
// No allocation, force evaluation, frame advance, state change or energy receipt.
// Every output byte and input remains unchanged on failure, including alias cases.
TL_RIGID_OBSERVATION_HD inline ObservationReport ObserveGroupForceStageKinetic(
    const GroupForceStageKineticInput& input,GroupForceStageKineticObservation& output) {
  using namespace force_stage_detail;
  if(!Ranges(input,output)) return {ObservationStatus::InvalidInput};
  if(!Phase(input.phase)) return {ObservationStatus::UnsupportedPhase};
  if(!observation_detail::ValidMetric(input.metric)||!UniqueMembers(input.metric))
    return {ObservationStatus::InvalidMetric};
  if(!detail::Orthonormal(input.force_frame)) return {ObservationStatus::InvalidInput};
  GroupForceStageKineticObservation next; next.phase=input.phase;
  next.source_group_id=input.metric.group->source_group_id;
  next.source_node_set_id=input.metric.group->source_node_set_id;next.member_count=input.metric.member_count;
  const double half_previous=.5*input.phase.durations.previous_drift_dt;
  if(!Collocate(input.before_primary,input.primary_acceleration,half_previous,next.collocated_primary))
    return {ObservationStatus::NonfiniteResult};
  MemberMotion members[observation_detail::MaxMembers];
  for(std::size_t i=0;i<input.metric.member_count;++i)
    if(!Collocate(input.before_members[i],input.member_acceleration[i],half_previous,members[i]))
      return {ObservationStatus::NonfiniteResult,i};
  // Phase-independent shared arithmetic; no fake PhysicalInitialization phase.
  const NodalRigidGroupState group{input.metric.group->center,next.collocated_primary.velocity,
                                  next.collocated_primary.omega,input.force_frame};
  GroupKineticObservation values;
  const auto report=observation_detail::KineticValues({input.metric,members,group,{}},values);
  if(!report) return report;
  next.members=values.members;next.aggregate=values.aggregate;next.replacement=values.replacement;
  output=next;return {};
}
} // namespace tl::fea::rigid
