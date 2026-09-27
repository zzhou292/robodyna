// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
namespace tl::fea::cin_advance::group_motion {
template<bool Capture>
TL_SURFACE_HD inline State Begin(const Input& input,std::uint32_t group) {
  State state{};
  state.report={0,UINT32_MAX,UINT32_MAX,NodalStatus::Ok,false,false};
  const auto view=input.groups;
  if (!view.groups||!view.members||!input.accepted||!input.trial||!input.loads||group>=view.group_count) {
    Fail(state,NodalStatus::InvalidOutput,UINT32_MAX);return state;
  }
  if constexpr(Capture) {
    const auto sink=input.capture;
    if (!sink.node||!sink.node_rotation||!sink.group||!sink.group_rotation) {
      Fail(state,NodalStatus::InvalidOutput,UINT32_MAX);return state;
    }
  }
  const auto range=view.groups[group];
  if (range.count<2||range.offset>view.member_count||range.count>view.member_count-range.offset) {
    Fail(state,NodalStatus::InvalidOutput,UINT32_MAX);return state;
  }
  state.offset=range.offset;state.count=range.count;
  state.dependent_coefficients=range.dependent_coefficients;
  const auto offset=19*std::size_t(input.model.node_count)+rigid::GroupStateValues*group;
  const auto prior=rigid::ReadGroupState(input.accepted+offset);
  // Copy only. Primary numerical validation must follow the entire wrench fold.
  state.input=Store(rigid::PrimaryStepInput{{prior.principal_axes,range.principal_inertia},
      prior.center,prior.velocity,prior.omega,range.mass,{},input.durations});
  return state;
}
TL_SURFACE_HD inline void PreparePrimary(const Input& input,State& state) {
  const auto body=Read(state.input);
  rigid::PrimaryStepTrial primary;
  const auto status=state.count==2 ? rigid::EvaluateTwoMemberPrimaryStep(body,primary)
      : rigid::EvaluatePrimaryStep(body,primary);
  if (status!=rigid::StepStatus::Success) {
    Fail(state,Status(status),input.groups.members[state.offset].node);return;
  }
  state.primary=Store(primary);
}
template<bool Capture>
TL_SURFACE_HD inline void PublishGroup(const Input& input,std::uint32_t group,const State& state) {
  const auto primary=Read(state.primary);
  const auto offset=19*std::size_t(input.model.node_count)+rigid::GroupStateValues*group;
  rigid::WriteGroupState(input.trial+offset,
      {primary.center,primary.velocity,primary.omega,primary.force_frame.axes});
  if constexpr(Capture) {
    rigid::AccelerationSink::Write(input.capture.group,group,primary.acceleration);
    rigid::AccelerationSink::Write(input.capture.group_rotation,group,primary.angular_acceleration);
  }
}
} // namespace tl::fea::cin_advance::group_motion
