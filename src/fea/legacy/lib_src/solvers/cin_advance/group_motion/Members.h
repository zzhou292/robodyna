// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
namespace tl::fea::cin_advance::group_motion {
TL_SURFACE_HD inline NodalStatus PrepareMember(const Input& input,const State& state,
    std::uint32_t local,rigid::MemberStepTrial& next) {
  using namespace rigid::candidate_detail;
  const auto m=input.groups.members[state.offset+local];const auto node=m.node,nodes=input.model.node_count;
  // The completed wrench phase has already checked every node before this phase.
  const rigid::MemberStepInput member{Node(input.accepted,node),Node(input.accepted+3*nodes,node),
      Node(input.accepted+6*nodes,node),Load(input.loads,node,nodes),Load(input.loads+3*nodes,node,nodes),m.mass,m.inertia};
  const auto policy=state.dependent_coefficients ? rigid::MemberCoefficientPolicy::NonnegativeDependent
      : rigid::MemberCoefficientPolicy::PositiveIndependent;
  const auto body=Read(state.input);const auto primary=Read(state.primary);
  return Status(state.count==2 ? rigid::EvaluateTwoMemberStep(body,primary,member,
      input.groups.source_length_to_m,next,policy) : rigid::EvaluateMemberStep(body,primary,member,next,policy));
}
// Leader selection finishes before any worker publishes its private candidate.
// Failure keeps the same successful source-member prefix, including prior tiles.
TL_SURFACE_HD inline void SelectPrefix(const Input& input,std::uint32_t first,
    unsigned count,Tile& tile) {
  auto& state=tile.state;state.prefix=count;
  for (unsigned local=0;local<count;++local) {
    if (tile.status[local]==NodalStatus::Ok) continue;
    state.prefix=local;
    Fail(state,tile.status[local],input.groups.members[state.offset+first+local].node);
    return;
  }
}
template<bool Capture>
TL_SURFACE_HD inline void PublishMember(const Input& input,const State& state,
    std::uint32_t local,const rigid::MemberStepTrial& next) {
  using rigid::candidate_detail::WriteNode;
  const auto node=input.groups.members[state.offset+local].node,nodes=input.model.node_count;
  WriteNode(input.trial,node,next.position);WriteNode(input.trial+3*nodes,node,next.velocity);
  WriteNode(input.trial+6*nodes,node,next.omega);WriteNode(input.trial+13*nodes,node,next.reaction_force);
  WriteNode(input.trial+16*nodes,node,next.reaction_couple);
  if constexpr(Capture) {
    rigid::AccelerationSink::Write(input.capture.node,node,next.acceleration);
    rigid::AccelerationSink::Write(input.capture.node_rotation,node,next.angular_acceleration);
  }
}
} // namespace tl::fea::cin_advance::group_motion
