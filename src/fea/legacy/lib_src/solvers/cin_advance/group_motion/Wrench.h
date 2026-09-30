// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
namespace tl::fea::cin_advance::group_motion {
TL_SURFACE_HD inline WrenchSlot PrepareWrench(const Input& input,const State& state,std::uint32_t local) {
  using namespace rigid::candidate_detail;
  WrenchSlot out{};out.node=input.groups.members[state.offset+local].node;
  out.status=NodalStatus::InvalidOutput;
  const auto node=out.node,nodes=input.model.node_count;
  if (node>=nodes) return out; // Never dereference a speculative invalid node.
  const auto x=Node(input.accepted,node),f=Load(input.loads,node,nodes),c=Load(input.loads+3*nodes,node,nodes);
  rigid::Wrench contribution;
  if (rigid::AggregateWrench(Read(state.input.center),&x,&f,&c,1,contribution)!=rigid::MathStatus::Success)
    return out;
  out.force=Store(contribution.force);out.couple=Store(contribution.couple);
  out.status=NodalStatus::Ok;return out;
}
TL_SURFACE_HD inline void FoldWrench(const WrenchSlot& value,State& state) {
  if (value.status!=NodalStatus::Ok) {Fail(state,value.status,value.node);return;}
  // The original left fold, including its intermediate overflow check. Neither
  // a tree sum nor a pre-summed tile preserves this failure/value contract.
  const auto force=rigid::detail::Add(Read(state.input.force),Read(value.force));
  const auto couple=rigid::detail::Add(Read(state.input.couple),Read(value.couple));
  state.input.force=Store(force);state.input.couple=Store(couple);
  if (!rigid::detail::Finite(force)||!rigid::detail::Finite(couple))
    Fail(state,NodalStatus::InvalidOutput,value.node);
}
} // namespace tl::fea::cin_advance::group_motion
