// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
namespace tl::fea::cin_advance::group_motion {
TL_SURFACE_HD inline NodalStatus PrepareOrientation(const Input& input,const State& state,
    std::uint32_t local,tl::math::Quaternion& next) {
  const auto node=input.groups.members[state.offset+local].node;
  return nodal_detail::PrepareNodeOrientationValue(input.accepted,input.trial,node,
      input.model.node_count,input.durations.drift_dt,input.maximum_angle,false,next);
}
TL_SURFACE_HD inline void PublishOrientation(const Input& input,const State& state,
    std::uint32_t local,const tl::math::Quaternion& next) {
  const auto node=input.groups.members[state.offset+local].node;
  auto* q=input.trial+9*input.model.node_count+4*node;
  q[0]=next.w;q[1]=next.x;q[2]=next.y;q[3]=next.z;
}
} // namespace tl::fea::cin_advance::group_motion
