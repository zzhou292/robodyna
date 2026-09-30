// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Boundary.h"
namespace tlfea::contact::radioss_type25::initial_state {
TL_MATH_HOST_DEVICE inline bool Supported(const Profile& p) {
  return p.gap_mode==1&&p.initial_penetration==5&&p.damping_flag==1&&
      (p.sharp==1||p.sharp==2)&&p.arithmetic_precision==0&&p.partitions==1;
}
TL_MATH_HOST_DEVICE inline Status EvaluatePair(const PairInput& input,PairResult* output) {
  const auto& p=input.profile;const auto& in=input.geometry;
  if(!Supported(p))return Status::UnsupportedProfile;
  NativeGeometryHistory virgin;virgin.secondary_source_id=in.key.secondary_source_id;virgin.generation=in.key.generation;
  // Reuse the existing geometric coherence check without inventing finite
  // requirements for native-unread response coefficients or unbound VTX slots.
  auto checked=in;checked.main_coefficient=0;checked.secondary_coefficient=0;
  for(unsigned k=0;k<4;++k)if(!checked.boundary_ids[k])
    for(unsigned j=0;j<2;++j)checked.vertex_bisector[k][j]={};
  if(!output||input.expanded_main_count<=0||input.expanded_main_count>1073741823||in.local_main>input.expanded_main_count||
      in.key.main_segment>input.expanded_main_count||in.segment_type==(-2147483647-1)||
      in.segment_type>2*input.expanded_main_count||in.segment_type< -2*input.expanded_main_count||
      in.main_gap_max!=native_constant::ep20*native_constant::ep10||
      in.radiation_range!=0||in.applied_gap!=0||!selection::detail::Valid(checked,virgin))return Status::InvalidInput;
  PairResult next;next.key=in.key;next.local_main=in.local_main;
  // The opposite to a coating shell is excluded by original PEN3's INDX.
  if(in.segment_type< -input.expanded_main_count){*output=next;return Status::Ok;}
  detail::PairWork work;
  if(!detail::Project(input,work))return Status::UndefinedNativeInput;
  detail::FreeCone(input,work);detail::Boundary(input,work);detail::Sharp(input,work);
  const auto& q=work.projection.sector[work.selected];
  next.sector=int(work.selected+1);next.far=q.far;next.distance_squared=q.distance_squared;
  next.penetration_offset=q.penetration;
  if(!normal_detail::Nonnegative(next.distance_squared)||!normal_detail::Nonnegative(next.penetration_offset))return Status::NonfiniteResult;
  next.considered=in.segment_type==0 || (q.far!=2 &&
      (in.segment_type>input.expanded_main_count||work.value.plane_distance[work.selected]<=0));
  *output=next;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::initial_state
