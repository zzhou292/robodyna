// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../NormalResponse.h"
namespace tlfea::contact::radioss_type25::initial_state {
// One writer per source secondary. Distance/tie order is Starter PEN3, not
// Engine maximum-penetration classification. Scratch is never runtime TIME_S.
TL_MATH_HOST_DEVICE inline Status Consider(const PairResult& value,Winner& winner) {
  if(!value.considered)return Status::Ok;
  if(value.key.main_segment<=0||value.local_main<=0||value.sector<1||value.sector>4||
      !normal_detail::Nonnegative(value.distance_squared)||!normal_detail::Nonnegative(value.penetration_offset)||
      !normal_detail::Nonnegative(winner.distance_squared)||!normal_detail::Nonnegative(winner.row.penetration_offset))return Status::InvalidInput;
  if(value.distance_squared<winner.distance_squared||
      (value.distance_squared==winner.distance_squared&&winner.row.irtlm[0]<value.key.main_segment)) {
    winner.distance_squared=value.distance_squared;
    winner.row.irtlm[0]=value.key.main_segment;winner.row.irtlm[1]=value.sector;
    winner.row.irtlm[2]=value.local_main;winner.row.penetration_offset=value.penetration_offset;
  }
  return Status::Ok;
}
TL_MATH_HOST_DEVICE inline void FinalizeInacti5(Winner& winner) {
  // Original PWR3 keeps nonzero offsets; selected zero-offset rows become cold.
  if(winner.row.penetration_offset==0)for(unsigned i=0;i<3;++i)winner.row.irtlm[i]=0;
}
// PREPARE_INT25's source-produced table supplies a local ordinal and partition.
// Native tied-removal normalization must run BEFORE this mapping.
TL_MATH_HOST_DEVICE inline Status MapPreparedMain(int local,int partition,Winner& winner) {
  if(winner.row.irtlm[2]==0)return Status::Ok;
  if(winner.row.irtlm[2]<0||winner.row.irtlm[0]<=0||local<=0||partition!=1)return Status::InvalidInput;
  winner.row.irtlm[2]=local;winner.row.irtlm[3]=partition;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::initial_state
