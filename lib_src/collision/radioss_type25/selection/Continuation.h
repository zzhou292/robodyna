// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ContinuationBoundary.h"
namespace tlfea::contact::radioss_type25::selection {
namespace detail {
TL_MATH_HOST_DEVICE inline void PublishContinuationWinner(const Profile& profile,
    const NativeContinuationInput& in,NativeContinuationResult& out) {
  if(out.selected_subtriangle==0)return;
  if(out.sector[out.selected_subtriangle-1].penetration==0) {
    out.selected_subtriangle=0;return;
  }
  auto& row=out.history.row;
  if(out.distance_squared<onep02*row.selection_metric[0]&&
     (out.distance_squared<row.selection_metric[1]||
      (out.distance_squared==row.selection_metric[1]&&row.irtlm[0]<in.pair.key.main_segment))) {
    row.irtlm[0]=in.pair.key.main_segment;row.irtlm[1]=out.selected_subtriangle;
    row.irtlm[2]=in.pair.local_main;row.irtlm[3]=profile.local_processor;
    row.selection_metric[1]=out.distance_squared;out.row_replaced=true;
  }
}
} // namespace detail
// Pure complete local COR3_21/DST3_21/GLOB packet. The caller owns the original
// continuation-list phase snapshot; scalar evaluation grants no runtime authority.
TL_MATH_HOST_DEVICE inline Status EvaluateNativeContinuation(const Profile& profile,
    const NativeContinuationInput& input,const NativeGeometryHistory& prior,
    NativeContinuationResult* output) {
  using namespace detail;
  if(!output||!Valid(input,prior))return Status::InvalidInput;
  if(!Supported(profile)||input.pair.radiation_range!=0||input.pair.applied_gap!=0)
    return Status::UnsupportedProfile;
  NativeContinuationResult result;Initialize(input,prior,result);
  if(!tl::math::Finite(result.classification_product))return Status::NonfiniteResult;
  if(result.active) {
    Work work;Prepare(input.pair,work);RawProjection(work,result);
    const unsigned count=work.frame.triangle?1:4;
    for(unsigned i=0;i<count;++i) {
      const double gap=ProjectSector(input.pair,work,result,i,true);
      if(result.sector[i].distance_squared<=gap*gap&&work.plane_distance[i]<=0)
        result.cylindrical_gap[i]=1;
    }
    result.selected_subtriangle=SelectClosest(work,result,0);
    for(unsigned i=0;i<4;++i)if(int(i+1)!=result.selected_subtriangle)result.sector[i].penetration=0;
    ConstrainContinuation(input,work,result);
    ContinuationBoundary(input,work,result);
    PublishContinuationWinner(profile,input,result);
  }
  if(!Finite(result))return Status::NonfiniteResult;
  PublishCache(result);*output=result;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::selection
