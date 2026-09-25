// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "RetainedDecision.h"
namespace tlfea::contact::radioss_type25::selection {
// Complete defined retained COR3_1/DST3_1/GLOB_1 packet. Runtime admission still
// requires the authentic retained list; this pure function creates no owner.
TL_MATH_HOST_DEVICE inline Status EvaluateNativeRetained(const Profile& profile,
    const NativePairInput& input,const NativeGeometryHistory& prior,
    NativeRetainedResult* output) {
  using namespace detail;
  if(!output||!Valid(input,prior))return Status::InvalidInput;
  if(!Supported(profile)||input.radiation_range!=0||input.applied_gap!=0)
    return Status::UnsupportedProfile;
  NativeRetainedResult result;Initialize(input,prior,result);
  if(!tl::math::Finite(result.classification_product))return Status::NonfiniteResult;
  if(result.active) {
    const bool triangle=input.main_node_ids[2]==input.main_node_ids[3];
    if(result.prior_subtriangle<1||result.prior_subtriangle>4||
        (triangle&&result.prior_subtriangle!=1))return Status::UndefinedNativeInput;
    if(prior.row.irtlm[0]!=input.key.main_segment||prior.row.irtlm[2]!=input.local_main||
       prior.row.irtlm[3]!=profile.local_processor)return Status::InvalidInput;
    Work work;Prepare(input,work);
    RetainedGeometry(input,work,result);
    SelectRetained(input,work,result);
  }
  if(!Finite(result))return Status::NonfiniteResult;
  PublishCache(result);
  *output=result;
  return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::selection
