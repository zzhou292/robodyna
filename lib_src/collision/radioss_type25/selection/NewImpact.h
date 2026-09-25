// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NewImpactDecision.h"
#include <climits>
namespace tlfea::contact::radioss_type25::selection {
// Complete selected local COR3_22/DST3_22/GLOB22 arithmetic and native crossing
// helpers. Authenticated phase membership, partner binding and row ownership are
// coordinator prerequisites; the value API creates no physical authority.
TL_MATH_HOST_DEVICE inline Status EvaluateNativeNewImpact(const Profile& profile,
    const NativeNewImpactInput& input,const NativeGeometryHistory& prior,
    NativeNewImpactResult* output) {
  using namespace detail;
  if(!output||!Valid(input,prior)||prior.row.irtlm[0]==INT_MIN)return Status::InvalidInput;
  if(!Supported(profile)||input.pair.radiation_range!=0||input.pair.applied_gap!=0)
    return Status::UnsupportedProfile;
  NativeNewImpactResult result;Initialize(input,prior,result);
  if(!tl::math::Finite(result.classification_product))return Status::NonfiniteResult;
  ImpactWork work;Prepare(input,work);
  const auto crossings=Intersections(input,work);
  result.primary.intersection=crossings.primary;result.opposite.intersection=crossings.opposite;
  result.recontact_intersection=crossings.recontact;
  ProjectNewImpact(input,work,result);
  ImpactPenetrations(input,work,result);
  BoundNewImpactSide(input.pair,work.geometry,work.geometry.frame.normal,result.primary,false);
  if(work.opposite_local!=0)
    BoundNewImpactSide(work.opposite_boundary,work.geometry,work.opposite_normal,result.opposite,true);
  ChooseNewImpact(profile,input,result);
  if(!Finite(result))return Status::NonfiniteResult;
  *output=result;return Status::Ok;
}
} // namespace tlfea::contact::radioss_type25::selection
